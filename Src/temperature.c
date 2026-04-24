/* USER CODE BEGIN Header */
/**
 * Professional Weather Station Firmware - Temperature Module
 * Optimized with Energy Balance Model & Robust I2C State Machine
 */
/* USER CODE END Header */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "stm32l1xx.h"
#include "my_time.h"
#include "temperature.h"
#include "I2C.h"
#include "debug.h"

// --- COSTANTI FISICHE PER BILANCIO ENERGETICO ---
#define LUX_TO_W_M2      0.0079f
#define SHIELD_ALPHA     0.0014f
#define CONV_A           4.50f
#define CONV_B           2.10f
#define MAX_CORR_DECI    25

// --- VARIABILI ESTERNE ---
extern uint32_t lightMeasurement;
extern int16_t lastSampleX;
extern int16_t lastSampleY;

#ifdef DEBUG_TEMP
const uint8_t tempMeasurementInterval_s = 3;
#else
const uint8_t tempMeasurementInterval_s = 30;
#endif

enum TempSensorEnum { TS_Missing, SHT3x, SHT4x, HTU21C };
enum TempSensorEnum tempSensorType;

// FIX: Prevenzione overflow silenzioso
const uint32_t tempMeasurementInterval_rtc = (uint32_t)tempMeasurementInterval_s * WAKEUP_FREQUENCY;
const uint16_t tempSensorDelay = 1;

uint32_t lastTempMeasurementTicks = 0xFFFF;
int16_t g_tempMeasurement = 0;
uint16_t g_humidityMeasurement = 0;

enum TempSensorState { TS_Idle, TS_Acquiring };
enum TempSensorState tempSensorState = TS_Idle;

void DetermineTempSensor();
bool WakeUpTempSensor();
bool GetTemperature(int16_t *temperature, uint16_t *humidity);

/**
 * @brief Modello a bilancio energetico
 */
int16_t Apply_Energy_Balance(int16_t t_raw_deci) {
    // FIX: Sostituito g_lux con lightMeasurement
    if (lightMeasurement < 1500) return t_raw_deci;

    float fx = (float)lastSampleX;
    float fy = (float)lastSampleY;
    float wind_speed_mps = sqrtf((fx * fx) + (fy * fy)) / 100.0f;

    if (wind_speed_mps < 0.1f) wind_speed_mps = 0.1f;

    float Rs = (float)lightMeasurement * LUX_TO_W_M2;
    float heat_dissipation = CONV_A + CONV_B * sqrtf(wind_speed_mps);
    float delta_t = (SHIELD_ALPHA * Rs) / heat_dissipation;

    int16_t correction = (int16_t)(delta_t * 10.0f);
    if (correction > MAX_CORR_DECI) correction = MAX_CORR_DECI;

    #ifdef DEBUG_TEMP
    // FIX: Sostituito g_lux con lightMeasurement anche nel print di debug
    debug_print("Temp Corr: -%d.%d C (Wind: %d cm/s, Lux: %ld)\r\n",
                correction/10, correction%10, (int)(wind_speed_mps*100), lightMeasurement);
    #endif

    return t_raw_deci - correction;
}

void ProcessTemperature()
{
    uint32_t ticks = g_rtcTicks;
    switch (tempSensorState)
    {
        case TS_Idle:
            if (ticks - lastTempMeasurementTicks > tempMeasurementInterval_rtc)
            {
                if (!WakeUpTempSensor()) {
                    #ifdef DEBUG_TEMP
                    debug_print("Failed to begin measurement.\r\n");
                    #endif
                    lastTempMeasurementTicks = ticks; // Ritenta al prossimo giro
                } else {
                    tempSensorState = TS_Acquiring;
                    lastTempMeasurementTicks = ticks;
                    WaitForStop();
                }
            }
            break;

        case TS_Acquiring:
            if (ticks - lastTempMeasurementTicks > tempSensorDelay)
            {
                if (!GetTemperature(&g_tempMeasurement, &g_humidityMeasurement)) {
                    #ifdef DEBUG_TEMP
                    debug_print("Failed to get temperature.\r\n");
                    #endif
                }
                tempSensorState = TS_Idle;
                WaitForStop();
            }
            break;
    }
}

bool GetTemperature(int16_t *temperature, uint16_t *humidity)
{
    uint8_t address;
    uint16_t tempMultiplier = 1750;
    uint16_t tempOffset = 50;
    uint8_t humidityFactor = 125;
    uint8_t humidityOffset = 6;
    uint16_t tempRaw, humidityRaw, tempPlus400;

    if (tempSensorType == TS_Missing) return false;

    if (tempSensorType == HTU21C) {
        address = 0x80;
        tempMultiplier = 1757;
        tempOffset = 68;
    } else {
        address = 0x88;
        if (tempSensorType == SHT3x) {
            humidityFactor = 100;
            humidityOffset = 0;
        }
    }

    uint8_t buffer[6];

    if (tempSensorType != HTU21C) {
        if (!I2CReadFromAddress(address, buffer, 6)) return false;

        tempRaw = ((uint16_t)buffer[0] << 8) | buffer[1];
        humidityRaw = ((uint16_t)buffer[3] << 8) | buffer[4];

    } else {
        if (!I2CReadFromAddress(address, buffer, 3)) return false;

        uint8_t cmd_temp = 0xF3;
        if (!I2CWriteToAddress(address, &cmd_temp, 1)) return false;
        delay_stopped(50);

        if (!I2CReadFromAddress(address, buffer + 3, 3)) return false;

        humidityRaw = ((uint16_t)buffer[0] << 8) | buffer[1];
        tempRaw = ((uint16_t)buffer[3] << 8) | buffer[4];
    }

    #ifdef DEBUG_TEMP
    debug_print(
        "buf: %02x %02x %02x %02x %02x %02x\r\n",
        buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
    debug_print("tempRaw: %d\r\n", tempRaw);
    #endif

    // Rimosso tempRaw == 0 perché può indicare esattamente -45°C su SHT3x
    if (tempRaw == 0xFFFF) return false;

    // --- Conversione Temperatura ---
    uint16_t tempTemp = ((uint32_t)tempRaw * tempMultiplier) >> 16;
    if (tempTemp < tempOffset) tempPlus400 = 0;
    else tempPlus400 = tempTemp - tempOffset;

    // RIPRISTINATA LOGICA ORIGINALE DI CLIPPING ED ERRORE RADIO
    if (tempPlus400 > 1050)
        tempPlus400 = 0xFFFF;
    else if (tempPlus400 > 1000)
        tempPlus400 = 1000;

    // --- Conversione Umidità ---
    uint16_t tempHumidity = ((uint32_t)humidityFactor * humidityRaw) >> 16;
    if (tempHumidity < humidityOffset) tempHumidity = 0;
    else tempHumidity -= humidityOffset;

    // Cap esplicito sull'umidità massima (100.0%)
    if (tempHumidity > 1000) tempHumidity = 1000;

    *humidity = tempHumidity;

    // --- APPLICAZIONE MODELLO PRO ---
    // Se il sensore ha dato errore (0xFFFF), bypassiamo il bilancio energetico
    if (tempPlus400 == 0xFFFF) {
        *temperature = 0xFFFF - 400; // Mantiene l'errore per la radio
    } else {
        int16_t t_raw_final = (int16_t)tempPlus400 - 400;
        *temperature = Apply_Energy_Balance(t_raw_final);
    }

    #ifdef DEBUG_TEMP
    debug_print("temp: %d\r\n", *temperature);
    debug_print("humidity: %d\r\n", *humidity);
    #endif

    return true;
}

bool WakeUpTempSensor() 
{
    if (tempSensorType == TS_Missing) {
        DetermineTempSensor();
    }

    #ifdef DEBUG_TEMP
    debug_print("B4 wakeup Sensor Type: %d\r\n", tempSensorType);
    #endif

    switch (tempSensorType)
    {
        case SHT3x: {
            uint8_t cmd1[2] = { 0x24, 0x00 };
            return I2CWriteToAddress(0x88, cmd1, 2);
        }
        case SHT4x: {
            uint8_t cmd2 = 0xfd;
            return I2CWriteToAddress(0x88, &cmd2, 1);
        }
        case HTU21C: {
            uint8_t cmd3 = 0xf5;
            return I2CWriteToAddress(0x80, &cmd3, 1);
        }
        case TS_Missing:
        default:
            return false;
    }
}

void DetermineTempSensor()
{
    #ifdef DEBUG_TEMP
    debug_print("Determining Sensor\r\n");
    #endif

    tempSensorType = TS_Missing;
    uint8_t cmd1[2] = { 0x30, 0xa2 };
    uint8_t cmd2 = 0x94;
    uint8_t cmd3 = 0xfe;

    if (I2CWriteToAddress(0x88, cmd1, 2))
        tempSensorType = SHT3x;
    else if (I2CWriteToAddress(0x88, &cmd2, 1))
        tempSensorType = SHT4x;
    else if (I2CWriteToAddress(0x80, &cmd3, 1))
        tempSensorType = HTU21C;

    #ifdef DEBUG_TEMP
    debug_print("DTS Sensor Type: %d\r\n", tempSensorType);
    #endif
}
