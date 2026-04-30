#include "my_time.h"
#include "wind.h"
//#include "wind_phys.h"
#include "scope.h"
#include "wind_calc.h"
#include "debug.h"
#include "light.h"

volatile uint16_t* g_wind_measurement; //[WIND_SAMPLE_SIZE];

// We need six volume values
// 01  02  03  12  13  23
// NS, NE, NW, SE, SW, EW
uint32_t g_signalPowers[6][2];
uint8_t g_windRingCounts[6];

static uint32_t s_last_wind_sample = 0xFFFF;
static bool s_calibrationRecallAttempted = false;

uint8_t wind_sample_frequency = 4;
const uint8_t max_volume = 12;

void sample_wind(bool calibrating);

// Assicuriamoci che la funzione di background sia visibile al compilatore
extern void wind_calc_background_task(void);

bool process_wind()
{
    #ifdef DELAYED_ADC
    g_wind_measurement = scopePacket.Buffer;
    #else
    g_wind_measurement = scopePacket.Buffer + 192;
    #endif
    uint32_t rtcTicksLocal = g_rtcTicks;
    uint8_t wind_sample_interval = WAKEUP_FREQUENCY / wind_sample_frequency;
    if ((rtcTicksLocal - s_last_wind_sample >= wind_sample_interval) || doContinuousMonitoring())
    {
        if (g_wind_init_required)
        {
            WIND_PRINT("Reinitialising wind.");
            InitScope();
        }
        if (!s_calibrationRecallAttempted)
        {
            // FIX: Inizializza l'intero modulo (carica Fase e Scia dalla EEPROM)
            wind_calc_init();
            s_calibrationRecallAttempted = true;
        }
        // WIND_PRINT("Sampling Wind!\r\n");
        s_last_wind_sample = rtcTicksLocal;

        // Campiona il vento
        sample_wind(false);

        // FIX: Esegue il task di auto-apprendimento della scia aerodinamica in background
        wind_calc_background_task();

        return true;
    }
    return false;
}

const uint8_t g_channel_transducers[6][2] =
{ { 0, 1 },
  { 0, 2 },
  { 0, 3 },
  { 1, 2 },
  { 1, 3 },
  { 2, 3 } };


void sample_wind(bool calibrating)
{
    #ifdef DEBUG_WIND_TIME
    timeMeasurementTypdef entryMeas = measureTime();
    timeDifferenceTypedef measurementTime = {0};
    timeDifferenceTypedef processTime = {0};
    #endif
    for (uint8_t dir = 0; dir < 2; dir++)
    {
        for (uint8_t chan = 0; chan < 6; chan++)
        {
            ON_WIND_TIME(timeMeasurementTypdef preScope = measureTime());
            ProcessScope(chan, dir);
            ON_WIND_TIME(timeMeasurementTypdef between = measureTime());
            processWindWaveform(chan, dir);
            ON_WIND_TIME(timeMeasurementTypdef afterProcess = measureTime());
            #ifdef DEBUG_WIND_TIME
            measurementTime = addDiffs(measurementTime, subtractTimes(between, preScope));
            processTime = addDiffs(processTime, subtractTimes(afterProcess, between));
            #endif

            // We need to wait for the transducers to ring down between measurements.
            // Original firmware used 8ms delay, but my testing suggests 4ms is fine.
            // We have an additional 2ms delay while we wait for the analog circuitry to stabilise.
            delay_stopped(3);
        }
    }
    #ifdef DEBUG_WIND_TIME
    timeMeasurementTypdef exitMeas = measureTime();
    WIND_TIME_PRINT("measurement:   ");
    printMeasurementDifference(measurementTime);
    WIND_TIME_PRINT("process:       ");
    printMeasurementDifference(processTime);
    WIND_TIME_PRINT("physical wind: ");
    printMeasurementDifference(subtractTimes(exitMeas, entryMeas));
    // The individual measurements give 182 + 253 = 435k cycles
    // The overall measurement gives 576k cycles.
    // Where are the missing 135k cycles?
    entryMeas = measureTime();
    #endif

    if (!calibrating)
    {
        int16_t x_cmps, y_cmps;
        calculate_wind(&x_cmps, &y_cmps);
        store_wind_sample(x_cmps, y_cmps);
    }
    else
        store_calibration_sample();
    adjustRingCounts();

    #ifdef DEBUG_WIND_TIME
    exitMeas = measureTime();
    WIND_TIME_PRINT("calc wind:     ");
    printMeasurementDifference(subtractTimes(exitMeas, entryMeas));
    #endif
}

void printWindDebug()
{
    #ifdef DEBUG_WIND
    print_wind_phys_debug();
    print_wind_calc_debug();
    WIND_PRINT("Measurement: \r\n");
    for (int i = 0; i < 5; i++)
    {
        int idx = i * 30;
        WIND_PRINT(" %4d, %4d, %4d, %4d, %4d\r\n", 
            g_wind_measurement[idx + 0], g_wind_measurement[idx + 1], g_wind_measurement[idx + 2], g_wind_measurement[idx + 3],
            g_wind_measurement[idx + 4]);
    }
    #endif
}

void calibrateWind()
{
    WIND_PRINT("Beginning calibration\r\n");
    begin_calibration();
    bool ledOn = true;
    do
    {
        GPIOA->BSRR = 1 << (16 * ledOn);
        ledOn = !ledOn;
        sample_wind(true);
    } while (!maybe_end_calibration());
    GPIOA->BSRR = 1 << 16;
}
