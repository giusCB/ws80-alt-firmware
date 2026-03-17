#include "main.h"
#include "my_time.h"
#include "debug.h"
#include "wind.h"
#include "battery.h"

#define BATTERY_PRINT(...) debug_print(__VA_ARGS__)

const uint8_t battSwPin = 2;
const uint8_t batt_timeout = 10;


int8_t g_batteryMeasurement;

uint32_t last_batt_measurement = 0xFFFF;
const uint16_t batt_measurement_interval_s = 600;
const uint16_t batt_measurement_interval_rtc = batt_measurement_interval_s * WAKEUP_FREQUENCY;

bool processBattery()
{
    uint32_t ticks = g_rtcTicks;
    if (ticks - last_batt_measurement > batt_measurement_interval_rtc)
    {
        measureBattery();
        last_batt_measurement = ticks;
        return true;
    }
    return false;
}

void measureBattery()
{
    // Record that we have messed with the ADC:
    g_wind_init_required = true;

    // Set the switch to output
    GPIOA->MODER = (GPIOA->MODER & (~3 << battSwPin)) 
        | 1 << battSwPin;
    // Set the switch low
    GPIOA->BSRR = 1 << (battSwPin + 16);
    delay_stopped(10);
    // Setup the ADC.
    // Turn it off before we start messing with it.
    ADC1->CR2 &= ~ADC_CR2_ADON;
    // Clear ADC_SR
    ADC1->SR &= ~0x1Ful;
    // Nothing fancy. No DMA, trigger, or continuous
    ADC1->CR2 = (ADC1->CR2 & (0x8080F088));
    // Change to channel 9:
    ADC1->SQR5 = (ADC1->SQR5 & ~ADC_SQR5_SQ1_Msk) | 9;
    // Enable the conversion complete event:

    // Turn on the ADC:
    ADC1->CR2 |= ADC_CR2_ADON;
    // Wait for the ADC to turn on and the regular channel to become ready:
    uint32_t entryTicks = HAL_GetTick();
    while ((ADC1->SR & (ADC_SR_ADONS | ADC_SR_RCNR)) != (ADC_SR_ADONS | ADC_SR_RCNR))
    {
        if (HAL_GetTick() - entryTicks > batt_timeout)
        {
            BATTERY_PRINT("Battery: TIMEOUT on ADONS | RCNR!\r\n");
            return;
        }
    }
    ADC1->CR2 |= ADC_CR2_SWSTART;
    // Wait for our conversion
    entryTicks = HAL_GetTick();
    while (!(ADC1->SR & ADC_SR_EOCS))
    {
        if (HAL_GetTick() - entryTicks > batt_timeout)
        {
            BATTERY_PRINT("Battery: TIMEOUT on EOCS!\r\n");
            return;
        }
    }
    uint16_t batt_raw = ADC1->DR;
    // Turn off the ADC:
    ADC1->CR2 &= ~ADC_CR2_ADON;
    // Set the battery switch analog:
    GPIOA->MODER = (GPIOA->MODER & ~(3 << (battSwPin * 2))) 
        | MODE_ANALOG << (battSwPin * 2);
    g_batteryMeasurement = batt_raw / 25;
    BATTERY_PRINT("Battery raw: %d, processed: %d\r\n", batt_raw, g_batteryMeasurement);
}