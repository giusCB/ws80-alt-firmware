#include "main.h"
#include "debug.h"
#include "my_time.h"
#include "temperature.h"
#include "wind.h"
#include "radio.h"
#include "light.h"
#include "battery.h"

void sleep_radio();
void sleepLight();

void sleepLikeEcowitt();
void set_many_pins_analog();

void Test()
{
    //init_radio();//(true, 3);
    //g_canStop = false;
    sleep_radio();
    ProcessTemperature();
    //sleepLight();
    uint32_t lastTimePrint = g_rtcTicks;
    while (1)
    {
        /*sleepLikeEcowitt();
        continue;
        #if 1
        if (g_rtcTicks & 1)
            GPIOA->BSRR = 1;
        else
            GPIOA->BSRR = 1 << 16;
        #endif*/

        // With everything, we're at 7.0mV (2.2mV diff)
        processRadio();
        process_wind();
        processLight();
        ProcessTemperature();
        processBattery();
        stopLowPower();
        //sleepLikeEcowitt();
        //stop_until_event(true);
        continue;
        uint32_t ticks = g_rtcTicks;
        if (ticks - lastTimePrint >= WAKEUP_FREQUENCY)
        {
            timePrintDebug();
            lastTimePrint = ticks;
        }
        
        //debug_print("Time time is: %lu, %ld, %lu\r\n", millis32(), HAL_GetTick(), g_rtcTicks);

        //debug_print("I'm going to Alarm delay now.\r\n");
        wait_for_continue();
        debug_print("\r\n ----- Battery ----\r\n");
        if (!processBattery())
        {
            debug_print("Manual battery measurement\r\n");
            measureBattery();
        }
        debug_print("\r\n ----- Light ------\r\n");
        processLight();
        readPossibleLightInterface();
        debug_print("\r\n ------Temp -------\r\n");
        ProcessTemperature();
        continue;

        bool processed = process_wind();
        // ProcessTemperature();
        // debug_print("Finished process_wind\r\n");

        if (processed)
        {
            printWindDebug();
            //wait_for_continue();
        }

        if (processRadio())
            wait_for_continue();

        delay_stopped(100);
    }
}

void preparePinsForStop();
void resumePinsAfterStop();
void sleepLikeEcowitt()
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_CWUF;
    PWR->CR = (PWR->CR & ~PWR_CR_PVDE) | PWR_CR_ULP | PWR_CR_FWU;
    //set_many_pins_analog();
    preparePinsForStop();
    stop_until_event(true);
    resumePinsAfterStop();

    PWR->CR = (PWR->CR & ~PWR_CR_ULP) | PWR_CR_PVDE;
}

#if 0
void preparePinsForStop()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // A0, LED
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = MODE_ANALOG;
    GPIO_InitStruct.Pull = 0;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    // B12: Radio interrupt.
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = MODE_ANALOG;
    GPIO_InitStruct.Pull = 0;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

    // All C pins
    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = MODE_ANALOG;
    GPIO_InitStruct.Pull = 0;
    HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);

    // Oscillator, 2.4 mV
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = MODE_ANALOG;
    GPIO_InitStruct.Pull = 0;
    HAL_GPIO_Init(GPIOH,&GPIO_InitStruct);
    return;
}

void resumePinsAfterStop()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // A0, LED
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = 0;
    HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

    // Radio interrupt is handled by our radio code.

    // C pins are handled by our radio code?

    // Oscillator pins don't need to be set back?
}
#endif

void set_many_pins_analog()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  
  //B13 not used
  //B12 radio int
  //B0  not used
  GPIO_InitStruct.Pin = GPIO_PIN_12;
                    /* analog */
  // Already set: 13, 0
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

  // 4.4 mv if we quit here
                    /* pins 0, 2, 3, 6, 7, 11, 12, 15 */
    // * indicates already set
    // A0 LED
    // A2 calibration switch
    // A3 Not used*
    // A6,7 analog mux switch
    // A11, 12 USB*
    // A15 USART *
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  // Try remove GPIO_PIN_6|GPIO_PIN_7
  // No apparent change: GPIO_PIN_0|GPIO_PIN_2
  //Already set: GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15
                    /* analog */
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);
  // 4.4 mV if we quit here / 5.0 settled

                    /* all pins except 8 */
  //C8 radio data
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
                    /* pin 2 */
  //5.6mV if we quit here
  //D2 not used
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOD,&GPIO_InitStruct);
  // 5.6mV if we quit here
                    /* pin 1 and 2 */
  // Oscillator, 2.4 mV
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOH,&GPIO_InitStruct);
  return;
}