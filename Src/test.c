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
    sleepLight();
    uint32_t lastTimePrint = g_rtcTicks;
    while (1)
    {
        //sleepLikeEcowitt();
        //continue;
        #if 0
        if (g_rtcTicks & 1)
            GPIOA->BSRR = 1;
        else
            GPIOA->BSRR = 1 << 16;
        #endif

        // Measure analog circuitry power consumption:
        GPIOA->BSRR = GPIO_PIN_4 << 16;

        //processRadio();
        //process_wind();
        //processLight();
        //processBattery();
        //ProcessTemperature();
        //sleepLikeEcowitt();
        stop_until_event(true);
        uint32_t ticks = g_rtcTicks;
        if (ticks - lastTimePrint >= WAKEUP_FREQUENCY)
        {
            timePrintDebug();
            lastTimePrint = ticks;
        }
        continue;
        
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

void sleepLikeEcowitt()
{
    PWR->CR |= PWR_CR_CWUF;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR = (PWR->CR & ~PWR_CR_PVDE) | PWR_CR_ULP | PWR_CR_FWU;
    set_many_pins_analog();
    stop_until_event(true);
    PWR->CR = (PWR->CR & ~PWR_CR_ULP) | PWR_CR_PVDE;
}

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

  return;
  //15.8mV if this code is not run. BECAUSE OF THE LED YOU FOOL.
    #if 1
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

  // #else //8.2mV if only the above is run

                    /* all pins except 8 */
  //C8 radio data
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0;
                    /* analog */
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOC,&GPIO_InitStruct);
    // #else // Code below here doesn't affect draw
                    /* pin 2 */
  //D2 not used
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOD,&GPIO_InitStruct);
                    /* pin 1 and 2 */
  // Oscillator
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
  GPIO_InitStruct.Mode = MODE_ANALOG;
  GPIO_InitStruct.Pull = 0;
  HAL_GPIO_Init(GPIOH,&GPIO_InitStruct);
  #endif
  return;
}