#include "main.h"
#include "debug.h"
#include "my_time.h"
#include "temperature.h"
#include "wind.h"
#include "radio.h"
#include "light.h"
#include "battery.h"

void Test()
{
    //init_radio();//(true, 3);
    //g_canStop = false;
    while (1)
    {
        GPIOA->ODR ^= 1;
        
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