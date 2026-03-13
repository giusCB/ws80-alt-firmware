#include "main.h"
#include "debug.h"
#include "my_time.h"
#include "temperature.h"
#include "wind.h"
#include "radio.h"

void Test()
{
    init_radio();//(true, 3);
    //g_canStop = false;
    while (1)
    {
        GPIOA->ODR ^= 1;
        
        //debug_print("Time time is: %lu, %ld, %lu\r\n", millis32(), HAL_GetTick(), g_rtcTicks);

        //debug_print("I'm going to Alarm delay now.\r\n");

        bool processed = process_wind();
        // ProcessTemperature();
        // debug_print("Finished process_wind\r\n");

        if (processed)
        {
            printWindDebug();
            //wait_for_continue();
        }

        processRadio();

        delay_stopped(100);
    }
}