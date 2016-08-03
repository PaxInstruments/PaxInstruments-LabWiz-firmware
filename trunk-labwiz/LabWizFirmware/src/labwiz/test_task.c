/*****************************************************************************
 **
 ** Test task source code
 ** August 2016
 **
 ****************************************************************************/

#include "labwiz/defs.h"
#include "labwiz/port.h"
#include "labwiz/test_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "labwiz/drv_serial.h"

#define TOGGLE_PERIOD_MS        100

char m_scratch[100];

void TestTaskFunction( void *pvParameters )
{
    uint8_t bytes;

    for( ;; )
    {
        // Do all the things
        vTaskDelay(portTICK_PERIOD_MS*TOGGLE_PERIOD_MS);

        // Togle LED and send string
        led1(toggle());

        bytes = drv_serial1_rx(NULL, 100);
        if(bytes>0)
        {
            uint8_t n;
            n = (uint8_t)sprintf(m_scratch,"Received %d bytes\n",bytes);
            drv_serial1_tx((uint8_t*)m_scratch,n);
        }
    }

    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );
    return;
}

//eof
