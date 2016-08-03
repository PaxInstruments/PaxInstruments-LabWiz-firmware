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

#define TOGGLE_PERIOD_MS        100

void TestTaskFunction( void *pvParameters )
{
    //int iVariableExample = 0;

    for( ;; )
    {
        // Do all the things
        vTaskDelay(portTICK_PERIOD_MS*TOGGLE_PERIOD_MS);
        led1(toggle());
    }

    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );
    return;
}

//eof
