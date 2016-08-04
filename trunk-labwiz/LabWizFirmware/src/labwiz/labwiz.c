/*****************************************************************************
 **
 ** LabWiz related functions.
 ** August 2016
 **
 ****************************************************************************/

#include "labwiz/defs.h"
#include "labwiz/labwiz.h"

#include "labwiz/drv_serial.h"
#include "labwiz/test_task.h"
#include "labwiz/drv_esp8266.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


void labwiz_init()
{
    drv_uart_init();
    drv_esp8266_init();

    return;
}

void labwiz_task_init()
{
    xTaskCreate( TestTaskFunction,
              "TestTask",
              configMINIMAL_STACK_SIZE,
              NULL,
              osPriorityNormal,
              NULL
    );
    xTaskCreate( drv_uart_task,
            "UARTTask",
            configMINIMAL_STACK_SIZE,
            NULL,
            osPriorityNormal,
            NULL
    );
    return;
}

// eof
