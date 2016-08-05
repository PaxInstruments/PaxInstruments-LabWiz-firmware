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
#include "labwiz/drv_spi.h"
#include "labwiz/drv_lcd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


void labwiz_init()
{
    drv_uart_init();
    drv_esp8266_init();
    drv_spi_init();
    drv_lcd_init();
    return;
}

void labwiz_task_init()
{
    // What is stack requirements for each task
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
    xTaskCreate( drv_lcd_task,
            "LCDTask",
            configMINIMAL_STACK_SIZE*3,
            NULL,
            osPriorityNormal,
            NULL
    );

    return;
}

// eof
