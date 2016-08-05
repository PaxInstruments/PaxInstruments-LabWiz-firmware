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
    BaseType_t result;
    nop();
    // What is stack requirements for each task

    // The stack argument is the number of words the stack
    // will be, not bytes.  Our stack width is 4, so a value
    // of 100entries  will really be 400 bytes

    // configMINIMAL_STACK_SIZE = 128 = 512 bytes
    // if we have 3k of stack, this is 6 functions!
    result = xTaskCreate( TestTaskFunction,
              "TestTask",
              configMINIMAL_STACK_SIZE,
              NULL,
              osPriorityNormal,
              NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(1) nop();
    result = xTaskCreate( drv_uart_task,
            "UARTTask",
            configMINIMAL_STACK_SIZE,
            NULL,
            osPriorityNormal,
            NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(1) nop();
#if 1
    result = xTaskCreate( drv_lcd_task,
            "LCDTask",
            configMINIMAL_STACK_SIZE*2,
            NULL,
            osPriorityNormal,
            NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(1) nop();
#endif

    return;
}

// eof
