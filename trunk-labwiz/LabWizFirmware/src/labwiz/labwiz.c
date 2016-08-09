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


#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

// Definitions and types
// ----------------------------------------------------------------------------
#define DEBUG   1

#define SW_A_EXTI           15
#define SW_B_EXTI           6
#define SW_C_EXTI           7
#define SW_D_EXTI           12
#define SW_E_EXTI           13
#define SW_PWR_EXTI         0

#define SW_A_EXTI_MASK      (1<<(SW_A_EXTI))
#define SW_B_EXTI_MASK      (1<<(SW_B_EXTI))
#define SW_C_EXTI_MASK      (1<<(SW_C_EXTI))
#define SW_D_EXTI_MASK      (1<<(SW_D_EXTI))
#define SW_E_EXTI_MASK      (1<<(SW_E_EXTI))
#define SW_PWR_EXTI_MASK    (1<<(SW_PWR_EXTI))

#define SW_A                SW_A_EXTI
#define SW_B                SW_B_EXTI
#define SW_C                SW_C_EXTI
#define SW_D                SW_D_EXTI
#define SW_E                SW_E_EXTI
#define SW_PWR              SW_PWR_EXTI

// Local variables
// ----------------------------------------------------------------------------
EXTI_TypeDef * m_exti_struct = ((EXTI_TypeDef *) EXTI_BASE);
uint32_t m_exti_mask = 0;

xSemaphoreHandle m_labwiz_isr_semaphore;

// Local prototypes
// ----------------------------------------------------------------------------
void _labwiz_button_press(uint8_t button);
void _labwiz_task( void *pvParameters );
void EXTI15_10_IRQHandler(void);
void WWDG_IRQHandler(void);
void _labwiz_app_task( void *pvParameters );

// Public functions
// ----------------------------------------------------------------------------

void labwiz_init()
{
    // All HAL init functions have been called at this point
    drv_uart_init();
    drv_esp8266_init();
    drv_spi_init();
    lcd_init();

    m_exti_mask = 0;
    vSemaphoreCreateBinary(m_labwiz_isr_semaphore);


    // Enable the External interrupts 10-15 global interrupt
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_DisableIRQ(WWDG_IRQn);

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
    // if we have 3k of stack, this is 6 tasks!
    result = xTaskCreate( TestTaskFunction,
              "TestTask",
              configMINIMAL_STACK_SIZE,
              NULL,
              osPriorityNormal,
              NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(DEBUG) nop();
    result = xTaskCreate( drv_uart_task,
            "UARTTask",
            configMINIMAL_STACK_SIZE,
            NULL,
            osPriorityNormal,
            NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(DEBUG) nop();
    result = xTaskCreate( lcd_task,
            "LCDTask",
            configMINIMAL_STACK_SIZE*2,
            NULL,
            osPriorityNormal,
            NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(DEBUG) nop();
    result = xTaskCreate( _labwiz_task,
            "LabWiz",
            configMINIMAL_STACK_SIZE*2,
            NULL,
            osPriorityNormal,
            NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(DEBUG) nop();

    result = xTaskCreate( _labwiz_app_task,
            "AppTask",
            configMINIMAL_STACK_SIZE*2,
            NULL,
            osPriorityLow,
            NULL
    );
    // DEBUG
    if(result!=pdPASS)
        while(DEBUG) nop();
    return;
}


// Private functions
// ---------------------------------------------------------------------------

void _labwiz_button_press(uint8_t button)
{
    switch(button){
    case SW_A: break;
    case SW_B: break;
    case SW_C: break;
    case SW_D: break;
    case SW_E: lcd_backlight_toggle(); break;
    case SW_PWR: break;
    default: break;
    }
    return;
}
void _labwiz_task( void *pvParameters )
{
    nop();

    for(;;)
    {
        // Block here until we get an interrupt
        xSemaphoreTake(m_labwiz_isr_semaphore,portMAX_DELAY);

        // m_exti_mask holds the pending interrupt for each exti
        if(m_exti_mask & SW_A_EXTI_MASK)
        { m_exti_mask&=(uint32_t)(~SW_A_EXTI_MASK);_labwiz_button_press(SW_A);}

        if(m_exti_mask & SW_B_EXTI_MASK)
        { m_exti_mask&=(uint32_t)(~SW_A_EXTI_MASK);_labwiz_button_press(SW_B);}

        if(m_exti_mask & SW_C_EXTI_MASK)
        { m_exti_mask&=(uint32_t)(~SW_A_EXTI_MASK);_labwiz_button_press(SW_C);}

        if(m_exti_mask & SW_D_EXTI_MASK)
        { m_exti_mask&=(uint32_t)(~SW_A_EXTI_MASK);_labwiz_button_press(SW_D);}

        if(m_exti_mask & SW_E_EXTI_MASK)
        { m_exti_mask&=(uint32_t)(~SW_A_EXTI_MASK);_labwiz_button_press(SW_E);}
    }

    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );

    return;
}

void EXTI15_10_IRQHandler(void)
{
    volatile uint32_t pr;
    pr = m_exti_struct->PR;
    // Save the interrupts
    m_exti_mask = m_exti_mask|pr;
    // Clear all pending interrupts
    m_exti_struct->PR = 0x000FFFFF;
    xSemaphoreGiveFromISR(m_labwiz_isr_semaphore,NULL);
    return;
}
void WWDG_IRQHandler(void)
{
    nop();
    return;
}

void _labwiz_app_task( void *pvParameters )
{
    setup();
    for(;;)
    {
        loop();
    }

    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );

    return;
}

__weak void setup()
{
    return;
}
__weak void loop()
{
    return;
}

// eof
