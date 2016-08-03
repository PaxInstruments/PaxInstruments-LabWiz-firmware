/*****************************************************************************
 **
 ** Serial access functions
 ** August 2016
 **
 ****************************************************************************/

#include "string.h"
#include "limits.h"

#include "labwiz/defs.h"
#include "labwiz/drv_serial.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "stm32f1xx_hal.h"

// Definitions and externs
// ---------------------------------------------------------------------------
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#ifndef UART1_RX_BUFFER_SIZE
#define UART1_RX_BUFFER_SIZE        32
#endif

#ifndef UART2_RX_BUFFER_SIZE
#define UART2_RX_BUFFER_SIZE        32
#endif

#if (UART1_RX_BUFFER_SIZE > UCHAR_MAX)
#error "Uart buffer too big for uint8"
#endif
#if (UART2_RX_BUFFER_SIZE > UCHAR_MAX)
#error "Uart buffer too big for uint8"
#endif

// Local module variables
// ---------------------------------------------------------------------------
// A mutex to synchronize the serial port from different tasks
xSemaphoreHandle m_serial1_mutex_tx;
xSemaphoreHandle m_serial1_mutex_rx;

xSemaphoreHandle m_serial_semaphore_rx;

uint8_t m_uart1_receive_byte;
bool m_uart1_recevie_flag = false;
uint8_t m_uart2_receive_byte;
bool m_uart2_recevie_flag = false;

uint8_t m_uart1_buffer[UART1_RX_BUFFER_SIZE];
uint8_t m_uart1_buffer_index = 0;
bool m_uart1_buffer_overflow = false;

uint8_t m_uart2_buffer[UART2_RX_BUFFER_SIZE];
uint8_t m_uart2_buffer_index = 0;
bool m_uart2_buffer_overflow = false;

// Public functions
// ---------------------------------------------------------------------------

void drv_serial_init()
{
    // HAL_UART_Init for uart1 and uart2 has been executed at this point

    m_serial1_mutex_tx = xSemaphoreCreateMutex();
    m_serial1_mutex_rx = xSemaphoreCreateMutex();
    vSemaphoreCreateBinary(m_serial_semaphore_rx);
    // If semaphores are NULL, not enough space

    // Enable the UART1 global interrupt
    HAL_NVIC_SetPriority(USART1_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    return;
}

// This is blocking
void drv_serial1_tx(uint8_t * pData, uint16_t size)
{
    xSemaphoreTake( m_serial1_mutex_tx, portMAX_DELAY );
    // The following will only execute once the mutex has been obtained
    {
        HAL_StatusTypeDef result;

        // Send things to the serial port, use blocking mode, no timeout
        #if 1
        // If this blocks for a long time the scheduler will push us to
        // BLOCKED and resume when we have a slice.  Since we are guarded
        // by a mutex other tasks will not jump into the middle of the
        // active tasks transmit
        result = HAL_UART_Transmit(&huart1, pData, size, HAL_MAX_DELAY);
        #else
        nop();
        #endif

        // The mutex MUST be given back
        xSemaphoreGive( m_serial1_mutex_tx );
    }
    return;
}

uint8_t drv_serial1_rx(uint8_t* pData, uint8_t max_size)
{
    uint8_t bytes = max_size;
    // We have a buffer in RAM for all the bytes we have received, we need to
    // pass back up to max_size bytes, return the actual number of bytes we
    // have copied back;

    // Short cut the logic if we don't have data.  We can do this outside
    // the semaphore because we are not modifying the index.
    if(m_uart1_buffer_index==0) return 0;

    xSemaphoreTake( m_serial1_mutex_rx, portMAX_DELAY );
    if(m_uart1_buffer_index<max_size)
    {
        if(pData!=NULL) memcpy(pData,m_uart1_buffer,(size_t)m_uart1_buffer_index);
        bytes = m_uart1_buffer_index;
        m_uart1_buffer_index = 0;
    }else{
        // Send back a subset and shift stuff down
        // NOTE: We should use a circular buffer but, honestly, we are
        // not going to gain a huge decrease in time.  Best practice is try
        // and read a large chunk of data so we are always clearing the buffer
        // and not executing this memmove
        if(pData!=NULL) memcpy(pData,m_uart1_buffer,(size_t)max_size);
        bytes = max_size;

        // Move down by m_uart1_buffer_index-max_size
        memmove(m_uart1_buffer,&(m_uart1_buffer[m_uart1_buffer_index]),
                (size_t)(m_uart1_buffer_index-max_size) );
        // Decrease index by max_size
        m_uart1_buffer_index =(uint8_t)(m_uart1_buffer_index-max_size);
    }
    xSemaphoreGive( m_serial1_mutex_rx);

    return bytes;
}

void drv_uart_task( void *pvParameters )
{
    // This is a task to process data from the ISR, we take a
    // received byte and add it to an array.  We have to do this
    // in a task because we are using a mutex to protect the read
    // of the buffer

    HAL_StatusTypeDef uart_result;

    // The UART hardware only has a single receive register, so we need to
    // provide a buffer to store data for each UART.  We also need to
    // kick off the interrupt based receive function to return our data
    // to our local callback
    uart_result = HAL_UART_Receive_IT(&huart1, &m_uart1_receive_byte, 1);

    for(;;)
    {
        // Block here until we get some data
        xSemaphoreTake(m_serial_semaphore_rx,portMAX_DELAY);

        // UART1 receive
        if(m_uart1_recevie_flag)
        {
            xSemaphoreTake( m_serial1_mutex_rx, portMAX_DELAY );
            // UART1 receive
            if(m_uart1_buffer_index>=UART1_RX_BUFFER_SIZE)
            {
                m_uart1_buffer_overflow = true;
                // Drop the byte
            }else{
                m_uart1_buffer[m_uart1_buffer_index] = m_uart1_receive_byte;
                m_uart1_buffer_index++;
            }
            xSemaphoreGive( m_serial1_mutex_rx);
            uart_result = HAL_UART_Receive_IT(&huart1, &m_uart1_receive_byte, 1);
        }

        // UART2 receive
        if(m_uart2_recevie_flag)
        {
            //xSemaphoreTake( m_serial2_mutex_rx, portMAX_DELAY );
            nop();
            //xSemaphoreTake( m_serial2_mutex_rx, portMAX_DELAY );
        }


    }

    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );

    return;
}

// Private functions
// ---------------------------------------------------------------------------

// This callback is called when we have received the number of
// bytes we requested from the interrupt
// HOLY CRAP, STM32 DRIVERS CALL THIS FROM INTERRUPT!
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart==&huart1)
        m_uart1_recevie_flag = true;
    else if(huart==&huart2)
        m_uart2_recevie_flag = true;
    xSemaphoreGiveFromISR(m_serial_semaphore_rx,NULL);
    return;
}



// eof
