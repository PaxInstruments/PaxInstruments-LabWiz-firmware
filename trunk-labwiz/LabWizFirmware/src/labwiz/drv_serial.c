/*****************************************************************************
 **
 ** Serial access functions
 ** August 2016
 **
 ** This driver provides a task to read in serial data from uart 1 and 2 and
 ** provides RX and TX functions to other tasks in the system.
 **
 ** The driver uses the TX functions provided by STMicro HAL for wait
 ** timeouts. The RX functions are updates of the STMicro RX functions
 ** to not-disable interrupts once data is received, since we don't know how
 ** much data is being transmitted by the device.
 **
 ****************************************************************************/

#include "string.h"
#include "limits.h"

#include "labwiz/labwizdefs.h"
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
bool m_uart1_buffer_overrun = false;

uint8_t m_uart2_buffer[UART2_RX_BUFFER_SIZE];
uint8_t m_uart2_buffer_index = 0;
bool m_uart2_buffer_overflow = false;
bool m_uart2_buffer_overrun = false;

// Private prototypes
// ---------------------------------------------------------------------------
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);

// Public functions
// ---------------------------------------------------------------------------

void drv_uart_init()
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
void drv_uart1_tx(uint8_t * pData, uint16_t size)
{
    xSemaphoreTake( m_serial1_mutex_tx, portMAX_DELAY );
    // The following will only execute once the mutex has been obtained
    {
        HAL_StatusTypeDef result;

        // Send things to the serial port, use blocking mode, no timeout

        // If this blocks for a long time the scheduler will push us to
        // BLOCKED and resume when we have a slice.  Since we are guarded
        // by a mutex other tasks will not jump into the middle of the
        // active tasks transmit
        result = HAL_UART_Transmit(&huart1, pData, size, HAL_MAX_DELAY);
    }
    // The mutex MUST be given back
    xSemaphoreGive( m_serial1_mutex_tx );
    return;
}

uint8_t drv_uart1_rx(uint8_t* pData, uint8_t max_size)
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
        // We can hold all the data, just copy it over and reset buffer
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
    // in a task because we are using a mutex to protect the
    // buffer index

    HAL_StatusTypeDef uart_result;

    // The UART hardware only has a single receive register, so we need to
    // provide a buffer to store data for each UART.  We also need to
    // kick off the interrupt based receive function to pass the register
    // data to our RAM buffer
    uart_result = HAL_UART_Receive_IT(&huart1, &m_uart1_receive_byte, 1);
    //uart_result = HAL_UART_Receive_IT(&huart2, &m_uart2_receive_byte, 1);

    for(;;)
    {
        // Block here until we get some data
        xSemaphoreTake(m_serial_semaphore_rx,portMAX_DELAY);

        // UART1 receive
        //////////////////////////////////////////
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
        }
        // Catch errors
        if(m_uart1_buffer_overflow && m_uart1_buffer_overrun)
            nop();

        // UART2 receive
        //////////////////////////////////////////
        if(m_uart2_recevie_flag)
        {
            //xSemaphoreTake( m_serial2_mutex_rx, portMAX_DELAY );
            nop();
            //xSemaphoreTake( m_serial2_mutex_rx, portMAX_DELAY );
        }
        // Catch errors
        if(m_uart2_buffer_overflow && m_uart2_buffer_overrun)
            nop();

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

void USART1_IRQHandler()
{
    HAL_NVIC_ClearPendingIRQ(USART1_IRQn);

    // Copied from the original HAL ISR

    // Check each interrupt flag ORd with the interrupt enable flag

#if 0
    /* UART parity error interrupt occurred ------------------------------------*/
    if( (huart1.Instance->SR & UART_FLAG_PE) &&
            (huart1.Instance->CR1 & USART_CR1_PEIE) )
    {
        huart1.ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred -------------------------------------*/
    if( (huart1.Instance->SR & UART_FLAG_FE) &&
            (huart1.Instance->CR3 & USART_CR3_EIE) )
    {
    huart1.ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred -------------------------------------*/
    if( (huart1.Instance->SR & UART_FLAG_NE) &&
            (huart1.Instance->CR3 & USART_CR3_EIE) )
    {
    huart1.ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred ----------------------------------------*/
    if( (huart1.Instance->SR & UART_FLAG_ORE) &&
            (huart1.Instance->CR3 & USART_CR3_EIE) )
    {
    huart1.ErrorCode |= HAL_UART_ERROR_ORE;
    }
#endif
    /* UART in mode Receiver ---------------------------------------------------*/
    if( (huart1.Instance->SR & UART_FLAG_RXNE) &&
            (huart1.Instance->CR1 & USART_CR1_RXNEIE) )
    {
        // We recevied a byte

        // Save the byte
        m_uart1_receive_byte = (uint8_t)(huart1.Instance->DR);

        // If the process flag was already set, we have an overrun
        if(m_uart1_recevie_flag)
            m_uart1_buffer_overrun = true;

        // Release the RX semaphore
        m_uart1_recevie_flag = true;
        xSemaphoreGiveFromISR(m_serial_semaphore_rx,NULL);
    }
#if 0
    /* UART in mode Transmitter ------------------------------------------------*/
    if( (huart1.Instance->SR & UART_FLAG_TXE) &&
            (huart1.Instance->CR1 & USART_CR1_TXEIE) )
    {
        // We transmitted a byte
        //UART_Transmit_IT(huart1);
    }

    /* UART in mode Transmitter end --------------------------------------------*/
    if( (huart1.Instance->SR & UART_FLAG_TC) &&
            (huart1.Instance->CR1 & USART_CR1_TCIE) )
    {
        // End transmit
        //UART_EndTransmit_IT(huart1);
    }
    if(huart1.ErrorCode != HAL_UART_ERROR_NONE)
    {
        uint32_t tmpreg;
        // Reads will clear the error flags?
        //__HAL_UART_CLEAR_PEFLAG(huart);
        tmpreg = huart1.Instance->SR;
        tmpreg = huart1.Instance->DR;

        // Set the UART state ready to be able to start again the process?
        //huart1.State = HAL_UART_STATE_READY;

        // We had an error
        //HAL_UART_ErrorCallback(huart1);
    }
#else

    // Clear the error flags
    {
        uint32_t tmpreg;
        // Reads will clear the error flags?
        //__HAL_UART_CLEAR_PEFLAG(huart);
        tmpreg = huart1.Instance->SR;
        tmpreg = huart1.Instance->DR;
    }
#endif
    return;
}

void USART2_IRQHandler()
{
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);

    // Copied from the original HAL ISR

    // Check each interrupt flag ORd with the interrupt enable flag

#if 0
    /* UART parity error interrupt occurred ------------------------------------*/
    if( (huart2.Instance->SR & UART_FLAG_PE) &&
            (huart2.Instance->CR1 & USART_CR1_PEIE) )
    {
        huart2.ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred -------------------------------------*/
    if( (huart2.Instance->SR & UART_FLAG_FE) &&
            (huart2.Instance->CR3 & USART_CR3_EIE) )
    {
        huart2.ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred -------------------------------------*/
    if( (huart2.Instance->SR & UART_FLAG_NE) &&
            (huart2.Instance->CR3 & USART_CR3_EIE) )
    {
        huart2.ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred ----------------------------------------*/
    if( (huart2.Instance->SR & UART_FLAG_ORE) &&
            (huart2.Instance->CR3 & USART_CR3_EIE) )
    {
        huart2.ErrorCode |= HAL_UART_ERROR_ORE;
    }
#endif
    /* UART in mode Receiver ---------------------------------------------------*/
#if 0
    if( (huart2.Instance->SR & UART_FLAG_RXNE) &&
            (huart2.Instance->CR1 & USART_CR1_RXNEIE) )
    {
        // We recevied a byte

        // Save the byte
        m_uart2_receive_byte = (uint8_t)(huart2.Instance->DR);
        #if 0
        // If the process flag was already set, we have an overrun
        if(m_uart2_recevie_flag)
            m_uart2_buffer_overrun = true;

        // Release the RX semaphore
        m_uart2_recevie_flag = true;
        xSemaphoreGiveFromISR(m_serial_semaphore_rx,NULL);
        #endif

    }
#endif

#if 0
    /* UART in mode Transmitter ------------------------------------------------*/
    if( (huart2.Instance->SR & UART_FLAG_TXE) &&
            (huart2.Instance->CR1 & USART_CR1_TXEIE) )
    {
        // We transmitted a byte
        //UART_Transmit_IT(huart2);
    }

    /* UART in mode Transmitter end --------------------------------------------*/
    if( (huart2.Instance->SR & UART_FLAG_TC) &&
            (huart2.Instance->CR1 & USART_CR1_TCIE) )
    {
        // End transmit
        //UART_EndTransmit_IT(huart2);
    }
    if(huart2.ErrorCode != HAL_UART_ERROR_NONE)
    {
        uint32_t tmpreg;
        // Reads will clear the error flags?
        //__HAL_UART_CLEAR_PEFLAG(huart);
        tmpreg = huart2.Instance->SR;
        tmpreg = huart2.Instance->DR;

        // Set the UART state ready to be able to start again the process?
        //huart2.State = HAL_UART_STATE_READY;

        // We had an error
        //HAL_UART_ErrorCallback(huart2);
    }
#else

    // Clear the error flags
    {
        //uint32_t tmpreg;
        // Reads will clear the error flags?
        //__HAL_UART_CLEAR_PEFLAG(huart);
        //tmpreg = huart2.Instance->SR;
    }
#endif
    return;
}


// eof
