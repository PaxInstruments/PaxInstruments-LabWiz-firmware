/*****************************************************************************
 **
 ** LabWiz UART driver
 ** August 2016
 **
 ** This driver provides a task to read in serial data from uart 1 and 2 and
 ** provides RX and TX functions to other tasks in the system.
 **
 ****************************************************************************/

#ifndef __DRV_UART_H__
#define __DRV_UART_H__

void drv_uart_init(void);

// Send a buffer of data to the serial port.  This will
// block until all data has been sent
void drv_uart1_tx(uint8_t * pData, uint16_t size);

// Helper macro to send a fixed string
#define drv_uart1_tx_str(S)   drv_serial1_tx((uint8_t*)(S), sizeof(S)-1);

// Receive function, can be called from a task and will return
// back the number of bytes in the buffer up to max_size
uint8_t drv_uart1_rx(uint8_t* pData, uint8_t max_size);

// This is the uart driver task, reading the RX data and putting
// it into a RAM buffer
void drv_uart_task( void *pvParameters );
void TestTaskFunction2( void *pvParameters );

#endif

//eof
