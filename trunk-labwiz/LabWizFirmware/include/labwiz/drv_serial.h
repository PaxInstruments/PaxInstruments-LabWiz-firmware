#ifndef __DRV_SERIAL_H__
#define __DRV_SERIAL_H__

void drv_serial_init(void);

#define drv_serial1_tx_str(S)   drv_serial1_tx((uint8_t*)(S), sizeof(S)-1);
void drv_serial1_tx(uint8_t * pData, uint16_t size);

uint8_t drv_serial1_rx(uint8_t* pData, uint8_t max_size);

void drv_uart_task( void *pvParameters );

#endif

//eof
