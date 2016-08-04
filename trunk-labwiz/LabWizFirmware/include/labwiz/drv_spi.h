/*****************************************************************************
 **
 ** SPI driver
 ** August 2016
 **
 ** This driver provides functions to read and write SPI busses
 **
 ****************************************************************************/

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

void drv_spi_init(void);

bool drv_spi1_txrx(uint8_t *pTxData, uint8_t *pRxData, uint16_t size);

bool drv_spi2_txrx(uint8_t *pTxData, uint8_t *pRxData, uint16_t size);

bool drv_spi3_txrx(uint8_t *pTxData, uint8_t *pRxData, uint16_t size);

bool drv_spi3_tx(uint8_t *pTxData, uint16_t size);

#endif

//eof
