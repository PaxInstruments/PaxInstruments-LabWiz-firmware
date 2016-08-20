/*****************************************************************************
 **
 ** SPI driver
 ** August 2016
 **
 ** This driver provides functions to read and write SPI busses
 **
 ****************************************************************************/

#include "labwiz/labwizdefs.h"
#include "labwiz/drv_spi.h"

#include "stm32f1xx_hal.h"

// Definitions and externs
// ---------------------------------------------------------------------------
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

// Local module variables
// ---------------------------------------------------------------------------

// Private prototypes
// ---------------------------------------------------------------------------

// Public functions
// ---------------------------------------------------------------------------

void drv_spi_init()
{
    // HAL_SPI_Init() has been called at this point for all SPI busses

    return;
}

bool drv_spi1_txrx(uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
    // Need uint32_t Timeout?
    HAL_StatusTypeDef result;
    result = HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, size, HAL_MAX_DELAY);
    return (result==HAL_OK);
}
bool drv_spi1_tx(uint8_t *pTxData, uint16_t size)
{
    // Need uint32_t Timeout?
    HAL_StatusTypeDef result;
    result = HAL_SPI_Transmit(&hspi1, pTxData, size, HAL_MAX_DELAY);
    return (result==HAL_OK);
}


bool drv_spi2_txrx(uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
    // Need uint32_t Timeout?
    HAL_StatusTypeDef result;
    result = HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, size, HAL_MAX_DELAY);
    return (result==HAL_OK);
}

bool drv_spi3_txrx(uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
    // Need uint32_t Timeout?
    HAL_StatusTypeDef result;
    result = HAL_SPI_TransmitReceive(&hspi3, pTxData, pRxData, size, HAL_MAX_DELAY);
    return (result==HAL_OK);
}
bool drv_spi3_tx(uint8_t *pTxData, uint16_t size)
{
    // Need uint32_t Timeout?
    HAL_StatusTypeDef result;
    result = HAL_SPI_Transmit(&hspi3, pTxData, size, HAL_MAX_DELAY);
    return (result==HAL_OK);
}

// eof
