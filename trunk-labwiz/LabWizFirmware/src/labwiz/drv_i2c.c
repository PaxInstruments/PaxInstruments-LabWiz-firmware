/*****************************************************************************
 **
 ** I2C driver
 ** September 2016
 **
 ** This driver provides functions to read and write I2C busses
 **
 ****************************************************************************/

#include "labwiz/labwizdefs.h"
#include "labwiz/drv_i2c.h"

#include "stm32f1xx_hal.h"

// Definitions and externs
// ---------------------------------------------------------------------------
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

// Local module variables
// ---------------------------------------------------------------------------

int m_i2c1_busy_count=0;

// Private prototypes
// ---------------------------------------------------------------------------

// Public functions
// ---------------------------------------------------------------------------

void drv_i2c_init()
{
    // HAL_I2C_Init() has been called at this point for all I2C busses

	//(#) To check if target device is ready for communication, use the function HAL_I2C_IsDeviceReady()

    return;
}

I2C_HandleTypeDef * i2c_get_bus_handle(int i2c_bus)
{
    if(i2c_bus==I2C_BUS_1) return &hi2c1;
    //if(i2c_bus==I2C_BUS_2) return &hi2c2;
    return NULL;
}

#if 0
// We could do this too?
HAL_I2C_IsDeviceReady()
// Check to see if we get an ACK
bool drv_i2c1_device_connected()
{
    if(hi2c->State == HAL_I2C_STATE_READY)
    {

        /* Wait until BUSY flag is reset */
        if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG) != HAL_OK)
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Disable Pos */
        CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

        hi2c->State = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode = HAL_I2C_MODE_MASTER;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Send Slave Address */
        if(I2C_MasterRequestRead(hi2c, DevAddress, Timeout) != HAL_OK)
        {
            if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);
                return HAL_ERROR;
            }else{
                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);
                return HAL_TIMEOUT;
            }
        }

        /* Generate Stop */
        SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
    }else{
        return HAL_BUSY;
    }

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return true;
}
#endif

// I2C 1 functions
////////////////////////////
bool drv_i2c1_busy()
{
    return (hi2c1.State == HAL_I2C_STATE_BUSY);
}

bool drv_i2c1_ready()
{
    return (hi2c1.State == HAL_I2C_STATE_READY);
}

int drv_i2c1_busy_count()
{
	return m_i2c1_busy_count;
}

HAL_StatusTypeDef drv_i2c1_write(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef ret;

    // Check if busy
    //if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
    //    return HAL_BUSY;
    // Not busy, do things
    //ret = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, pData, Size, HAL_MAX_DELAY);
    ret = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, pData, Size, 10);
    switch(ret){
    case HAL_OK:
    	m_i2c1_busy_count=0;
        break;
    case HAL_ERROR:
        nop();
        break;
    case HAL_BUSY:
    	m_i2c1_busy_count++;
        break;
    case HAL_TIMEOUT:
    default:
        nop();
        break;
    }

    return ret;
}
HAL_StatusTypeDef drv_i2c1_read(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef ret;

    // Check if busy
    //if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
    //    return HAL_BUSY;
    // Not busy, do things

    ret = HAL_I2C_Master_Receive(&hi2c1, DevAddress, pData, Size, 10);
    switch(ret){
    case HAL_OK:
    	m_i2c1_busy_count=0;
        break;
    case HAL_ERROR:
        nop();
        break;
    case HAL_BUSY:
    	m_i2c1_busy_count++;
        break;
    case HAL_TIMEOUT:
    default:
        nop();
        break;
    }

    return ret;
}

// I2C 2 functions
////////////////////////////
#if 0
bool drv_i2c2_busy()
{
    return (hi2c1.State == HAL_I2C_STATE_BUSY);
}

bool drv_i2c2_ready()
{
    return (hi2c2.State == HAL_I2C_STATE_READY);
}

HAL_StatusTypeDef drv_i2c2_write(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef ret;

    // Check if busy
    //if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
    //    return HAL_BUSY;
    // Not busy, do things
    //ret = HAL_I2C_Master_Transmit(&hi2c2, DevAddress, pData, Size, HAL_MAX_DELAY);
    ret = HAL_I2C_Master_Transmit(&hi2c2, DevAddress, pData, Size, 10);
    switch(ret){
    case HAL_OK:
        nop();
        break;
    case HAL_ERROR:
        nop();
        break;
    case HAL_BUSY:
        nop();
        break;
    case HAL_TIMEOUT:
    default:
        nop();
        break;
    }

    return ret;
}
HAL_StatusTypeDef drv_i2c2_read(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef ret;

    // Check if busy
    //if(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
    //    return HAL_BUSY;
    // Not busy, do things
    //ret = HAL_I2C_Master_Receive(&hi2c2, DevAddress, pData, Size, HAL_MAX_DELAY);
    ret = HAL_I2C_Master_Receive(&hi2c2, DevAddress, pData, Size, 10);
    switch(ret){
    case HAL_OK:
        nop();
        break;
    case HAL_ERROR:
        nop();
        break;
    case HAL_BUSY:
        nop();
        break;
    case HAL_TIMEOUT:
    default:
        nop();
        break;
    }

    return ret;
}
#endif


// eof
