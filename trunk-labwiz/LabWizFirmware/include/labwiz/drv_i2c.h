/*****************************************************************************
 **
 ** I2C driver
 ** September 2016
 **
 ** This driver provides functions to read and write I2C busses
 **
 ****************************************************************************/

#ifndef __DRV_I2C_H__
#define __DRV_I2C_H__

void drv_i2c_init(void);

I2C_HandleTypeDef * i2c_get_bus_handle(int i2c_bus);

bool drv_i2c1_busy(void);
bool drv_i2c1_ready(void);
HAL_StatusTypeDef drv_i2c1_write(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef drv_i2c1_read(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

bool drv_i2c2_busy(void);
bool drv_i2c2_ready(void);
HAL_StatusTypeDef drv_i2c2_write(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef drv_i2c2_read(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

#endif

//eof
