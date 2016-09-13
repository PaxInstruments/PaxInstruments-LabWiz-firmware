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

bool drv_i2c1_busy();

bool drv_i2c1_ready();

void drv_i2c1_write(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

void drv_i2c1_read(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

#endif

//eof
