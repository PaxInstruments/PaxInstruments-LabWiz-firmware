/*****************************************************************************
 **
 ** MCP9800 driver
 ** September 2016
 **
 ** This driver provides functions to control an MCP9800
 **
 ****************************************************************************/

#include "defs.h"
#include "labwiz/drv_i2c.h"
#include "stm32f1xx_hal.h"

#include "drv_mcp9800.h"

#include "string.h"

#ifdef ENABLE_MCP9800

// Definitions and externs
// ---------------------------------------------------------------------------
#define ADDR_TEMP       0x00
#define ADDR_CONFIG     0x01
#define ADDR_HYST       0x02
#define ADDR_LIMIT      0x03

// This is fixed for the MCP980x
#define DEV_ADDR            0x48

// Local module variables
// ---------------------------------------------------------------------------
static I2C_HandleTypeDef * m_i2cbus_handle = NULL;

static uint8_t m_id_number = 0;

mcp9800_resolution_e m_resolution;

int m_current_temp=0;

// Private prototypes
// ---------------------------------------------------------------------------
bool _mcp9800_read_temperature(int * tempval);
bool _mcp9800_write_config(uint8_t config);
uint8_t _mcp9800_read_config(void);

// Public functions
// ---------------------------------------------------------------------------

bool mcp9800_init(int i2c_bus, uint8_t id_number)
{
    m_i2cbus_handle = i2c_get_bus_handle(i2c_bus);

    // The ID is 0-7
    if(id_number>7) id_number=7;
    m_id_number = id_number;

    // Check to see if the MCP responded
    //if(i2c_check(DEV_ADDR|m_id_number)==false)
    //return false;

    return true;
}

void mcp9800_configure(mcp9800_resolution_e resolution)
{
    uint8_t config;

    m_resolution = resolution;

    config = resolution;
    config<<=5;

    config|=0x80; // one shot

    _mcp9800_write_config(config);

    _mcp9800_read_config();

    return;
}

void mcp9800_update()
{
    int temp;

    _mcp9800_read_temperature(&temp);

    m_current_temp = temp;

    return;
}

int mcp9800_get_temperature()
{
    return m_current_temp;
}

// Private functions
// ---------------------------------------------------------------------------
bool _mcp9800_read_temperature(int * tempval)
{
    int tmp1,tmp2;
    uint8_t data[2];
    HAL_StatusTypeDef ret;

    // Write address
    data[0] = ADDR_TEMP;
    ret = drv_i2c1_write((DEV_ADDR|m_id_number)<<1, &data, 1);
    if(ret!=HAL_OK)
    {
        *tempval = MCP9800_INVALID_TEMP;
        return false;
    }

    // Read data
    ret = drv_i2c1_read((DEV_ADDR|m_id_number)<<1, &data, 2);
    if(ret!=HAL_OK)
    {
        *tempval = MCP9800_INVALID_TEMP;
        return false;
    }

    // Full degrees, mult by 10
    tmp1 = data[0];
    tmp1*=10;

    // Get fraction 0.1 deg C resolution
    tmp2 = (data[1]>>4);
    tmp2 = (tmp2*10)>>4;

    // Add in
    tmp1+=tmp2;

    *tempval = tmp1;

    nop();

    return true;
}

bool _mcp9800_write_config(uint8_t config)
{
    uint8_t data[2];
    HAL_StatusTypeDef ret;

    // Write address + config
    data[0] = ADDR_CONFIG;
    data[1] = config;
    ret = drv_i2c1_write((DEV_ADDR|m_id_number)<<1, &data, 2);
    if(ret!=HAL_OK) return false;

    return true;
}

uint8_t _mcp9800_read_config()
{
    uint8_t data;
    HAL_StatusTypeDef ret;

    // Write address
    data = ADDR_CONFIG;
    ret = drv_i2c1_write((DEV_ADDR|m_id_number)<<1, &data, 1);
    if(ret!=HAL_OK) return 0;

    // Read config
    ret = drv_i2c1_read((DEV_ADDR|m_id_number)<<1, &data, 1);
    if(ret!=HAL_OK) return 0;

    return data;
}


#endif
// eof
