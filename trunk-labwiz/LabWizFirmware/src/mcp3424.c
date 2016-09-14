/*****************************************************************************
 **
 ** MCP3424 driver
 ** September 2016
 **
 ** This driver provides functions to control an MCP3424
 **
 ****************************************************************************/

#include "defs.h"
#include "labwiz/drv_i2c.h"
#include "stm32f1xx_hal.h"

#include "mcp3424.h"

#ifdef ENABLE_MCP3424

// Definitions and externs
// ---------------------------------------------------------------------------
typedef struct{
    unsigned char ready         :1;
    unsigned char channel       :2;
    unsigned char mode          :1;
    unsigned char sample_rate   :2;
    unsigned char gain          :2;
}mcp_config_t;

// This is fixed for the MCP342x
#define DEV_ADDR            0x68

#define NUM_CHANNELS        4

// Local module variables
// ---------------------------------------------------------------------------
I2C_HandleTypeDef * m_i2cbus_handle = NULL;

mcp_config_t m_config_reg;

uint8_t m_id_number = 0;

uint32_t m_adc_data[NUM_CHANNELS];

// Private prototypes
// ---------------------------------------------------------------------------
void _mcp3424_write_config(void);
uint8_t _mcp3424_read_config(void);
void _mcp3424_next(void);


// Public functions
// ---------------------------------------------------------------------------

bool mcp3424_init(int i2c_bus, uint8_t id_number)
{
    m_i2cbus_handle = i2c_get_bus_handle(i2c_bus);

    // The ID is how the A0,A1 lines are configured
    if(id_number>3) id_number=3;
    m_id_number = id_number;

    // Check to see if the MCP responded
    //if(i2c_check(DEV_ADDR|m_id_number)==false)
    //return false;

    return true;
}

void mcp3424_configure(mcp_format_e format, mcp_gain_e gain)
{
    m_config_reg.mode = 0 ; // 0 = One shot
    m_config_reg.channel = 0;
    m_config_reg.gain = gain;
    m_config_reg.sample_rate = format;

    m_config_reg.ready = 1; // Start a one shot conv.

    // Write the configuration register
    _mcp3424_write_config();

    return;
}

void mcp3424_poll()
{
    uint8_t config;
    mcp_config_t * config_ptr;
    config_ptr = (mcp_config_t*)&config;

    //config = _mcp3424_read_config();

    nop();

#if 0
    if(config_ptr->ready==0)
    {
        uint32_t data;
        data = _mcp_read_data();
        m_adc_data[m_config_reg.channel] = data;
        _mcp_next();
    }
#endif

    return;
}

uint32_t mcp3424_get_channel(int channel)
{
#if 0
    if(channel>=NUM_CHANNELS) channel = 0;
    return m_adc_data[channel];
#elif 0
    return 0;
#else
    return (uint32_t)channel;
#endif
}

#if 0
// array must be a uint32_t[4], anything shorter and there
// will be a buffer overflow
void mcp3432_get_channels(uint32_t * array)
{
    //memcpy(array,m_adc_data,sizeof(m_adc_data));
    return;
}
#endif

// Private functions
// ---------------------------------------------------------------------------
void _mcp3424_write_config()
{
    return;
}

uint8_t _mcp3424_read_config()
{
    uint8_t config=0x80; // by default /ready=1 which is NOT ready
    HAL_StatusTypeDef ret;
    ret = drv_i2c1_read((DEV_ADDR|m_id_number), (uint8_t*)&config, 1);
    nop();
    return config;
}

void _mcp3424_next()
{
    m_config_reg.channel++;
    if(m_config_reg.channel>(NUM_CHANNELS-1)) m_config_reg.channel = 0;
    // Start conv.
    m_config_reg.ready = 1;
    _mcp3424_write_config();
    return;
}

#endif
// eof
