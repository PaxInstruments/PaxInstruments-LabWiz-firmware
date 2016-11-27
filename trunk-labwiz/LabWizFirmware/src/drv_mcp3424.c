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

#include "drv_mcp3424.h"

#include "string.h"

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
//#define DEV_ADDR            0x48

#define NUM_CHANNELS        4

// Local module variables
// ---------------------------------------------------------------------------
static I2C_HandleTypeDef * m_i2cbus_handle = NULL;
static uint8_t m_id_number = 0;

mcp_config_t m_config_reg;

int32_t m_adc_data[NUM_CHANNELS];

mcp_format_e m_data_format = FORMAT_12BITS;

// Private prototypes
// ---------------------------------------------------------------------------
bool _mcp3424_write_config(mcp_config_t*config);
bool _mcp3424_read_data(mcp_config_t * config,int32_t * data);
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
    m_data_format = format;

    m_config_reg.ready = 1; // Start a one shot conv.

    // Write the configuration register
    _mcp3424_write_config(&m_config_reg);

    return;
}

void mcp3424_poll()
{
    mcp_config_t config;
    int32_t data;

    if(_mcp3424_read_data(&config,&data))
    {
        if(config.ready==0)
        {
            m_adc_data[config.channel] = data;
            _mcp3424_next();
        }
    }

    return;
}

int32_t mcp3424_get_channel(int channel)
{
#if 1
    if(channel>=NUM_CHANNELS) channel = 0;
    return m_adc_data[channel];
#elif 0
    return 0;
#else
    return (int32_t)channel;
#endif
}

#if 0
// array must be a int32_t[4], anything shorter and there
// will be a buffer overflow
void mcp3432_get_channels(int32_t * array)
{
    //memcpy(array,m_adc_data,sizeof(m_adc_data));
    return;
}
#endif

// Private functions
// ---------------------------------------------------------------------------
bool _mcp3424_write_config(mcp_config_t*configptr)
{
    HAL_StatusTypeDef ret;
    uint8_t configval;

    // TODO, can we just overlay the struct on the byte?
    configval = configptr->ready;
    configval<<=2;
    configval |= configptr->channel;
    configval<<=1;
    configval |= configptr->mode;
    configval<<=2;
    configval |= configptr->sample_rate;
    configval<<=2;
    configval |= configptr->gain;

    nop();

    ret = drv_i2c1_write((DEV_ADDR|m_id_number)<<1, &configval, 1);
    if(ret!=HAL_OK) return false;

    return true;
}

bool _mcp3424_read_data(mcp_config_t * config,int32_t * data)
{
    uint8_t rawdata[4];
    uint8_t configbyte=0x80; // by default /ready=1 which is NOT ready
    uint32_t datavalue = 0;
    int32_t datavalue_signed = 0;
    HAL_StatusTypeDef ret;

    ret = drv_i2c1_read((DEV_ADDR|m_id_number)<<1, rawdata, 4);
    if(ret!=HAL_OK)
        return false;
    switch(m_data_format){
    case FORMAT_18BITS:
        datavalue = (rawdata[0]&0x03);
        datavalue<<=8;
        datavalue |= rawdata[1];
        datavalue<<=8;
        datavalue |= rawdata[2];

        // value in uint32 represents a signed
        // 18bit value, turn into a signed int32
        datavalue_signed = (int32_t)(datavalue << 14) / 16384;
        // Shift up to match sign, then divide by 2^14 to get back to
        // the original value
        // http://stackoverflow.com/questions/34075922/convert-raw-14-bit-twos-complement-to-signed-16-bit-integer

        configbyte = rawdata[3];
        break;
    case FORMAT_12BITS:
        datavalue = rawdata[0]&0x0F;
        datavalue<<=8;
        datavalue |= rawdata[1];

        // value in uint32 represents a signed
        // 12bit value, turn into a signed int32
        datavalue_signed = (int32_t)(datavalue << 20) / 1048576;
        // Shift up to match sign, then divide by 2^20 to get back to
        // the original value

        configbyte = rawdata[2];
        break;
    case FORMAT_14BITS:
        datavalue = rawdata[0]&0x3F;
        datavalue<<=8;
        datavalue |= rawdata[1];

        // value in uint32 represents a signed
        // 14bit value, turn into a signed int32
        datavalue_signed = (int32_t)(datavalue << 18) / 262144;
        // Shift up to match sign, then divide by 2^18 to get back to
        // the original value

        configbyte = rawdata[2];
        break;
    case FORMAT_16BITS:
    default:
        datavalue = rawdata[0];
        datavalue<<=8;
        datavalue |= rawdata[1];

        // value in uint32 represents a signed
        // 16bit value, turn into a signed int32
        datavalue_signed = (int32_t)(datavalue << 16) / 65536;
        // Shift up to match sign, then divide by 2^16 to get back to
        // the original value

        configbyte = rawdata[2];
        break;
    }
    *data = datavalue_signed;

    // TODO, can we just overlay the struct on the byte?
    config->gain = configbyte&0x03;
    configbyte>>=2;
    config->sample_rate = configbyte&0x03;
    configbyte>>=2;
    config->mode = configbyte&0x01;
    configbyte>>=1;
    config->channel = configbyte&0x03;
    configbyte>>=2;
    config->ready = configbyte&0x01;

    return true;
}

void _mcp3424_next()
{
    m_config_reg.channel++;
    if(m_config_reg.channel>(NUM_CHANNELS-1)) m_config_reg.channel = 0;
    // Start conv.
    m_config_reg.ready = 1;
    if(_mcp3424_write_config(&m_config_reg)==false)
        asm("nop");
    return;
}


#endif
// eof
