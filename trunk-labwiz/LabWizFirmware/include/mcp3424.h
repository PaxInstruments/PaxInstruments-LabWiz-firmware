/*****************************************************************************
 **
 ** MCP3424 driver
 ** September 2016
 **
 ** This driver provides functions to control an MCP3424
 **
 ****************************************************************************/

#ifndef __MCP3424_H__
#define __MCP3424_H__
#ifdef ENABLE_MCP3424

typedef enum{
    MCP_OK = 0,
    MCP_ERROR,
    MCP_BUSY,
    MCP_TIMEOUT,
}mcp3424_result_e;

typedef enum{
    FORMAT_12BITS = 0,  // 240 SPS
    FORMAT_14BITS,      // 60 SPS
    FORMAT_16BITS,      // 15 SPS
    FORMAT_18BITS,      // 3.75 SPS
}mcp_format_e;

typedef enum{
    GAIN_1 = 0,
    GAIN_2,
    GAIN_4,
    GAIN_8,
}mcp_gain_e;

bool mcp3424_init(int i2c_bus, uint8_t id_number);

void mcp3424_configure(mcp_format_e format, mcp_gain_e gain);

void mcp3424_poll(void);

int32_t mcp3424_get_channel(int channel);

#endif
#endif

//eof
