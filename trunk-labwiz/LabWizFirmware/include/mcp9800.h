/*****************************************************************************
 **
 ** MCP9800 driver
 ** September 2016
 **
 ** This driver provides functions to control an MCP9800
 **
 ****************************************************************************/

#ifndef __MCP9800_H__
#define __MCP9800_H__

#ifdef ENABLE_MCP9800

typedef enum{
    MCP9800_9BIT = 0,   //9 bit or 0.5°C (Power-up default)
    MCP9800_10BIT,      //10 bit or 0.25°C
    MCP9800_11BIT,      //11 bit or 0.125°C
    MCP9800_12BIT,      //12 bit or 0.0625°C
}mcp9800_resolution_e;

#define MCP9800_INVALID_TEMP       -1000

bool mcp9800_init(int i2c_bus, uint8_t id_number);

void mcp9800_configure(mcp9800_resolution_e resolution);

void mcp9800_update(void);

int mcp9800_get_temperature(void);

#endif
#endif

//eof
