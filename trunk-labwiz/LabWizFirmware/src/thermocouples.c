/*****************************************************************************
 **
 ** Thermocouple interface
 ** September 2016
 **
 ** This module provides functions related to thermocouples
 **
 ****************************************************************************/

#include "defs.h"
#include "mcp3424.h"
#include "thermocouples.h"
#include "thermocouples_private.h"
#include "thermocouples_LUT.h"


// Definitions and externs
// ---------------------------------------------------------------------------

// Local module variables
// ---------------------------------------------------------------------------
int32_t m_ambient_uvolts;

// Private prototypes
// ---------------------------------------------------------------------------

// Public functions
// ---------------------------------------------------------------------------

void thrm_init()
{
    // Setup MCP3424
    mcp3424_init(I2C_BUS_1,1);
    mcp3424_configure(FORMAT_16BITS,GAIN_8);

    return;
}

void thrm_poll()
{
    // This does all the MCP processing
    mcp3424_poll();

    return;
}

// Get temperature value in tenths of deg C for channel 1-4
int thrm_get_temperature(int channel)
{
    int32_t tempval=0;
    int32_t adc_value;

    switch(channel){
    default:
    case 1: adc_value = mcp3424_get_channel(1); break;
    case 2: adc_value = mcp3424_get_channel(0); break;
    case 3: adc_value = mcp3424_get_channel(3); break;
    case 4: adc_value = mcp3424_get_channel(2); break;
    }

    // We have the ADC reading for 0-3.3v, we need to get
    // it into tenths of degC so 254 = 25.4C

    // So we have 16bit signed (15bits of resolution) from -3300mv to 0mv to 3300mv
    // so each bit is 3300/2^15 = 3300/32768 = 0.100708008mv per bit!
    // Lets call this 101uv/bit
    {
        int uvolts,celcius;
        uvolts = adc_value*101;
        //tempval = uvolts;

        uvolts += m_ambient_uvolts;
        celcius = thrmMicroVoltsToC(uvolts);
        tempval = celcius;
    }

    return tempval;
}

void thrm_set_ambient(int ambient)
{
    int32_t ambient_uvolts;

    ambient_uvolts = thrmCToMicroVolts(ambient);

    m_ambient_uvolts = ambient_uvolts;
    return;
}


// Private functions
// ---------------------------------------------------------------------------

// eof
