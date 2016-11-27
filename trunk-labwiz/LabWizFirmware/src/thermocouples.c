/*****************************************************************************
 **
 ** Thermocouple interface
 ** September 2016
 **
 ** This module provides functions related to thermocouples
 **
 ****************************************************************************/

#include "defs.h"
#include "drv_mcp3424.h"
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

int thrm_get_uvolts(int channel)
{
    int32_t adc_value;

    switch(channel){
    default:
    case 1: adc_value = mcp3424_get_channel(1); break;
    case 2: adc_value = mcp3424_get_channel(0); break;
    case 3: adc_value = mcp3424_get_channel(3); break;
    case 4: adc_value = mcp3424_get_channel(2); break;
    }

    // So we have 16bit signed (15bits of resolution) from -2048mv to 0mv to 2048mv
    // so each bit is 2048/2^15 = 2048/32768 = 0.0625mv per bit!
    // Lets call this 6.3uv/bit
    adc_value = adc_value*63; // take rounding into account
    adc_value = adc_value/10;  // div by 10 to get in uV

    // Based off measurements, we are off by this much
	#if 1
    // Given an input of 175mV, we were seeing 139.5mV.  This
    // scales the input by 1.25
    adc_value = adc_value*125;
	adc_value = adc_value/100;
	// Hard wiring a sensor to 0, shows a 7-10uV offset, remove
	// this offset
	adc_value -= 7;
	#endif

    // adc_value is now in uvolts

    return adc_value;

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

    tempval = adc_value;

#if 1
    // We have the ADC reading, we need to get
    // it into tenths of degC so 254 = 25.4C

    // So we have 16bit signed (15bits of resolution) from -2048mv to 0mv to 2048mv
	// so each bit is 2048/2^15 = 2048/32768 = 0.0625mv per bit!
	// Lets call this 6.3uv/bit
	adc_value = adc_value*63; // take rounding into account
	adc_value = adc_value/10;  // div by 10 to get in uV

	// Based off measurements, we are off by this much
	#if 1
	// Given an input of 175mV, we were seeing 139.5mV.  This
	// scales the input by 1.25
	adc_value = adc_value*125;
	adc_value = adc_value/100;
	// Hard wiring a sensor to 0, shows a 7-10uV offset, remove
	// this offset
	adc_value -= 7;
	#endif


    {
        int uvolts,celcius;
        uvolts = adc_value;
        tempval = uvolts;

        #if 1
        uvolts += m_ambient_uvolts;
        celcius = thrmMicroVoltsToC(uvolts);
        tempval = celcius;
        #endif
    }
#endif
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
