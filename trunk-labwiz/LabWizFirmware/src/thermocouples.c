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


// Definitions and externs
// ---------------------------------------------------------------------------

// Local module variables
// ---------------------------------------------------------------------------
int m_ambient;

// Private prototypes
// ---------------------------------------------------------------------------
int32_t _thrm_uvolt_to_cel(int32_t microVolts);

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


    #if 0
    // TODO: Magic
    #elif 1
    // So we have 16bit signed (15bits of resolution) from -3300mv to 0mv to 3300mv
    // so each bit is 3300/2^15 = 3300/32768 = 0.100708008mv per bit!
    // Lets call this 100uv/bit
    {
        int uvolts,celcius;
        uvolts = adc_value*100;

        //celcius = _thrm_uvolt_to_cel(uvolts);

        tempval = uvolts;

        // This is milivolts
        //tempval = uvolts/1000;

    }
    #else
    tempval = (int16_t)adc_value;
    #endif

    return tempval;
}

void thrm_set_ambient(int ambient)
{
    m_ambient = ambient;
    return;
}


// Private functions
// ---------------------------------------------------------------------------

// This is a lookup for temperature given microvolts
int32_t _thrm_uvolt_to_cel(int32_t microVolts)
{
#if 0
  float tmpflt,tmpflt2,tmpflt3;
  float LookedupValue = 0.0;
#else
  int16_t tmp16,tmp16_2;
  int32_t interpolated_temp_value=0;
  int32_t tmp32;
#endif
  uint16_t tempWindowLowMicrovolts;
  uint16_t tempWindowHighMicrovolts;

#if 0
  // Input the junction temperature compensated voltage such that the junction
  // temperature is compensated to 0°C

  //Add an offset for the adjusted lookup table.
  microVolts += TK_OFFSET;

  // Check if it's in range
  if(microVolts > TEMP_TYPE_K_MAX_CONVERSION || microVolts < TEMP_TYPE_K_MIN_CONVERSION)
  {
    return OUT_OF_RANGE_INT;
  }

  // Now itterate through the temperature lookup table to find
  // a temperature range for our microvolts.

  // TODO: Binary search here to decrease lookup time
  for(uint16_t i = 0; i<TEMP_TYPE_K_LENGTH; i++)
  {
      // tempLow is the
    tempWindowLowMicrovolts = thermocoupleMicrovoltLookup(i);
    tempWindowHighMicrovolts = thermocoupleMicrovoltLookup(i + 1);

    // NOTE: I think there is a bug here, the lookupThermocouleData macro
    // returns a temperature, but we are comparing that to micovolts
    if(microVolts >= tempWindowLowMicrovolts && microVolts <= tempWindowHighMicrovolts)
    {

        // NOTE: This is a

        #if 1
        // The window lowest temperature
        tmp16 = ( (TK_MIN_TEMP*10) + (i)*100); // max is 16400-2700=13700. Min is -270=-2700

        // The number of microvolts above the lower temp value, times 100
        // Max delta is 520, so max value here is 52000 (this is why we need the int32)
        tmp32 = (100 *(microVolts - tempWindowLowMicrovolts));

        // NOTE: The max could be 52000, why not divide by 2 (>>1) to get within the
        // int16 range, then multiply by 2 after the next operation?

        // Microvolt delta for the window
        // Max delta is 520
        tmp16_2 = (tempWindowHighMicrovolts - tempWindowLowMicrovolts);


        // worst case1 is 13700 + (51900/5200)
        // worst case2 is -2700 + (51900/5200)
        interpolated_temp_value =  tmp16 + ( tmp32 / tmp16_2);
        // NOTE: This operation ^ will always be an int16, since tmp32 and tmp16_2 are related.

        #else
        // ORIGINAL
        /******************* Float Math Start ********************/
        // Float representation of the window lowest temperature
        tmpflt = ( (float)TK_MIN_TEMP + (i)*10);
        // Float representation of the number of microvolts above the lower temp value, times 10
        tmpflt2 = (10 *(float)(microVolts - tempWindowLowMicrovolts));
        // Microvolt delta for the window
        tmpflt3 = ((float)(tempWindowHighMicrovolts - tempWindowLowMicrovolts));

        LookedupValue =  tmpflt + ( tmpflt2 / tmpflt3);
        // Convert from float to int
        interpolated_temp_value = ((int16_t)(LookedupValue*10));
        /******************* Float Math End ********************/
        #endif


      break;
    }

  }
#endif

  return interpolated_temp_value;
}

// eof
