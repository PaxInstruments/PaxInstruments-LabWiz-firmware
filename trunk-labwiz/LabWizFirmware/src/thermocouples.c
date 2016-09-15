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


// Definitions and externs
// ---------------------------------------------------------------------------

// Local module variables
// ---------------------------------------------------------------------------
int32_t m_ambient_uvolts;

// Private prototypes
// ---------------------------------------------------------------------------
int32_t _thrm_cel_to_uvolt(int32_t celcius);
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
    // Lets call this 101uv/bit
    {
        int uvolts,celcius;
        uvolts = adc_value*101;

        uvolts = uvolts + m_ambient_uvolts;

        // This gets us to 1/100ths of a deg C, div by 10 to get 1/10ths
        celcius = _thrm_uvolt_to_cel(uvolts);
        tempval = celcius/10;

        //tempval = uvolts;
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
    int32_t ambient_uvolts;

    ambient_uvolts = _thrm_cel_to_uvolt(ambient);

    m_ambient_uvolts = ambient_uvolts;
    return;
}


// Private functions
// ---------------------------------------------------------------------------


// This is a lookup from temperature to microvolts
int32_t _thrm_cel_to_uvolt(int32_t celcius)
{
  int voltage = 0;
  int lut_index = 0;
  int tempWindowLowMicrovolts;
  int tempWindowHighMicrovolts;

  // lut = Look Up Table

  // Original
  //lut_index = ((float)celcius) /10 + 27;

  // Int math, we are in 10ths of degs.  We know 0C is at index 27, so if each step
  // is 10C, ournum/100 is the number of steps above/below 0...add index 27
  lut_index = ((celcius/100)+27);

  // This gets us a 10C range this value could be in
  tempWindowLowMicrovolts = tempTypK[lut_index];
  tempWindowHighMicrovolts = tempTypK[lut_index + 1];

  // This interpolates the voltage between the 2 points??
  // ??? (low - offset) + ((temp - VAR) * (delta/10))
  voltage = tempWindowLowMicrovolts - TK_OFFSET + (celcius - (lut_index*10-270)) * (tempWindowHighMicrovolts - tempWindowLowMicrovolts)/10;

  //return i; // Displays '29.0' on the LCD as expected
  //return tempTypK[29]; // Displays '7256.0'on LCD
  return voltage;
}

// This is a lookup for temperature given microvolts
int32_t _thrm_uvolt_to_cel(int32_t microVolts)
{
#if 0
  float tmpflt,tmpflt2,tmpflt3;
  float LookedupValue = 0.0;
#else
  //int16_t tmp16,tmp16_2;
  int32_t interpolated_temp_value=0;
  int32_t tmp,tmp2,tmp3,i;
#endif
  int32_t tempWindowLowMicrovolts;
  int32_t tempWindowHighMicrovolts;

  // Input the junction temperature compensated voltage such that the junction
  // temperature is compensated to 0°C

  //Add an offset for the adjusted lookup table. (This gets to to mv above abs zero)
  microVolts += TK_OFFSET;

  // Check if it's in range
  if(microVolts > TEMP_TYPE_K_MAX_CONVERSION || microVolts < TEMP_TYPE_K_MIN_CONVERSION)
  {
    return THRM_OUT_OF_RANGE;
  }

  // Now itterate through the temperature lookup table to find
  // a temperature range for our microvolts.

  for(i = 0; i<TEMP_TYPE_K_LENGTH; i++)
  {
    // Get the lower and upper uV entry for this 10C window
    tempWindowLowMicrovolts = tempTypK[i];
    tempWindowHighMicrovolts = tempTypK[i+1];

    // If our microvolts are in this window...
    if(microVolts >= tempWindowLowMicrovolts && microVolts <= tempWindowHighMicrovolts)
    {


        #if 1

        // The window lowest temperature
        tmp = ( (TK_MIN_TEMP*10) + (i)*100); // max is 16400-2700=13700. Min is -270=-2700

        // The number of microvolts above the lower temp value, times 100
        // Max delta is 520, so max value here is 52000
        tmp2 = (100 *(microVolts - tempWindowLowMicrovolts));

        // Microvolt delta for the window
        // Max delta is 520
        tmp3 = (tempWindowHighMicrovolts - tempWindowLowMicrovolts);


        // worst case1 is 13700 + (51900/5200)
        // worst case2 is -2700 + (51900/5200)
        interpolated_temp_value =  tmp + ( tmp2 / tmp3);

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

  return interpolated_temp_value;
}

// eof
