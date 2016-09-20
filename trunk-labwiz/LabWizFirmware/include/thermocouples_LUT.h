// Source: https://wemakethings.net/2014/04/24/k-thermocouple-lib/
// Modified September 2016 - Andrew Gaylo
#ifndef _THRM_LUT_H_
#define _THRM_LUT_H_

typedef struct {
    int32_t temperature;
    int32_t microvolts;
} thrm_lookup_t;

/**
 * Returns temperature as a function of the measured thermocouple voltage.
 *
 * int32_t microvoltsMeasured - Microvolts from 0v +/-
 *
 * Return: int32_t of the temperature in C in 1/10ths of a deg C
 *
  **/
int32_t thrmMicroVoltsToC(int32_t microvoltsMeasured);

/**
 * Return the microvolts of a given temperature
 *
 * int32_t measured_temp - Temperature in 1/10ths of deg C
 *
 * Return: uint32_t of the microvolts for the temperature from 0v +/-
 *
 **/
int32_t thrmCToMicroVolts(int32_t measured_temp);

#endif
