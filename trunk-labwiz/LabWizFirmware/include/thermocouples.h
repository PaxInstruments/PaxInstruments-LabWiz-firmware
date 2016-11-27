/*****************************************************************************
 **
 ** Thermocouple interface
 ** September 2016
 **
 ** This module provides functions related to thermocouples
 **
 ****************************************************************************/

#ifndef __THERMOCOUPLES_H__
#define __THERMOCOUPLES_H__

#define THRM_OUT_OF_RANGE       60000
#define THRM_OUT_OF_RANGE_NEG   (-1*THRM_OUT_OF_RANGE)

void thrm_init(void);

void thrm_poll(void);

int thrm_get_uvolts(int channel);

int thrm_get_temperature(int channel);

void thrm_set_ambient(int ambient);

#endif

//eof
