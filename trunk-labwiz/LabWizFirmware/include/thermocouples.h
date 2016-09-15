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

void thrm_init(void);

void thrm_poll(void);

int thrm_get_temperature(int channel);

void thrm_set_ambient(int ambient);

#endif

//eof
