/*****************************************************************************
 **
 ** LabWiz Definitions
 ** August 2016
 **
 ****************************************************************************/

#ifndef __DEFS_H__
#define __DEFS_H__  // Include guard

#include "stdbool.h"
#include "stm32f1xx.h"

#define FIRMWARE_VERSION    "0.1"

// Intrinsics for the processor
#ifndef nop
#define nop()   asm("nop")
#endif

// Here is where we should specify which pins are defined
// See port.h for more information on pin IO
#define led1(func)          portc_13(func)


#endif // End include guard

// eof
