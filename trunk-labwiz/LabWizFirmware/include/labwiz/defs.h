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

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

// Intrinsics for the processor
#ifndef nop
#define nop()   asm("nop")
#endif

// Here is where we should specify which pins are defined
// See port.h for more information on pin IO
#define led1(func)          porta_8(func)

#define interrupts_enable()     __enable_irq ();
#define interrupts_disable()    __disable_irq ();


#endif // End include guard

// eof
