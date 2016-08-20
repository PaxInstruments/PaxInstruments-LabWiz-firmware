/*****************************************************************************
 **
 ** LabWiz Definitions
 ** August 2016
 **
 ****************************************************************************/

#ifndef __LABWIZDEFS_H__
#define __LABWIZDEFS_H__  // Include guard

#include "stdint.h"
#include "stdbool.h"
#include "stm32f1xx.h"
#include "labwiz/port.h"

#define LABWIZ_VERSION    "1.0b"

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"

// Intrinsics for the processor
#ifndef nop
#define nop()   asm("nop")
#endif

#define MS_PER_SECOND       1000

// Here is where we should specify which pins are defined
// See port.h for more information on pin IO

#define interrupts_enable()     __enable_irq ();
#define interrupts_disable()    __disable_irq ();

void setup(void);
void loop(void);

#define led1(func)          porta_8(func)

#endif // End include guard

// eof
