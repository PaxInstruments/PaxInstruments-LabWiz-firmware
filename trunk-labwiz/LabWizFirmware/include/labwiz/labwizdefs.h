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

#include "FreeRTOS.h"
#include "task.h"

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

#define led1(func)          portc_1(func)
#define btnA(func)          portb_15(func)
#define btnB(func)          portc_6(func)
#define btnC(func)          portc_7(func)
#define btnD(func)          portb_12(func)
#define btnE(func)          portb_13(func)
#define btnPwr(func)        porta_0(func)

#endif // End include guard

// eof
