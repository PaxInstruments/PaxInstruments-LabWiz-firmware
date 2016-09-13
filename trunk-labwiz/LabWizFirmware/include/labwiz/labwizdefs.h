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

#define btnA(func)          portc_11(func)
#define btnB(func)          portc_10(func)
#define btnC(func)          portc_7(func)
#define btnD(func)          porta_8(func)
#define btnE(func)          portb_2(func)
#define btnPwr(func)        porta_0(func)


#define mod1_gpio1(func)    portc_4(func)
#define mod2_gpio0(func)    porta_1(func)
#define mod2_gpio1(func)    portc_3(func)
#define mod3_gpio1(func)    portc_2(func)
#define mod4_gpio0(func)    portc_5(func)
#define mod4_gpio1(func)    portb_1(func)

#endif // End include guard

// eof
