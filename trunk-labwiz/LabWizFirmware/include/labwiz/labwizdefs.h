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

#define btnA(func)          portc_9(func)
#define btnB(func)          porta_8(func)
#define btnC(func)          porta_9(func)
#define btnD(func)          portb_2(func)
#define btnE(func)          portb_2(func)
#define btnPwr(func)        portb_10(func)


#define mod1_gpio0(func)    porta_4(func)
#define mod1_gpio1(func)    porta_1(func)
#define mod2_gpio0(func)    portb_0(func)
#define mod2_gpio1(func)    portc_3(func)
#define mod3_gpio0(func)    portb_12(func)
#define mod3_gpio1(func)    portc_2(func)
#define mod4_gpio0(func)    portb_1(func)
#define mod4_gpio1(func)    portc_0(func)


#define I2C_BUS_1       1
//#define I2C_BUS_2       2

#define SPI_BUS_1       1
#define SPI_BUS_2       2
#define SPI_BUS_3       3


#endif // End include guard

// eof
