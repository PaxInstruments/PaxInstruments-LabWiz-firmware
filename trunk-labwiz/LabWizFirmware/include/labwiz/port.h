/*****************************************************************************
 **
 ** Port header for pin IO
 ** August 2016
 **
 ****************************************************************************/

#ifndef __PORT_H__
#define __PORT_H__  // Include guard

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"


// Port struct pointers to registers (Evaluated at compile time)
#define PORTA ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(0)))
#define PORTB ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(1)))
#define PORTC ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(2)))

// Macros for SET, GET, READ, READ_LAT, TOGGLE. These
// are macros because they compile down to a very tight
// set of instructions.
#define SET(REG, BIT)       do{((REG)->BSRR)=(uint32_t)(1<<(BIT));}while(0)
#define CLR(REG, BIT)       do{((REG)->BRR)=(uint32_t)(1<<(BIT));}while(0)
#define WRITE(REG,BIT,V)    do{if(V){SET((REG),(BIT));}else{CLR((REG),(BIT));}}while(0)
#define READ(REG,BIT)       (((REG)->IDR)&((uint32_t)(1<<BIT)))
#define READ_LAT(REG,BIT)   (((REG)->ODR)&((uint32_t)(1<<BIT)))
#define TOGGLE(R,B)         do{if(READ_LAT((R),(B))){CLR((R),(B));}else{SET((R),(B));}}while(0)

// Macros for pin operations, easy to use from C and human readable. Each
// macro has a portX_#(func) at the top, this is to allow calls such
// as portX_#(set()).  A new more human readable definition can be created
// to reference the port macros.
//
// Example:
//
// // LED on porta pin 0
// #define led(func)        port1_0(func)
//
// led(set);
// leg(toggle);
//
// Should the pin ever change it's trivial to update code

// PortA
#define porta_0(func)       porta_0_##func
#define porta_0_set()       do{SET(PORTA,0);}while(0)
#define porta_0_clear()     do{CLR(PORTA,0);}while(0)
#define porta_0_write(V)    do{WRITE(PORTA,0,(V));}while(0)
#define porta_0_read()      (READ(PORTA,0))
#define porta_0_read_lat()  (READ_LAT(PORTA,0))
#define porta_0_toggle()    do{TOGGLE(PORTA,0);}while(0)

#define porta_1(func)       porta_1_##func
#define porta_1_set()       do{SET(PORTA,1);}while(0)
#define porta_1_clear()     do{CLR(PORTA,1);}while(0)
#define porta_1_write(V)    do{WRITE(PORTA,1,(V));}while(0)
#define porta_1_read()      (READ(PORTA,1))
#define porta_1_read_lat()  (READ_LAT(PORTA,1))
#define porta_1_toggle()    do{TOGGLE(PORTA,1);}while(0)

#define porta_8(func)       porta_8_##func
#define porta_8_set()       do{SET(PORTA,8);}while(0)
#define porta_8_clear()     do{CLR(PORTA,8);}while(0)
#define porta_8_write(V)    do{WRITE(PORTA,8,(V));}while(0)
#define porta_8_read()      (READ(PORTA,8))
#define porta_8_read_lat()  (READ_LAT(PORTA,8))
#define porta_8_toggle()    do{TOGGLE(PORTA,8);}while(0)

// PortB

#define portb_9(func)       portb_9_##func
#define portb_9_set()       do{SET(PORTB,9);}while(0)
#define portb_9_clear()     do{CLR(PORTB,9);}while(0)
#define portb_9_write(V)    do{WRITE(PORTB,9,(V));}while(0)
#define portb_9_read()      (READ(PORTB,9))
#define portb_9_read_lat()  (READ_LAT(PORTB,9))
#define portb_9_toggle()    do{TOGGLE(PORTB,9);}while(0)

#define portb_13(func)      portb_13_##func
#define portb_13_set()      do{SET(PORTB,13);}while(0)
#define portb_13_clear()    do{CLR(PORTB,13);}while(0)
#define portb_13_write(V)   do{WRITE(PORTB,13,(V));}while(0)
#define portb_13_read()     (READ(PORTB,13))
#define portb_13_read_lat() (READ_LAT(PORTB,13))
#define portb_13_toggle()   do{TOGGLE(PORTB,13);}while(0)

// PortC
#define portc_13(func)      portc_13_##func
#define portc_13_set()      do{SET(PORTC,13);}while(0)
#define portc_13_clear()    do{CLR(PORTC,13);}while(0)
#define portc_13_write(V)   do{WRITE(PORTC,13,(V));}while(0)
#define portc_13_read()     (READ(PORTC,13))
#define portc_13_read_lat() (READ_LAT(PORTC,13))
#define portc_13_toggle()   do{TOGGLE(PORTC,13);}while(0)

#ifndef HIGH
#define HIGH  (1)
#endif
#ifndef LOW
#define LOW   (0)
#endif

#endif // End include guard

// eof
