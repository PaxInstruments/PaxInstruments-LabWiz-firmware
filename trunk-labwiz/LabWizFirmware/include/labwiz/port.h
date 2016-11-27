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

#define port_none(func)       port_none_##func
#define port_none_set()       asm("nop")
#define port_none_clear()     asm("nop")
#define port_none_write(V)    asm("nop")
#define port_none_read()      (0)
#define port_none_read_lat()  (0)
#define port_none_toggle()    asm("nop")

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

#define porta_3(func)       porta_3_##func
#define porta_3_set()       do{SET(PORTA,3);}while(0)
#define porta_3_clear()     do{CLR(PORTA,3);}while(0)
#define porta_3_write(V)    do{WRITE(PORTA,3,(V));}while(0)
#define porta_3_read()      (READ(PORTA,3))
#define porta_3_read_lat()  (READ_LAT(PORTA,3))
#define porta_3_toggle()    do{TOGGLE(PORTA,3);}while(0)

#define porta_8(func)       porta_8_##func
#define porta_8_set()       do{SET(PORTA,8);}while(0)
#define porta_8_clear()     do{CLR(PORTA,8);}while(0)
#define porta_8_write(V)    do{WRITE(PORTA,8,(V));}while(0)
#define porta_8_read()      (READ(PORTA,8))
#define porta_8_read_lat()  (READ_LAT(PORTA,8))
#define porta_8_toggle()    do{TOGGLE(PORTA,8);}while(0)

#define porta_9(func)       porta_9_##func
#define porta_9_set()       do{SET(PORTA,9);}while(0)
#define porta_9_clear()     do{CLR(PORTA,9);}while(0)
#define porta_9_write(V)    do{WRITE(PORTA,9,(V));}while(0)
#define porta_9_read()      (READ(PORTA,9))
#define porta_9_read_lat()  (READ_LAT(PORTA,9))
#define porta_9_toggle()    do{TOGGLE(PORTA,9);}while(0)

#define porta_10(func)       porta_10_##func
#define porta_10_set()       do{SET(PORTA,10);}while(0)
#define porta_10_clear()     do{CLR(PORTA,10);}while(0)
#define porta_10_write(V)    do{WRITE(PORTA,10,(V));}while(0)
#define porta_10_read()      (READ(PORTA,10))
#define porta_10_read_lat()  (READ_LAT(PORTA,10))
#define porta_10_toggle()    do{TOGGLE(PORTA,10);}while(0)

#define porta_15(func)       porta_15_##func
#define porta_15_set()       do{SET(PORTA,15);}while(0)
#define porta_15_clear()     do{CLR(PORTA,15);}while(0)
#define porta_15_write(V)    do{WRITE(PORTA,15,(V));}while(0)
#define porta_15_read()      (READ(PORTA,15))
#define porta_15_read_lat()  (READ_LAT(PORTA,15))
#define porta_15_toggle()    do{TOGGLE(PORTA,15);}while(0)

// PortB
#define portb_1(func)       portb_1_##func
#define portb_1_set()       do{SET(PORTB,1);}while(0)
#define portb_1_clear()     do{CLR(PORTB,1);}while(0)
#define portb_1_write(V)    do{WRITE(PORTB,1,(V));}while(0)
#define portb_1_read()      (READ(PORTB,1))
#define portb_1_read_lat()  (READ_LAT(PORTB,1))
#define portb_1_toggle()    do{TOGGLE(PORTB,1);}while(0)

#define portb_2(func)       portb_2_##func
#define portb_2_set()       do{SET(PORTB,2);}while(0)
#define portb_2_clear()     do{CLR(PORTB,2);}while(0)
#define portb_2_write(V)    do{WRITE(PORTB,2,(V));}while(0)
#define portb_2_read()      (READ(PORTB,2))
#define portb_2_read_lat()  (READ_LAT(PORTB,2))
#define portb_2_toggle()    do{TOGGLE(PORTB,2);}while(0)

#define portb_4(func)       portb_4_##func
#define portb_4_set()       do{SET(PORTB,4);}while(0)
#define portb_4_clear()     do{CLR(PORTB,4);}while(0)
#define portb_4_write(V)    do{WRITE(PORTB,4,(V));}while(0)
#define portb_4_read()      (READ(PORTB,4))
#define portb_4_read_lat()  (READ_LAT(PORTB,4))
#define portb_4_toggle()    do{TOGGLE(PORTB,4);}while(0)

#define portb_6(func)       portb_6_##func
#define portb_6_set()       do{SET(PORTB,6);}while(0)
#define portb_6_clear()     do{CLR(PORTB,6);}while(0)
#define portb_6_write(V)    do{WRITE(PORTB,6,(V));}while(0)
#define portb_6_read()      (READ(PORTB,6))
#define portb_6_read_lat()  (READ_LAT(PORTB,6))
#define portb_6_toggle()    do{TOGGLE(PORTB,6);}while(0)

#define portb_7(func)       portb_7_##func
#define portb_7_set()       do{SET(PORTB,7);}while(0)
#define portb_7_clear()     do{CLR(PORTB,7);}while(0)
#define portb_7_write(V)    do{WRITE(PORTB,7,(V));}while(0)
#define portb_7_read()      (READ(PORTB,7))
#define portb_7_read_lat()  (READ_LAT(PORTB,7))
#define portb_7_toggle()    do{TOGGLE(PORTB,7);}while(0)

#define portb_8(func)       portb_8_##func
#define portb_8_set()       do{SET(PORTB,8);}while(0)
#define portb_8_clear()     do{CLR(PORTB,8);}while(0)
#define portb_8_write(V)    do{WRITE(PORTB,8,(V));}while(0)
#define portb_8_read()      (READ(PORTB,8))
#define portb_8_read_lat()  (READ_LAT(PORTB,8))
#define portb_8_toggle()    do{TOGGLE(PORTB,8);}while(0)


#define portb_9(func)       portb_9_##func
#define portb_9_set()       do{SET(PORTB,9);}while(0)
#define portb_9_clear()     do{CLR(PORTB,9);}while(0)
#define portb_9_write(V)    do{WRITE(PORTB,9,(V));}while(0)
#define portb_9_read()      (READ(PORTB,9))
#define portb_9_read_lat()  (READ_LAT(PORTB,9))
#define portb_9_toggle()    do{TOGGLE(PORTB,9);}while(0)

#define portb_10(func)      portb_10_##func
#define portb_10_set()      do{SET(PORTB,10);}while(0)
#define portb_10_clear()    do{CLR(PORTB,10);}while(0)
#define portb_10_write(V)   do{WRITE(PORTB,10,(V));}while(0)
#define portb_10_read()     (READ(PORTB,10))
#define portb_10_read_lat() (READ_LAT(PORTB,10))
#define portb_10_toggle()   do{TOGGLE(PORTB,10);}while(0)

#define portb_12(func)      portb_12_##func
#define portb_12_set()      do{SET(PORTB,12);}while(0)
#define portb_12_clear()    do{CLR(PORTB,12);}while(0)
#define portb_12_write(V)   do{WRITE(PORTB,12,(V));}while(0)
#define portb_12_read()     (READ(PORTB,12))
#define portb_12_read_lat() (READ_LAT(PORTB,12))
#define portb_12_toggle()   do{TOGGLE(PORTB,12);}while(0)

#define portb_13(func)      portb_13_##func
#define portb_13_set()      do{SET(PORTB,13);}while(0)
#define portb_13_clear()    do{CLR(PORTB,13);}while(0)
#define portb_13_write(V)   do{WRITE(PORTB,13,(V));}while(0)
#define portb_13_read()     (READ(PORTB,13))
#define portb_13_read_lat() (READ_LAT(PORTB,13))
#define portb_13_toggle()   do{TOGGLE(PORTB,13);}while(0)

#define portb_15(func)      portb_15_##func
#define portb_15_set()      do{SET(PORTB,15);}while(0)
#define portb_15_clear()    do{CLR(PORTB,15);}while(0)
#define portb_15_write(V)   do{WRITE(PORTB,15,(V));}while(0)
#define portb_15_read()     (READ(PORTB,15))
#define portb_15_read_lat() (READ_LAT(PORTB,15))
#define portb_15_toggle()   do{TOGGLE(PORTB,15);}while(0)

// PortC
#define portc_0(func)       portc_0_##func
#define portc_0_set()       do{SET(PORTC,0);}while(0)
#define portc_0_clear()     do{CLR(PORTC,0);}while(0)
#define portc_0_write(V)    do{WRITE(PORTC,0,(V));}while(0)
#define portc_0_read()      (READ(PORTC,0))
#define portc_0_read_lat()  (READ_LAT(PORTC,0))
#define portc_0_toggle()    do{TOGGLE(PORTC,0);}while(0)

#define portc_1(func)       portc_1_##func
#define portc_1_set()       do{SET(PORTC,1);}while(0)
#define portc_1_clear()     do{CLR(PORTC,1);}while(0)
#define portc_1_write(V)    do{WRITE(PORTC,1,(V));}while(0)
#define portc_1_read()      (READ(PORTC,1))
#define portc_1_read_lat()  (READ_LAT(PORTC,1))
#define portc_1_toggle()    do{TOGGLE(PORTC,1);}while(0)

#define portc_2(func)       portc_2_##func
#define portc_2_set()       do{SET(PORTC,2);}while(0)
#define portc_2_clear()     do{CLR(PORTC,2);}while(0)
#define portc_2_write(V)    do{WRITE(PORTC,2,(V));}while(0)
#define portc_2_read()      (READ(PORTC,2))
#define portc_2_read_lat()  (READ_LAT(PORTC,2))
#define portc_2_toggle()    do{TOGGLE(PORTC,2);}while(0)

#define portc_3(func)       portc_3_##func
#define portc_3_set()       do{SET(PORTC,3);}while(0)
#define portc_3_clear()     do{CLR(PORTC,3);}while(0)
#define portc_3_write(V)    do{WRITE(PORTC,3,(V));}while(0)
#define portc_3_read()      (READ(PORTC,3))
#define portc_3_read_lat()  (READ_LAT(PORTC,3))
#define portc_3_toggle()    do{TOGGLE(PORTC,3);}while(0)

#define portc_4(func)       portc_4_##func
#define portc_4_set()       do{SET(PORTC,4);}while(0)
#define portc_4_clear()     do{CLR(PORTC,4);}while(0)
#define portc_4_write(V)    do{WRITE(PORTC,4,(V));}while(0)
#define portc_4_read()      (READ(PORTC,4))
#define portc_4_read_lat()  (READ_LAT(PORTC,4))
#define portc_4_toggle()    do{TOGGLE(PORTC,4);}while(0)

#define portc_5(func)       portc_5_##func
#define portc_5_set()       do{SET(PORTC,5);}while(0)
#define portc_5_clear()     do{CLR(PORTC,5);}while(0)
#define portc_5_write(V)    do{WRITE(PORTC,5,(V));}while(0)
#define portc_5_read()      (READ(PORTC,5))
#define portc_5_read_lat()  (READ_LAT(PORTC,5))
#define portc_5_toggle()    do{TOGGLE(PORTC,5);}while(0)

#define portc_6(func)       portc_6_##func
#define portc_6_set()       do{SET(PORTC,6);}while(0)
#define portc_6_clear()     do{CLR(PORTC,6);}while(0)
#define portc_6_write(V)    do{WRITE(PORTC,6,(V));}while(0)
#define portc_6_read()      (READ(PORTC,6))
#define portc_6_read_lat()  (READ_LAT(PORTC,6))
#define portc_6_toggle()    do{TOGGLE(PORTC,6);}while(0)

#define portc_7(func)       portc_7_##func
#define portc_7_set()       do{SET(PORTC,7);}while(0)
#define portc_7_clear()     do{CLR(PORTC,7);}while(0)
#define portc_7_write(V)    do{WRITE(PORTC,7,(V));}while(0)
#define portc_7_read()      (READ(PORTC,7))
#define portc_7_read_lat()  (READ_LAT(PORTC,7))
#define portc_7_toggle()    do{TOGGLE(PORTC,7);}while(0)

#define portc_9(func)       portc_9_##func
#define portc_9_set()       do{SET(PORTC,9);}while(0)
#define portc_9_clear()     do{CLR(PORTC,9);}while(0)
#define portc_9_write(V)    do{WRITE(PORTC,9,(V));}while(0)
#define portc_9_read()      (READ(PORTC,9))
#define portc_9_read_lat()  (READ_LAT(PORTC,9))
#define portc_9_toggle()    do{TOGGLE(PORTC,9);}while(0)

#define portc_10(func)      portc_10_##func
#define portc_10_set()      do{SET(PORTC,10);}while(0)
#define portc_10_clear()    do{CLR(PORTC,10);}while(0)
#define portc_10_write(V)   do{WRITE(PORTC,10,(V));}while(0)
#define portc_10_read()     (READ(PORTC,10))
#define portc_10_read_lat() (READ_LAT(PORTC,10))
#define portc_10_toggle()   do{TOGGLE(PORTC,10);}while(0)

#define portc_11(func)      portc_11_##func
#define portc_11_set()      do{SET(PORTC,11);}while(0)
#define portc_11_clear()    do{CLR(PORTC,11);}while(0)
#define portc_11_write(V)   do{WRITE(PORTC,11,(V));}while(0)
#define portc_11_read()     (READ(PORTC,11))
#define portc_11_read_lat() (READ_LAT(PORTC,11))
#define portc_11_toggle()   do{TOGGLE(PORTC,11);}while(0)

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
