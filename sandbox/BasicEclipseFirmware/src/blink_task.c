#include "stm32f1xx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f1xx_hal.h"

#include "blink_task.h"

#ifndef nop
#define nop()   asm("nop")
#endif

// Port struct pointers to registers
GPIO_TypeDef * gPortaPtr = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(0));
GPIO_TypeDef * gPortbPtr = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(1));
GPIO_TypeDef * gPortcPtr = (GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(2));

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
#define porta_0_set()       do{SET(gPortaPtr,0);}while(0)
#define porta_0_clear()     do{CLR(gPortaPtr,0);}while(0)
#define porta_0_write(V)    do{WRITE(gPortaPtr,0,(V));}while(0)
#define porta_0_read()      (READ(gPortaPtr,0))
#define porta_0_read_lat()  (READ_LAT(gPortaPtr,0))
#define porta_0_toggle()    do{TOGGLE(gPortaPtr,0);}while(0)

#define porta_1(func)       porta_1_##func
#define porta_1_set()       do{SET(gPortaPtr,1);}while(0)
#define porta_1_clear()     do{CLR(gPortaPtr,1);}while(0)
#define porta_1_write(V)    do{WRITE(gPortaPtr,1,(V));}while(0)
#define porta_1_read()      (READ(gPortaPtr,1))
#define porta_1_read_lat()  (READ_LAT(gPortaPtr,1))
#define porta_1_toggle()    do{TOGGLE(gPortaPtr,1);}while(0)

// PortB
#define portb_13(func)      portb_13_##func
#define portb_13_set()      do{SET(PportbPtr,13);}while(0)
#define portb_13_clear()    do{CLR(gPortbPtr,13);}while(0)
#define portb_13_write(V)   do{WRITE(gPortbPtr,13,(V));}while(0)
#define portb_13_read()     (READ(gPortbPtr,13))
#define portb_13_read_lat() (READ_LAT(gPortbPtr,13))
#define portb_13_toggle()   do{TOGGLE(gPortbPtr,13);}while(0)

// PortC
#define portc_13(func)      portc_13_##func
#define portc_13_set()      do{SET(gPortcPtr,13);}while(0)
#define portc_13_clear()    do{CLR(gPortcPtr,13);}while(0)
#define portc_13_write(V)   do{WRITE(gPortcPtr,13,(V));}while(0)
#define portc_13_read()     (READ(gPortcPtr,13))
#define portc_13_read_lat() (READ_LAT(gPortcPtr,13))
#define portc_13_toggle()   do{TOGGLE(gPortcPtr,13);}while(0)

#define HIGH  (1)
#define LOW   (0)


#define led1(func)          portc_13(func)
#define LED_ON  HIGH
#define LED_OFF LOW

void blink_init()
{
    led1(write(LED_ON));
    return;
}

void ATaskFunction( void *pvParameters )
{
    //int iVariableExample = 0;

    for( ;; )
    {
        // Do all the things
        vTaskDelay(portTICK_PERIOD_MS*100);
        led1(toggle());
    }

    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );
    return;
}
