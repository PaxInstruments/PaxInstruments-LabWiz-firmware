#if 1
/*****************************************************************************
 **
 ** Test application
 ** September 2016
 **
 ****************************************************************************/
#include "defs.h"
#include "labwiz/labwiz.h"
#include "labwiz/drv_lcd.h"

#include "labwiz/drv_i2c.h"

// Definitions and types
// ----------------------------------------------------------------------------
#define STATE_INIT          0
#define STATE_OPERATING     1

#define DEV_ADDR            0x68
// ID Values: 0-3
#define DEV_ID              0

#define PERIOD_MS           1000

#define led1    port_none

// Local variables
// ----------------------------------------------------------------------------
lcd_screen_t m_testscreen;

uint32_t m_button_mask=0;

static int m_state;

TickType_t m_last_tick;

// Local prototypes
// ----------------------------------------------------------------------------
void _test_btn_press(uint8_t button);

// Public functions
// ----------------------------------------------------------------------------

void setup()
{

    m_button_mask = 0;

    labwiz_set_btn_callback(_test_btn_press);

    lcd_blank();


    lcd_blank();
    lcd_print("Pax Instruments TEST",30,2);

    lcd_latch();
    return;
}

char m_scratch[30];
void loop()
{
    uint16_t addr;
    uint8_t data[20];
    uint16_t size;

    switch(m_state){
    case STATE_INIT:
        vTaskDelay(portTICK_PERIOD_MS*2000);
        m_state = STATE_OPERATING;
        break;
    case STATE_OPERATING:
    {
        // This is the main loop
        // -------------------------------------------------------------------
        TickType_t current_tick = xTaskGetTickCount();

        // Do things every PERIOD_MS
        if(current_tick >= (m_last_tick+(portTICK_PERIOD_MS*PERIOD_MS)))
        {
            m_last_tick = current_tick;
            led1(toggle());

            lcd_set_screen(&m_testscreen);

            sprintf(&(m_scratch),"Testing");
            lcd_print(m_scratch,0,0);


            addr = DEV_ADDR | DEV_ID;
            sprintf(data,"Test");
            size = 4;
            drv_i2c1_write(addr, data, size);
            drv_i2c1_read(addr, data, size);


            lcd_latch();
        }
    }
    break;
    default: break;
    }
    return;
}

// Private functions
// ----------------------------------------------------------------------------
void _test_btn_press(uint8_t button)
{
    // When called as a callback, the function is running in the labwiz
    // task, not the loop() task
    m_button_mask |= SW_MASK(button);
    return;
}

// eof
#endif

