#if 0
/*****************************************************************************
 **
 ** Test application
 ** September 2016
 **
 ****************************************************************************/
#include "defs.h"
#include "labwiz/labwiz.h"
#include "labwiz/drv_lcd.h"

#include "mcp3424.h"
#include "mcp9800.h"
#include "thermocouples.h"

// Definitions and types
// ----------------------------------------------------------------------------
#define STATE_INIT          0
#define STATE_OPERATING     1

#define PERIOD_MS           250

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
    char outstring[20];

    m_button_mask = 0;

    // Get button presses
    labwiz_set_btn_callback(_test_btn_press);

    // Thermocouple setup
    thrm_init();

    // Setup MCP9800 for ambient
    mcp9800_init(I2C_BUS_1,0);
    mcp9800_configure(MCP9800_12BIT);

    // Setup LCD screen
    lcd_blank();
    lcd_print("Pax Instruments TEST",0,0);
    sprintf(outstring,"Firmware: %s",FIRMWARE_VERSION);
    lcd_print(outstring,10,0);
    lcd_latch();


    return;
}

char m_scratch[30];
void loop()
{

    thrm_poll();

    switch(m_state){
    case STATE_INIT:
        vTaskDelay(portTICK_PERIOD_MS*1000);
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
            int x;
            m_last_tick = current_tick;
            led1(toggle());

            mcp9800_update();

            lcd_set_screen(&m_testscreen);

            sprintf(m_scratch,"%c",lcd_spinner());
            lcd_print(m_scratch,0,60);

            sprintf(m_scratch,"Testing");
            lcd_print(m_scratch,0,0);

#if 1
            for(x=0;x<4;x++)
            {
                int temp,uv;
                temp = thrm_get_temperature(x+1);
                uv = thrm_get_uvolts(x+1);
#if 0
                if(temp>THRM_OUT_OF_RANGE_NEG)
                    sprintf(m_scratch,"Ch %d:%d.%d C",x,temp/10,temp%10);
                else
                    sprintf(m_scratch,"Ch %d: N/A",x);
#else
                sprintf(m_scratch,"Ch %d:%d uV, %d",x,uv,temp);
#endif
                lcd_print(m_scratch,(x*10)+10,0);
            }
            {
                int temp=mcp9800_get_temperature();
                //int temp=0;
                thrm_set_ambient(temp);
                sprintf(m_scratch,"Ambient:%d.%d",temp/10,temp%10);
                lcd_print(m_scratch,49,0);
            }
#endif

            {
            	int cnt = drv_i2c1_busy_count();
            	sprintf(m_scratch,"Busy: %d",cnt);
				lcd_print(m_scratch,57,0);
            }

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

