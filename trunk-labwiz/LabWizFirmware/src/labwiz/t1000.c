/*****************************************************************************
 **
 ** t1000 related functions.
 ** August 2016
 **
 ****************************************************************************/

#include "labwiz/defs.h"
#include "labwiz/t1000.h"

#include "labwiz/labwiz.h"
#include "labwiz/drv_lcd.h"
#include "labwiz/drv_filesystem.h"

// Definitions and types
// ----------------------------------------------------------------------------

// Local variables
// ----------------------------------------------------------------------------
int m_button_mask = 0;

// Local prototypes
// ----------------------------------------------------------------------------
void _t1000_btn_press(uint8_t button);

lcd_screen_t lcd_screen1;

// Public functions
// ----------------------------------------------------------------------------

void setup()
{
    int8_t x;
    lcd_blank();
    // Top Row section
    lcd_line(8,0, 8, MAX_COL);
    for(x=0;x<4;x++)
        lcd_line(0, (x*33),7,(x*33));
    lcd_line(0, 131,7,131);
    // Status bar
    lcd_line(18, 0,18,131);
    // Side bar
    lcd_line(19, 18,MAX_ROW,18);
    // Tick marks
    for(x=0;x<5;x++)
        lcd_set_pixel((x*9)+19,17);

    // Test
    lcd_print("123.4",0,(0*33)+2);
    lcd_print("123.4",0,(1*33)+2);
    lcd_print("123.4",0,(2*33)+2);
    lcd_print("123.4",0,(3*33)+2);
    lcd_print("TypK-Not Logging-1s-E",10,0);


    lcd_get_screen(&lcd_screen1);

    lcd_blank();
    labwiz_set_btn_callback(_t1000_btn_press);

    return;
}


void loop()
{
    // Do things with buttons
    if(m_button_mask!=0)
    {
        if(m_button_mask&SW_MASK(SW_A))
        {
            lcd_set_screen(&lcd_screen1);
            m_button_mask&=~SW_MASK(SW_A);
        }
        if(m_button_mask&SW_MASK(SW_B))
        {
            m_button_mask&=~SW_MASK(SW_B);
        }
        if(m_button_mask&SW_MASK(SW_C))
        {
            m_button_mask&=~SW_MASK(SW_C);
        }
        if(m_button_mask&SW_MASK(SW_D))
        {
            m_button_mask&=~SW_MASK(SW_D);
        }
        if(m_button_mask&SW_MASK(SW_E))
        {
            lcd_backlight_toggle();
            m_button_mask&=~SW_MASK(SW_E);
        }
    }
    return;
}

// Private functions
// ----------------------------------------------------------------------------
void _t1000_btn_press(uint8_t button)
{
    // When called as a callback, the function is running in the labwiz
    // task, not the loop() task
    m_button_mask |= SW_MASK(button);
    // TODO: mutex?
    return;
}

// eof
