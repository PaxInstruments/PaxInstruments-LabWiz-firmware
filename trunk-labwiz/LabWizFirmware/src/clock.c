#if 1
/*****************************************************************************
 **
 ** Clock application
 ** August 2016
 **
 ****************************************************************************/
#include "defs.h"
#include "labwiz/labwiz.h"
#include "labwiz/drv_lcd.h"

#include "sin_lut.h"

// Definitions and types
// ----------------------------------------------------------------------------
#define STATE_INIT          0
#define STATE_OPERATING     1

#define CLOCK_CENTER_ROW    32
#define CLOCK_CENTER_COL    64

// Local variables
// ----------------------------------------------------------------------------
lcd_screen_t m_clockscreen;

uint32_t m_button_mask=0;

int m_state;

TickType_t m_last_tick;

// Local prototypes
// ----------------------------------------------------------------------------
void _clock_btn_press(uint8_t button);
void _clock_pos(int row, int col, int radius,int deg, int * newrow, int * newcol);
void _clock_circle(int row, int col, int radius);
int _clock_sin(int deg);
int _clock_cos(int deg);

// Public functions
// ----------------------------------------------------------------------------

void setup()
{

    m_state=STATE_OPERATING;
    m_button_mask = 0;

    labwiz_set_btn_callback(_clock_btn_press);

    lcd_blank();
    int d,n;
    int newrow,newcol;
    char buf[5];
    _clock_circle(CLOCK_CENTER_ROW,CLOCK_CENTER_COL,31);
    for(d=30,n=1;d<=360;d+=30,n++)
    {
        _clock_pos(CLOCK_CENTER_ROW+1,CLOCK_CENTER_COL,25 ,d, &newrow, &newcol);
        sprintf(buf,"%d",n);
        if(d==180) newcol-=2;
        if(d>180) newcol-=5;
        newrow-=5;
        lcd_print(buf,newrow,newcol);
    }
    lcd_set_pixel(32,64);
    lcd_get_screen(&m_clockscreen);



    lcd_blank();
    lcd_print("Pax Instruments CLOCK!",30,2);

    lcd_latch();
    return;
}

char m_scratch[30];
void loop()
{
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

        // Do things every PERIODIC_PERIOD_MS
        if(current_tick >= (m_last_tick+(portTICK_PERIOD_MS*1000)))
        {
            int d;
            int newrow,newcol;
            labwiz_time_t tm;

            m_last_tick = current_tick;
           led1(toggle());

            labwiz_get_time(&tm);

            lcd_set_screen(&m_clockscreen);

            // Draw hour
            d = (tm.Hours%12)*30;
            _clock_pos(CLOCK_CENTER_ROW,CLOCK_CENTER_COL,10 ,d, &newrow, &newcol);
            lcd_line(CLOCK_CENTER_ROW,CLOCK_CENTER_COL,newrow,newcol);
            // Draw minute
            //d = (((tm.minutes*100)/60)*360)/100;
            d = (tm.Minutes*36000)/6000;
            _clock_pos(CLOCK_CENTER_ROW,CLOCK_CENTER_COL,15 ,d, &newrow, &newcol);
            lcd_line(CLOCK_CENTER_ROW,CLOCK_CENTER_COL,newrow,newcol);
            // Draw second
            d = (tm.Seconds*36000)/6000;
            _clock_pos(CLOCK_CENTER_ROW,CLOCK_CENTER_COL,20 ,d, &newrow, &newcol);
            lcd_line(CLOCK_CENTER_ROW,CLOCK_CENTER_COL,newrow,newcol);

            sprintf(&(m_scratch),"%d-%d-%d",tm.Month,tm.Day,tm.Year+2000);
            lcd_print(m_scratch,0,0);
            sprintf(&(m_scratch),"%02d:%02d:%02d",tm.Hours,tm.Minutes,tm.Seconds);
            lcd_print(m_scratch,0,82);

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
void _clock_btn_press(uint8_t button)
{
    // When called as a callback, the function is running in the labwiz
    // task, not the loop() task
    m_button_mask |= SW_MASK(button);
    return;
}
void _clock_pos(int row, int col, int radius,int deg, int * newrow, int * newcol)
{
    int s,c;
    int row_offset,col_offset;
    s = _clock_sin(deg);
    c = _clock_cos(deg);
    row_offset = ((radius*c)/1000);
    col_offset = ((radius*s)/1000);
    *newrow = row-row_offset;
    *newcol = col+col_offset;
    return;
}
void _clock_circle(int row, int col, int radius)
{
    int d;
    int newrow,newcol;
    for(d=1;d<=360;d++)
    {
      _clock_pos(row, col, radius,d, &newrow,&newcol);
      lcd_set_pixel(newrow,newcol);
    }
    return;
}

int _clock_sin(int deg)
{
    int tmp,result=1;
    tmp = deg%180;
    if(tmp<=0) tmp=179;
    if(tmp>=180) tmp=179;
    if(tmp<90) result= sin_deg(tmp);
    else if(tmp<180) result= sin_deg(90-(tmp-90));
    if(deg>180) result*=-1;
    return result;
}
int _clock_cos(int deg)
{
    int tmp,result=1;
    tmp = deg%180;
    if(tmp<=0) tmp=179;
    if(tmp<=90) result= sin_deg((90-tmp)+1);
    else if(tmp<=180) result= sin_deg((tmp-90));
    if(deg>=90 && deg<=270) result*=-1;
    return result;
}

// eof
#endif

