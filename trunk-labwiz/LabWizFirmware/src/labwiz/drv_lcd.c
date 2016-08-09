/*****************************************************************************
 **
 ** LCD driver
 ** August 2016
 **
 ** This driver provides functions to write the LCD
 **
 ****************************************************************************/

#include "string.h"

#include "labwiz/defs.h"
#include "labwiz/drv_spi.h"
#include "labwiz/drv_lcd.h"
#include "labwiz/port.h"
#include "labwiz/font5x7.h"

// NOTE: The LCD driver is an ST7567
// RESET is active low
// A0 is data vs cmd

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// Definitions and externs
// ---------------------------------------------------------------------------
// The LCD is a 132x64 pixel graphical LCD.  If split into 8 bit 'pages' then
// it is 133 x 8pages
#define LCD_COLS            132
#define LCD_ROWS            64
#define LCD_ROWS_PER_PAGE   8
#define LCD_PAGES           (LCD_ROWS/LCD_ROWS_PER_PAGE)    // 8

// The data/cmd GPIO
#define pin_data_cmd(func)  portb_9(func)
//#define pin_data_cmd(func)  port_none(func)
#define m_set_data()        pin_data_cmd(set())
#define m_set_cmd()         pin_data_cmd(clear())

// Reset line
#define pin_reset(func)     portb_8(func)
//#define pin_reset(func)  port_none(func)
#define m_reset_on()        pin_reset(clear())
#define m_reset_off()       pin_reset(set())

//#define pin_cs(func)        porta_1(func)
#define pin_cs(func)        portb_7(func)
//#define pin_cs(func)        port_none(func)
#define m_cs_enable()       pin_cs(clear())
#define m_cs_disable()      pin_cs(set())

#define pin_bl(func)        portb_6(func)
#define m_bl_enable()       pin_bl(clear())
#define m_bl_disable()      pin_bl(set())

typedef enum{
    LCD_POWER = 0 ,
    LCD_SET_START_LINE,
    LCD_SET_PAGE,
    LCD_SET_COLUMN_MSB,
    LCD_SET_COLUMN_LSB,
    LCD_READ_STATUS,    //5
    LCD_WRITE_DATA,
    LCD_READ_DATA,
    LCD_SEG_DIRECTION,
    LCD_INVERT_DISPLAY,
    LCD_ALL_PIXEL_ON,   //10
    LCD_BIAS_SELECT,
    LCD_READMODIFYWRITE,
    LCD_END,
    LCD_RESET,
    LCD_COM_DIRECTION,  //15
    LCD_POWER_CONTROL,
    LCD_REGULATION_RATIO,
    LCD_NOP,            // 18
    LCD_CONTRAST_START,
    LCD_CONTRAST_VALUE,
    LCD_MAX
}e_lcd_commands;

uint8_t lcdcmds[LCD_MAX] = {
        0xA7, //LCD_POWER
        0x40, //LCD_SET_START_LINE
        0xB0, //LCD_SET_PAGE
        0x10, //LCD_SET_COLUMN_MSB
        0x00, //LCD_SET_COLUMN_LSB
        0x00, //LCD_READ_STATUS
        0x00, //LCD_WRITE_DATA
        0x00, //LCD_READ_DATA
        0xA0, //LCD_SEG_DIRECTION
        0xA6, //LCD_INVERT_DISPLAY
        0xA4, //LCD_ALL_PIXEL_ON
        0xA2, //LCD_BIAS_SELECT
        0xE0, //LCD_READMODIFYWRITE
        0xEE, //LCD_END
        0xE2, //LCD_RESET
        0xC0, //LCD_COM_DIRECTION
        0x28, //LCD_POWER_CONTROL
        0x20, //LCD_REGULATION_RATIO
        0xE3,  //LCD_NOP
        0x81, //LCD_CONTRAST_START
        0x00, //LCD_CONTRAST_VALUE
};

#define OP_DISPLAY_ON       (0x01)
#define OP_DISPLAY_OFF      (0x00)
#define OP_START_LINE(L)    ((L)&0x1F)
#define OP_PAGE(P)          ((P)&0xF)
#define OP_COL_MSB(C)       ((C)>>4)
#define OP_COL_LSB(C)       ((C)&0xF)
#define OP_SEG_REVERSE      (0x01)
#define OP_SEG_NORMAL       (0x00)
#define OP_INVERT           (0x01)
#define OP_NORMAL           (0x00)
#define OP_ALL_PIX_ON       (0x01)
#define OP_ALL_PIX_OFF      (0x00)
#define OP_BIAS_1_9         (0x01)
#define OP_BIAS_1_7         (0x00)
#define OP_COM_DIR_REV      (0x01)
#define OP_COM_DIR_NORM     (0x00)
#define OP_REG_RATIO(R)     ((R)&0x7)
#define OP_CONTRAST(C)      ( (C)&0x3F)
#define OP_CONTRAST_PCT(C)  ( (((C)*64)/100))&0x3F)
#define OP_VB_ON            (0x04)
#define OP_VR_ON            (0x02)
#define OP_VF_ON            (0x01)

// Local module variables
// ---------------------------------------------------------------------------


#ifndef LCD_DOUBLE_BUFFER
uint8_t m_lcd_buffer[LCD_PAGES][LCD_COLS];
#else
#define WRITE_BUFFER        0
#define SEND_BUFFER         1
uint8_t m_lcd_buffer[2][LCD_PAGES][LCD_COLS];
#endif


// Private prototypes
// ---------------------------------------------------------------------------
void _lcd_setup(void);
void _lcd_draw(void);
void _lcdDEBUG_test_pattern1(void);

// Public functions
// ---------------------------------------------------------------------------

void lcd_init()
{
    m_cs_disable();
    m_reset_on();
    HAL_Delay(1);  /* delay 1 ms */
    m_set_cmd();

    lcd_blank();

    _lcd_setup();

    m_bl_enable();


    // DEBUG tests
    #if 1
    lcd_set_pixel(0,0);
    lcd_set_pixel(0,131);
    lcd_set_pixel(63,131);
    lcd_set_pixel(63,0);
    lcd_set_pixel(32,66);
    #endif

    #if 1
    lcd_line(32, 0, 32, 131);
    lcd_line(0, 66, 63, 66);
    #endif

    _lcd_draw();

    nop();

    return;
}


void lcd_task( void *pvParameters )
{
    volatile UBaseType_t lcd_max_stack_depth;
    // This is a task to send data to the LCD, we block
    // on a delay and then send all data at once
    nop();

    for(;;)
    {
        // Get our max stack depth in words
        lcd_max_stack_depth = uxTaskGetStackHighWaterMark( NULL);

        // DEBUG, run every XXXms (slow for testing)
        vTaskDelay(portTICK_PERIOD_MS*1000);
        //pin_bl(toggle());
        #ifndef LCD_DOUBLE_BUFFER
        _lcd_draw();
        #else
        memcpy(send buffer, draw buffer, size);
        //lcd_draw(send_buffer);
        #endif

    }

    /* Should the task implementation ever break out of the above loop
    then the task must be deleted before reaching the end of this function.
    The NULL parameter passed to the vTaskDelete() function indicates that
    the task to be deleted is the calling (this) task. */
    vTaskDelete( NULL );

    return;
}


void lcd_blank()
{
    memset(m_lcd_buffer,0,sizeof(m_lcd_buffer));
    return;
}

// Disable conversion warnings, we are doing all sorts of bit operations
// and the compiler complains about everything.
#pragma GCC diagnostic ignored "-Wconversion"

void lcd_set_pixel(uint8_t row, uint8_t col)
{
    volatile uint8_t i,p;
    //p = (row/LCD_ROWS_PER_PAGE);    // p is the page
    p = (row>>3); // div by 8
    i = (uint8_t)(row-(p*8));       // i is the index in the byte

    #ifndef LCD_DOUBLE_BUFFER
    m_lcd_buffer[p][col] = (uint8_t)(m_lcd_buffer[p][col]|(1<<(7-i)));
    #else
    nop();
    #endif

    return;
}
void lcd_clear_pixel(uint8_t row, uint8_t col)
{
    volatile uint8_t i,p;
    //p = (row/LCD_ROWS_PER_PAGE);
    p = (row>>3);
    i = (uint8_t)(row-(p*8));
    // i is the page index, p is the row in the page
    #ifndef LCD_DOUBLE_BUFFER
    m_lcd_buffer[i][col] = (uint8_t)(m_lcd_buffer[p][col]&(~(1<<i)));
    #else
    nop();
    #endif
    return;
}

// This is kinda ugly, is there a better way to write text across pages?
// TODO: Improve
void lcd_print(char * st,uint8_t row,uint8_t col)
{
    char c;
    uint8_t * ptr;
    uint8_t page,bits,x,data,pos,cnt,src;
    if(row>LCD_ROWS || col>130) return;
    row+=8;
    while(*st)
    {
        c = *st;
        pos = col;
        ptr=&(Font5x7[(c-' ')*5]);

        // Draw X bits in this row
        page = (row>>3);
        bits = row%8;
        if(bits>0 && page<(LCD_PAGES-1))
        {
            for(x=0;x<5;x++)
            {
                data = *ptr;
                data = data>>(8-bits);
                // Flip font vvvvvvvvvvv
                src = 0;
                for(cnt=0;cnt<8;cnt++){ src<<=1; src|=(data&1); data>>=1;}
                // Flip font ^^^^^^^^^^^
                m_lcd_buffer[page][pos++] |= src; // or it? or overwrite with just =  ?
                ptr++;
            }
        }
        // If we displayed less than the full 8 bits, then we have 8-bits left
        // to draw on the next page
        if(bits<8 && page>0)
        {
            // 8-bits remain in the lower page
            bits = 8-bits;
            pos = col;
            page--;
            ptr=&(Font5x7[(c-' ')*5]);
            for(x=0;x<5;x++)
            {
                data = *ptr;
                data = data<<(8-bits);
                // Flip font vvvvvvvvvvv
                src = 0;
                for(cnt=0;cnt<8;cnt++){ src<<=1; src|=(data&1); data>>=1;}
                // Flip font ^^^^^^^^^^^
                m_lcd_buffer[page][pos++] |= src; // or it? or overwrite with just =  ?
                ptr++;
            }
        }

        st++;
        col+=6;
    };
    return;
}

// Found here: http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C
void lcd_line(uint8_t r0, uint8_t c0, uint8_t r1, uint8_t c1)
{

  int16_t dr = abs(r1-r0), sr = r0<r1 ? 1 : -1;
  int16_t dc = abs(c1-c0), sc = c0<c1 ? 1 : -1;
  int16_t err = (dr>dc ? dr : -dc)>>1, e2;

  for(;;)
  {
    lcd_set_pixel(r0,c0);
    if (r0==r1 && c0==c1) break;
    e2 = err;
    if (e2 >-dr) { err -= dc; r0 += sr; }
    if (e2 < dc) { err += dr; c0 += sc; }
  }
  return;
}

// Re-enable the warnings
#pragma GCC diagnostic warning "-Wconversion"


void lcd_backlight_toggle()
{
    pin_bl(toggle());
}
void lcd_backlight_enable(bool enable)
{
    if(enable)
        pin_bl(set());
    else
        pin_bl(clear());
}
// Private functions
// ---------------------------------------------------------------------------


// All these functions are still for debug/testing and need work

void _lcd_setup()
{
    uint8_t commands[15];

    /*
    This is the LCD controller strtup sequence
    From the datasheet:
    (11) Bias Select
    (8) SEG Direction
    (15) COM Direction
    (17) Regulation Ratio
    (18) Set EV
    (16) Power Control
    */
    m_reset_off();
    HAL_Delay(1);  /* delay 1 ms */

    m_cs_enable();

    // Set 1
    commands[0] = (lcdcmds[LCD_BIAS_SELECT]|OP_BIAS_1_9);
    commands[1] = (lcdcmds[LCD_SEG_DIRECTION]|OP_SEG_NORMAL);
    commands[2] = (lcdcmds[LCD_COM_DIRECTION]|OP_COM_DIR_NORM);
    commands[3] = (lcdcmds[LCD_SET_START_LINE]|OP_START_LINE(0));
    drv_spi3_tx(commands,4);

    // Set 2
    HAL_Delay(50);  /* delay 50 ms */
    commands[0] = (lcdcmds[LCD_POWER_CONTROL]|OP_VB_ON|OP_VR_ON|OP_VF_ON);
    drv_spi3_tx(commands,1);

    // Set 3
    HAL_Delay(50);  /* delay 50 ms */
    commands[0] = (lcdcmds[LCD_REGULATION_RATIO]|OP_REG_RATIO(6));  // 6 based on u8glib
    commands[1] = (lcdcmds[LCD_INVERT_DISPLAY]|OP_NORMAL);  // NOT inverted
    commands[2] = (lcdcmds[LCD_CONTRAST_START]);
    commands[3] = (lcdcmds[LCD_CONTRAST_VALUE]|OP_CONTRAST(0x28)); // Contrast value
    //commands[4] = (lcdcmds[LCD_POWER]|OP_DISPLAY_ON);
    commands[4] = 0xAF; // force on
    // Now send these commands to LCD
    drv_spi3_tx(commands,5);

    m_cs_disable();

    return;
}


void _lcd_draw()
{
    /* Commands to write display
     * From datasheet:
     * (2) Display Start Line Set
     * (3) Page Address Set
     * (4) Column Address Set
     * (6) Display Data Write
     * (1) Display ON/OFF <-- optional????
     */
    uint8_t page;
    uint8_t commands[5];

    // Write all the pages
    for(page=0;page<LCD_PAGES;page++)
    {
        m_set_cmd();
        commands[0] = (uint8_t)(lcdcmds[LCD_SET_START_LINE]|OP_START_LINE(0));
        commands[1] = (uint8_t)(lcdcmds[LCD_SET_PAGE]|OP_PAGE((LCD_PAGES-1)-page));
        commands[2] = (uint8_t)(lcdcmds[LCD_SET_COLUMN_MSB]|OP_COL_MSB(0));
        commands[3] = (uint8_t)(lcdcmds[LCD_SET_COLUMN_LSB]|OP_COL_LSB(0));
        // Now send these commands to LCD
        m_cs_enable();
        drv_spi3_tx(commands,4);

        m_set_data();
        // Now send data to LCD
        drv_spi3_tx(&(m_lcd_buffer[page][0]), LCD_COLS);
        m_cs_disable();

    }

    return;
}

// DEBUG functions
void _lcdDEBUG_test_pattern1()
{
    uint8_t row,col;
    lcd_blank();
    for(row=0;row<LCD_ROWS;row=(uint8_t)(row+1))
    {
        nop();
        // every other pixel
        for(col=(row&1);col<LCD_COLS;col=(uint8_t)(col+2))
        {
            lcd_set_pixel(row,col);
        }
    }
    return;
}

// eof
