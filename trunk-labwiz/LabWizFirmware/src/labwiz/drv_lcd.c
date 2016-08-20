/*****************************************************************************
 **
 ** LCD driver
 ** August 2016
 **
 ** This driver provides functions to write the LCD
 **
 ****************************************************************************/
#include "stdlib.h"
#include "string.h"

#include "labwiz/labwizdefs.h"
#include "labwiz/drv_spi.h"
#include "labwiz/drv_lcd.h"
#include "labwiz/font5x7.h"

// NOTE: The LCD driver is an ST7567
// RESET is active low
// A0 is data vs cmd

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// Definitions and externs
// ---------------------------------------------------------------------------

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

// Double buffering
lcd_screen_t m_lcd_screen_A;
lcd_screen_t m_lcd_screen_B;
lcd_screen_t * m_lcd_write_ptr = &m_lcd_screen_A;
lcd_screen_t * m_lcd_send_ptr = &m_lcd_screen_B;

bool m_swap_request = false;


// Private prototypes
// ---------------------------------------------------------------------------
void _lcd_setup(void);
void _lcd_draw(void);
//void _lcdDEBUG_test_pattern1(void);
//void lcd_print_test();

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
    #if 0
    lcd_set_pixel(0,0);
    lcd_set_pixel(0,131);
    lcd_set_pixel(63,131);
    lcd_set_pixel(63,0);
    lcd_set_pixel(32,66);
    #endif

    #if 0
    lcd_line(32, 0, 32, 131);
    lcd_line(0, 66, 63, 66);
    #endif

    #if 0
    lcd_print("!\"#$%&'()*+,-./0123456",0,0);
    lcd_print("789:;<=>?@ABCDEFGHIJKL",9,0);
    lcd_print("MNOPQRSTUVWXYZ[\]^_`ab",18,0);
    lcd_print("cdefghijklmnopqrstuvwx",27,0);
    lcd_print("yz{|}~~",37,0);
    #endif

    //_lcd_draw();

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
        vTaskDelay(portTICK_PERIOD_MS*100);
        //pin_bl(toggle());
        _lcd_draw();

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
    memset(&(m_lcd_write_ptr->data),0,sizeof(lcd_screen_t));
    return;
}

void lcd_latch(void)
{
    m_swap_request = true;
    return;
}

void lcd_set_pixel(int row, int col)
{
    volatile uint8_t i,p;
    //p = (row/LCD_ROWS_PER_PAGE);    // p is the page
    p = (uint8_t)(row>>3); // div by 8
    i = (uint8_t)(row-(p*8));       // i is the index in the byte

    m_lcd_write_ptr->data[p][col] = (uint8_t)(m_lcd_write_ptr->data[p][col]|(1<<(7-i)));

    return;
}
void lcd_clear_pixel(int row, int col)
{
    volatile uint8_t i,p;
    //p = (row/LCD_ROWS_PER_PAGE);
    p = (uint8_t)(row>>3);
    i = (uint8_t)(row-(p*8));
    // i is the page index, p is the row in the page

    m_lcd_write_ptr->data[i][col] = (uint8_t)(m_lcd_write_ptr->data[p][col]&(~(1<<i)));

    return;
}

// This is kinda ugly, is there a better way to write text across pages?
// TODO: Improve
#if 1
#define CHAR_WIDTH  5
#define CHAR_HEIGHT 8
#define FONT        Font5x7
#else
#define CHAR_WIDTH  4
#define CHAR_HEIGHT 7
#define FONT        fonttest
#endif
void lcd_print(char * st,int row,int col)
{
    char c;
    uint8_t * ptr;
    uint8_t page,bits,x,data,pos,cnt,src;
    if(row>LCD_ROWS || col>130) return;
    row+=CHAR_HEIGHT;
    while(*st)
    {
        c = *st;
        pos = (uint8_t)col;
        ptr=&(FONT[(c-' ')*CHAR_WIDTH]);

        // Draw X bits in this row
        page = (uint8_t)(row>>3);
        bits = (uint8_t)(row%8); // We can write this many bits in the current row
        if(bits>0 && page<(LCD_PAGES))
        {
            for(x=0;x<CHAR_WIDTH;x++)
            {
                data = *ptr;
                data = data>>(CHAR_HEIGHT-bits);
                // Flip font vvvvvvvvvvv
                src = 0;
                for(cnt=0;cnt<8;cnt++){ src<<=1; src|=(data&1); data>>=1;}
                // Flip font ^^^^^^^^^^^
                m_lcd_write_ptr->data[page][pos++] |= src; // or it? or overwrite with just =  ?
                ptr++;
            }
        }
        // If we displayed less than the full HEIGHT bits, then we have 8-bits left
        // to draw on the next page
        if(bits<8 && page>0)
        {
            // HEIGHT-bits remain in the lower page
            bits = CHAR_HEIGHT-bits; // We have this many bits to go
            pos = col;
            page--;
            ptr=&(FONT[(c-' ')*CHAR_WIDTH]);
            for(x=0;x<CHAR_WIDTH;x++)
            {
                data = *ptr;
                data = data<<(CHAR_HEIGHT-bits);
                // Flip font vvvvvvvvvvv
                src = 0;
                for(cnt=0;cnt<8;cnt++){ src<<=1; src|=(data&1); data>>=1;}
                // Flip font ^^^^^^^^^^^
                m_lcd_write_ptr->data[page][pos++] |= src; // or it? or overwrite with just =  ?
                ptr++;
            }
        }
        st++;
        col+=6;
    };
    return;
}

// Found here: http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C
void lcd_line(int r0, int c0, int r1, int c1)
{

  int16_t dr = (int16_t)abs(r1-r0), sr = r0<r1 ? 1 : -1;
  int16_t dc = (int16_t)abs(c1-c0), sc = c0<c1 ? 1 : -1;
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

void lcd_get_screen(lcd_screen_t * screen)
{
    if(screen==NULL) return;
    memcpy(screen,m_lcd_write_ptr,sizeof(lcd_screen_t));
    return;
}
void lcd_set_screen(lcd_screen_t * screen)
{
    if(screen==NULL) return;
    memcpy(m_lcd_write_ptr,screen,sizeof(lcd_screen_t));
    return;
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
        drv_spi3_tx( &(m_lcd_send_ptr->data[page][0]) , LCD_COLS);
        m_cs_disable();

    }

    // This is double buffering, after we write a full screen, if we have a new
    // screen, switch the buffer pointers
    if(m_swap_request)
    {
        lcd_screen_t * tmpsptr;
        tmpsptr = m_lcd_write_ptr;
        m_lcd_write_ptr = m_lcd_send_ptr;
        m_lcd_send_ptr = tmpsptr;
        m_swap_request=false;
    }

    return;
}

#if 0
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
void lcd_print_test()
{
    uint8_t * ptr;
    ptr = &(m_lcd_write_ptr->data[0][0]);
    memcpy(ptr,fonttest,30);
    return;
}
#endif
// eof
