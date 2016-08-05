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
    LCD_POWER_SAVE,
    LCD_NOP,            // 19
    LCD_MAX
}e_lcd_commands;

uint8_t lcdcmds[LCD_MAX] = {0xA7,0x40,0xB0,0x01,0x00,0x00,
                            0x00,0x00,0xA0,0xA3,0xA2,
                            0xA2,0xE0,0xEE,0xE2,0xC0,
                            0x28,0x20,0x81,0xE3};

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
void _lcd_set_pixel(uint8_t row, uint8_t col);
void _lcd_clear_pixel(uint8_t row, uint8_t col);
void _lcdDEBUG_test_pattern1(void);

// Public functions
// ---------------------------------------------------------------------------

void drv_lcd_init()
{
    m_reset_on();
    HAL_Delay(1);  /* delay 1 ms */
    m_bl_enable();
    m_cs_disable();
    m_set_cmd();

    _lcd_setup();

    return;
}


void drv_lcd_task( void *pvParameters )
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
        //_lcd_draw();
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


void drv_lcd_blank()
{
    memset(m_lcd_buffer,0,sizeof(m_lcd_buffer));
    return;
}

// Private functions
// ---------------------------------------------------------------------------


// All these functions are still for debug/testing and need work

void _lcd_setup()
{
    uint8_t commands[15];
#if 0
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
    m_set_cmd();
    commands[0] = (lcdcmds[LCD_BIAS_SELECT]|OP_BIAS_1_9);
    commands[1] = (lcdcmds[LCD_SEG_DIRECTION]|OP_SEG_NORMAL);
    commands[2] = (lcdcmds[LCD_COM_DIRECTION]|OP_COM_DIR_NORM);
    commands[3] = (lcdcmds[LCD_REGULATION_RATIO]|OP_REG_RATIO(7));  // NEEDS TEST!!!
    commands[4] = 0x81; // Magic! Look at datasheet for EV setting
    commands[5] = 0x00; // Electronic volume??? Look at datasheet for EV setting
    commands[6] = (lcdcmds[LCD_POWER_CONTROL]|OP_VB_ON|OP_VR_ON|OP_VF_ON);

    // Release from reset
    m_reset_off();

    // Now send these commands to LCD
    m_cs_enable();
    drv_spi3_tx(commands,7);
    m_cs_disable();

    // Then get the LCD into a known state
    commands[0] = (lcdcmds[LCD_POWER]|OP_DISPLAY_ON);
    // Now send these commands to LCD
    m_cs_enable();
    drv_spi3_tx(commands,1);
    m_cs_disable();
#else
    m_cs_enable();
    commands[0] = 0xa3; /* 0x0a3: LCD bias 1/9 (suggested for the pi13264) */
    commands[1] = 0xa1;            /* 0x0a1: ADC set to reverse (suggested for the pi13264) */
    commands[2] = 0xc0;            /* common output mode: set scan direction normal operation/SHL Select, 0x0c0 --> SHL = 0, normal, 0x0c8 --> SHL = 1 */
    commands[3] = 0x40;            /* set display start line */

    commands[4] = 0x28 | 0x04;     /* power control: turn on voltage converter */
    drv_spi3_tx(commands,5);
    HAL_Delay(50);  /* delay 50 ms */


     commands[0] = 0x28 | 0x06;     /* power control: turn on voltage regulator */
     drv_spi3_tx(commands,1);
     HAL_Delay(50);  /* delay 50 ms */

     commands[0] = 0x28 | 0x07;     /* power control: turn on voltage follower */
     drv_spi3_tx(commands,1);
     HAL_Delay(50);  /* delay 50 ms */

     commands[0] = 0x26;            /* set V0 voltage resistor ratio to 6 */

     commands[1] = 0xa6;            /* display normal, bit val 0: LCD pixel off. */

     commands[2] = 0xC0;            /* set contrast */
     commands[3] = 0x18;            /* contrast value*/

     /*0x0ac,*/        /* indicator */
     /*0x000,*/        /* disable */

     commands[4] = 0xaf;            /* display on */
     drv_spi3_tx(commands,5);
     HAL_Delay(100);  /* delay 100 ms */

     #if 0

     commands[0] = 0xa5;            /* display all points, ST7565 */
     drv_spi3_tx(commands,1);
     HAL_Delay(200);  /* delay 200 ms */

     commands[0] = 0xa4;            /* normal display */
     drv_spi3_tx(commands,1);
    #endif

     m_cs_disable();
#endif

    m_bl_enable();

    // DEBUG
    _lcdDEBUG_test_pattern1();
    _lcd_draw();

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
#if 0
    uint8_t page;
    uint8_t commands[5];

    // Write all the pages
    for(page=0;page<LCD_PAGES;page++)
    {
        m_set_cmd();
        commands[0] = (uint8_t)(lcdcmds[LCD_SET_START_LINE]|OP_START_LINE(0));
        commands[1] = (uint8_t)(lcdcmds[LCD_SET_PAGE]|OP_PAGE(page));
        commands[2] = (uint8_t)(lcdcmds[LCD_SET_COLUMN_MSB]|OP_COL_MSB(0));
        commands[3] = (uint8_t)(lcdcmds[LCD_SET_COLUMN_LSB]|OP_COL_LSB(0));
        // Now send these commands to LCD
        m_cs_enable();
        drv_spi3_tx(commands,4);

        m_set_data();
        // Now send data to LCD
#if 0
        //drv_spi3_tx(&(m_lcd_buffer[page][0]), LCD_COLS);
#else
        m_lcd_buffer[page][0] = 0x55;
        m_lcd_buffer[page][1] = 0xAA;
        m_lcd_buffer[page][2] = 0x55;
        m_lcd_buffer[page][3] = 0xAA;
        drv_spi3_tx(&(m_lcd_buffer[page][0]), 4);
#endif
        m_cs_disable();

    }
#else
    uint8_t commands[10];
    uint8_t x;
    // Now send these commands to LCD
    m_set_cmd();
    commands[0] = (uint8_t)(lcdcmds[LCD_SET_START_LINE]|OP_START_LINE(0));
    commands[1] = (uint8_t)(lcdcmds[LCD_SET_PAGE]|OP_PAGE(1));
    commands[2] = (uint8_t)(lcdcmds[LCD_SET_COLUMN_MSB]|OP_COL_MSB(0));
    commands[3] = (uint8_t)(lcdcmds[LCD_SET_COLUMN_LSB]|OP_COL_LSB(0));
    m_cs_enable();
    drv_spi3_tx(commands,4);
    m_cs_disable();

    HAL_Delay(5);  /* delay 5 ms */

    // Now send data to LCD
    m_set_data();
    m_cs_enable();
    for(x=0;x<50;x++)
        m_lcd_buffer[1][x] = 0x55;
    drv_spi3_tx(&(m_lcd_buffer[1][0]), 50);
    m_cs_disable();
#endif

    return;
}


void _lcd_set_pixel(uint8_t row, uint8_t col)
{
    uint8_t i,p;
    i = (row%LCD_ROWS_PER_PAGE);
    p = (uint8_t)(row-(i*8));
    // i is the page index, p is the row in the page
    #ifndef LCD_DOUBLE_BUFFER
    m_lcd_buffer[i][col] = (uint8_t)(m_lcd_buffer[i][col]&(~(1<<p)));
    #else
    nop();
    #endif

    return;
}
void _lcd_clear_pixel(uint8_t row, uint8_t col)
{
    uint8_t i,p;
    i = (row%LCD_ROWS_PER_PAGE);
    p = (uint8_t)(row-(i*8));
    // i is the page index, p is the row in the page
    #ifndef LCD_DOUBLE_BUFFER
    m_lcd_buffer[i][col] = (uint8_t)(m_lcd_buffer[i][col]|(1<<p));
    #else
    nop();
    #endif
    return;
}

void _lcdDEBUG_test_pattern1()
{
    uint8_t row,col;
    drv_lcd_blank();
    for(row=0;row<LCD_ROWS;row++)
    {
        // every other pixel
        //for(col=(uint8_t)(row&1);col<LCD_COLS;col+=2)
        for(col=0;col<LCD_COLS;col=(uint8_t)(col+2))
        {
            _lcd_set_pixel(row,col);
        }
    }
    return;
}
// eof
