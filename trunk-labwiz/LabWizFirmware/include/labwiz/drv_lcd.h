/*****************************************************************************
 **
 ** LCD driver
 ** August 2016
 **
 ** This driver provides functions to write the LCD
 **
 ****************************************************************************/

#ifndef __DRV_LCD_H__
#define __DRV_LCD_H__

// The LCD is a 132x64 pixel graphical LCD.  If split into 8 bit 'pages' then
// it is 133 x 8pages
#define LCD_COLS            132
#define LCD_ROWS            64
#define LCD_ROWS_PER_PAGE   8
#define LCD_PAGES           (LCD_ROWS/LCD_ROWS_PER_PAGE)    // 8

#define MAX_ROW     (LCD_ROWS-1)
#define MAX_COL     (LCD_COLS-1)

typedef struct{
    uint8_t data[LCD_ROWS][LCD_COLS];
}lcd_screen_t;


void lcd_init(void);

void lcd_task( void *pvParameters );

void lcd_blank(void);

void lcd_set_pixel(uint8_t row, uint8_t col);

void lcd_clear_pixel(uint8_t row, uint8_t col);

void lcd_print(char * st,uint8_t row,uint8_t col);

void lcd_line(uint8_t r0, uint8_t c0, uint8_t r1, uint8_t c1);

void lcd_backlight_toggle(void);

void lcd_backlight_enable(bool enable);

void lcd_get_screen(lcd_screen_t * screen);

void lcd_set_screen(lcd_screen_t * screen);

#endif

//eof
