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

void lcd_init(void);

void lcd_task( void *pvParameters );

void lcd_blank(void);

void lcd_set_pixel(uint8_t row, uint8_t col);

void lcd_clear_pixel(uint8_t row, uint8_t col);

void lcd_print(char * st,uint8_t row,uint8_t col);

void lcd_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

#endif

//eof
