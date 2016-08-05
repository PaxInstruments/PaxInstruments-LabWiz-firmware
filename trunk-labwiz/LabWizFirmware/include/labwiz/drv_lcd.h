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

void drv_lcd_init(void);

void drv_lcd_task( void *pvParameters );

void drv_lcd_blank(void);

#endif

//eof
