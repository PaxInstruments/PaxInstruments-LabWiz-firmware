/*****************************************************************************
 **
 ** USB driver
 ** August 2016
 **
 ** This driver provides functions related to the USB
 **
 ****************************************************************************/

#include "labwiz/labwizdefs.h"
#include "labwiz/drv_usb.h"
#include "labwiz/usbd_cdc_if.h"

#include "stm32f1xx_hal.h"

// Definitions and externs
// ---------------------------------------------------------------------------

// Local module variables
// ---------------------------------------------------------------------------

// Private prototypes
// ---------------------------------------------------------------------------

// Public functions
// ---------------------------------------------------------------------------
uint8_t usb_write(uint8_t * buf, uint16_t size)
{
   uint8_t result;
   result = CDC_Transmit_FS(buf, size);
   return result;
}

// eof
