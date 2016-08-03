/*****************************************************************************
 **
 ** Serial access functions
 ** August 2016
 **
 ****************************************************************************/

#include "labwiz/defs.h"
#include "labwiz/drv_serial.h"

#include "FreeRTOS.h"
#include "semphr.h"

// A mutex to synchronize the serial port from different tasks
xSemaphoreHandle m_serial_mutex;

void drv_serial_init()
{
    m_serial_mutex = xSemaphoreCreateMutex();

    return;
}

// This is blocking
void drv_serial_tx(void * buf, int count)
{
    xSemaphoreTake( m_serial_mutex, portMAX_DELAY );
    // The following will only execute once the mutex has been obtained
    nop();
    // TODO: Send things to the serial port
    // The mutex MUST be given back
    xSemaphoreGive( m_serial_mutex );
    return;
}

// eof
