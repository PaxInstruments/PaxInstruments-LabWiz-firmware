/*****************************************************************************
 **
 ** LabWiz related functions.
 ** August 2016
 **
 ****************************************************************************/

#include "labwiz/defs.h"
#include "labwiz/labwiz.h"

#include "labwiz/drv_serial.h"

void labwiz_init()
{
    drv_serial_init();

    return;
}

// eof
