/*****************************************************************************
 **
 ** File system related functions.
 ** August 2016
 **
 ****************************************************************************/

#include "labwiz/defs.h"
#include "labwiz/drv_filesystem.h"

#include "ff.h"

// Definitions and types
// ----------------------------------------------------------------------------

// Local variables
// ----------------------------------------------------------------------------
FATFS m_FatFs;

// Local prototypes
// ----------------------------------------------------------------------------

// Public functions
// ----------------------------------------------------------------------------
bool fs_open_path(char * path)
{
    FRESULT ret;
    // Mount path
    ret = f_mount (&m_FatFs,path,0);
    if(ret!=FR_OK)
        return false;
    return true;
}
bool fs_card_detected()
{
    return (f_open_path("")==FR_OK);
}
// Private functions
// ----------------------------------------------------------------------------


// eof
