/*****************************************************************************
 **
 ** File system related functions.
 ** August 2016
 **
 ****************************************************************************/

#include "labwiz/defs.h"
#include "labwiz/drv_filesystem.h"

/*****************************************************************************
 *
 *       FIL fp;
 *       FRESULT ret;
 *       uint32_t bytes;
 *
 *       if(fs_card_detected() && fs_open_path(""))
 *       {
 *           ret = f_open (&fp,"ACGtest.txt", (FA_WRITE | FA_CREATE_ALWAYS) );
 *           if(ret!=FR_OK)
 *               nop();
 *           ret = f_write (&fp, "Testing1234", 11, (UINT*)&bytes);
 *           if(ret!=FR_OK)
 *               nop();
 *           ret = f_write (&fp, "AnotherTest", 11, (UINT*)&bytes);
 *           if(ret!=FR_OK) // FR_INVALID_OBJECT is ejected, f_mount(NULL,"",0);
 *               nop();
 *           ret = f_close (&fp);
 *           if(ret!=FR_OK)
 *               nop();
 *       }
 *
 ****************************************************************************/

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
    return (fs_open_path("")==FR_OK);
}

bool fs_exists(char *filename)
{
    FIL fp;
    FRESULT result;
    result = f_open(&fp, filename, FA_READ);
    f_close(&fp);
    return (result==FR_OK);
}
// Private functions
// ----------------------------------------------------------------------------


// eof
