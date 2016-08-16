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
DIR m_log_dir;
FRESULT fret;
FIL m_tmp_fp;
// Local prototypes
// ----------------------------------------------------------------------------

// Public functions
// ----------------------------------------------------------------------------
bool fs_intialize_card()
{
    uint8_t result;
    result = BSP_SD_Init();
    if(result != MSD_OK)
        return false;
    return true;
}
bool fs_open_path(char * path)
{
    // Mount path
    fret = f_mount (&m_FatFs,path,0);
    if(fret!=FR_OK)
        return false;
    fret = f_opendir(&m_log_dir,"");
    if(fret!=FR_OK)
        return false;
    return true;
}
bool fs_close_path()
{
    bool result=true;

    while(BSP_SD_GetStatus()==SD_TRANSFER_BUSY);

    fret = f_closedir(&m_log_dir);
    if(fret!=FR_OK)
        result = false;
    fret = f_mount(NULL,"",0);
    if(fret!=FR_OK)
        result = false;
    return result;
}
bool fs_card_detected()
{
    return true;
}

bool fs_exists(char *filename)
{
    fret = f_open(&m_tmp_fp, filename, FA_READ);
    f_close(&m_tmp_fp);
    return (fret==FR_OK);
}
// Private functions
// ----------------------------------------------------------------------------


// eof
