/*****************************************************************************
 **
 ** File system related functions - Header
 ** August 2016
 **
 ****************************************************************************/

#ifndef __DRV_FILESYSTEM_H__
#define __DRV_FILESYSTEM_H__

#include "ff.h"

bool fs_intialize_card();

bool fs_open_path(char * path);

bool fs_close_path();

bool fs_card_detected(void);

bool fs_exists(char *filename);

#endif

//eof
