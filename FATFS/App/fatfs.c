/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "app_debug.h"

/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
FRESULT fresult;

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

uint32_t fatfs_read_file(const char *file, uint8_t *data, uint32_t size)
{
    UINT byte_read = 0;
//    if (!m_sdcard_is_mounted)
//    {
//        goto end;
//    }
    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("Seek file %s failed\r\n", file);
        f_close(&USERFile);
        goto end;
    }

    fresult = f_read(&USERFile, data, size, &byte_read);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("Read file %s failed %d\r\n", file, fresult);
        f_close(&USERFile);
        goto end;
    }

    f_close(&USERFile);

end:
    return byte_read;
}

int32_t fatfs_read_file_at_pos(const char *file, uint8_t *data, uint32_t size, uint32_t pos)
{
    UINT byte_read = 0;
//    if (!m_sdcard_is_mounted)
//    {
//        goto end;
//    }
    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }
    DEBUG_VERBOSE("File %s opened\r\n", file);

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("[0] Seek file %s failed\r\n", file);
        f_close(&USERFile);
        goto end;
    }

    fresult = f_lseek(&USERFile, pos);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("[1] Seek file %s failed\r\n", file);
        f_close(&USERFile);
        goto end;
    }

    fresult = f_read(&USERFile, data, size, &byte_read);
    if (FR_OK != fresult)
    {
        DEBUG_ERROR("Read file %s failed %d\r\n", file, fresult);
        f_close(&USERFile);
        goto end;
    }

    f_close(&USERFile);
    DEBUG_VERBOSE("File %s closed\r\n", file);
end:
    return byte_read;
}

int32_t fatfs_get_file_size(const char *file)
{
    int32_t size = -1;
//    if (!m_sdcard_is_mounted)
//    {
//        goto end;
//    }
    fresult = f_open(&USERFile, file, FA_READ);
    if (fresult != FR_OK)
    {
        DEBUG_ERROR("Open file %s failed %d\r\n", file, fresult);
        goto end;
    }

    fresult = f_lseek(&USERFile, SEEK_SET);
    if (FR_OK != fresult)
    {
        f_close(&USERFile);
        DEBUG_ERROR("Seek file %s failed %d\r\n", file, fresult);
        goto end;
    }

    size = f_size(&USERFile);
    f_close(&USERFile);

end:
    return size;
}
/* USER CODE END Application */
