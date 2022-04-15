/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
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
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#include "app_debug.h"
#include "app_spi_flash.h"
#include "app_drv_spi.h"
#include "spi.h"
#include "user_diskio.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static app_flash_drv_t m_spi_flash;
static void spi_flash_delay(void *arg, uint32_t ms);
//static QueueHandle_t m_cmd_queue;

static void spi_flash_delay(void *arg, uint32_t ms)
{
    uint32_t now = xTaskGetTickCount();
    vTaskDelayUntil(&now, ms);
}


void storage_flash_initialize(void)
{
    app_drv_spi_initialize();
    m_spi_flash.error = false;
    m_spi_flash.spi = &hspi1;
    m_spi_flash.callback.spi_cs = app_drv_spi_cs;
    m_spi_flash.callback.spi_rx_buffer = app_drv_spi_receive_frame;
    m_spi_flash.callback.spi_tx_buffer = app_drv_spi_transmit_frame;
    m_spi_flash.callback.spi_tx_rx = app_drv_spi_transmit_receive_frame;
    m_spi_flash.callback.spi_tx_byte = app_drv_spi_transmit_byte;
    m_spi_flash.callback.delay_ms = spi_flash_delay;

    if (app_spi_flash_initialize(&m_spi_flash) == false)
    {
        m_spi_flash.error = true;
        DEBUG_ERROR("SPI flash error\r\n");
    }

//    if (!m_cmd_queue)
//    {
//        m_cmd_queue = xQueueCreate(4, sizeof(uint8_t));
//    }
}
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    Stat = RES_OK;
    storage_flash_initialize();
    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    Stat = STA_NOINIT;
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	uint32_t i;
	uint32_t addr = sector * APP_SPI_FLASH_SECTOR_SIZE;
	DEBUG_VERBOSE("Read data at sector %d to %d\r\n", sector, sector + count);
	for (i=0; i<count; i++)
	{
		app_spi_flash_read_bytes(&m_spi_flash, addr, buff, APP_SPI_FLASH_SECTOR_SIZE);
		sector++;
		buff += APP_SPI_FLASH_SECTOR_SIZE;
		addr += APP_SPI_FLASH_SECTOR_SIZE;
	}
    return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
	  /* USER CODE HERE */
	uint32_t i;
	uint32_t addr = sector * APP_SPI_FLASH_SECTOR_SIZE;
	for (i=0; i<count ;i++)
	{
		app_spi_flash_erase_sector_4k(&m_spi_flash, sector);
		app_spi_flash_write(&m_spi_flash, addr, buff, APP_SPI_FLASH_SECTOR_SIZE);
		sector++;
		buff += APP_SPI_FLASH_SECTOR_SIZE;
		addr += APP_SPI_FLASH_SECTOR_SIZE;
	}

    return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	DRESULT res = RES_ERROR;
	switch(cmd)
	{
		case CTRL_SYNC :
			res = RES_OK;
			break;

		//????
//		case CTRL_ERASE_SECTOR:
//			nFrom = *((DWORD*)buff);
//			nTo = *(((DWORD*)buff)+1);
//			for(i=nFrom;i<=nTo;i++)
//				W25X_Erase_Sector(i);
//
//			break;

		case GET_BLOCK_SIZE:
			*(DWORD*)buff = 65536;
//			DEBUG_INFO ("get block size \r\n");
			res = RES_OK;
		//	buf[1] = (u8)(FLASH_BLOCK_SIZE & 0xFF);
		//	buf[0] = (u8)(FLASH_BLOCK_SIZE >> 8);
			break;


		case GET_SECTOR_SIZE:
			*(DWORD*)buff = APP_SPI_FLASH_SECTOR_SIZE;
//			DEBUG_INFO ("get sector size \r\n");
			res = RES_OK;
		//	buf[0] = (u8)(FLASH_SECTOR_SIZE & 0xFF);
		//	buf[1] = (u8)(FLASH_SECTOR_SIZE >> 8);
			break;

		case GET_SECTOR_COUNT:
			*(DWORD*)buff = m_spi_flash.info.size/APP_SPI_FLASH_SECTOR_SIZE;
			DEBUG_VERBOSE("User diskio sector count %u\r\n", *(DWORD*)buff);
			res = RES_OK;
		//	buf[0] = (u8)(FLASH_SECTOR_COUNT & 0xFF);
		//	buf[1] = (u8)(FLASH_SECTOR_COUNT >> 8);
			break;

		case DISKIO_CMD_WRITE_RAW:
		{
			user_diskio_raw_cmd_t *cmd = (user_diskio_raw_cmd_t*)buff;
			app_spi_flash_write(&m_spi_flash, cmd->addr, cmd->buffer, cmd->size);
		}
			break;
		case DISKIO_CMD_READ_RAW:
		{
			user_diskio_raw_cmd_t *cmd = (user_diskio_raw_cmd_t*)buff;
			app_spi_flash_erase_sector_4k(&m_spi_flash, cmd->addr/APP_SPI_FLASH_SECTOR_SIZE);
			app_spi_flash_read_bytes(&m_spi_flash, cmd->addr, cmd->buffer, cmd->size);
		}
			break;

		case DISKIO_CMD_ERASE:
		{
			DWORD sector = *(DWORD*)buff;
			app_spi_flash_erase_sector_4k(&m_spi_flash, sector);
		}
			break;

		default:
			res = RES_PARERR;
			break;
	}
	return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

