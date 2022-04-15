/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

//#include "bsp/board.h"
#include "tusb.h"
#include "app_spi_flash.h"
#include "app_drv_spi.h"
#include "spi.h"
#include "app_debug.h"
#include "fatfs.h"

typedef struct
{
	int32_t rd_sector;
	int32_t wr_sector;
//	uint32_t wr_offset;
	uint8_t *wr_buffer;
} usb_msc_cache_t;

#if CFG_TUD_MSC
//
//
//enum
//{
//  DISK_BLOCK_NUM  = 16, // 8KB is the smallest size that windows allow to mount
//  DISK_BLOCK_SIZE = 512
//};

// Cache flash data in ram, for faster read and write speed
usb_msc_cache_t m_disk_cache =
{
	.rd_sector = -1,
	.wr_sector = -1,
//		.wr_offset = 0,
	.wr_buffer = 0
};


// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
  (void) lun;

  const char vid[] = "BSAFE";
  const char pid[] = "Mass Storage";
  const char rev[] = "1.0";

  memcpy(vendor_id  , vid, strlen(vid));
  memcpy(product_id , pid, strlen(pid));
  memcpy(product_rev, rev, strlen(rev));
}


// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  (void) lun;

  return true; // RAM disk is always ready
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
uint32_t m_disk_block_size = 4096;

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
  (void) lun;
  if (lun > 0)
  {
	  DEBUG_ERROR("invalid lun number %u", lun);
	  return;
  }

//  *block_count = DISK_BLOCK_NUM; //m_spi_flash.info.size/DISK_BLOCK_SIZE;
//  *block_size  = DISK_BLOCK_SIZE;
//  DEBUG_VERBOSE("Block count %u, size %u\r\n", *block_count, *block_size);
	uint32_t tmp;
	disk_ioctl(0, GET_SECTOR_COUNT, &tmp);
	*block_count = tmp;
	disk_ioctl(0, GET_SECTOR_SIZE, &tmp);
	*block_size = tmp;
//	m_disk_block_size = *block_size;
	if (!m_disk_cache.wr_buffer)
	{
		m_disk_cache.wr_buffer = pvPortMalloc(m_disk_block_size);
	}
	DEBUG_VERBOSE("Disk has %u block, size of block %u\r\n", *block_count, m_disk_block_size);
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun;
  (void) power_condition;

  if ( load_eject )
  {
    if (start)
    {
      // load disk storage
    }else
    {
      // unload disk storage
    }
  }

  return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
	 if (lun != 0)
	 {
		DEBUG_ERROR("Invalid lun number %u", lun);
		return 0;
	}

	const uint32_t block_count = (bufsize + m_disk_block_size -1) / m_disk_block_size;
	if (bufsize < m_disk_block_size)
	{
		if (m_disk_cache.wr_sector == -1
			|| m_disk_cache.rd_sector == -1
			|| lba != m_disk_cache.rd_sector)		// If invalid sector =>> read data from flash immediately
		{
			DEBUG_VERBOSE("Read from flash\r\n");
			m_disk_cache.rd_sector = lba;
			m_disk_cache.wr_sector = lba;
			disk_read(0, m_disk_cache.wr_buffer, lba, block_count);
		}
		else
		{
			DEBUG_VERBOSE("Read from cache\r\n");
		}
//		disk_read(0, m_cache, lba, block_count);
		memcpy(buffer, m_disk_cache.wr_buffer+offset, bufsize);
	}
	else		// never happen
	{
//		disk_read(0, buffer, lba, block_count);
		configASSERT(0);
	}
	DEBUG_VERBOSE("Disk read %u bytes, LBA=%u, offset %u, block = %u, size = %u\r\n", bufsize, lba, offset, block_count, m_disk_block_size);

	return bufsize;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
	(void) lun;

	const uint32_t block_count = (bufsize + m_disk_block_size -1) / m_disk_block_size;

	if (bufsize <= m_disk_block_size)		// always jump into here
	{
		// Read back all data from sector in flash to cache buffer
		if (m_disk_cache.wr_sector == -1
			|| lba != m_disk_cache.wr_sector)		// If invalid sector =>> read data from flash immediately
		{
			m_disk_cache.wr_sector = lba;
			disk_read(0, m_disk_cache.wr_buffer, lba, block_count);
		}

		m_disk_cache.rd_sector = lba;
		// Copy content
		memcpy(&m_disk_cache.wr_buffer[offset], buffer, bufsize);

		if (offset + 512 == m_disk_block_size)
		{
			// Sync now
			DEBUG_WARN("Sync to flash now\r\n");
			disk_write(0, m_disk_cache.wr_buffer, lba, block_count);
		}
	}
	else		// never happen
	{
		configASSERT(0);
	}
	DEBUG_VERBOSE("Disk write %u bytes, LBA=%u, offset %u, block = %u, size = %u\r\n", bufsize, lba, offset, block_count, m_disk_block_size);
	return bufsize;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb (uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
  // read10 & write10 has their own callback and MUST not be handled here

  void const* response = NULL;
  int32_t resplen = 0;

  // most scsi handled is input
  bool in_xfer = true;

  switch (scsi_cmd[0])
  {
    default:
      // Set Sense = Invalid Command Operation
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

      // negative means error -> tinyusb could stall and/or response with failed status
      resplen = -1;
    break;
  }

  // return resplen must not larger than bufsize
  if ( resplen > bufsize ) resplen = bufsize;

  if ( response && (resplen > 0) )
  {
    if(in_xfer)
    {
      memcpy(buffer, response, resplen);
    }else
    {
      // SCSI output
    }
  }

  return resplen;
}

#endif
