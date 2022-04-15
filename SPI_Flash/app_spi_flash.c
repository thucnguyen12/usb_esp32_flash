 #include "app_spi_flash.h"
#include "app_debug.h"
#include <string.h>

typedef union
{
    struct
    {
        uint8_t manufacture_id[2];
        uint8_t device_id[2];
    } __attribute__((packed)) name;
    uint8_t raw[4];
} __attribute__((packed)) app_spi_flash_device_id_t;

#define VERIFY_FLASH 1
#define DEBUG_FLASH 0
#define FLASH_INIT_MAX_RETRIES 3

#define READ_DATA_CMD 0x03
#define FAST_READ_DATA_CMD 0x0B
#define RDID_CMD 0x9F
#define READ_ID_CMD 0x90

#define WREN_CMD 0x06
#define WRDI_CMD 0x04

#define SE_CMD 0x20  // Sector Erase - 4KB
#define BE_CMD 0xD8  // Block Erase - 64KB
#define CE_CMD 0xC7  // 0x60 Chip Erase
#define WRR_CMD 0x01 // Ghi vao thanh ghi trang thai
#define PP_CMD 0x02
#define RDSR_CMD 0x05 /* Read status register */
#define CLSR_CMD 0x30 /* Dua Flash ve trang thai mac dinh */

#define SPI_DUMMY 0x00

/* Dinh nghia cac lenh cho dia chi do rong 4-byte */
#define READ_DATA_CMD4 0x13      // Read
#define FAST_READ_DATA_CMD4 0x0C // FastRead
#define PP_CMD4 0x12             // Page Program
#define SE_CMD4 0x21             // Sector Erase - 4KB - 32bit addrress
#define BE_CMD4 0xDC             // Block Erase - 64KB - 32bits address

/* Mot so lenh danh rieng cho Flash GigaDevice */
#define EN4B_MODE_CMD 0xB7 // Vao che do 32bits dia chi
#define EX4B_MODE_CMD 0xE9 // Thoat che do 32bits dia chi
#define RDSR1_CMD 0x05     // Read status register 1
#define RDSR2_CMD 0x35     // Read status register 2
#define RDSR3_CMD 0x15     // Read status register 3

#define WRSR1_CMD 0x01 // Write status register 1
#define WRSR2_CMD 0x31 // Write status register 2
#define WRSR3_CMD 0x11 // Write status register 3

/* Winbond */
#define WB_RESET_STEP0_CMD 0x66
#define WB_RESET_STEP1_CMD 0x99
#define WB_POWER_DOWN_CMD 0xB9
#define WB_WAKEUP_CMD 0xAB

#define SPI_FLASH_SHUTDOWN_ENABLE 0
#define HAL_SPI_Initialize() while (0)

#define SECTOR_ERASE_TIME_MS 400
#define FLASH_WRITE_TIMEOUT_MS 2000
#define FLASH_ERASE_TIMEOUT_MS 60000

static bool flash_get_device_id(app_flash_drv_t *flash_drv);
//static bool is_the_first_time_device_run(app_flash_drv_t *flash_drv);
bool flash_check_header(app_flash_drv_t *flash_drv);

static const app_flash_info_t m_info_table[] =
    {
        {APP_SPI_FLASH_DEVICE_INVALID, 0, APP_SPI_DEVICE_ERROR}, // bytes
        {
            APP_SPI_FLASH_FL164K,
            2 * 1024 * 1024,
            APP_SPI_FRAM,
        },
        {
            APP_SPI_FLASH_FL127S,
            16 * 1024 * 1024,
            APP_SPI_FRAM,
        },
        {
            APP_SPI_FLASH_FL256S,
            32 * 1024 * 1024,
            APP_SPI_FRAM,
        },
        {
            APP_SPI_FLASH_GD256,
            32 * 1024 * 1024,
            APP_SPI_FLASH,
        },
        {
            APP_SPI_FLASH_W25Q256JV,
            32 * 1024 * 1024,
            APP_SPI_FLASH,
        },
        {
            APP_SPI_FLASH_W25Q80D,
            2 * 1024 * 1024,
            APP_SPI_FLASH,
        },
        {
            APP_SPI_FLASH_W25Q128,
            16 * 1024 * 1024,
            APP_SPI_FLASH,
        },
        {
            APP_SPI_FLASH_W25Q32,
            4 * 1024 * 1024,
            APP_SPI_FLASH,
        },
        {
            APP_SPI_FLASH_W25Q64,
            8 * 1024 * 1024,
            APP_SPI_FLASH,
        },
        {
            APP_SPI_FRAM_MB85RS16,
            8 * 1024,
            APP_SPI_FRAM,
        },
        {
            APP_SPI_FLASH_AT25SF128,
            16 * 1024 * 1024,
            APP_SPI_FLASH,
        },
        {
            APP_SPI_FRAM_FM25V02A,
            32 * 1024,
            APP_SPI_FRAM,
        },
        //{  APP_SPI_FRAM_FM25V02A, 	        16*1024,  	    APP_SPI_FRAM,			},
        {
            APP_SPI_FRAM_MB85RS64V,
            8 * 1024,
            APP_SPI_FRAM,
        },
        {
            APP_SPI_FRAM_FM25V01,
            16 * 1024,
            APP_SPI_FRAM,
        },
        {
            APP_SPI_FRAM_FM25V10,
            16 * 1024,
            APP_SPI_FRAM,
        },
        {
            APP_SPI_FLASH_W25Q16,
            2 * 1024* 1024,
            APP_SPI_FLASH,
        },

        {APP_SPI_FLASH_MAX, 0, APP_SPI_DEVICE_ERROR},
};

enum
{
    NUM_CURRENT_LOOK_UP = sizeof(m_info_table) / sizeof(app_flash_info_t)
};

static app_flash_info_t *lookup_flash(app_flash_device_t id)
{
    if (id > APP_SPI_FLASH_MAX)
    {
        return NULL;
    }

    return (app_flash_info_t *)&m_info_table[id];
}

bool app_spi_flash_initialize(app_flash_drv_t *flash_drv)
{
    // bool flash_test_status = 0;

    //	HAL_SPI_Initialize();
    app_spi_flash_wakeup();
    if (flash_get_device_id(flash_drv))
    {
        app_flash_info_t *info = lookup_flash(flash_drv->info.device);
        if (info)
        {
            memcpy(&flash_drv->info, info, sizeof(app_flash_info_t));
        }
        else
        {
            flash_drv->info.type = APP_SPI_DEVICE_ERROR;
        }
//        DEBUG_INFO("Flash self test[OK]\r\nFlash type: ");
        switch (flash_drv->info.device)
        {
        case APP_SPI_FLASH_FL164K: // 8MB
            DEBUG_RAW("APP_SPI_FLASH_FL164K, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_FL127S: // 16MB
            DEBUG_RAW("APP_SPI_FLASH_FL127S, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_FL256S: // 32MB
            DEBUG_RAW("APP_SPI_FLASH_FL256S, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_GD256: // 32MB
            DEBUG_RAW("APP_SPI_FLASH_GD256, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_W25Q256JV: // 32MB
            DEBUG_RAW("APP_SPI_FLASH_256JV, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_W25Q80D:
            DEBUG_RAW("APP_SPI_FLASH_W25Q80DL, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_W25Q128:
            DEBUG_RAW("APP_SPI_FLASH_W25Q128, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_W25Q32:
            DEBUG_RAW("APP_SPI_FLASH_W25Q32FV, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_W25Q64:
            DEBUG_RAW("APP_SPI_FLASH_W25Q64, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FRAM_MB85RS16:
            DEBUG_RAW("APP_SPI_FRAM_MB85RS16, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_AT25SF128:
            DEBUG_RAW("APP_SPI_FLASH_AT25SF128, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FRAM_FM25V02A:
            DEBUG_RAW("APP_SPI_FRAM_FM25V02A, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FRAM_MB85RS64V:
            DEBUG_RAW("APP_SPI_FRAM_MB85RS64V, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FRAM_FM25V01:
            DEBUG_RAW("APP_SPI_FRAM_FM25V01, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FRAM_FM25V10:
            DEBUG_RAW("APP_SPI_FRAM_FM25V10, size %u bytes", flash_drv->info.size);
            break;
        case APP_SPI_FLASH_W25Q16:
            DEBUG_RAW("APP_SPI_FLASH_W25Q16, size %u bytes", flash_drv->info.size);
            break;
        default:
            DEBUG_RAW("UNKNOWNN: %u", flash_drv->info.device);
            break;
        }
        DEBUG_RAW("\r\n");
    }
    else
    {
        DEBUG_ERROR("SPI mem init failed\r\n");
        flash_drv->error = true;
        flash_drv->info.size = 0;
        return false;
    }

    return flash_drv->error ? false : true;
}

bool app_spi_flash_is_ok(app_flash_drv_t *flash_drv)
{
    return (flash_drv->error ? false : true);
}

static void flash_write_control(app_flash_drv_t *flash_drv, uint8_t enable)
{
    flash_drv->callback.spi_cs(flash_drv->spi, 0);
    if (enable)
    {
        flash_drv->callback.spi_tx_byte(flash_drv->spi, WREN_CMD);
    }
    else
    {
        flash_drv->callback.spi_tx_byte(flash_drv->spi, WRDI_CMD);
    }
    flash_drv->callback.spi_cs(flash_drv->spi, 1);
}

static void wait_write_in_process(app_flash_drv_t *flash_drv, uint32_t timeout_ms)
{
    uint8_t status[2];
    // uint8_t cmd;

    /* Read status register */
    flash_drv->callback.spi_cs(flash_drv->spi, 0);
    uint8_t tmp[2] = {RDSR_CMD, SPI_DUMMY};

    while (1)
    {
        flash_drv->callback.spi_tx_rx(flash_drv->spi, tmp, status, 2);

        if ((status[0] & 1) == 0)
            break;
        if (timeout_ms)
        {
            timeout_ms--;
            flash_drv->callback.delay_ms(flash_drv, 1);
        }
        else
        {
            break;
        }
    }

    if (timeout_ms == 0)
    {
        DEBUG_ERROR("[%s-%d] error\r\n", __FUNCTION__, __LINE__);
    }
    flash_drv->callback.spi_cs(flash_drv->spi, 1);
}

void app_spi_flash_direct_write_bytes(app_flash_drv_t *flash_drv, uint32_t addr, uint8_t *buffer, uint16_t length)
{
    if (flash_drv->error)
    {
        return;
    }
    DEBUG_VERBOSE("Flash write page addr 0x%08X, size %u\r\n", addr, length);


    flash_write_control(flash_drv, 1);
    flash_drv->callback.spi_cs(flash_drv->spi, 0);

    uint8_t tmp[32];
    uint32_t buffer_size = 0;

    // uint8_t cmd;
    if (flash_drv->info.device == APP_SPI_FLASH_FL256S || flash_drv->info.device == APP_SPI_FLASH_GD256 || flash_drv->info.device == APP_SPI_FLASH_W25Q256JV)
    {
        /* Send write cmd */
        tmp[buffer_size++] = PP_CMD4;

        /* Send 4 byte addr */
        tmp[buffer_size++] = (addr >> 24) & 0xFF;
    }
    else
    {
        /* Send write cmd */
        tmp[buffer_size++] = PP_CMD;
    }
#if 1
    if (flash_drv->info.type == APP_SPI_FLASH)
    {
        /* Send 3 bytes address */
        tmp[buffer_size++] = (addr >> 16) & 0xFF;
        tmp[buffer_size++] = (addr >> 8) & 0xFF;
        tmp[buffer_size++] = addr & 0xFF;
    }
    else // FRAM : only 2 byte address send
    {
        /* Send 2 bytes address */
        tmp[buffer_size++] = (addr & 0xFF00) >> 8;
        tmp[buffer_size++] = addr & 0xFF;
    }

    flash_drv->callback.spi_tx_buffer(flash_drv->spi, tmp, buffer_size);
    /* Send data to flash */
    flash_drv->callback.spi_tx_buffer(flash_drv->spi, buffer, length);
#endif
    flash_drv->callback.spi_cs(flash_drv->spi, 1);

    wait_write_in_process(flash_drv, FLASH_WRITE_TIMEOUT_MS);

#if VERIFY_FLASH
    uint32_t i = 0;
    uint32_t old_addr = addr;
    bool found_error = false;
    for (i = 0; i < length; i++) // Debug only
    {
        uint8_t rb;
        app_spi_flash_read_bytes(flash_drv, old_addr + i, (uint8_t *)&rb, 1);
        if (memcmp(&rb, buffer + i, 1))
        {
            found_error = true;
            DEBUG_ERROR("Flash write error at addr 0x%08X, readback 0x%02X, expect 0x%02X\r\n", old_addr + i, rb, *(buffer + i));
            break;
        }
        else
        {
            DEBUG_VERBOSE("Flash write success at addr 0x%08X, readback 0x%02X, expect 0x%02X\r\n", old_addr + i, rb, *(buffer + i));
        }
    }
    if (found_error == false)
    {
        DEBUG_VERBOSE("Page write success\r\n");
    }
#endif
    // vPortFree(tmp);
}

/*****************************************************************************/
/**
 * @brief	:  PageSize = 256 (Flash 8,16,32MB); 512 (Flash >= 64MB)
 * @param	:
 * @retval	:
 * @author	:
 * @created	:	15/01/2014
 * @version	:
 * @reviewer:
 */
void app_spi_flash_write(app_flash_drv_t *flash_drv, uint32_t addr, uint8_t *buffer, uint32_t length)
{
    if (flash_drv->error)
    {
        return;
    }
    /* Split data into page size (256), only availble in Flash */
    uint32_t offset_addr = 0;
    uint32_t length_need_to_write = 0;
    uint32_t nb_bytes_written = 0;

    DEBUG_VERBOSE("Flash write %u bytes, from addr 0x%08X\r\n", length, addr);
    if (addr + length > flash_drv->info.size)
    {
        DEBUG_ERROR("Flash write from 0x%08X to 0x%08X is over flash size %u bytes\r\n",
                    addr,
                    addr + length, flash_drv->info.size);
        return;
    }
    uint32_t max_write_size = APP_SPI_FLASH_PAGE_SIZE;
    if (flash_drv->info.type == APP_SPI_FRAM)
    {
        max_write_size = length + 1;
    }
    while (length)
    {
        offset_addr = addr % max_write_size;

        if (offset_addr > 0)
        {
            if (offset_addr + length > max_write_size)
            {
                length_need_to_write = max_write_size - offset_addr;
            }
            else
            {
                length_need_to_write = length;
            }
        }
        else
        {
            if (length > max_write_size)
            {
                length_need_to_write = max_write_size;
            }
            else
            {
                length_need_to_write = length;
            }
        }

        length -= length_need_to_write;

        app_spi_flash_direct_write_bytes(flash_drv,
                                         addr,
                                         &buffer[nb_bytes_written],
                                         length_need_to_write);

        nb_bytes_written += length_need_to_write;

        addr += length_need_to_write;
    }
}

void app_spi_flash_read_bytes(app_flash_drv_t *flash_drv, uint32_t addr, uint8_t *buffer, uint16_t length)
{
    if (flash_drv->error)
    {
        return;
    }

    flash_drv->callback.spi_cs(flash_drv->spi, 0);
    uint8_t cmd_buffer[6];
    uint8_t index = 0;
    uint32_t next_sector_offset = 0;
    if (addr + length > flash_drv->info.size)
    {
        next_sector_offset = (addr + length) - flash_drv->info.size;
        length = flash_drv->info.size - addr;
    }
    while (1)
    {
        if (flash_drv->info.device == APP_SPI_FLASH_FL256S || flash_drv->info.device == APP_SPI_FLASH_GD256 || flash_drv->info.device == APP_SPI_FLASH_W25Q256JV)
        {
            /* Send read cmd */
            cmd_buffer[index++] = READ_DATA_CMD4;
            cmd_buffer[index++] = (addr >> 24) & 0xFF;
        }
        else
        {
            /* Send read cmd*/
            cmd_buffer[index++] = READ_DATA_CMD;
        }

        if (flash_drv->info.type == APP_SPI_FLASH)
        {
            /* Send 3 bytes address */
            cmd_buffer[index++] = (addr >> 16) & 0xFF;
            ;
            cmd_buffer[index++] = (addr >> 8) & 0xFF;
            cmd_buffer[index++] = addr & 0xFF;
        }
        else
        {
            /* Send 2 bytes address */
            cmd_buffer[index++] = (addr & 0xFF00) >> 8;
            cmd_buffer[index++] = addr & 0xFF;
        }
        flash_drv->callback.spi_tx_buffer(flash_drv->spi, cmd_buffer, index);

        // Read data
        flash_drv->callback.spi_rx_buffer(flash_drv->spi, buffer, length);

        if (next_sector_offset == 0)
        {
            break;
        }
        else
        {
            buffer += length;
            addr = next_sector_offset;
            next_sector_offset = 0;
            continue;
        }
    }
    flash_drv->callback.spi_cs(flash_drv->spi, 1);
}

void app_spi_flash_erase_sector_4k(app_flash_drv_t *flash_drv, uint32_t sector_count)
{
    if (flash_drv->info.type == APP_SPI_FRAM)
    {
        DEBUG_WARN("FRAM doesnt need to erase sector 4KB\r\n");
        return;
    }
    if (flash_drv->error)
    {
        return;
    }

    DEBUG_INFO("Erase sector %u\r\n", sector_count);
    uint32_t addr = 0;
    // uint32_t old_addr = 0;
    addr = sector_count * APP_SPI_FLASH_SECTOR_SIZE; // Sector 4KB
//    old_addr = addr;

    flash_write_control(flash_drv, 1);
    flash_drv->callback.delay_ms(flash_drv, 5);
    uint8_t cmd_buffer[32];
    uint32_t cmd_count = 0;

    flash_drv->callback.spi_cs(flash_drv->spi, 0);
    if (flash_drv->info.device == APP_SPI_FLASH_FL256S || flash_drv->info.device == APP_SPI_FLASH_GD256 || flash_drv->info.device == APP_SPI_FLASH_W25Q256JV)
    {
        /* Gui lenh */
        cmd_buffer[cmd_count++] = SE_CMD4;

        /* Send 4 bytes address */
        cmd_buffer[cmd_count++] = (addr >> 24) & 0xFF;
    }
    else
    {
        /* Gui lenh */
        cmd_buffer[cmd_count++] = SE_CMD;
    }

    /* Send 3 byte address */
    cmd_buffer[cmd_count++] = (addr >> 16) & 0xFF;
    cmd_buffer[cmd_count++] = (addr >> 8) & 0xFF;
    cmd_buffer[cmd_count++] = addr & 0xFF;
    flash_drv->callback.spi_tx_buffer(flash_drv->spi, cmd_buffer, cmd_count);
    flash_drv->callback.spi_cs(flash_drv->spi, 1);
    flash_drv->callback.delay_ms(flash_drv, SECTOR_ERASE_TIME_MS);
    wait_write_in_process(flash_drv, 50);
    if (app_spi_flash_is_sector_empty(flash_drv, sector_count))
    {
        DEBUG_INFO("Success\r\n", sector_count);
    }
    else
    {
        DEBUG_ERROR("Failed\r\n", sector_count);
    }
}

#if 0
void flash_erase_block_64K(uint16_t sector_count)
{
    uint32_t addr = 0;
    uint32_t old_addr = 0;
    uint8_t cmd;

    addr = sector_count * 65536; //Sector 64KB
    old_addr = addr;

    flash_write_control(1, addr);

    flash_drv->callback.spi_cs(flash_drv->spi, 0);

    if (m_flash_version == APP_SPI_FLASH_FL256S || m_flash_version == APP_SPI_FLASH_GD256 || m_flash_version == APP_SPI_FLASH_256JV)
    {
        cmd = BE_CMD4;
        flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

        /* Send 4 bytes address */
        cmd = (addr >> 24) & 0xFF;
        flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
    }
    else
    {
        cmd = BE_CMD;
        flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
    }

    /* Send 3 bytes address */
    cmd = (addr >> 16) & 0xFF;
    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

    cmd = (addr >> 8) & 0xFF;
    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

    cmd = addr & 0xFF;
    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

    flash_drv->callback.spi_cs(flash_drv->spi, 1);
    wait_write_in_process(old_addr);
}
#endif

bool flash_get_device_id(app_flash_drv_t *flash_drv)
{
    uint8_t reg_status = 0;
    uint8_t tries;
    uint8_t cmd;
    app_spi_flash_device_id_t id;
    bool val = false;

    flash_drv->error = false;
    flash_drv->info.device = APP_SPI_FLASH_DEVICE_INVALID;

    for (tries = 0; tries < FLASH_INIT_MAX_RETRIES; tries++)
    {
        flash_drv->callback.spi_cs(flash_drv->spi, 0);
#if 0
        cmd = READ_ID_CMD;
#else
        cmd = RDID_CMD;
#endif
        if (cmd == READ_ID_CMD)
        {
            uint8_t buffer_tx[6] = {READ_ID_CMD, 0x00, 0x00, 0x00, 0xFF, 0x00};
            uint8_t buffer_rx[6];

#if 0
			flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

			/* 3 byte address */
			cmd = 0;
			flash_drv->callback.spi_tx_byte(flash_drv->spi, 0x00);
			flash_drv->callback.spi_tx_byte(flash_drv->spi, 0x00);
			flash_drv->callback.spi_tx_byte(flash_drv->spi, 0x00);

			cmd = 0xFF;
			flash_drv->callback.spi_tx_rx(flash_drv->spi, &cmd, &id.name.manufacture_id[1], 1);
			flash_drv->callback.spi_tx_rx(flash_drv->spi, &cmd, &id.name.device_id[0], 1);
#else
            flash_drv->callback.spi_tx_rx(flash_drv->spi, buffer_tx, buffer_rx, 6);
            flash_drv->callback.spi_cs(flash_drv->spi, 1);
            id.name.manufacture_id[1] = buffer_rx[4];
            id.name.device_id[0] = buffer_rx[4];

#endif
            DEBUG_INFO("device id: 0x0x%02, manufacture id: 0x%02X\r\n", id.name.device_id[0],
                       id.name.manufacture_id[1]);
        }
        else
        {
            uint8_t buffer_size = 1 + sizeof(app_spi_flash_device_id_t);
            uint8_t buffer_tx[buffer_size];
            uint8_t buffer_rx[buffer_size];
            buffer_tx[0] = cmd;

            flash_drv->callback.spi_tx_rx(flash_drv->spi, buffer_tx, buffer_rx, buffer_size);
            memcpy(&id, &buffer_rx[1], sizeof(app_spi_flash_device_id_t));
            flash_drv->callback.spi_cs(flash_drv->spi, 1);

            DEBUG_INFO("device id: 0x%02X%02X, manufacture id[0-1]: 0x%02X%02X\r\n", id.name.device_id[0],
                       id.name.device_id[1],
                       id.name.manufacture_id[0],
                       id.name.manufacture_id[1]);
        }

        if (cmd == READ_ID_CMD)     // read fram
        {
            if (id.name.manufacture_id[1] == 0x01)
            {
                DEBUG_INFO("FRAM\r\n");
                if (id.name.device_id[0] == 0x16)
                {
                    flash_drv->info.device = APP_SPI_FLASH_FL164K;
                    val = true;
                }
                else if (id.name.device_id[0] == 0x17)
                {
                    flash_drv->info.device = APP_SPI_FLASH_FL127S;
                    val = true;
                }
            }
            else if (id.name.manufacture_id[1] == 0xEF)
            {
                DEBUG_INFO("Windbond\r\n");
                if (id.name.device_id[0] == 0x13)
                {
                    DEBUG_INFO("W25Q80DL\r\n");
                    flash_drv->info.device = APP_SPI_FLASH_W25Q80D;
                    val = true;
                }
                if (id.name.device_id[0] == 0x15)
                {
                    DEBUG_INFO("W25Q32FV\r\n");
                    flash_drv->info.device = APP_SPI_FLASH_W25Q32;
                    val = true;
                }
                else if (id.name.device_id[0] == 0x17)
                {
                    DEBUG_INFO("W25Q128\r\n");
                    flash_drv->info.device = APP_SPI_FLASH_W25Q128;
                    val = true;
                }
                if (id.name.device_id[0] == 0x18)
                {
                    DEBUG_INFO("256JV\r\n");
                    flash_drv->info.device = APP_SPI_FLASH_W25Q256JV;
                    // Enter mode : 4 bytes address
                    flash_drv->callback.spi_cs(flash_drv->spi, 0);
                    cmd = EN4B_MODE_CMD;
                    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
                    flash_drv->callback.spi_cs(flash_drv->spi, 1);

                    flash_drv->callback.delay_ms(flash_drv, 10);
                    // Read status register 3, bit ADS  (S16) - bit 0
                    flash_drv->callback.spi_cs(flash_drv->spi, 0);
                    cmd = RDSR3_CMD;
                    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

                    cmd = SPI_DUMMY;
                    flash_drv->callback.spi_tx_rx(flash_drv->spi, &cmd, &reg_status, 1);

                    DEBUG_INFO("status register: %02X\r\n", reg_status);
                    if (reg_status & 0x01)
                    {
                        DEBUG_INFO("Address mode : 32 bit\r\n");
                    }
                    flash_drv->callback.spi_cs(flash_drv->spi, 1);
                    val = true;
                }
            }
            else if (id.name.manufacture_id[1] == 0xC8) /* APP_SPI_FLASH_GD256 - GigaDevice 256Mb */
            {
                DEBUG_INFO("Giga device\r\n");
                if (id.name.device_id[0] == 0x18)
                {
                    flash_drv->info.device = APP_SPI_FLASH_GD256;
                    DEBUG_INFO("GD256\r\n");

                    // Enter mode : 4 bytes address
                    flash_drv->callback.spi_cs(flash_drv->spi, 0);
                    cmd = EN4B_MODE_CMD;
                    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
                    flash_drv->callback.spi_cs(flash_drv->spi, 1);

                    flash_drv->callback.delay_ms(flash_drv, 10);
                    // Read register status 2, bit ADS - 5
                    flash_drv->callback.spi_cs(flash_drv->spi, 0);
                    cmd = RDSR2_CMD;
                    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

                    cmd = SPI_DUMMY;
                    flash_drv->callback.spi_tx_rx(flash_drv->spi, &cmd, &reg_status, 1);
                    flash_drv->callback.spi_cs(flash_drv->spi, 1);

                    DEBUG_INFO("status register: %02X\r\n", reg_status);
                    if (reg_status & 0x20)
                    {
                        DEBUG_INFO("Address mode : 32 bit\r\n");
                    }
                    else
                    {
                        flash_drv->callback.delay_ms(flash_drv, 500);
                        continue;
                    }
                    val = true;
                }
            }
            else if (id.name.manufacture_id[1] == 0x89) /* APP_SPI_FLASH_AT25SF128 */
            {
                DEBUG_INFO("Adesto Technologies\r\n");
                if (id.name.device_id[0] == 0x1F)
                {
                    flash_drv->info.device = APP_SPI_FLASH_AT25SF128;
                    DEBUG_INFO("AT25SF128A\r\n");

                    // Enter mode : 4 bytes address
                    flash_drv->callback.spi_cs(flash_drv->spi, 0);
                    cmd = EN4B_MODE_CMD;
                    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
                    flash_drv->callback.spi_cs(flash_drv->spi, 1);

                    flash_drv->callback.delay_ms(flash_drv, 10);
                    // Read register status 2, bit ADS - 5
                    flash_drv->callback.spi_cs(flash_drv->spi, 0);
                    cmd = RDSR2_CMD;
                    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

                    cmd = SPI_DUMMY;
                    flash_drv->callback.spi_tx_rx(flash_drv->spi, &cmd, &reg_status, 1);
                    flash_drv->callback.spi_cs(flash_drv->spi, 1);

                    DEBUG_INFO("status register: %02X\r\n", reg_status);
                    if (reg_status & 0x20)
                    {
                        DEBUG_INFO("Address mode : 32 bit\r\n");
                    }
                    else
                    {
                        flash_drv->callback.delay_ms(flash_drv, 500);
                        continue;
                    }
                    val = true;
                }
            }
            else if (id.name.manufacture_id[0] == 0x7F && id.name.manufacture_id[1] == 0x7F) /* APP_SPI_FLASH_AT25SF128 */
            {
                DEBUG_INFO("Cypress\r\n");
                // Cypress read total 9 byte,
                // 6 byte MSB = 7F7F7F7F7F7F
                // Byte 7 done care
                // Byte 89 = 2100 =>> FM25V01
                // Byte 89 = 2008 =>> FM25V02A

                uint8_t size = 10;
                uint8_t cypess_manufacture_data_tx[size];
                uint8_t cypess_manufacture_data_rx[size];
                cypess_manufacture_data_tx[0] = RDID_CMD;

                flash_drv->callback.spi_cs(flash_drv->spi, 0);
                flash_drv->callback.spi_tx_rx(flash_drv->spi, cypess_manufacture_data_tx, cypess_manufacture_data_rx, size);
                flash_drv->callback.spi_cs(flash_drv->spi, 1);
                for (uint32_t i = 1; i < size; i++)
                {
                    DEBUG_RAW("%02X ", cypess_manufacture_data_rx[i]);
                }
                DEBUG_RAW("\r\n");

                if (cypess_manufacture_data_rx[1] == 0x7F && cypess_manufacture_data_rx[2] == 0x7F && cypess_manufacture_data_rx[3] == 0x7F && cypess_manufacture_data_rx[4] == 0x7F && cypess_manufacture_data_rx[5] == 0x7F && cypess_manufacture_data_rx[6] == 0x7F && cypess_manufacture_data_rx[8] == 0x22 && cypess_manufacture_data_rx[9] == 0x08)
                {
                    flash_drv->info.device = APP_SPI_FRAM_FM25V02A;
                    DEBUG_INFO("FM25V02A\r\n");
                    val = true;
                }
                else if (cypess_manufacture_data_rx[1] == 0x7F && cypess_manufacture_data_rx[2] == 0x7F && cypess_manufacture_data_rx[3] == 0x7F && cypess_manufacture_data_rx[4] == 0x7F && cypess_manufacture_data_rx[5] == 0x7F && cypess_manufacture_data_rx[6] == 0x7F && cypess_manufacture_data_rx[8] == 0x21 && cypess_manufacture_data_rx[9] == 0x00)
                {
                    flash_drv->info.device = APP_SPI_FRAM_FM25V01;
                    DEBUG_INFO("FM25V01\r\n");
                    val = true;
                }
                else if (cypess_manufacture_data_rx[1] == 0x7F && cypess_manufacture_data_rx[2] == 0x7F && cypess_manufacture_data_rx[3] == 0x7F && cypess_manufacture_data_rx[4] == 0x7F && cypess_manufacture_data_rx[5] == 0x7F && cypess_manufacture_data_rx[6] == 0x7F && cypess_manufacture_data_rx[8] == 0x24 && cypess_manufacture_data_rx[9] == 0x00)
                {
                    flash_drv->info.device = APP_SPI_FRAM_FM25V10;
                    DEBUG_INFO("FM25V10\r\n");
                    val = true;
                }
            }
            else
            {
                DEBUG_ERROR("Unknown device\r\n");
                val = false;
            }
        }
        else if (cmd == RDID_CMD)
        {
//            if (id.name.manufacture_id[1] == 0xEF)
            if (id.name.manufacture_id[0] == 0xEF)
            {
                DEBUG_INFO("Winbond\r\n");
//                if (id.name.device_id[0] == 0x40 && id.name.device_id[1] == 0x17)
//                {
//                    DEBUG_INFO("W25Q65\r\n");
//                    flash_drv->info.device = APP_SPI_FLASH_W25Q64;
//                    val = true;
//                }
                
                if (id.name.manufacture_id[1] == 0x40 && id.name.device_id[0] == 0x15)
                {
                    DEBUG_INFO("W25Q16\r\n");
                    flash_drv->info.device = APP_SPI_FLASH_W25Q16;
                    val = true;
                }
                else if (id.name.manufacture_id[1] == 0x40 && id.name.device_id[0] == 0x17)
                {
                    DEBUG_INFO("W25Q64\r\n");
                    flash_drv->info.device = APP_SPI_FLASH_W25Q64;
                    val = true;
                }
            }
            else if (id.name.manufacture_id[0] == 0x04 && id.name.manufacture_id[1] == 0x7F)
            {
                if (id.name.device_id[0] == 0x01 && id.name.device_id[1] == 0x01)
                {
                    DEBUG_INFO("MB85RS16\r\n");
                    flash_drv->info.device = APP_SPI_FRAM_MB85RS16;
                    val = true;
                }
                else if (id.name.device_id[0] == 0x03 && id.name.device_id[1] == 0x02)
                {
                    DEBUG_INFO("MB85RS64V\r\n");
                    flash_drv->info.device = APP_SPI_FRAM_MB85RS64V;
                    val = true;
                }
            }
            else if (id.name.manufacture_id[0] == 0x1F && id.name.manufacture_id[1] == 0x89) /* APP_SPI_FLASH_AT25SF128 */
            {
                DEBUG_INFO("Adesto Technologies\r\n");
                flash_drv->info.device = APP_SPI_FLASH_AT25SF128;
                DEBUG_INFO("AT25SF128A\r\n");
                val = true;
            }
            else if (id.name.manufacture_id[0] == 0x7F && id.name.manufacture_id[1] == 0x7F) /* APP_SPI_FLASH_AT25SF128 */
            {
                DEBUG_INFO("Cypress\r\n");
                // Cypress read total 9 byte,
                // 6 byte MSB = 7F7F7F7F7F7F
                // Byte 7 done care
                // Byte 89 = 2100 =>> FM25V01
                // Byte 89 = 2008 =>> FM25V02A

                uint8_t size = 10;
                uint8_t cypess_manufacture_data_tx[size];
                uint8_t cypess_manufacture_data_rx[size];
                cypess_manufacture_data_tx[0] = RDID_CMD;

                flash_drv->callback.spi_cs(flash_drv->spi, 0);
                flash_drv->callback.spi_tx_rx(flash_drv->spi, cypess_manufacture_data_tx, cypess_manufacture_data_rx, size);
                flash_drv->callback.spi_cs(flash_drv->spi, 1);
                for (uint32_t i = 1; i < size; i++)
                {
                    DEBUG_RAW("%02X ", cypess_manufacture_data_rx[i]);
                }
                DEBUG_RAW("\r\n");

                if (cypess_manufacture_data_rx[1] == 0x7F && cypess_manufacture_data_rx[2] == 0x7F && cypess_manufacture_data_rx[3] == 0x7F && cypess_manufacture_data_rx[4] == 0x7F && cypess_manufacture_data_rx[5] == 0x7F && cypess_manufacture_data_rx[6] == 0x7F && cypess_manufacture_data_rx[8] == 0x22 && cypess_manufacture_data_rx[9] == 0x08)
                {
                    flash_drv->info.device = APP_SPI_FRAM_FM25V02A;
                    DEBUG_INFO("FM25V02A\r\n");
                    val = true;
                }
                else if (cypess_manufacture_data_rx[1] == 0x7F && cypess_manufacture_data_rx[2] == 0x7F && cypess_manufacture_data_rx[3] == 0x7F && cypess_manufacture_data_rx[4] == 0x7F && cypess_manufacture_data_rx[5] == 0x7F && cypess_manufacture_data_rx[6] == 0x7F && cypess_manufacture_data_rx[8] == 0x21 && cypess_manufacture_data_rx[9] == 0x00)
                {
                    flash_drv->info.device = APP_SPI_FRAM_FM25V01;
                    DEBUG_INFO("FM25V01\r\n");
                    val = true;
                }
                else if (cypess_manufacture_data_rx[1] == 0x7F && cypess_manufacture_data_rx[2] == 0x7F && cypess_manufacture_data_rx[3] == 0x7F && cypess_manufacture_data_rx[4] == 0x7F && cypess_manufacture_data_rx[5] == 0x7F && cypess_manufacture_data_rx[6] == 0x7F && cypess_manufacture_data_rx[8] == 0x24 && cypess_manufacture_data_rx[9] == 0x00)
                {
                    flash_drv->info.device = APP_SPI_FRAM_FM25V10;
                    DEBUG_INFO("FM25V10\r\n");
                    val = true;
                }
            }
        }

        if (val)
        {
            break;
        }
    }
    return val;
}

// bool flash_check_header(app_flash_drv_t *flash_drv)
//{
//     uint16_t flash_read_status = 0, i;
//     uint8_t buffer_test[10];
//
//     memset(buffer_test, 0xAA, sizeof(buffer_test));
//
//     /* Ghi vao Flash 10 bytes */
//     app_spi_flash_write(flash_drv, FLASH_CHECK_FIRST_RUN + 15, buffer_test, 10);
//
//     /* Verify */
//     memset(buffer_test, 0, 10);
//     app_spi_flash_read_bytes(flash_drv, FLASH_CHECK_FIRST_RUN + 15, buffer_test, 10);
//
//     for (i = 0; i < 10; i++)
//     {
//         if (buffer_test[i] != 0xAA)
//         {
//             flash_read_status++;
//         }
//     }
//     return flash_read_status ? false : true;
// }

bool app_spi_flash_erase_all(app_flash_drv_t *flash_drv, uint32_t timeout_ms)
{
    uint8_t status = 0xFF;
    uint8_t cmd;
    bool retval = false;
    DEBUG_VERBOSE("Erase all flash\r\n");

    flash_write_control(flash_drv, 1);
    flash_drv->callback.spi_cs(flash_drv->spi, 0);

    cmd = CE_CMD;
    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
    flash_drv->callback.spi_cs(flash_drv->spi, 1);

    /* Doc thanh ghi status */
    flash_drv->callback.delay_ms(flash_drv, 100);
    flash_drv->callback.spi_cs(flash_drv->spi, 0);
    cmd = RDSR_CMD;
    flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);

//    uint32_t begin_tick = xTaskGetTickCount();
    while (1)
    {
        cmd = SPI_DUMMY;
        flash_drv->callback.spi_tx_rx(flash_drv->spi, &cmd, &status, 1);
        if ((status & 1) == 0)
        {
            retval = true;
            break;
        }
        if (timeout_ms)
        {
            timeout_ms--;
            flash_drv->callback.delay_ms(flash_drv, 1);
        }

        if (timeout_ms == 0)
        {
            break;
        }
    }
    flash_drv->callback.spi_cs(flash_drv->spi, 1);

#if 0
//#if VERIFY_FLASH
    bool found_error = false;
    uint32_t old_addr = 0;
    uint8_t buffer[1] = {0xFF};
    for (uint32_t i = 0; i < APP_SPI_FLASH_SIZE; i++)      // Debug only
    {
        uint8_t tmp;
        app_spi_flash_read_bytes(old_addr + i, (uint8_t*)&tmp, 1);
        if (memcmp(&tmp, buffer, 1))
        {
            found_error = true;
            DEBUG_ERROR("Flash erase error at addr 0x%08X, readback 0x%02X, expect 0xFF\r\n", old_addr + i, tmp);
            break;
        }
    }
    if (found_error == false)
    {
        DEBUG_VERBOSE("Erase success\r\n");
    }
#endif

    if (retval)
    {
        DEBUG_INFO("Erase [DONE\r\n");
    }
    else
    {
        DEBUG_ERROR("Erase flash failed\r\n");
        flash_drv->error = true;
    }
    return retval;
}


void app_spi_flash_wakeup(void)
{
#if SPI_FLASH_SHUTDOWN_ENABLE
    DEBUG_INFO("Wakeup flash\r\n");
    for (uint8_t i = 0; i < 3; i++)
    {
        flash_drv->callback.spi_cs(flash_drv->spi, 0);
        uint8_t cmd = WB_WAKEUP_CMD;
        flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
        flash_drv->callback.spi_cs(flash_drv->spi, 1);
        for (volatile uint32_t i = 0; i < 48 * 3; i++) // 3us
        {
            __nop();
        }
    }
#endif
}

void app_spi_flash_shutdown(void)
{
#if SPI_FLASH_SHUTDOWN_ENABLE
    DEBUG_INFO("Shutdown flash\r\n");
    for (volatile uint8_t i = 0; i < 1; i++)
    {
        flash_drv->callback.spi_cs(flash_drv->spi, 0);
        uint8_t cmd = WB_POWER_DOWN_CMD;
        flash_drv->callback.spi_tx_byte(flash_drv->spi, cmd);
        flash_drv->callback.spi_cs(flash_drv->spi, 1);
    }
#endif
}

bool app_spi_flash_is_sector_empty(app_flash_drv_t *flash_drv, uint32_t sector_index)
{
    bool retval = true;
    if (flash_drv->info.type == APP_SPI_FLASH)
    {
        uint32_t addr = sector_index * APP_SPI_FLASH_SECTOR_SIZE;
        for (uint32_t i = 0; i < APP_SPI_FLASH_SECTOR_SIZE;) // Debug only
        {
            uint32_t tmp;
            app_spi_flash_read_bytes(flash_drv, addr + i, (uint8_t *)&tmp, 4);
            if (tmp != 0xFFFFFFFF)
            {
                retval = false;
                break;
            }
            i += 4;
        }
    }
    return retval;
}

void app_spi_flash_skip_to_end_flash_test(app_flash_drv_t *flash_drv)
{
    if (flash_drv->info.size)
    {
        flash_drv->current_write_address = flash_drv->info.size - 1;
    }
}

// Note : only support FRAM
void app_spi_flash_erase_from_to(app_flash_drv_t *flash_drv, uint32_t addr, uint32_t len)
{

    uint32_t i = 0;

    uint8_t tmp[96];
    if (addr + len > flash_drv->info.size)
    {
        len -= (addr + len - flash_drv->info.size);
    }
    DEBUG_INFO("Erase from addr 0x%08X, size %u\r\n", addr, len);
    while (len)
    {
        uint32_t buffer_size = 0;
        flash_write_control(flash_drv, 1);
//        uint8_t cmd;
        if (flash_drv->info.device == APP_SPI_FLASH_FL256S || flash_drv->info.device == APP_SPI_FLASH_GD256 || flash_drv->info.device == APP_SPI_FLASH_W25Q256JV)
        {
            /* Send write cmd */
            tmp[buffer_size++] = PP_CMD4;

            /* Send 4 byte addr */
            tmp[buffer_size++] = (addr >> 24) & 0xFF;
        }
        else
        {
            tmp[buffer_size++] = PP_CMD;
        }

        // FRAM MB85RS16 : only 2 byte address send
        if (flash_drv->info.type == APP_SPI_FLASH)
        {
            /* Send 3 bytes address */
            tmp[buffer_size++] = (addr >> 16) & 0xFF;
            tmp[buffer_size++] = (addr >> 8) & 0xFF;
            tmp[buffer_size++] = addr & 0xFF;
        }
        else
        {
            // DEBUG_INFO("FRAM =>> Send 2 byte addr to erase device at %u\r\n", addr);
            /* Send 2 bytes address */
            tmp[buffer_size++] = (addr & 0xFF00) >> 8;
            tmp[buffer_size++] = addr & 0xFF;
        }

        /* Send data to flash */
        uint32_t wr_size = 64;
        if (len > 64)
        {
            len -= 64;
        }
        else
        {
            wr_size = len;
            len = 0;
        }
        for (i = 0; i < wr_size; i++)
        {
            tmp[buffer_size++] = 0xFF;
        }
        flash_drv->callback.spi_cs(flash_drv->spi, 0);
        flash_drv->callback.spi_tx_buffer(flash_drv->spi, tmp, buffer_size);
        flash_drv->callback.spi_cs(flash_drv->spi, 1);

        addr += wr_size;
        // wait_write_in_process(flash_drv, FLASH_WRITE_TIMEOUT_MS);

#if VERIFY_FLASH
        uint32_t old_addr = addr;
        bool found_error = false;
        for (i = 0; i < wr_size; i++) // Debug only
        {
            uint8_t rb;
            app_spi_flash_read_bytes(flash_drv, old_addr + i, (uint8_t *)&rb, 1);
            if (rb != 0xFF)
            {
                found_error = true;
                DEBUG_ERROR("Flash erase error at addr 0x%08X, readback 0x%02X, expect 0x%02X\r\n", old_addr + i, rb, 0xFF);
                break;
            }
            else
            {
                DEBUG_VERBOSE("Flash erase success at addr 0x%08X, readback 0x%02X, expect 0x%02X\r\n", old_addr + i, rb, 0xFF);
            }
        }
        if (found_error == false)
        {
            DEBUG_VERBOSE("Flash erase success\r\n");
        }
#endif
    }
    flash_drv->callback.spi_cs(flash_drv->spi, 1);
}
