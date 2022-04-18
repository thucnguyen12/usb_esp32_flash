/* Copyright 2020 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "serial_comm_prv.h"
#include "serial_comm.h"
#include "serial_io.h"
#include "esp_loader.h"
#include "esp_targets.h"
#include "md5_hash.h"
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "app_debug.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b)) ? (a) : (b)
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b)) ? (a) : (b)
#endif

static const uint32_t DEFAULT_TIMEOUT = 1000;
static const uint32_t DEFAULT_FLASH_TIMEOUT = 3000;        // timeout for most flash operations
static const uint32_t ERASE_REGION_TIMEOUT_PER_MB = 10000; // timeout (per megabyte) for erasing a region
static const uint8_t PADDING_PATTERN = 0xFF;

typedef enum
{
    SPI_FLASH_READ_ID = 0x9F
} spi_flash_cmd_t;

#if MD5_ENABLED

static const uint32_t MD5_TIMEOUT_PER_MB = 800;
static inline void init_md5(void *config, uint32_t address, uint32_t size)
{
    ((esp_loader_config_t*)config)->start_address = address;
    ((esp_loader_config_t*)config)->image_size = size;
    struct MD5Context *ctx = ((esp_loader_config_t*)config)->md5_context;
    MD5Init(ctx);
}

static inline void md5_update(void *config, const uint8_t *data, uint32_t size)
{
    struct MD5Context *ctx = ((esp_loader_config_t*)config)->md5_context;
    MD5Update(ctx, data, size);
}

static inline void md5_final(void *config, uint8_t digets[16])
{
    struct MD5Context *ctx = ((esp_loader_config_t*)config)->md5_context;
    MD5Final(digets, ctx);
}

#else

static inline void init_md5(void *config, uint32_t address, uint32_t size)
{
}
static inline void md5_update(void *config, const uint8_t *data, uint32_t size) {}
static inline void md5_final(void *config, uint8_t digets[16]) {}

#endif

static uint32_t timeout_per_mb(uint32_t size_bytes, uint32_t time_per_mb)
{
    uint32_t timeout = time_per_mb * (size_bytes / 1e6);
    return MAX(timeout, DEFAULT_FLASH_TIMEOUT);
}

esp_loader_error_t esp_loader_connect(void *config)
{
    uint32_t spi_config;
    esp_loader_error_t err;
    int32_t trials = ((esp_loader_config_t*)config)->trials;
    
    loader_port_enter_bootloader(config);

    do
    {
        loader_port_start_timer(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->sync_timeout);
        err = loader_sync_cmd(config);
        if (err == ESP_LOADER_ERROR_TIMEOUT)
        {
            if (--trials == 0)
            {
            	DEBUG_INFO("ESP TIME OUT\r\n");
                return ESP_LOADER_ERROR_TIMEOUT;
            }
            loader_port_delay_ms(((esp_loader_config_t*)config), 100);
        }
        else if (err != ESP_LOADER_SUCCESS)
        {
            return err;
        }
    } while (err != ESP_LOADER_SUCCESS);
    
    RETURN_ON_ERROR(loader_detect_chip(config, &((esp_loader_config_t*)config)->target, &((esp_loader_config_t*)config)->reg));
    if (((esp_loader_config_t*)config)->target == ESP8266_CHIP)
    {
        err = loader_flash_begin_cmd(((esp_loader_config_t*)config), 0, 0, 0, 0, ((esp_loader_config_t*)config)->target);
    }
    else
    {
        RETURN_ON_ERROR(loader_read_spi_config(config, ((esp_loader_config_t*)config)->target, &spi_config));
        loader_port_start_timer(((esp_loader_config_t*)config), DEFAULT_TIMEOUT);
        err = loader_spi_attach_cmd(((esp_loader_config_t*)config), spi_config);
    }

    return err;
}

target_chip_t esp_loader_get_target(void *config)
{
    return ((esp_loader_config_t*)config)->target;
}

static esp_loader_error_t spi_set_data_lengths(void *config, size_t mosi_bits, size_t miso_bits)
{
    if (mosi_bits > 0)
    {
        RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->mosi_dlen, mosi_bits - 1));
    }
    if (miso_bits > 0)
    {
        RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->miso_dlen, miso_bits - 1));
    }

    return ESP_LOADER_SUCCESS;
}

static esp_loader_error_t spi_set_data_lengths_8266(void *config, size_t mosi_bits, size_t miso_bits)
{
    uint32_t mosi_mask = (mosi_bits == 0) ? 0 : mosi_bits - 1;
    uint32_t miso_mask = (miso_bits == 0) ? 0 : miso_bits - 1;
    return esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->usr1, (miso_mask << 8) | (mosi_mask << 17));
}

static esp_loader_error_t spi_flash_command(void *config, spi_flash_cmd_t cmd, 
                                            void *data_tx, size_t tx_size, void *data_rx, size_t rx_size)
{
    RETURN_ON_ERROR(rx_size <= 32); // Reading more than 32 bits back from a SPI flash operation is unsupported
    RETURN_ON_ERROR(tx_size <= 64); // Writing more than 64 bytes of data with one SPI command is unsupported

    uint32_t SPI_USR_CMD = (((uint32_t)1) << 31);
    uint32_t SPI_USR_MISO = (1 << 28);
    uint32_t SPI_USR_MOSI = (1 << 27);
    uint32_t SPI_CMD_USR = (1 << 18);
    uint32_t CMD_LEN_SHIFT = 28;

    // Save SPI configuration
    uint32_t old_spi_usr;
    uint32_t old_spi_usr2;
    RETURN_ON_ERROR(esp_loader_read_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->usr, &old_spi_usr));
    RETURN_ON_ERROR(esp_loader_read_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->usr2, &old_spi_usr2));

    if (((esp_loader_config_t*)config)->target == ESP8266_CHIP)
    {
        RETURN_ON_ERROR(spi_set_data_lengths_8266(((esp_loader_config_t*)config), tx_size, rx_size));
    }
    else
    {
        RETURN_ON_ERROR(spi_set_data_lengths(((esp_loader_config_t*)config), tx_size, rx_size));
    }

    uint32_t usr_reg_2 = (7 << CMD_LEN_SHIFT) | cmd;
    uint32_t usr_reg = SPI_USR_CMD;
    if (rx_size > 0)
    {
        usr_reg |= SPI_USR_MISO;
    }
    if (tx_size > 0)
    {
        usr_reg |= SPI_USR_MOSI;
    }

    RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->usr, usr_reg));
    RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->usr2, usr_reg_2));

    if (tx_size == 0)
    {
        // clear data register before we read it
        RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->w0, 0));
    }
    else
    {
        uint32_t *data = (uint32_t *)data_tx;
        uint32_t words_to_write = (tx_size + 31) / (8 * 4);
        uint32_t data_reg_addr = ((esp_loader_config_t*)config)->reg->w0;

        while (words_to_write--)
        {
            uint32_t word = *data++;
            RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), data_reg_addr, word));
            data_reg_addr += 4;
        }
    }

    RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->cmd, SPI_CMD_USR));

    uint32_t trials = 10;
    while (trials--)
    {
        uint32_t cmd_reg;
        RETURN_ON_ERROR(esp_loader_read_register(config, ((esp_loader_config_t*)config)->reg->cmd, &cmd_reg));
        if ((cmd_reg & SPI_CMD_USR) == 0)
        {
            break;
        }
    }

    if (trials == 0)
    {
        return ESP_LOADER_ERROR_TIMEOUT;
    }

    RETURN_ON_ERROR(esp_loader_read_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->w0, data_rx));

    // Restore SPI configuration
    RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->usr, old_spi_usr));
    RETURN_ON_ERROR(esp_loader_write_register(((esp_loader_config_t*)config), ((esp_loader_config_t*)config)->reg->usr2, old_spi_usr2));

    return ESP_LOADER_SUCCESS;
}

static esp_loader_error_t detect_flash_size(void *config, size_t *flash_size)
{
    uint32_t flash_id = 0;

    RETURN_ON_ERROR(spi_flash_command(((esp_loader_config_t*)config), SPI_FLASH_READ_ID, NULL, 0, &flash_id, 24));
    uint32_t size_id = flash_id >> 16;

    if (size_id < 0x12 || size_id > 0x18)
    {
        return ESP_LOADER_ERROR_UNSUPPORTED_CHIP;
    }

    *flash_size = 1 << size_id;

    return ESP_LOADER_SUCCESS;
}

esp_loader_error_t esp_loader_flash_start(void *config, uint32_t offset, uint32_t image_size, uint32_t block_size)
{
    uint32_t blocks_to_write = (image_size + block_size - 1) / block_size;
    uint32_t erase_size = block_size * blocks_to_write;
    ((esp_loader_config_t*)config)->flash_write_size = block_size;

    size_t flash_size = 0;
    if (detect_flash_size(((esp_loader_config_t*)config), &flash_size) == ESP_LOADER_SUCCESS)
    {
        if (image_size > flash_size)
        {
            return ESP_LOADER_ERROR_IMAGE_SIZE;
        }
        loader_port_start_timer(((esp_loader_config_t*)config), DEFAULT_TIMEOUT);
        RETURN_ON_ERROR(loader_spi_parameters(((esp_loader_config_t*)config), flash_size));
    }
    else
    {
        loader_port_debug_print(config, "Flash size detection failed, falling back to default\r\n");
    }

    init_md5(((esp_loader_config_t*)config), offset, image_size);

    bool encryption_in_cmd = encryption_in_begin_flash_cmd(((esp_loader_config_t*)config)->target);

    loader_port_start_timer(((esp_loader_config_t*)config), timeout_per_mb(erase_size, ERASE_REGION_TIMEOUT_PER_MB));
    return loader_flash_begin_cmd(((esp_loader_config_t*)config), offset, erase_size, block_size, blocks_to_write, encryption_in_cmd);
}

esp_loader_error_t esp_loader_flash_write(void *config, void *payload, uint32_t size)
{
    uint32_t padding_bytes = ((esp_loader_config_t*)config)->flash_write_size - size;
    uint8_t *data = (uint8_t *)payload;
    uint32_t padding_index = size;

    while (padding_bytes--)
    {
        data[padding_index++] = PADDING_PATTERN;
    }

    md5_update(config, payload, (size + 3) & ~3);

    loader_port_start_timer(((esp_loader_config_t*)config), DEFAULT_TIMEOUT);

    return loader_flash_data_cmd(((esp_loader_config_t*)config), data, ((esp_loader_config_t*)config)->flash_write_size);
}

esp_loader_error_t esp_loader_flash_finish(void *config, bool reboot)
{
    loader_port_start_timer(((esp_loader_config_t*)config), DEFAULT_TIMEOUT);

    return loader_flash_end_cmd(((esp_loader_config_t*)config), !reboot);
}

esp_loader_error_t esp_loader_read_register(void *config, uint32_t address, uint32_t *reg_value)
{
    loader_port_start_timer(((esp_loader_config_t*)config), DEFAULT_TIMEOUT);

    return loader_read_reg_cmd(((esp_loader_config_t*)config), address, reg_value);
}

esp_loader_error_t esp_loader_write_register(void *config, uint32_t address, uint32_t reg_value)
{
    loader_port_start_timer(((esp_loader_config_t*)config), DEFAULT_TIMEOUT);

    return loader_write_reg_cmd(((esp_loader_config_t*)config), address, reg_value, 0xFFFFFFFF, 0);
}

esp_loader_error_t esp_loader_change_baudrate(void *config, uint32_t baudrate)
{
    if (((esp_loader_config_t*)config)->target == ESP8266_CHIP)
    {
        return ESP_LOADER_ERROR_UNSUPPORTED_FUNC;
    }

    loader_port_start_timer(((esp_loader_config_t*)config), DEFAULT_TIMEOUT);

    return loader_change_baudrate_cmd(((esp_loader_config_t*)config), baudrate);
}

#if MD5_ENABLED

static void hexify(const uint8_t raw_md5[16], uint8_t hex_md5_out[32])
{
    static const uint8_t dec_to_hex[] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    for (int i = 0; i < 16; i++)
    {
        *hex_md5_out++ = dec_to_hex[raw_md5[i] >> 4];
        *hex_md5_out++ = dec_to_hex[raw_md5[i] & 0xF];
    }
}

esp_loader_error_t esp_loader_flash_verify(void *config)
{
    if (((esp_loader_config_t*)config)->target == ESP8266_CHIP)
    {
        return ESP_LOADER_ERROR_UNSUPPORTED_FUNC;
    }

    uint8_t raw_md5[16] = {0};

    /* Zero termination and new line character require 2 bytes */
    uint8_t hex_md5[MD5_SIZE + 2] = {0};
    uint8_t received_md5[MD5_SIZE + 2] = {0};

    md5_final(config, raw_md5);
    hexify(raw_md5, hex_md5);

    loader_port_start_timer(((esp_loader_config_t*)config), timeout_per_mb(((esp_loader_config_t*)config)->image_size, MD5_TIMEOUT_PER_MB));

    RETURN_ON_ERROR(loader_md5_cmd(((esp_loader_config_t*)config), 
                                ((esp_loader_config_t*)config)->start_address, 
                                ((esp_loader_config_t*)config)->image_size, received_md5));

    bool md5_match = memcmp(hex_md5, received_md5, MD5_SIZE) == 0;

    if (!md5_match)
    {
        hex_md5[MD5_SIZE] = '\n';
        received_md5[MD5_SIZE] = '\n';

        loader_port_debug_print(config, "Error: MD5 checksum does not match:\n");
        loader_port_debug_print(config, "Expected:\n");
        loader_port_debug_print(config, (char *)received_md5);
        loader_port_debug_print(config, "Actual:\n");
        loader_port_debug_print(config, (char *)hex_md5);

        return ESP_LOADER_ERROR_INVALID_MD5;
    }

    return ESP_LOADER_SUCCESS;
}

#endif

void esp_loader_reset_target(void *config)
{
    loader_port_reset_target(config);
}
