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

#include <stdio.h>
#include <string.h>
// #include <sys/param.h>
#include "serial_io.h"
#include "esp_loader.h"
#include "example_common.h"



#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#ifdef STM32
#include "fatfs.h"
#include "app_debug.h"
#endif

//#ifndef SINGLE_TARGET_SUPPORT

//#define BOOTLOADER_ADDRESS_8266 0x1000
//#define BOOTLOADER_ADDRESS 0x1000
//#define PARTITION_ADDRESS 0x8000
//#define APPLICATION_ADDRESS 0x10000

//extern const uint8_t ESP32_bootloader_bin[];
//extern const uint32_t ESP32_bootloader_bin_size;
//extern const uint8_t ESP32_hello_world_bin[];
//extern const uint32_t ESP32_hello_world_bin_size;
//extern const uint8_t ESP32_partition_table_bin[];
//extern const uint32_t ESP32_partition_table_bin_size;

//extern const uint8_t ESP32_S2_bootloader_bin[];
//extern const uint32_t ESP32_S2_bootloader_bin_size;
//extern const uint8_t ESP32_S2_hello_world_bin[];
//extern const uint32_t ESP32_S2_hello_world_bin_size;
//extern const uint8_t ESP32_S2_partition_table_bin[];
//extern const uint32_t ESP32_S2_partition_table_bin_size;

//extern const uint8_t ESP8266_bootloader_bin[];
//extern const uint32_t ESP8266_bootloader_bin_size;
//extern const uint8_t ESP8266_hello_world_bin[];
//extern const uint32_t ESP8266_hello_world_bin_size;
//extern const uint8_t ESP8266_partition_table_bin[];
//extern const uint32_t ESP8266_partition_table_bin_size;

//void get_example_binaries(void *config, target_chip_t target, example_binaries_t *bins)
//{
//    if (target == ESP8266_CHIP)
//    {
//        bins->boot.data = ESP8266_bootloader_bin;
//        bins->boot.size = ESP8266_bootloader_bin_size;
//        bins->boot.addr = BOOTLOADER_ADDRESS_8266;
//        bins->part.data = ESP8266_partition_table_bin;
//        bins->part.size = ESP8266_partition_table_bin_size;
//        bins->part.addr = PARTITION_ADDRESS;
//        bins->app.data = ESP8266_hello_world_bin;
//        bins->app.size = ESP8266_hello_world_bin_size;
//        bins->app.addr = APPLICATION_ADDRESS;
//    }
//    else if (target == ESP32_CHIP)
//    {
//        bins->boot.data = ESP32_bootloader_bin;
//        bins->boot.size = ESP32_bootloader_bin_size;
//        bins->boot.addr = BOOTLOADER_ADDRESS;
//        bins->part.data = ESP32_partition_table_bin;
//        bins->part.size = ESP32_partition_table_bin_size;
//        bins->part.addr = PARTITION_ADDRESS;
//        bins->app.data = ESP32_hello_world_bin;
//        bins->app.size = ESP32_hello_world_bin_size;
//        bins->app.addr = APPLICATION_ADDRESS;
//    }
//    else
//    {
//        bins->boot.data = ESP32_S2_bootloader_bin;
//        bins->boot.size = ESP32_S2_bootloader_bin_size;
//        bins->boot.addr = BOOTLOADER_ADDRESS;
//        bins->part.data = ESP32_S2_partition_table_bin;
//        bins->part.size = ESP32_S2_partition_table_bin_size;
//        bins->part.addr = PARTITION_ADDRESS;
//        bins->app.data = ESP32_S2_hello_world_bin;
//        bins->app.size = ESP32_S2_hello_world_bin_size;
//        bins->app.addr = APPLICATION_ADDRESS;
//    }
//}

//#endif

//esp_loader_error_t connect_to_target(void *config, uint32_t higher_baudrate)
//{
//    esp_loader_error_t err = esp_loader_connect(config);
//    if (err != ESP_LOADER_SUCCESS)
//    {
//        DEBUG_INFO("Port[%u]  Cannot connect to target. Error: %u\n", ((esp_loader_config_t*)config)->uart_port, err);
//        return err;
//    }
//    DEBUG_INFO("Port[%u]  Connected to target\r\n", ((esp_loader_config_t*)config)->uart_port);

//    if (higher_baudrate && esp_loader_get_target(config) != ESP8266_CHIP)
//    {
//        err = esp_loader_change_baudrate(config, higher_baudrate);
//        if (err == ESP_LOADER_ERROR_UNSUPPORTED_FUNC)
//        {
//            DEBUG_INFO("Port[%u] : ESP8266 does not support change baudrate command\r\n", ((esp_loader_config_t*)config)->uart_port);
//            return err;
//        }
//        else if (err != ESP_LOADER_SUCCESS)
//        {
//            DEBUG_INFO("Port[%u] : Unable to change baud rate on target\r\n", ((esp_loader_config_t*)config)->uart_port);
//            return err;
//        }
//        else
//        {
//            err = loader_port_change_baudrate(config, higher_baudrate);
//            if (err != ESP_LOADER_SUCCESS)
//            {
//                DEBUG_INFO("Port[%u] : Unable to change baud rate\r\n", ((esp_loader_config_t*)config)->uart_port);
//                return err;
//            }
//            DEBUG_INFO("Port[%u] : Baudrate changed\n", ((esp_loader_config_t*)config)->uart_port);
//        }
//    }

//    return ESP_LOADER_SUCCESS;
//}

//esp_loader_error_t flash_binary(void *config, const uint8_t *bin, size_t size, size_t address)
//{
//    esp_loader_error_t err;
//    #warning "Large payload on stack, optimize later"
//    uint8_t payload[1024];
//    uint8_t retry = 3;
//    while (retry)
//    {
//        const uint8_t *bin_addr = bin;
//        retry--;
//        DEBUG_INFO("Port[%u]  Erasing flash (this may take a while)...\r\n", ((esp_loader_config_t*)config)->uart_port);
//        err = esp_loader_flash_start(config, address, size, sizeof(payload));
//        if (err != ESP_LOADER_SUCCESS)
//        {
//            DEBUG_INFO("Port[%u]  Erasing flash failed with error %d.\n", ((esp_loader_config_t*)config)->uart_port, err);
//            return err;
//        }
//        DEBUG_INFO("Port[%u]  Flash erased\r\n", ((esp_loader_config_t*)config)->uart_port);
//        DEBUG_INFO("Port[%u]  Start programming size %u bytes\r\n", ((esp_loader_config_t*)config)->uart_port, size);

//        size_t binary_size = size;
//        size_t written = 0;

//        while (size > 0)
//        {
//            size_t to_read = MIN(size, sizeof(payload));
//            memcpy(payload, bin_addr, to_read);

//            err = esp_loader_flash_write(config, payload, to_read);
//            if (err != ESP_LOADER_SUCCESS)
//            {
//                DEBUG_INFO("\nPort[%u] : Packet could not be written! Error %d\r\n", ((esp_loader_config_t*)config)->uart_port, err);
//                retry--;
//                if (retry == 0)
//                {
//                    return err;
//                }
//                size = binary_size;
//                written = 0;
//                bin_addr = bin;
//            }
//            else
//            {
//                size -= to_read;
//                bin_addr += to_read;
//                written += to_read;

//                int progress = (int)(((float)written / binary_size) * 100);
//                DEBUG_INFO("\rPort[%u] Progress: %d %%", ((esp_loader_config_t*)config)->uart_port, progress);
//                fflush(stdout);
//            }
//        };

//        DEBUG_INFO("\nPort[%u] Finished programming\n", ((esp_loader_config_t*)config)->uart_port);

//    #if MD5_ENABLED
//        err = esp_loader_flash_verify(config);
//        if (err == ESP_LOADER_ERROR_UNSUPPORTED_FUNC)
//        {
//            DEBUG_INFO("Port[%u] ESP8266 does not support flash verify command.\r\n", ((esp_loader_config_t*)config)->uart_port);
//            return err;
//        }
//        else if (err != ESP_LOADER_SUCCESS)
//        {
//            DEBUG_INFO("Port[%u] MD5 does not match. err: %d\n", ((esp_loader_config_t*)config)->uart_port, err);
//            return err;
//        }
//        DEBUG_INFO("Port[%u] Flash verified\n", ((esp_loader_config_t*)config)->uart_port);
//    #endif
//    }

//    return ESP_LOADER_SUCCESS;
//}

esp_loader_error_t flash_binary_stm32(void *config, partition_attr_t *part)
{
    esp_loader_error_t err;
    uint8_t retry = 3;
    uint32_t buffer_size;
    uint8_t *payload = ((esp_loader_config_t*)config)->buffer;
    buffer_size = ((esp_loader_config_t*)config)->buffer_size;
    int last_progress = 0;
    while (retry)
    {
        retry--;
        DEBUG_INFO("Port[%u]  Erasing flash (this may take a while)...\r\n", ((esp_loader_config_t*)config)->uart_addr);
        err = esp_loader_flash_start(config, part->addr, part->size, buffer_size);
        if (err != ESP_LOADER_SUCCESS)
        {
            DEBUG_INFO("Port[%u]  Erasing flash failed with error %d.\n", ((esp_loader_config_t*)config)->uart_addr, err);
            return err;
        }
        DEBUG_INFO("Port[%u]  Flash erased\r\n", ((esp_loader_config_t*)config)->uart_addr);
        DEBUG_INFO("Port[%u]  Start programming size %u bytes\r\n", ((esp_loader_config_t*)config)->uart_addr, part->size);
        
        uint32_t addr = part->addr;
        uint32_t tmp_size = part->size;
        size_t written = 0;

        while (tmp_size > 0)
        {

            size_t to_read = MIN(tmp_size, buffer_size);
            int read = -1;
            read = fatfs_read_file_at_pos(part->file_name, payload, to_read, written);
            if (read <= 0)
            {
                DEBUG_ERROR("Flash rom read file error\r\n");
                retry--;
                if (retry == 0)
                {
                    err = ESP_LOADER_ERROR_FAIL;
                    return err;
                }
                continue;
            }
            // memcpy(payload, bin_addr, to_read);
            
            err = esp_loader_flash_write(config, payload, to_read);
            if (err != ESP_LOADER_SUCCESS)
            {
                DEBUG_INFO("Port[%u] : Packet could not be written! Error %d\r\n", ((esp_loader_config_t*)config)->uart_addr, err);
                retry--;
                if (retry == 0)
                {
                    return err;
                }
                tmp_size = part->size;
                written = 0;
                addr = part->addr;
                continue;
            }
            else
            {
                tmp_size -= to_read;
                addr += to_read;
                written += to_read;

                int progress = (int)(((float)written / part->size) * 100);
                if (progress != last_progress)
                {
                DEBUG_INFO("Port[%u] Progress: %d %%\r\n", ((esp_loader_config_t*)config)->uart_addr, progress);
                last_progress = progress;
                }
                fflush(stdout);
            }
        };

        DEBUG_INFO("Port[%u] Finished programming\r\n", ((esp_loader_config_t*)config)->uart_addr);

    #if MD5_ENABLED
        err = esp_loader_flash_verify(config);
        if (err == ESP_LOADER_ERROR_UNSUPPORTED_FUNC)
        {
            DEBUG_INFO("Port[%u] ESP8266 does not support flash verify command.\r\n", ((esp_loader_config_t*)config)->uart_addr);
            return err;
        }
        else if (err != ESP_LOADER_SUCCESS)
        {
            DEBUG_INFO("Port[%u] MD5 does not match. err: %d\r\n", ((esp_loader_config_t*)config)->uart_addr, err);
            return err;
        }
        DEBUG_INFO("Port[%u] Flash verified\r\n", ((esp_loader_config_t*)config)->uart_addr);
        break;
    #endif
    }
    return ESP_LOADER_SUCCESS;
}

