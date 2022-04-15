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

#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_loader.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_loader_error_t loader_flash_begin_cmd(void *config, uint32_t offset, uint32_t erase_size, uint32_t block_size, uint32_t blocks_to_write, bool encryption);

esp_loader_error_t loader_flash_data_cmd(void *config, const uint8_t *data, uint32_t size);

esp_loader_error_t loader_flash_end_cmd(void *config, bool stay_in_loader);

esp_loader_error_t loader_write_reg_cmd(void *config, uint32_t address, uint32_t value, uint32_t mask, uint32_t delay_us);

esp_loader_error_t loader_read_reg_cmd(void *config, uint32_t address, uint32_t *reg);

esp_loader_error_t loader_sync_cmd(void *config);

esp_loader_error_t loader_spi_attach_cmd(void *loader_configuration, uint32_t config);

esp_loader_error_t loader_change_baudrate_cmd(void *config, uint32_t baudrate);

esp_loader_error_t loader_md5_cmd(void *config, uint32_t address, uint32_t size, uint8_t *md5_out);

esp_loader_error_t loader_spi_parameters(void *config, uint32_t total_size);

void loader_flush_rx_data(void *config);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_COM_H */
