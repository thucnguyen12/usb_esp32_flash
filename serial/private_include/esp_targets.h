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

#ifndef ESP_TARGET_H
#define ESP_TARGET_H

#include <stdint.h>
#include "esp_loader.h"


esp_loader_error_t loader_detect_chip(void *config, target_chip_t *target, const target_registers_t **regs);
esp_loader_error_t loader_read_spi_config(void *config, target_chip_t target_chip, uint32_t *spi_config);
bool encryption_in_begin_flash_cmd(target_chip_t target);

#endif /* ESP_TARGET_H */
