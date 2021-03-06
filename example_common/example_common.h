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

#ifndef EXAMPLE_COMMON_H
#define EXAMPLE_COMMON_H

#include <stdint.h>
#include "esp_loader.h"

#define EXAMPLE_COMMON_USE_SDCARD   1


typedef struct 
{
    const uint8_t *data;
    uint32_t size;
    uint32_t addr;
#if EXAMPLE_COMMON_USE_SDCARD
    const char *file_name;
#endif
} partition_attr_t;

typedef struct {
    partition_attr_t boot;
    partition_attr_t part;
    partition_attr_t app;
} example_binaries_t;

void get_example_binaries(void *config, target_chip_t target, example_binaries_t *binaries);
esp_loader_error_t connect_to_target(void *config, uint32_t higrer_baudrate);
esp_loader_error_t flash_binary(void *config, const uint8_t *bin, size_t size, size_t address);
esp_loader_error_t flash_binary_stm32(void *config, partition_attr_t *part);

#endif /* EXAMPLE_COMMON_H */
