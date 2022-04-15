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

 #pragma once

#include "serial_io.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef struct
{
    uint32_t queue_size;        /*!< Set to zero for default UART queue size */
    QueueHandle_t *uart_queue;  /*!< Set to NULL, if UART queue handle is not  
                                    necessary. Otherwise, it will be assigned here */
} loader_esp32_config_t;

/**
  * @brief Initializes serial interface.
  *
  * @param baud_rate[in]       Communication speed.
  *
  * @return
  *     - ESP_LOADER_SUCCESS Success
  *     - ESP_LOADER_ERROR_FAIL Initialization failure
  */
esp_loader_error_t loader_port_esp32_init(void *config);

/**
  * @brief Deinitialize serial interface.
  */
void loader_port_esp32_deinit(void *config);
