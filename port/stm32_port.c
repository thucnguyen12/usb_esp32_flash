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

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32_port.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "freertos.h"
#include "task.h"
#include "usart.h"
#include "app_debug.h"
// #define SERIAL_DEBUG_ENABLE

//static UART_HandleTypeDef *uart;
//static GPIO_TypeDef* gpio_port_io0, *gpio_port_rst;
//static uint16_t gpio_num_io0, gpio_num_rst;


#ifdef SERIAL_DEBUG_ENABLE

static void dec_to_hex_str(const uint8_t dec, uint8_t hex_str[3])
{
    static const uint8_t dec_to_hex[] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
    };

    hex_str[0] = dec_to_hex[(dec >> 4)];
    hex_str[1] = dec_to_hex[(dec & 0xF)];
    hex_str[2] = '\0';
}

static void serial_debug_print(const uint8_t *data, uint16_t size, bool write)
{
    static bool write_prev = false;
    uint8_t hex_str[3];

    if(write_prev != write) {
        write_prev = write;
        printf("\n--- %s ---\n", write ? "WRITE" : "READ");
    }

    for(uint32_t i = 0; i < size; i++) {
        dec_to_hex_str(data[i], hex_str);
        printf("%s ", hex_str);
    }
}

#else

static void serial_debug_print(const uint8_t *data, uint16_t size, bool write) 
{
    
}

#endif


esp_loader_error_t loader_port_serial_write(void *config, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    serial_debug_print(data, size, true);
    (void)timeout;

    usart_send_bytes(tmp->uart_addr, (uint8_t *)data, size);
    return ESP_LOADER_SUCCESS;
}


esp_loader_error_t loader_port_serial_read(void *config, uint8_t *data, uint16_t size, uint32_t timeout)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    
    memset(data, 0x22, size);

    HAL_StatusTypeDef err = (HAL_StatusTypeDef)usart_get_bytes(tmp->uart_addr, data, size, timeout);

    serial_debug_print(data, size, false);

    if (err == HAL_OK) 
    {
        return ESP_LOADER_SUCCESS;
    } 
    else if (err == HAL_TIMEOUT) 
    {
        return ESP_LOADER_ERROR_TIMEOUT;
    } 
    else 
    {
        return ESP_LOADER_ERROR_FAIL;
    }
}


void loader_port_serial_flush(void *config)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    usart_flush_rx_ringbuffer(tmp->uart_addr);
}

void loader_port_stm32_init(esp_loader_config_t *config)
{
//    uart = config->huart;
//    gpio_port_io0 = config->port_io0; 
//    gpio_port_rst = config->port_rst;
//    gpio_num_io0 = config->pin_num_io0;
//    gpio_num_rst = config->pin_num_rst;
}

// Set GPIO0 LOW, then
// assert reset pin for 100 milliseconds.
void loader_port_enter_bootloader(void *config)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    HAL_GPIO_WritePin((GPIO_TypeDef*)tmp->reset_trigger_port, tmp->reset_trigger_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin((GPIO_TypeDef*)tmp->gpio0_trigger_port, tmp->gpio0_trigger_pin, GPIO_PIN_RESET);
    uint32_t now = xTaskGetTickCount();
    vTaskDelayUntil(&now, 2);
    HAL_GPIO_WritePin((GPIO_TypeDef*)tmp->reset_trigger_port, tmp->reset_trigger_pin, GPIO_PIN_SET);
    vTaskDelayUntil(&now, 100);
}


void loader_port_reset_target(void *config)
{
    uint32_t now = xTaskGetTickCount();
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    HAL_GPIO_WritePin((GPIO_TypeDef*)tmp->reset_trigger_port, tmp->reset_trigger_pin, GPIO_PIN_RESET);
    vTaskDelayUntil(&now, 200);
    HAL_GPIO_WritePin((GPIO_TypeDef*)tmp->reset_trigger_port, tmp->reset_trigger_pin, GPIO_PIN_SET);
}


void loader_port_delay_ms(void *config, uint32_t ms)
{
    uint32_t now = xTaskGetTickCount();
    vTaskDelayUntil(&now, ms);
}


void loader_port_start_timer(void *config, uint32_t ms)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    tmp->time_end = xTaskGetTickCount() + ms;
}


uint32_t loader_port_remaining_time(void *config)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    int32_t remaining = tmp->time_end - xTaskGetTickCount();
    return (remaining > 0) ? (uint32_t)remaining : 0;
}


void loader_port_debug_print(void *config, const char *str)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    DEBUG_INFO("[0x%08X] DEBUG: %s\r\n", tmp->uart_addr, str);
}

esp_loader_error_t loader_port_change_baudrate(void *config, uint32_t baudrate)
{
    esp_loader_config_t *tmp = (esp_loader_config_t*)config;
    tmp->baud_rate = baudrate;
    usart_change_baudrate(tmp->uart_addr, tmp->baud_rate);
    return ESP_LOADER_SUCCESS;
}
