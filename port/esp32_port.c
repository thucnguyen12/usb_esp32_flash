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

#include "esp32_port.h"
#include "usart.h"
#include "gpio.h"
//#include "esp_timer.h"
//#include "esp_log.h"
#include <unistd.h>
#include "FreeRTOSConfig.h"
#include "task.h"

// #define SERIAL_DEBUG_ENABLE

#ifdef SERIAL_DEBUG_ENABLE

static void dec_to_hex_str(const uint8_t dec, uint8_t hex_str[3])
{
    static const uint8_t dec_to_hex[] = {
        '0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

    hex_str[0] = dec_to_hex[dec >> 4];
    hex_str[1] = dec_to_hex[dec & 0xF];
    hex_str[2] = '\0';
}

static void serial_debug_print(void *config, const uint8_t *data, uint16_t size, bool write)
{
    uint8_t hex_str[3];

    if (((esp_loader_config_t*)config)->write_prev != write)
    {
        ((esp_loader_config_t*)config)->write_prev = write;
        printf("\n--- [%u]%s ---\n", ((esp_loader_config_t*)config)->uart_port, write ? "WRITE" : "READ");
    }

    for (uint32_t i = 0; i < size; i++)
    {
        dec_to_hex_str(data[i], hex_str);
        printf("%s ", hex_str);
    }
}

#else

static void serial_debug_print(void *config, const uint8_t *data, uint16_t size, bool write) {}

#endif

esp_loader_error_t loader_port_esp32_init(void *config)
{
    // Initialize UART
    uart_config_t uart_config = 
    {
        .baud_rate = ((esp_loader_config_t*)config)->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    int rx_buffer_size = ((esp_loader_config_t*)config)->rx_buffer_size ? ((esp_loader_config_t*)config)->rx_buffer_size : 400;
    int tx_buffer_size = ((esp_loader_config_t*)config)->tx_buffer_size ? ((esp_loader_config_t*)config)->tx_buffer_size : 400;
    QueueHandle_t *uart_queue = ((esp_loader_config_t*)config)->uart_queue ? ((esp_loader_config_t*)config)->uart_queue : NULL;
    int queue_size = ((esp_loader_config_t*)config)->queue_size ? ((esp_loader_config_t*)config)->queue_size : 0;

    if (uart_param_config(((esp_loader_config_t*)config)->uart_port, &uart_config) != ESP_OK)
    {
        return ESP_LOADER_ERROR_FAIL;
    }
    if (uart_set_pin(((esp_loader_config_t*)config)->uart_port, 
                    ((esp_loader_config_t*)config)->uart_tx_pin, 
                    ((esp_loader_config_t*)config)->uart_rx_pin, 
                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        return ESP_LOADER_ERROR_FAIL;
    }
    if (uart_driver_install(((esp_loader_config_t*)config)->uart_port, 
                            rx_buffer_size, tx_buffer_size, 
                            queue_size, uart_queue, 0) != ESP_OK)
    {
        return ESP_LOADER_ERROR_FAIL;
    }

    // Initialize boot pin selection pins
    gpio_reset_pin(((esp_loader_config_t*)config)->reset_trigger_pin);
    gpio_set_pull_mode(((esp_loader_config_t*)config)->reset_trigger_pin, GPIO_PULLUP_ONLY);
    gpio_set_direction(((esp_loader_config_t*)config)->reset_trigger_pin, GPIO_MODE_OUTPUT);

    gpio_reset_pin(((esp_loader_config_t*)config)->gpio0_trigger_pin);
    gpio_set_pull_mode(((esp_loader_config_t*)config)->gpio0_trigger_pin, GPIO_PULLUP_ONLY);
    gpio_set_direction(((esp_loader_config_t*)config)->gpio0_trigger_pin, GPIO_MODE_OUTPUT);

    return ESP_LOADER_SUCCESS;
}

void loader_port_esp32_deinit(void *config)
{
    uart_driver_delete(((esp_loader_config_t*)config)->uart_port);
}

esp_loader_error_t loader_port_serial_write(void *config, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    serial_debug_print(((esp_loader_config_t*)config), data, size, true);

    uart_write_bytes(((esp_loader_config_t*)config)->uart_port, (const char *)data, size);
    esp_err_t err = uart_wait_tx_done(((esp_loader_config_t*)config)->uart_port, pdMS_TO_TICKS(timeout));

    if (err == ESP_OK)
    {
        return ESP_LOADER_SUCCESS;
    }
    else if (err == ESP_ERR_TIMEOUT)
    {
        return ESP_LOADER_ERROR_TIMEOUT;
    }
    else
    {
        return ESP_LOADER_ERROR_FAIL;
    }
}

esp_loader_error_t loader_port_serial_read(void *config, uint8_t *data, uint16_t size, uint32_t timeout)
{
    int read = uart_read_bytes(((esp_loader_config_t*)config)->uart_port, data, size, pdMS_TO_TICKS(timeout));

    serial_debug_print(((esp_loader_config_t*)config), data, read, false);

    if (read < 0)
    {
        return ESP_LOADER_ERROR_FAIL;
    }
    else if (read < size)
    {
        return ESP_LOADER_ERROR_TIMEOUT;
    }
    else
    {
        return ESP_LOADER_SUCCESS;
    }
}

// Set GPIO0 LOW, then
// assert reset pin for 50 milliseconds.
void loader_port_enter_bootloader(void *config)
{
    gpio_set_level(((esp_loader_config_t*)config)->gpio0_trigger_pin, 0);
    loader_port_reset_target(config);
    loader_port_delay_ms(((esp_loader_config_t*)config), 50);
    gpio_set_level(((esp_loader_config_t*)config)->reset_trigger_pin, 1);
}

void loader_port_reset_target(void *config)
{
    gpio_set_level(((esp_loader_config_t*)config)->reset_trigger_pin, 0);
    loader_port_delay_ms(((esp_loader_config_t*)config), 50);
    gpio_set_level(((esp_loader_config_t*)config)->reset_trigger_pin, 1);
}

void loader_port_delay_ms(void *config, uint32_t ms)
{
    (void)config;
    vTaskDelay(ms);
}

void loader_port_start_timer(void *config, uint32_t ms)
{
    ((esp_loader_config_t*)config)->time_end = xTaskGetTickCount() + ms;
}

uint32_t loader_port_remaining_time(void *config)
{
    int64_t remaining = (((esp_loader_config_t*)config)->time_end - xTaskGetTickCount());
    return (remaining > 0) ? (uint32_t)remaining : 0;
}

void loader_port_debug_print(void *config, const char *str)
{
    printf("[%u] DEBUG: %s\n", ((esp_loader_config_t*)config)->uart_port, str);
}

esp_loader_error_t loader_port_change_baudrate(void *config, uint32_t baudrate)
{
    esp_err_t err = uart_set_baudrate(((esp_loader_config_t*)config)->uart_port, baudrate);
    return (err == ESP_OK) ? ESP_LOADER_SUCCESS : ESP_LOADER_ERROR_FAIL;
}
