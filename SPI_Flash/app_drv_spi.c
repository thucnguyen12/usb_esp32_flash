/*
 * app_drv_spi.c
 *
 *  Created on: Aug 1, 2021
 *      Author: huybk
 */
#include "app_drv_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "spi.h"

void app_drv_spi_cs(void *spi, bool level);
static SemaphoreHandle_t m_sem_spi = NULL;

static inline void on_spi_done(void)
{
    BaseType_t ctx_sw;
    xSemaphoreGiveFromISR(m_sem_spi, &ctx_sw);
    portYIELD_FROM_ISR(ctx_sw);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    on_spi_done();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    on_spi_done();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    on_spi_done();
}


void app_drv_spi_initialize(void)
{
    if (!m_sem_spi)
    {
        m_sem_spi = xSemaphoreCreateBinary();
    }        
    
}

void app_drv_spi_transmit_frame(void *spi, uint8_t *tx_data, uint32_t length)
{
    if (length && tx_data)
    {
        HAL_SPI_Transmit_DMA(spi, tx_data, length);
        xSemaphoreTake(m_sem_spi, portMAX_DELAY);
    }
}

void app_drv_spi_receive_frame(void *spi, uint8_t *rx_data, uint32_t length)
{
    if (length && rx_data)
    {
        HAL_SPI_Receive_DMA(spi, rx_data, length);
        xSemaphoreTake(m_sem_spi, portMAX_DELAY);
    }
}

void app_drv_spi_transmit_receive_frame(void *spi, uint8_t *tx_data, uint8_t *rx_data, uint32_t length)
{
    if (length && tx_data && rx_data)
    {
        HAL_SPI_TransmitReceive_DMA(spi, tx_data, rx_data, length);
        xSemaphoreTake(m_sem_spi, portMAX_DELAY);
    }
}

uint8_t app_drv_spi_transmit_byte(void *spi, uint8_t data)
{
    uint8_t tmp[2] = {data, 0xFF};
    HAL_SPI_TransmitReceive(spi, tmp, tmp+1, 1, 10);
    return tmp[1];
}

void app_drv_spi_cs(void *spi, bool level)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
