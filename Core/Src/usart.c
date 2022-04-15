/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>
#include "lwrb.h"
#include "app_debug.h"
#include "FreeRTOSConfig.h"
#include "freertos.h"
#include "task.h"
#include "semphr.h"

#define USART3_RX_RING_BUFFER_SIZE    1024
#define USART3_RX_DMA_BUFFER_SIZE     256

static inline void usart3_hw_uart_rx_raw(uint8_t *data, uint32_t length);
uint8_t m_usart3_rx_buffer[USART3_RX_RING_BUFFER_SIZE];
__ALIGNED(4) uint8_t m_usart3_rx_dma_buffer[USART3_RX_DMA_BUFFER_SIZE];
volatile uint32_t m_last_usart3_transfer_size = 0;
static volatile size_t m_old_usart3_dma_rx_pos;
static uint32_t m_last_usart3_baud = 115200;
lwrb_t m_ringbuffer_usart3_rx;
static volatile bool m_usart3_rx_ongoing = false;
static SemaphoreHandle_t m_sem_uart3_tx;
/* USER CODE END 0 */

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 DMA Init */

  /* USART3_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_1, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

  /* USART3_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_3, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* USER CODE BEGIN 1 */
static void usart3_transmit_dma(uint8_t *data, uint32_t size)
{
    LL_DMA_ConfigTransfer(DMA1, LL_DMA_STREAM_4,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                        LL_DMA_PRIORITY_HIGH              |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_4,
                         (uint32_t)data,
                         LL_USART_DMA_GetRegAddr(USART3),
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_4));
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, size);

      /* Enable DMA TX Interrupt */
    LL_USART_EnableDMAReq_TX(USART3);

    /* Enable DMA Channel Tx */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
}

uint32_t usart_get_bytes(uint32_t uart_addr, uint8_t *data, uint32_t size, uint32_t timeout)
{
    if ((USART_TypeDef*)uart_addr == USART3)
    {
        while (lwrb_get_full(&m_ringbuffer_usart3_rx) < size && timeout)
        {
            uint32_t now = xTaskGetTickCount();
            timeout--;
            vTaskDelayUntil(&now, 1);
    //        vTaskDelay(1);
        }

        if (lwrb_get_full(&m_ringbuffer_usart3_rx) < size)
        {
            // lwrb_reset(&m_ringbuffer_usart3_rx);
    //         DEBUG_ERROR("Serial read timeout, expected %u/%u\r\n", size, lwrb_get_full(&m_ringbuffer_usart3_rx));
            return HAL_TIMEOUT;
        }
        lwrb_read(&m_ringbuffer_usart3_rx, data, size);
    }
    return HAL_OK;
}

uint32_t usart_flush_rx_ringbuffer(uint32_t uart_addr)
{
    if ((USART_TypeDef*)uart_addr == USART3)
    {
        lwrb_reset(&m_ringbuffer_usart3_rx);
    }
    return HAL_OK;
}

void usart_send_bytes(uint32_t uart_addr, uint8_t *data, uint32_t size)
{
    DEBUG_VERBOSE("Send frame\r\n");
    if ((USART_TypeDef*)uart_addr == USART3)
    {
        usart3_transmit_dma(data, size);
        xSemaphoreTake(m_sem_uart3_tx, portMAX_DELAY);
    }
}

void usart3_tx_cplt_cb(void)
{
//    DEBUG_ISR("TX cplt\r\n");
    BaseType_t ctx_sw;
    xSemaphoreGiveFromISR(m_sem_uart3_tx, &ctx_sw);
    portYIELD_FROM_ISR(ctx_sw);
}

void usart_change_baudrate(uint32_t uart_addr, uint32_t baudrate)
{
    if ((USART_TypeDef*)uart_addr == USART3)
    {
        if (m_last_usart3_baud != baudrate)
        {
            DEBUG_INFO("USART3 change baudrate to %ubps\r\n", baudrate);
            m_last_usart3_baud = baudrate;

            // TX
            LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_4);
            LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_4);

            // RX
            LL_DMA_DisableIT_HT(DMA1, LL_DMA_STREAM_1);
            LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_1);
            LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_1);

            LL_DMA_ClearFlag_TC4(DMA1);
            LL_DMA_ClearFlag_HT4(DMA1);

            LL_DMA_ClearFlag_TE4(DMA1);

            NVIC_DisableIRQ(USART3_IRQn);
            LL_USART_Disable(USART3);
            /* Peripheral clock enable */
            m_old_usart3_dma_rx_pos = 0;
            MX_USART3_UART_Init();
        }
    }
}

static inline void usart3_hw_uart_rx_raw(uint8_t *data, uint32_t length)
{
    NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    if (!m_usart3_rx_ongoing)
    {
        m_usart3_rx_ongoing = true;
        NVIC_EnableIRQ(DMA1_Stream1_IRQn);

        /* Enable DMA Channel Rx */
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

        LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1,
                              (uint32_t)&(USART3->DR),
                             (uint32_t)data,
                             LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, length);

        /* Enable DMA RX Interrupt */
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
        LL_USART_EnableDMAReq_RX(USART3);

    }
    else
    {
        NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    }
}

void usart3_start_dma_rx()
{
    usart3_hw_uart_rx_raw(m_usart3_rx_dma_buffer, sizeof(m_usart3_rx_dma_buffer));
}


void usart3_rx_complete_callback(bool status)
{
    if (status)
    {
        // DEBUG_ISR("RX cplt cb\r\n");
        size_t pos;

        /* Calculate current position in buffer */
        pos = USART3_RX_DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1);
        if (pos != m_old_usart3_dma_rx_pos)
        {
            /* Check change in received data */
            uint32_t bytes_write;
            if (pos > m_old_usart3_dma_rx_pos)
            {   /* Current position is over previous one */
                /* We are in "linear" mode */
                /* Process data directly by subtracting "pointers" */
//                DEBUG_RAW("%.*s", pos - m_old_usart3_dma_rx_pos, &m_usart3_rx_dma_buffer[m_old_usart3_dma_rx_pos]);
                bytes_write = pos-m_old_usart3_dma_rx_pos;
				if (bytes_write != lwrb_write(&m_ringbuffer_usart3_rx, &m_usart3_rx_dma_buffer[m_old_usart3_dma_rx_pos], bytes_write))
                {
                    lwrb_reset(&m_ringbuffer_usart3_rx);
                    DEBUG_ISR("Ringbuffer full\r\n");
                }
            }
            else
            {
                /* We are in "overflow" mode */
                /* First process data to the end of buffer */
                /* Check and continue with beginning of buffer */
                bytes_write = USART3_RX_DMA_BUFFER_SIZE - m_old_usart3_dma_rx_pos;
                if (bytes_write != lwrb_write(&m_ringbuffer_usart3_rx, &m_usart3_rx_dma_buffer[m_old_usart3_dma_rx_pos], bytes_write))
                {
                    lwrb_reset(&m_ringbuffer_usart3_rx);
                    DEBUG_ISR("Ringbuffer full\r\n");
                }

                if (pos > 0)
                {
                    if (pos != lwrb_write(&m_ringbuffer_usart3_rx, &m_usart3_rx_dma_buffer[0], pos))
                    {
                        lwrb_reset(&m_ringbuffer_usart3_rx);
                        DEBUG_ISR("Ringbuffer full\r\n");
                    }
                }
                bytes_write += pos;
            }
            m_old_usart3_dma_rx_pos = pos;                          /* Save current position as old */
            // DEBUG_ISR("Fill %u bytes\r\n", bytes_write);
        }
//        m_usart3_rx_ongoing = false;
    }
    else
    {
        m_usart3_rx_ongoing = false;
    }
}
/* USER CODE END 1 */
