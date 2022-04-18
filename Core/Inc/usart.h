/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
uint32_t usart_logger_put(const void *buffer, uint32_t size);
void usart3_rx_complete_callback(bool status);
void usart3_tx_cplt_cb(void);
void usart_change_baudrate(uint32_t uart_addr, uint32_t baudrate);
uint32_t usart_get_bytes(uint32_t uart_addr, uint8_t *data, uint32_t size, uint32_t timeout);
void usart_send_bytes(uint32_t uart_addr, uint8_t *data, uint32_t size);
uint32_t usart_flush_rx_ringbuffer(uint32_t uart_addr);
void usart3_start_dma_rx(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

