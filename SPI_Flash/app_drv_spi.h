/*
 * app_drv_spi.h
 *
 *  Created on: Aug 1, 2021
 *      Author: huybk
 */

#ifndef APP_DRV_SPI_H_
#define APP_DRV_SPI_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief		Initialize spi driver
 */
void app_drv_spi_initialize(void);

/**
 * @brief		Send a spi frame
 * @param[in]	spi SPI peripheral
 * @param[in]	tx_data Data send to spi port
 * @param[in] 	length Data length
 */
void app_drv_spi_transmit_frame(void *spi, uint8_t *tx_data, uint32_t length);

/**
 * @brief		Received a spi frame
 * @param[in]	spi SPI peripheral
 * @param[in]	rx_data Buffer hold rx data
 * @param[in] 	length Data length
 */
void app_drv_spi_receive_frame(void *spi, uint8_t *rx_data, uint32_t length);

/**
 * @brief		Transmit and received a spi frame
 * @param[in]	spi SPI peripheral
 * @param[in]	tx_data Buffer hold tx data
 * @param[in]	rx_data Buffer hold rx data
 * @param[in] 	length Data length
 */
void app_drv_spi_transmit_receive_frame(void *spi, uint8_t *tx_data, uint8_t *rx_data, uint32_t length);

/**
 * @brief		Transmit and received 1 bytes
 * @param[in]	data Bytes data
 * @retval	 	received data
 */
uint8_t app_drv_spi_transmit_byte(void *spi, uint8_t data);

/**
 * @brief		Set cs level
 * @param[in]	spi SPI peripheral
 * @param[in]	level CS level
 */
void app_drv_spi_cs(void *spi, bool level);

/**
 * @brief		Get HSPI handle
 * @retval		HSPI handle
 */
void *app_drv_spi_handle(void);

#endif /* APP_DRV_SPI_H_ */
