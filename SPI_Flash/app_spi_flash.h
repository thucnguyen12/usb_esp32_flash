#ifndef APP_SPI_FLASH_H
#define APP_SPI_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#define APP_SPI_FLASH_DATA_HEADER_KEY                   0x9813567A
#define APP_SPI_FLASH_PAGE_SIZE 						256
#define APP_SPI_FLASH_SECTOR_SIZE 						4096
#define APP_SPI_FLASH_BLOCK_SIZE						65536
#define APP_SPI_FLASH_HEADER_ADDR						0
#define APP_SPI_FLASH_CACHE_ENABLE						1

typedef struct
{
    uint32_t key;       // Neu co co nay =>> du lieu la hop le
    uint8_t data[256-4-2];		// PAGE_SIZE - sizeof(key) - sizeof(crc)
    uint16_t crc;		// crc16 must be the end of structure
} __attribute__((packed)) app_flash_header_t;

typedef struct
{
    uint32_t key;       // Neu co co nay =>> du lieu la hop le
    uint32_t timestamp;
    uint16_t length;
    uint8_t data[54];
    uint16_t crc;		// crc16 must be the end of structure
} __attribute__((packed)) app_spi_flash_data_t;

typedef enum
{
	APP_SPI_FLASH_DEVICE_INVALID,
	APP_SPI_FLASH_FL164K,
	APP_SPI_FLASH_FL127S,
	APP_SPI_FLASH_FL256S,
	APP_SPI_FLASH_GD256,
	APP_SPI_FLASH_W25Q256JV,
	APP_SPI_FLASH_W25Q80D,
	APP_SPI_FLASH_W25Q128,
	APP_SPI_FLASH_W25Q32,
	APP_SPI_FLASH_W25Q64,
	APP_SPI_FRAM_MB85RS16,
    APP_SPI_FLASH_AT25SF128,
    APP_SPI_FRAM_FM25V02A,
    APP_SPI_FRAM_MB85RS64V,
    APP_SPI_FRAM_FM25V01,
    APP_SPI_FRAM_FM25V10,
    APP_SPI_FLASH_W25Q16,
	APP_SPI_FLASH_MAX
} app_flash_device_t;

typedef enum
{
	APP_SPI_DEVICE_ERROR,
	APP_SPI_FLASH,
	APP_SPI_FRAM
} app_flash_type_t;

typedef struct
{
	app_flash_device_t device;
	uint32_t size;		// bytes
	app_flash_type_t type;
} app_flash_info_t;

typedef struct
{
	void (*spi_tx_buffer)(void *spi, uint8_t *tx_data, uint32_t length);
	void (*spi_rx_buffer)(void *spi, uint8_t *rx_data, uint32_t length);
	void (*spi_tx_rx)(void *spi, uint8_t *tx_data, uint8_t *rx_data, uint32_t length);
	uint8_t (*spi_tx_byte)(void *spi, uint8_t data);
	void (*spi_cs)(void *spi, bool level);
    void (*delay_ms)(void *spi, uint32_t ms);
} app_spi_flash_cb_t;

typedef struct
{
    void *spi;
	app_spi_flash_cb_t callback;
	app_flash_info_t info;
	uint32_t current_write_address;
	uint32_t next_write_addr;
	uint32_t current_read_address;
	bool error;
	bool is_the_first_time_flash_run;
} app_flash_drv_t;

/**
 * @brief       Initialize spi flash
 * @param[in]	flash_drv Pointer to flash driver
 * @retval		TRUE Flash init success
 * 				FALSE Flash init failed
 */
bool app_spi_flash_initialize(app_flash_drv_t *flash_drv);


/**
 * @brief       Estimate write address
 * @param[in]	flash_drv Pointer to flash driver
 * @param[in]   flash_full : Flash full or not
 */
uint32_t app_flash_estimate_next_write_addr(app_flash_drv_t *flash_drv, bool *flash_full);

/**
 * @brief       Dump all valid data
 */
uint32_t app_spi_flash_dump_all_data(void);

/**
 * @brief       Erase all data in flash
 * @param[in]	flash_drv Pointer to flash driver
 * @param[in]	timeout_ms Timeout in ms
 * @retval		TRUE Erase flash success
 * 				FALSE Erase flash failed
 */
bool app_spi_flash_erase_all(app_flash_drv_t *flash_drv, uint32_t timeout_ms);

/**
 * @brief       Check flash ok status
 * @param[in]	flash_drv Pointer to flash driver
 */
bool app_spi_flash_is_ok(app_flash_drv_t *flash_drv);

/**
 * @brief       Wakeup the flash
 */
void app_spi_flash_wakeup(void);

/**
 * @brief       Power down then flash
 */
void app_spi_flash_shutdown(void);


/**
 * @brief       Check sector is empty or not
 * @param[in]   sector Sector count
 * @retval		TRUE Sector is empty
 * 				FALSE Sector is not empty
 */
bool app_spi_flash_is_sector_empty(app_flash_drv_t *flash_drv, uint32_t sector_count);
    
/**
 * @brief       Flash stress write test
 * @param[in]   nb_of_write_times Number of write times
 */
void app_spi_flash_stress_test(uint32_t nb_of_write_times);

/**
 * @brief       Flash read all retransmission pending data
 * @param[in]   nb_of_write_times Number of write times
 */
void app_spi_flash_retransmission_data_test(void);

/**
 * @brief       Test write behavior if flash full
 */
void app_spi_flash_skip_to_end_flash_test(app_flash_drv_t *flash_drv);

/**
 * @brief       Flash read retransmission pending data
 * @param[in]   addr Address of data
 * @param[in]   rd_data Output data
 */
bool app_spi_flash_get_retransmission_data(uint32_t addr, app_spi_flash_data_t *rd_data);

/**
 * @brief       Write data to flash
 * @param[in]	flash_drv Flash driver
 * @param[in]   addr Address of data
 * @param[in]   buffer Data write to flash
 * @param[in]	length Size of data
 */
void app_spi_flash_write(app_flash_drv_t *flash_drv, uint32_t addr, uint8_t *buffer, uint32_t length);

/**
 * @brief       Read data from flash
 * @param[in]	flash_drv Flash driver
 * @param[in]   addr Address of data
 * @param[in]   buffer Data eadr from flash
 * @param[in]	length Size of data
 */
void app_spi_flash_read_bytes(app_flash_drv_t *flash_drv, uint32_t addr, uint8_t *buffer, uint16_t length);

/**
 * @brief       Erase block 4kb in flash
 * @param[in]	flash_drv Flash driver
 * @param[in]   sector_index Block index
 */
void app_spi_flash_erase_sector_4k(app_flash_drv_t *flash_drv, uint32_t sector_index);

/**
 * @brief       Erase flash data
 * @param[in]	flash_drv Flash driver
 * @param[in]   start_addr Start address
 * @param[in]   len Erase len
 */
void app_spi_flash_erase_from_to(app_flash_drv_t *flash_drv, uint32_t start_addr, uint32_t len);

/**
 * @brief       Write data to page
 * @param[in]	flash_drv Flash driver
 * @param[in]   addr Address of page
 * @param[in]   buffer Data write to flash
 * @param[in]	length Size of data
 */
void app_spi_flash_direct_write_bytes(app_flash_drv_t *flash_drv, uint32_t addr, uint8_t *buffer, uint16_t length);

#endif /* APP_SPI_FLASH_H */
