/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_storage_if.h"
#include "tusb.h"
#include "app_debug.h"
#include "fatfs.h"
#include "lwrb.h"
#include <stdbool.h>
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "esp_loader.h"
#include "example_common.h"
#include "md5_hash.h"
#include "app_btn.h"
#include "utilities.h"
#include "usart.h" /// needs add change baudrate function
#include "stm32_port.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CDC_STACK_SZIE      configMINIMAL_STACK_SIZE
#define USB_CDC_TX_RING_BUFFER_SIZE		1024
#define BIT_EVENT_GROUP_KEY_0_PRESSED        (1 << 0)
#define BIT_EVENT_GROUP_KEY_1_PRESSED        (1 << 1)
//#define BIT_EVENT_GROUP_BUTTON_2_PRESSED        (1 << 2)
#define HW_BTN_CONFIG                                       \
{                                                           \
    /* PinName       Last state   Idle level*/              \
    {0, 1, 1},                                              \
    {1, 1, 1},                                              \
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t m_buffer[4096];
static MD5Context_t md5_context;
static esp_loader_config_t m_loader_cfg =
{
    .baud_rate = 115200,
//    .uart_rx_pin = GPIO_NUM_26,
//    .uart_tx_pin = GPIO_NUM_25,
    .reset_trigger_pin = ESP32_RST_Pin,
    .gpio0_trigger_pin = ESP32_IO0_Pin,
//    .rx_buffer_size = 4096,
//    .tx_buffer_size = 4096,
    .buffer = m_buffer,
    .buffer_size = 4096,
    .sync_timeout = 100,
    .trials = 10,
    .md5_context = &md5_context
};

static example_binaries_t m_binary;
static char m_file_address[128];
volatile uint32_t led_busy_toggle = 0;

static app_btn_hw_config_t m_button_cfg[] = HW_BTN_CONFIG;
static EventGroupHandle_t m_button_event_group = NULL;
static const char *info_file = "info.txt";
static const char *bootloader_file = "bootloader.bin";
static const char *application_file = "app.bin";
static const char *partition_file = "partition-table.bin";
bool m_disk_is_mounted = false;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId testTaskHandle;
BYTE gFSWork[_MAX_SS];
UINT br, bw;  // File read/write count
UINT fbr, fbw;  // File read/write count
FRESULT flash_res;
StackType_t  cdc_stack[CDC_STACK_SZIE];
StaticTask_t cdc_taskdef;
void cdc_task(void* params);

void flash_task(void *argument);
static TaskHandle_t m_task_connect_handle = NULL;
void button_initialize(uint32_t button_num);
uint32_t btn_read(uint32_t pin);
void on_btn_pressed(int number, int event, void * pData);
void on_btn_release(int number, int event, void * pData);
void on_btn_hold(int number, int event, void * pData);
static void on_btn_hold_so_long(int index, int event, void * pData);
static const char *chip_des[] = {"ESP8266", "ESP32", "ESP32S2", "ESP32C3", "ESP32S3", "ESP32C2", "ESP32H2", "UNKNOWN"};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  /* USER CODE BEGIN StartDefaultTask */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    MX_FATFS_Init();

    vTaskDelay(500);		// time for usb renum

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    m_button_event_group = xEventGroupCreate(); /// create group buttons
    MX_USB_DEVICE_Init();
    DEBUG_INFO("tusb_init\r\n");
    // INIT BUTTON APP
    app_btn_config_t btn_conf;
    btn_conf.config = m_button_cfg;
    btn_conf.btn_count = 2;
    btn_conf.get_tick_cb = xTaskGetTickCount;
    btn_conf.btn_initialize = button_initialize;
    btn_conf.btn_read = btn_read;
    btn_conf.scan_interval_ms = 50;
    app_btn_initialize(&btn_conf);
//    app_btn_initialize(&btn_conf);
    app_btn_register_callback(APP_BTN_EVT_HOLD, on_btn_hold, NULL);
    app_btn_register_callback(APP_BTN_EVT_HOLD_SO_LONG, on_btn_hold_so_long, NULL);
    app_btn_register_callback(APP_BTN_EVT_PRESSED, on_btn_pressed, NULL);
    app_btn_register_callback(APP_BTN_EVT_RELEASED, on_btn_release, NULL);
    // INIT BUTTON END
    flash_res = f_mount(&USERFatFS, USERPath, 1);
	if (flash_res != FR_OK)
	{
		DEBUG_WARN("Mount flash fail\r\n");
		flash_res = f_mkfs(USERPath, FM_ANY, 0, gFSWork, sizeof gFSWork);
		flash_res = f_mount(&USERFatFS, USERPath, 1);
		if (flash_res == FR_OK)
		{
			m_disk_is_mounted = true;
			DEBUG_INFO ("format disk and mount again\r\n");
		}
		else
		{
			DEBUG_ERROR("Mount flash error\r\n");
		}
	}
	else
	{
		m_disk_is_mounted = true;
		DEBUG_INFO ("Mount flash ok\r\n");
	}
	TCHAR label[32];
	f_getlabel(USERPath, label, 0);
	DEBUG_INFO("Label %s\r\n", label);
	if (strcmp(label, "BSAFE JIG"))
	{
		DEBUG_INFO("Set label\r\n");
		f_setlabel("BSAFE JIG");
	}

    tusb_init();
  // Create CDC task
  (void) xTaskCreateStatic(cdc_task, "cdc", CDC_STACK_SZIE, NULL, 1, cdc_stack, &cdc_taskdef);// pio =2
  // Create flashtask
  if (m_task_connect_handle == NULL)
  {
	  xTaskCreate(flash_task, "flash_task", 4096, NULL, 0, &m_task_connect_handle);// pio =1
  }
  /* Infinite loop */
  for(;;)
  {
	  app_btn_scan(NULL);
	 // app_debug_isr_ringbuffer_flush();
	  if (led_busy_toggle == 0)
	  {
		  HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	  }
	  osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


static bool m_cdc_debug_register = false;
static lwrb_t m_ringbuffer_usb_cdc_tx;
static uint8_t m_lwrb_tx_raw_buffer[USB_CDC_TX_RING_BUFFER_SIZE];
uint32_t cdc_tx(const void *buffer, uint32_t size)
{
	lwrb_write(&m_ringbuffer_usb_cdc_tx, buffer, size);
	return size;
}

void cdc_task(void* params)
{
	DEBUG_INFO("ENTER CDC TASK\r\n");
	lwrb_init(&m_ringbuffer_usb_cdc_tx, m_lwrb_tx_raw_buffer, USB_CDC_TX_RING_BUFFER_SIZE);
	for (;;)
	{
//	    // connected() check for DTR bit
//	    // Most but not all terminal client set this when making connection
		tud_task();
	    if (tud_cdc_connected())
		{
	    	if (m_cdc_debug_register == false)
	    	{
	    		m_cdc_debug_register = true;
	    		app_debug_register_callback_print(cdc_tx);
	    	}
			// There are data available
			if (tud_cdc_available())
			{
				uint8_t buf[64];

				// read and echo back
				uint32_t count = tud_cdc_read(buf, sizeof(buf));
				(void) count;

				if (count && strstr((char*)buf, "RESET"))
				{
					tud_cdc_write_flush();
					tud_cdc_write_str("System reset\r\n");
					tud_cdc_write_flush();
					vTaskDelay(1000);
					NVIC_SystemReset();
				}
//				// Echo back
//				// Note: Skip echo by commenting out write() and write_flush()
//				// for throughput test e.g
//				//    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
//				tud_cdc_write(buf, count);
//				tud_cdc_write_flush();
			}
		}
	    else
	    {
	    	if (m_cdc_debug_register)
	    	{
	    		m_cdc_debug_register = false;
	    		app_debug_unregister_callback_print(cdc_tx);
		    	// Flush all cdc tx buffer
		    	char tmp[1];
		    	while (lwrb_read(&m_ringbuffer_usb_cdc_tx, tmp, 1))
		    	{

		    	}
	    	}
	    }

	    char buffer[ (TUD_OPT_HIGH_SPEED ? 512 : 64)];
	    uint32_t size;
	    while (1)
	    {
	    	uint32_t avai = tud_cdc_write_available();
	    	if (avai >= sizeof(buffer))
	    	{
	    		avai = sizeof(buffer);
	    	}
			size = lwrb_read(&m_ringbuffer_usb_cdc_tx, buffer, avai);
			if (size)
			{
				tud_cdc_write(buffer, size);
				tud_cdc_write_flush();
			}
			else
			{
				break;
			}
	    }
	    vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void flash_task(void *argument)
{
	DEBUG_INFO("ENTER flash TASK\r\n");
	int32_t file_size = 0;

	if (m_disk_is_mounted)
	{
		file_size = fatfs_read_file(info_file, (uint8_t*)m_file_address, sizeof(m_file_address) - 1);
		if (file_size > 0)
		{
			/*
			{
				"boot": 123456,
				"app": 1238123,
				"partition": 1238
			}
			*/
			char *ptr = strstr(m_file_address, "\"boot\":");
			if (ptr)
			{
				ptr += strlen("\"boot\":");
				m_binary.boot.addr = utilities_get_number_from_string(0, ptr);
			}

			ptr = strstr(m_file_address, "\"app\":");
			if (ptr)
			{
				ptr += strlen("\"app\":");
				m_binary.app.addr = utilities_get_number_from_string(0, ptr);
			}

			ptr = strstr(m_file_address, "\"partition\":");
			if (ptr)
			{
				ptr += strlen("\"partition\":");
				m_binary.part.addr = utilities_get_number_from_string(0, ptr);
			}
		}

		file_size = fatfs_get_file_size(bootloader_file);
		if (file_size > -1)
		{
			m_binary.boot.size = file_size;
			m_binary.boot.file_name = bootloader_file;
		}

		file_size = fatfs_get_file_size(application_file);
		if (file_size > -1)
		{
			m_binary.app.size = file_size;
			m_binary.app.file_name = application_file;
		}

		file_size = fatfs_get_file_size(partition_file);
		if (file_size > -1)
		{
			m_binary.part.size = file_size;
			m_binary.part.file_name = partition_file;
		}

		DEBUG_INFO("Bootloader offset 0x%08X, app 0x%08X, partition table 0x%08X\r\n", m_binary.boot.addr, m_binary.app.addr,  m_binary.part.addr);
		DEBUG_INFO("Bootloader %u bytes, app %u bytes, partition table %u bytes\r\n", m_binary.boot.size, m_binary.app.size,  m_binary.part.size);
	}
    m_loader_cfg.gpio0_trigger_port = (uint32_t)ESP32_IO0_GPIO_Port;
    m_loader_cfg.reset_trigger_port = (uint32_t)ESP32_RST_GPIO_Port;
    m_loader_cfg.uart_addr = (uint32_t)USART3;
    esp_loader_error_t err;

    // Clear led busy & success, set led error
    HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_SUCCESS_GPIO_Port, LED_SUCCESS_Pin, GPIO_PIN_SET);
    for (;;)
	{
    	DEBUG_INFO("ENTER flash LOOP\r\n");
    	if (led_busy_toggle > 10)
		{
			led_busy_toggle = 1;
		}
		xEventGroupWaitBits(m_button_event_group,
								BIT_EVENT_GROUP_KEY_0_PRESSED,
								pdTRUE,
								pdFALSE,
								portMAX_DELAY);
//		DEBUG_INFO("KEY IS PRESSED\r\n");
        HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_SUCCESS_GPIO_Port, LED_SUCCESS_Pin, GPIO_PIN_SET);
        uint32_t now = xTaskGetTickCount();
		uint32_t retry = 4;
		while (m_binary.part.size > 0
				&& m_binary.app.size > 0
				&& m_binary.part.size > 0
				)
		{
			if (retry == 0)
			{
				break;
			}
			retry--;
			loader_port_change_baudrate(&m_loader_cfg, 115200);
			DEBUG_INFO("Connecting to target remain %u times\r\n", retry);
			led_busy_toggle = 1000000;
			err = esp_loader_connect(&m_loader_cfg);
			if (err != ESP_LOADER_SUCCESS)
			{
				DEBUG_ERROR("Connect to target failed %d\r\n", err);
				continue;
			}
			else
			{
				DEBUG_INFO("Connected to target %s\r\n", chip_des[m_loader_cfg.target]);
			}
			DEBUG_INFO("Change baudrate\r\n");
			err = esp_loader_change_baudrate(&m_loader_cfg, 115200);
			if (err == ESP_LOADER_ERROR_UNSUPPORTED_FUNC)
			{
				DEBUG_ERROR("ESP8266 does not support change baudrate command\r\n");
			}
			else if (err != ESP_LOADER_SUCCESS)
			{
				DEBUG_ERROR("Unable to change baud rate on target\r\n");
			}
			else
			{
				err = loader_port_change_baudrate(&m_loader_cfg, 115200);
				if (err != ESP_LOADER_SUCCESS)
				{
					DEBUG_ERROR("Unable to change baud rate\r\n");
				}
				else
				{
					DEBUG_INFO("Port[%u] : Baudrate changed\r\n");
				}
			}

			DEBUG_INFO("Flash bootloader\r\n");
			if (flash_binary_stm32(&m_loader_cfg, &m_binary.boot) != ESP_LOADER_SUCCESS)
			{
				DEBUG_INFO("FLASH BOOTLOADER FAIL \r\n");
				xEventGroupClearBits(m_button_event_group,
									BIT_EVENT_GROUP_KEY_0_PRESSED);
				break;
			}

			DEBUG_INFO("Flash app\r\n");
			if (flash_binary_stm32(&m_loader_cfg, &m_binary.app) != ESP_LOADER_SUCCESS)
			{
				xEventGroupClearBits(m_button_event_group,
									BIT_EVENT_GROUP_KEY_0_PRESSED);
				break;
			}

			DEBUG_INFO("Flash parition table\r\n");
			if (flash_binary_stm32(&m_loader_cfg, &m_binary.part) != ESP_LOADER_SUCCESS)
			{
				xEventGroupClearBits(m_button_event_group,
									BIT_EVENT_GROUP_KEY_0_PRESSED);
				break;
			}
			retry = 0;
			xEventGroupClearBits(m_button_event_group,
								BIT_EVENT_GROUP_KEY_0_PRESSED);
			DEBUG_INFO("Total flash write time %us\r\n", (xTaskGetTickCount() - now)/1000);

			if (led_busy_toggle > 10)
			{
				led_busy_toggle = 1;
			}

			loader_port_change_baudrate(&m_loader_cfg, 115200);
			// loader_port_reset_target(&m_loader_cfg);
			// Led success on, led busy off
			HAL_GPIO_WritePin(LED_SUCCESS_GPIO_Port, LED_SUCCESS_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
			break;
		}
		vTaskDelay(1000);
	}
//    vTaskDelay(1000);
}



void button_initialize(uint32_t button_num)
{

}

uint32_t btn_read(uint32_t pin)
{
    if (pin == 0)
    {
        return HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);
    }
//    else if (pin == 1)
//    {
	return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
//    }
//    return HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
}

void on_btn_pressed(int number, int event, void * pData)
{
    DEBUG_INFO("On button %d pressed\r\n", number);
    if (number == 0)
    {
        xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_KEY_0_PRESSED);
    }
    else if (number == 1)
    {
        xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_KEY_1_PRESSED);
    }
    else
    {
        //xEventGroupSetBits(m_button_event_group, BIT_EVENT_GROUP_BUTTON_1_PRESSED);
    }
}

void on_btn_release(int number, int event, void * pData)
{
    DEBUG_VERBOSE("On button %d release\r\n", number);
    if (number == 0)
    {
        xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_KEY_0_PRESSED);
    }
    else if (number == 1)
    {
        xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_KEY_1_PRESSED);
    }
    else if (number == 2)
    {
  //      xEventGroupClearBits(m_button_event_group, BIT_EVENT_GROUP_BUTTON_2_PRESSED);
    }
}

void on_btn_hold(int number, int event, void * pData)
{
    DEBUG_INFO("On button %d pair hold, enter pair mode\r\n", number);
}


static void on_btn_hold_so_long(int index, int event, void * pData)
{
    DEBUG_INFO("Button hold so long\r\n");
}



/* USER CODE END Application */
