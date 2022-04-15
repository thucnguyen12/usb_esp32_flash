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
#define USB_CDC_TX_RING_BUFFER_SIZE		1024
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CDC_STACK_SZIE      configMINIMAL_STACK_SIZE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
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

    MX_USB_DEVICE_Init();
    DEBUG_INFO("tusb_init\r\n");


    flash_res = f_mount(&USERFatFS, USERPath, 1);
	if (flash_res != FR_OK)
	{
		DEBUG_WARN("Mount flash fail\r\n");
		flash_res = f_mkfs(USERPath, FM_ANY, 0, gFSWork, sizeof gFSWork);
		flash_res = f_mount(&USERFatFS, USERPath, 1);
		if (flash_res == FR_OK)
		{
			DEBUG_INFO ("format disk and mount again\r\n");
		}
		else
		{
			DEBUG_ERROR("Mount flash error\r\n");
		}
	}
	else
	{
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
  (void) xTaskCreateStatic(cdc_task, "cdc", CDC_STACK_SZIE, NULL, configMAX_PRIORITIES-2, cdc_stack, &cdc_taskdef);

  /* Infinite loop */
  for(;;)
  {
    tud_task();
    osDelay(1);
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
	lwrb_init(&m_ringbuffer_usb_cdc_tx, m_lwrb_tx_raw_buffer, USB_CDC_TX_RING_BUFFER_SIZE);
	for (;;)
	{
//	    // connected() check for DTR bit
//	    // Most but not all terminal client set this when making connection
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
/* USER CODE END Application */
