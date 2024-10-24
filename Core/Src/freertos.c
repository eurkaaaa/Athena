/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c_drv.h"
#include "spi_drv.h"
#include "uart_hal.h"
#include "tca6408a.h"
#include "vl53l5cx_api.h"
#include "test_tof.h"
#include "calibration.h"
#include "w25q64_ll.h"
#include "uart_receive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static VL53L5CX_Configuration vl53l5dev_f;
static VL53L5CX_ResultsData vl53l5_res_f;
SemaphoreHandle_t txComplete = NULL;
SemaphoreHandle_t rxComplete = NULL;
SemaphoreHandle_t spiMutex = NULL;
SemaphoreHandle_t UartRxReady = NULL;
static uint8_t Pos[26];
static uint8_t Pos_new[17];
static float para[4];
static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */


void StartDefaultTask(void *argument);
void compute();
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	txComplete = xSemaphoreCreateBinary();
	rxComplete = xSemaphoreCreateBinary();
	spiMutex = xSemaphoreCreateMutex();
	UartRxReady = xSemaphoreCreateBinary();
	CreateUartRxQueue();

	if (txComplete == NULL || rxComplete == NULL || spiMutex == NULL)
	{
	    while (1);
	}

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartDefaultTask */
void para_get()
{
  padX = para[0];
  padY = para[1];
  padZ = para[2];
}

void para_reget()
{
  para[0] = padX;
  para[1] = padY;
  para[2] = padZ;
}

void compute()
{
	switch(Pos[24])
	{
	case 0:
		//add more control 1up 0down
		if(Pos[25])
		{
			padZ = 0.3f;
			Pos_new[16] = 1;
		}
		else
		{
			Pos_new[16] = 0;
		}
		break;
	case 1:
		if(Pos[25])
		{
//      if(padX < 1.5f && padY < 1.5f && padZ < 1.5f)
//      {
         padX += 0.1f;
//      }
//      else if(padX >= 1.5f && padY < 1.5f && padZ < 1.5f)
//      {
//         padY += 0.1f;
//      }
//      else if(padX >= 1.5f && padY >= 1.5f && padZ < 1.5f)
//      {
//         padZ += 0.1f;
//      }
			Pos_new[16] = 2;
		}
		else
		{
			padZ = 0.3f;
			Pos_new[16] = 3;
		}
		break;
	}
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	uint8_t index = 0;
//	for(int i=0;i<36;i++)
//	{
//		Pos[i] = i+56;
//	}
	for(;;)
	{
//		LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_9);
//		LL_mDelay(100);
//		UART_DMA_Transmit(Pos, 16);
//		LL_mDelay(10);
	  if (xSemaphoreTake(UartRxReady, 0) == pdPASS) {

		  while (index < 26 && xQueueReceive(UartRxQueue, &Pos[index], 0) == pdPASS) {
				  index++;
		  }
		  if(index == 26)
		  {
        memcpy(para, (float *)Pos, 16);
        para_get();
        compute();
        para_reget();
        memcpy(Pos_new, (uint8_t *)para, 16);
		UART_DMA_Transmit(Pos_new, 17);
		index=0;
		  }
	  }
//	  else{
//		  BSP_W25Qx_Init();
//	  }
//	  BSP_W25Qx_Init();
//	  uint8_t ID[2]={0};
//	  BSP_W25Qx_Read_ID(ID);
//	  //BSP_W25Qx_Erase_Chip();
//	  BSP_W25Qx_Erase_Block(0x123456);
//	  uint8_t tx_data[128] = {0xef};
//	  uint8_t rx_data[128] = {0x00};
//	  BSP_W25Qx_Write(tx_data, 0x123456, 128);
//	  BSP_W25Qx_Read(rx_data, 0x123456, 4);
//	  BSP_W25Qx_Erase_Block(0x123456);
//	  BSP_W25Qx_Read(rx_data, 0x123456, 4);
//
//	  uint8_t ID[4];
//	  I2C_expander_initialize();
//	  initialize_sensors_I2C(&vl53l5dev_f,1);
//	  vl53l5cx_start_ranging(&vl53l5dev_f);
//	  while(1){
//		  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_9);
//	  	  LL_mDelay(100);
//	  	  get_sensor_data(&vl53l5dev_f, &vl53l5_res_f);
//	  }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

