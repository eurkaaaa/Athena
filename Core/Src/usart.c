/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration
  PC4   ------> USART3_TX
  PC5   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART3_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART3_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_USART_EnableIT_RXNE(USART3); // 使能接收缓冲区非空中断

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
//  LL_USART_Disable(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
//UART_HandleTypeDef huart3;
//DMA_HandleTypeDef hdma_usart3_rx;
//DMA_HandleTypeDef hdma_usart3_tx;
//
///* USART3 init function */
//
//void MX_USART3_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART3_Init 0 */
//
//  /* USER CODE END USART3_Init 0 */
//
//  /* USER CODE BEGIN USART3_Init 1 */
//
//  /* USER CODE END USART3_Init 1 */
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART3_Init 2 */
//
//  /* USER CODE END USART3_Init 2 */
//
//}
//
//void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(uartHandle->Instance==USART3)
//  {
//  /* USER CODE BEGIN USART3_MspInit 0 */
//
//  /* USER CODE END USART3_MspInit 0 */
//    LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);
//
//    /* USART3 clock enable */
//    __HAL_RCC_USART3_CLK_ENABLE();
//
//    __HAL_RCC_GPIOC_CLK_ENABLE();
//    /**USART3 GPIO Configuration
//    PC4     ------> USART3_TX
//    PC5     ------> USART3_RX
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//    /* USART3 DMA Init */
//    /* USART3_RX Init */
//    hdma_usart3_rx.Instance = DMA1_Channel3;
//    hdma_usart3_rx.Init.Request = DMA_REQUEST_2;
//    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
//    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
//    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);
//
//    /* USART3_TX Init */
//    hdma_usart3_tx.Instance = DMA1_Channel2;
//    hdma_usart3_tx.Init.Request = DMA_REQUEST_2;
//    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
//    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
//    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
//    {
//      Error_Handler();
//    }
//
//    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);
//
//    /* USART3 interrupt Init */
//    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
//    HAL_NVIC_EnableIRQ(USART3_IRQn);
//  /* USER CODE BEGIN USART3_MspInit 1 */
//
//  /* USER CODE END USART3_MspInit 1 */
//  }
//}
//
//void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
//{
//
//  if(uartHandle->Instance==USART3)
//  {
//  /* USER CODE BEGIN USART3_MspDeInit 0 */
//
//  /* USER CODE END USART3_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART3_CLK_DISABLE();
//
//    /**USART3 GPIO Configuration
//    PC4     ------> USART3_TX
//    PC5     ------> USART3_RX
//    */
//    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);
//
//    /* USART3 DMA DeInit */
//    HAL_DMA_DeInit(uartHandle->hdmarx);
//    HAL_DMA_DeInit(uartHandle->hdmatx);
//
//    /* USART3 interrupt Deinit */
//    HAL_NVIC_DisableIRQ(USART3_IRQn);
//  /* USER CODE BEGIN USART3_MspDeInit 1 */
//
//  /* USER CODE END USART3_MspDeInit 1 */
//  }
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
