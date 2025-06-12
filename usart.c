/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"
#include "util.h"

__IO uint8_t uart1Buffer[UART1_BUFFER_SIZE] = {0};
__IO uint8_t uart3Buffer[UART3_BUFFER_SIZE] = {0};

huart_port_t huart_p1;
huart_port_t huart_p3;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  huart_p1.p_huart = (UART_HandleTypeDef *) &huart1;
  huart_p1.p_cndtr = &(hdma_usart1_rx.Instance->CNDTR);
  huart_p1.p_rxBuf = (uint8_t *) &uart1Buffer;
  huart_p1.rxBufSize = ARRAY_LEN(uart1Buffer);
  huart_p1.rxPos = 0;
  HAL_UART_Receive_DMA(huart_p1.p_huart, huart_p1.p_rxBuf, huart_p1.rxBufSize);
  CLEAR_BIT(huart_p1.p_huart->Instance->CR3, USART_CR3_EIE);
  CLEAR_BIT(huart_p1.p_huart->Instance->CR1, USART_CR1_PEIE);

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  huart_p3.p_huart = (UART_HandleTypeDef *) &huart3;
  huart_p3.p_cndtr = &(hdma_usart3_rx.Instance->CNDTR);
  huart_p3.p_rxBuf = (uint8_t *) &uart3Buffer;
  huart_p3.rxBufSize = ARRAY_LEN(uart3Buffer);
  huart_p3.rxPos = 0;
  HAL_UART_Receive_DMA(huart_p3.p_huart, huart_p3.p_rxBuf, huart_p3.rxBufSize);
  CLEAR_BIT(huart_p3.p_huart->Instance->CR3, USART_CR3_EIE);
  CLEAR_BIT(huart_p3.p_huart->Instance->CR1, USART_CR1_PEIE);

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = USART1_TX_Pin|USART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = USART3_TX_Pin|USART3_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART1_TX_Pin|USART1_RX_Pin);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, USART3_TX_Pin|USART3_RX_Pin);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void uart_deinit(huart_port_t* huart){
    if (HAL_UART_DeInit(huart->p_huart) != HAL_OK){
    	Error_Handler();
    }
}

void uart_wait_tx_done(huart_port_t* huart){
	uint32_t start = HAL_GetTick();
	while (READ_BIT(huart->p_huart->Instance->ISR, USART_ISR_TC) == 0 &&
			(HAL_GetTick() - start <= DEFAULT_UART_TIMEOUT)) {
		__NOP();
	}
}

void uart_write_byte(huart_port_t* huart, void *byte){
	uart_wait_tx_done(huart);
	HAL_UART_Transmit_DMA(huart->p_huart, byte, 1);
	uart_wait_tx_done(huart);
}

void uart_write_string(huart_port_t* huart, void *p_buffer, uint16_t size){
	uart_wait_tx_done(huart);
	HAL_UART_Transmit_DMA(huart->p_huart, p_buffer, size);
	uart_wait_tx_done(huart);
}

bool uart_read_byte(huart_port_t* huart, void *byte){
	/* Calculate current position in buffer */
	size_t dmaPos = huart->rxBufSize - *huart->p_cndtr;
	/* Check for new data available */
	if(dmaPos != huart->rxPos){
		/* Copy the new data to the buffer */
		memcpy(byte, (char *) &huart->p_rxBuf[huart->rxPos], 1);
		/* Move pointer and process overflow */
		huart->rxPos++;
		if(huart->rxPos == huart->rxBufSize) huart->rxPos = 0;
		return true;
	}
	return false;
}

bool uart_read_string(huart_port_t* huart, void *p_dest, size_t len){
	size_t dataLen = 0, rxPos = huart->rxPos;
	/* Calculate current position in buffer */
	size_t dmaPos = huart->rxBufSize - *huart->p_cndtr;
	/* Calculate how much data is available inside the buffer */
	if(dmaPos > rxPos){
		dataLen = dmaPos - rxPos;
	}else if(dmaPos < rxPos){
		dataLen = huart->rxBufSize - rxPos;
		dataLen += dmaPos;
	}
	/* Check buffer size */
	if (dataLen >= len){
		/* Check for overflow */
		if((rxPos + len - 1) < huart->rxBufSize){
			memcpy((char *) p_dest, (char *) &huart->p_rxBuf[rxPos], len);
		}else{
			uint16_t tailLen = huart->rxBufSize - rxPos;
			uint16_t headLen = len - tailLen;
			memcpy((char *) p_dest, (char *) &huart->p_rxBuf[rxPos], tailLen);
			memcpy((char *) (p_dest+tailLen), (char *) &huart->p_rxBuf[0], headLen);
		}
		/* Move pointer and process overflow */
		rxPos += len;
		if (rxPos >= huart->rxBufSize) rxPos -= huart->rxBufSize;
		huart->rxPos = rxPos;
		return true;
	}
	return false;
}

bool uart_peek(huart_port_t* huart, void *p_dest, size_t len){
	size_t dataLen = 0, rxPos = huart->rxPos;
	/* Calculate current position in buffer */
	size_t dmaPos = huart->rxBufSize - *huart->p_cndtr;
	/* Calculate how much data is available inside the buffer */
	if(dmaPos > rxPos){
		dataLen = dmaPos - rxPos;
	}else if(dmaPos < rxPos){
		dataLen = huart->rxBufSize - rxPos;
		dataLen += dmaPos;
	}
	/* Check buffer size */
	if (dataLen > 0 && dataLen <= len){
		/* Check for overflow */
		if((rxPos + len - 1) < huart->rxBufSize){
			memcpy((char *) p_dest, (char *) &huart->p_rxBuf[rxPos], len);
		}else{
			uint16_t tailLen = huart->rxBufSize - rxPos;
			uint16_t headLen = len - tailLen;
			memcpy((char *) p_dest, (char *) &huart->p_rxBuf[rxPos], tailLen);
			memcpy((char *) (p_dest+tailLen), (char *) &huart->p_rxBuf[0], headLen);
		}
		return true;
	}
	return false;
}

void uart_flush_rx(huart_port_t* huart){
    huart->rxPos = huart->rxBufSize - *huart->p_cndtr;
}

/* USER CODE END 1 */
