/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "pre_prod.h"

/**
 * \brief           USART volatile data
 */
static uart_rx_buffer_t uart1_rx_data;
static uart_tx_buffer_t uart1_tx_data;

static uart_rx_buffer_t uart3_rx_data;
static uart_tx_buffer_t uart3_tx_data;

/**
 * \brief           USART1 setup
 */
static const uart_handler_t huart1 = {
    /* UART config */
    .uart = USART1,
    .baud_rate = 9600,
    .uart_tx_port = GPIOA,
    .uart_tx_pin = LL_GPIO_PIN_9,
    .uart_tx_pin_af = LL_GPIO_AF_7,
    .uart_rx_port = GPIOA,
    .uart_rx_pin = LL_GPIO_PIN_10,
    .uart_rx_pin_af = LL_GPIO_AF_7,
	/* UART flags */
    .dma_rx = DMA1,
    .dma_rx_ch = LL_DMA_CHANNEL_5,
    .dma_rx_req = LL_DMA_REQUEST_2,
	.dma_tx = DMA1,
    .dma_tx_ch = LL_DMA_CHANNEL_4,
    .dma_tx_req = LL_DMA_REQUEST_2,
	.dma_rx_is_tc_fn = LL_DMA_IsActiveFlag_TC5,
	.dma_rx_is_ht_fn = LL_DMA_IsActiveFlag_HT5,
	.dma_rx_clear_tc_fn = LL_DMA_ClearFlag_TC5,
	.dma_rx_clear_ht_fn = LL_DMA_ClearFlag_HT5,
	.dma_tx_is_tc_fn = LL_DMA_IsActiveFlag_TC4,
	.dma_tx_clear_tc_fn = LL_DMA_ClearFlag_TC4,
	.dma_tx_clear_ht_fn = LL_DMA_ClearFlag_HT4,
	.dma_tx_clear_gi_fn = LL_DMA_ClearFlag_GI4,
	.dma_tx_clear_te_fn = LL_DMA_ClearFlag_TE4,
	/* Interrupts config */
	.prio = 5,
    .uart_irq = USART1_IRQn,
	.dma_rx_irq = DMA1_Channel5_IRQn,
	.dma_tx_irq = DMA1_Channel4_IRQn,
    /* Volatile data */
    .rx_data = &uart1_rx_data,
	.tx_data = &uart1_tx_data,
};

/**
 * \brief           USART3 setup
 */
static const uart_handler_t huart3 = {
	    /* UART config */
	    .uart = USART3,
	    .baud_rate = 9600,
	    .uart_tx_port = GPIOB,
	    .uart_tx_pin = LL_GPIO_PIN_10,
	    .uart_tx_pin_af = LL_GPIO_AF_7,
	    .uart_rx_port = GPIOB,
	    .uart_rx_pin = LL_GPIO_PIN_11,
	    .uart_rx_pin_af = LL_GPIO_AF_7,
		/* UART flags */
	    .dma_rx = DMA1,
	    .dma_rx_ch = LL_DMA_CHANNEL_3,
	    .dma_rx_req = LL_DMA_REQUEST_2,
		.dma_tx = DMA1,
	    .dma_tx_ch = LL_DMA_CHANNEL_2,
	    .dma_tx_req = LL_DMA_REQUEST_2,
		.dma_rx_is_tc_fn = LL_DMA_IsActiveFlag_TC3,
		.dma_rx_is_ht_fn = LL_DMA_IsActiveFlag_HT3,
		.dma_rx_clear_tc_fn = LL_DMA_ClearFlag_TC3,
		.dma_rx_clear_ht_fn = LL_DMA_ClearFlag_HT3,
		.dma_tx_is_tc_fn = LL_DMA_IsActiveFlag_TC2,
		.dma_tx_clear_tc_fn = LL_DMA_ClearFlag_TC2,
		.dma_tx_clear_ht_fn = LL_DMA_ClearFlag_HT2,
		.dma_tx_clear_gi_fn = LL_DMA_ClearFlag_GI2,
		.dma_tx_clear_te_fn = LL_DMA_ClearFlag_TE2,
		/* Interrupts config */
		.prio = 5,
	    .uart_irq = USART3_IRQn,
		.dma_rx_irq = DMA1_Channel3_IRQn,
		.dma_tx_irq = DMA1_Channel2_IRQn,
	    /* Volatile data */
	    .rx_data = &uart3_rx_data,
		.tx_data = &uart3_tx_data,
};

const uart_handler_t *p_huart1 = &huart1;
const uart_handler_t *p_huart3 = &huart3;

/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
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
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART3_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 9600;
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
/**
 * \brief           Initialize UART
 * \param[in]       uart: Uart Handler
 */
void usart_init(const uart_handler_t* uart) {
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Init Ring Buffers */
    lwrb_init(&huart1.tx_data->usart_tx_rb, huart1.tx_data->usart_tx_rb_data, sizeof(huart1.tx_data->usart_tx_rb_data));
    lwrb_init(&huart1.rx_data->usart_rx_rb, huart1.rx_data->usart_rx_rb_data, sizeof(huart1.rx_data->usart_rx_rb_data));

    /* Peripheral clock enable */
    if(uart->uart == USART1){
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }else if(uart->uart == USART3){
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    }

    /* USART GPIO Configuration */
    /* RX pin */
    GPIO_InitStruct.Pin = uart->uart_rx_pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = uart->uart_rx_pin_af;
    LL_GPIO_Init(uart->uart_rx_port, &GPIO_InitStruct);
    /* TX pin */
    GPIO_InitStruct.Pin = uart->uart_tx_pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = uart->uart_tx_pin_af;
    LL_GPIO_Init(uart->uart_tx_port, &GPIO_InitStruct);

    /* USART1 RX DMA Init */
    LL_DMA_SetPeriphRequest(uart->dma_rx, uart->dma_rx_ch, uart->dma_rx_req);
    LL_DMA_SetDataTransferDirection(uart->dma_rx, uart->dma_rx_ch, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(uart->dma_rx, uart->dma_rx_ch, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(uart->dma_rx, uart->dma_rx_ch, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(uart->dma_rx, uart->dma_rx_ch, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(uart->dma_rx, uart->dma_rx_ch, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(uart->dma_rx, uart->dma_rx_ch, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(uart->dma_rx, uart->dma_rx_ch, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(uart->dma_rx, uart->dma_rx_ch, LL_USART_DMA_GetRegAddr(uart->uart, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(uart->dma_rx, uart->dma_rx_ch, (uint32_t) huart1.rx_data->usart_rx_dma_buffer);
    LL_DMA_SetDataLength(uart->dma_rx, uart->dma_rx_ch, ARRAY_LEN(huart1.rx_data->usart_rx_dma_buffer));

    /* USART1 TX DMA Init */
    LL_DMA_SetPeriphRequest(uart->dma_tx, uart->dma_tx_ch, LL_DMA_REQUEST_2);
    LL_DMA_SetDataTransferDirection(uart->dma_tx, uart->dma_tx_ch, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(uart->dma_tx, uart->dma_tx_ch, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(uart->dma_tx, uart->dma_tx_ch, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(uart->dma_tx, uart->dma_tx_ch, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(uart->dma_tx, uart->dma_tx_ch, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(uart->dma_tx, uart->dma_tx_ch, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(uart->dma_tx, uart->dma_tx_ch, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(uart->dma_tx, uart->dma_tx_ch, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT));

    /* Enable DMA RX HT & TC interrupts */
    LL_DMA_EnableIT_HT(uart->dma_rx, uart->dma_rx_ch);
    LL_DMA_EnableIT_TC(uart->dma_rx, uart->dma_rx_ch);
    /* Enable DMA TX TC interrupts */
    LL_DMA_EnableIT_TC(uart->dma_tx, uart->dma_tx_ch);

    /* DMA interrupt init */
    NVIC_SetPriority(uart->dma_rx_irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), uart->prio, 0));
    NVIC_EnableIRQ(uart->dma_rx_irq);
    NVIC_SetPriority(uart->dma_tx_irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), uart->prio, 0));
    NVIC_EnableIRQ(uart->dma_tx_irq);

    /* Configure USART1 */
    USART_InitStruct.BaudRate = uart->baud_rate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(uart->uart, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(uart->uart);
    LL_USART_EnableDMAReq_RX(uart->uart);
    LL_USART_EnableDMAReq_TX(uart->uart);
    LL_USART_EnableIT_IDLE(uart->uart);

    /* USART interrupt, same priority as DMA channel */
    NVIC_SetPriority(uart->uart_irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), uart->prio, 1));
    NVIC_EnableIRQ(uart->uart_irq);

    /* Enable USART and DMA RX */
    LL_DMA_EnableChannel(uart->dma_rx, uart->dma_rx_ch);
    LL_DMA_EnableChannel(uart->dma_tx, uart->dma_tx_ch);
    LL_USART_Enable(uart->uart);
}

/**
 * \brief           General purpose DMA interrupt handler
 * \note            Function must be called from DMA interrupt
 *
 * It handles half-transfer and transfer-complete interrupts and does the job accordingly
 *
 * \param[in]       uart: Uart description to handle
 */
void usart_dma_irq_handler(const uart_handler_t* uart){
	/* Events for DMA USART DMA RX */
	/* Check half-transfer complete interrupt */
	if (LL_DMA_IsEnabledIT_HT(uart->dma_rx, uart->dma_rx_ch) && uart->dma_rx_is_ht_fn(uart->dma_rx)) {
		uart->dma_rx_clear_ht_fn(uart->dma_rx);             /* Clear half-transfer complete flag */
		usart_rx_check(uart);                      			/* Check for data to process */
	}
	/* Check transfer-complete interrupt */
	if (LL_DMA_IsEnabledIT_TC(uart->dma_rx, uart->dma_rx_ch) && uart->dma_rx_is_tc_fn(uart->dma_rx)) {
		uart->dma_rx_clear_tc_fn(uart->dma_rx);             /* Clear transfer complete flag */
		usart_rx_check(uart);                       		/* Check for data to process */
	}
	/* Events for USART DMA TX */
	/* Check transfer complete */
	if (LL_DMA_IsEnabledIT_TC(uart->dma_tx, uart->dma_tx_ch) && uart->dma_tx_is_tc_fn(uart->dma_tx)) {
		/* Clear transfer complete flag */
		uart->dma_tx_clear_tc_fn(uart->dma_tx);
		/* Skip sent data, mark as read */
		lwrb_skip(&uart->tx_data->usart_tx_rb, uart->tx_data->usart_tx_dma_current_len);
		/* Clear length variable */
		uart->tx_data->usart_tx_dma_current_len = 0;
		/* Start sending more data */
		usart_start_tx_dma_transfer(uart);
	}
}

/**
 * \brief           General purpose UART interrupt handler
 * \note            Function must be called from UART interrupt
 *
 * It handles IDLE line detection interrupt and does the job accordingly
 *
 * \param[in]       uart: Uart description to handle
 */
void usart_irq_handler(const uart_handler_t* uart){
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(uart->uart) && LL_USART_IsActiveFlag_IDLE(uart->uart)) {
        LL_USART_ClearFlag_IDLE(uart->uart);        		/* Clear IDLE line flag */
        usart_rx_check(uart);                       		/* Check for data to process */
    }
}

/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void usart_rx_check(const uart_handler_t* uart) {
	/* Calculate current position in buffer and check for new data available */
    size_t pos = ARRAY_LEN(uart->rx_data->usart_rx_dma_buffer) -
    		LL_DMA_GetDataLength(uart->dma_rx, uart->dma_rx_ch);
    /* Check change in received data */
    if (pos != uart->rx_data->old_pos) {
    	/* Current position is over previous one */
		if (pos > uart->rx_data->old_pos) {
			/*
			 * Processing is done in "linear" mode.
			 *
			 * Application processing is fast with single data block,
			 * length is simply calculated by subtracting pointers
			 *
			 * [   0   ]
			 * [   1   ] <- old_pos |------------------------------------|
			 * [   2   ]            |                                    |
			 * [   3   ]            | Single block (len = pos - old_pos) |
			 * [   4   ]            |                                    |
			 * [   5   ]            |------------------------------------|
			 * [   6   ] <- pos
			 * [   7   ]
			 * [ N - 1 ]
			 */
			usart_process_data(uart,
					&uart->rx_data->usart_rx_dma_buffer[uart->rx_data->old_pos],
					pos - uart->rx_data->old_pos);
		} else {
			/*
			 * Processing is done in "overflow" mode..
			 *
			 * Application must process data twice,
			 * since there are 2 linear memory blocks to handle
			 *
			 * [   0   ]            |---------------------------------|
			 * [   1   ]            | Second block (len = pos)        |
			 * [   2   ]            |---------------------------------|
			 * [   3   ] <- pos
			 * [   4   ] <- old_pos |---------------------------------|
			 * [   5   ]            |                                 |
			 * [   6   ]            | First block (len = N - old_pos) |
			 * [   7   ]            |                                 |
			 * [ N - 1 ]            |---------------------------------|
			 */
			/* First block */
			usart_process_data(uart,
					&uart->rx_data->usart_rx_dma_buffer[uart->rx_data->old_pos],
					ARRAY_LEN(uart->rx_data->usart_rx_dma_buffer) - uart->rx_data->old_pos);
			/* Second block */
			if (pos > 0) usart_process_data(uart, &uart->rx_data->usart_rx_dma_buffer[0], pos);
		}
		/* Save current position as old for next transfers */
		uart->rx_data->old_pos = pos;
	}
}

/**
 * \brief           Process received data over UART
 * Data are written to RX ringbuffer for application processing at latter stage
 * \param[in]       uart: Uart Handler
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void usart_process_data(const uart_handler_t* uart, const void* data, size_t len) {
    lwrb_write(&uart->rx_data->usart_rx_rb, data, len);  /* Write data to receive buffer */
}

/**
 * \brief           Check if DMA is active and if not try to send data
 * \return          `1` if transfer just started, `0` if on-going or no data to transmit
 */
uint8_t usart_start_tx_dma_transfer(const uart_handler_t* uart) {
	uint8_t started = 0;
    if (uart->tx_data->usart_tx_dma_current_len == 0
            && (uart->tx_data->usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&uart->tx_data->usart_tx_rb)) > 0) {
        /* Disable channel if enabled */
        LL_DMA_DisableChannel(uart->dma_tx, uart->dma_tx_ch);
        /* Clear all flags */
        uart->dma_tx_clear_tc_fn(uart->dma_tx);
        uart->dma_tx_clear_ht_fn(uart->dma_tx);
        uart->dma_tx_clear_gi_fn(uart->dma_tx);
        uart->dma_tx_clear_te_fn(uart->dma_tx);
        /* Prepare DMA data and length */
        LL_DMA_SetDataLength(uart->dma_tx, uart->dma_tx_ch, uart->tx_data->usart_tx_dma_current_len);
        LL_DMA_SetMemoryAddress(uart->dma_tx, uart->dma_tx_ch, (uint32_t)lwrb_get_linear_block_read_address(&uart->tx_data->usart_tx_rb));
        /* Start transfer */
        LL_DMA_EnableChannel(uart->dma_tx, uart->dma_tx_ch);
        LL_USART_EnableDMAReq_TX(uart->uart);
        started = 1;
    }
    return started;
}

/**
 * \brief           Send string over USART
 * \param[in]       uart: Uart Handler
 * \param[in]       str: String to send
 */
bool usart_send_string(const uart_handler_t* uart, const char* str, size_t len) {
	/* Write data to transmit buffer */
    lwrb_write(&uart->tx_data->usart_tx_rb, str, len);
    /* Start transfer */
    return usart_start_tx_dma_transfer(uart);
}

/* USER CODE END 1 */
