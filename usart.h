/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define UART1_BUFFER_SIZE 64
#define UART3_BUFFER_SIZE 2048

#define DEFAULT_UART_TIMEOUT 500

typedef struct {
	UART_HandleTypeDef* p_huart;
	volatile uint32_t* p_cndtr;
	uint8_t* p_rxBuf;
	uint16_t rxBufSize;
	uint16_t rxPos;
} huart_port_t;

extern huart_port_t huart_p1;
extern huart_port_t huart_p3;

#define USER_HUART huart_p1
#define NBIOT_HUART huart_p3

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void uart_deinit(huart_port_t* huart);
void uart_wait_tx_done(huart_port_t* huart);
void uart_write_byte(huart_port_t* huart, void *byte);
void uart_write_string(huart_port_t* huart, void *p_buffer, uint16_t size);
bool uart_read_byte(huart_port_t* huart, void *byte);
bool uart_read_string(huart_port_t* huart, void *p_dest, size_t len);
bool uart_peek(huart_port_t* huart, void *p_dest, size_t len);
void uart_flush_rx(huart_port_t* huart);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

