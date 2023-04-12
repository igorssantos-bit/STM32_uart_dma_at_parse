/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#include "lwrb.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

typedef struct {
	uint8_t usart_rx_dma_buffer[64];
	size_t old_pos;
	lwrb_t usart_rx_rb;
	uint8_t usart_rx_rb_data[256];
} uart_rx_buffer_t;

typedef struct {
	lwrb_t usart_tx_rb;
	uint8_t usart_tx_rb_data[256];
	size_t usart_tx_dma_current_len;
} uart_tx_buffer_t;

typedef struct {
	/* UART config */
	USART_TypeDef* uart;                        /*!< UART/USART/LPUART instance */
    uint32_t baud_rate;
	GPIO_TypeDef* uart_tx_port;
    GPIO_TypeDef* uart_rx_port;
    uint16_t uart_tx_pin;
    uint16_t uart_rx_pin;
    uint16_t uart_tx_pin_af;
    uint16_t uart_rx_pin_af;
	/* DMA config & flags management */
    DMA_TypeDef* dma_rx;                        /*!< RX DMA instance */
    uint32_t dma_rx_ch;                         /*!< RX DMA channel */
    uint32_t dma_rx_req;                        /*!< RX DMA request */
	DMA_TypeDef* dma_tx;                        /*!< TX DMA instance */
    uint32_t dma_tx_ch;                         /*!< TX DMA channel */
    uint32_t dma_tx_req;                        /*!< TX DMA request */
	uint32_t (*dma_rx_is_ht_fn)(DMA_TypeDef *);
	uint32_t (*dma_rx_is_tc_fn)(DMA_TypeDef *);
    void (*dma_rx_clear_tc_fn)(DMA_TypeDef *);
    void (*dma_rx_clear_ht_fn)(DMA_TypeDef *);
    uint32_t (*dma_tx_is_tc_fn)(DMA_TypeDef *);
    void (*dma_tx_clear_tc_fn)(DMA_TypeDef *);
    void (*dma_tx_clear_ht_fn)(DMA_TypeDef *);
    void (*dma_tx_clear_gi_fn)(DMA_TypeDef *);
    void (*dma_tx_clear_te_fn)(DMA_TypeDef *);
    /* Interrupts config */
    uint8_t prio;                               /*!< Preemption priority number */
    IRQn_Type uart_irq;                         /*!< UART IRQ instance */
    IRQn_Type dma_rx_irq;                       /*!< DMA RX IRQ instance */
    IRQn_Type dma_tx_irq;                       /*!< DMA RX IRQ instance */
	/* Volatile data */
	uart_rx_buffer_t* rx_data;                 	/*!< Pointer to volatile rx data */
	uart_tx_buffer_t* tx_data;                 	/*!< Pointer to volatile tx data */
} uart_handler_t;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void usart_init(const uart_handler_t* uart);
void usart_dma_irq_handler(const uart_handler_t* uart);
void usart_irq_handler(const uart_handler_t* uart);
void usart_rx_check(const uart_handler_t* uart);
void usart_process_data(const uart_handler_t* uart, const void* data, size_t len);
uint8_t usart_start_tx_dma_transfer(const uart_handler_t* uart);
bool usart_send_string(const uart_handler_t* uart, const char* str, size_t len);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

