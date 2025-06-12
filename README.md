# STM32_uart_dma_at_parser

## Overview
This repo uses [STM32 UART DMA RX and TX](https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx) for USART DMA Rx and Tx implementation and [Stm32_uart_frame
](https://github.com/TienHuyIoT/stm32_uart_frame) to read and processes the commands coming from usart.

## File Information
* usart.h/.c : This files contains the USART DMA Rx and TX implementation.
* nbiot.h/.c : This files contains the AT parser APIs.
* stm32l4xx_it.h/.c: This files contains the STM32 interruptions and callbacks.

## Usage Guide
The user have to make a STM32 project using CubeMx, add nbiot.h/.c to the project and adapt the usart / interruptions files gererated by the CubeMX. 
Add nbiot_st_mch to Main loop to start the parser API:
```
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
    nbiot_st_mch();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```
