# STM32_uart_dma_at_parser

## Overview
This repo uses [STM32 UART DMA RX and TX](https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx) for USART DMA Rx and Tx implementation and [Stm32_uart_frame
](https://github.com/TienHuyIoT/stm32_uart_frame) to read and processes the commands coming from usart.

## File Information
* usart.h/.c : This files contains the USART DMA Rx and TX implementation.
* board_prod.h/.c : This files contains the AT parser APIs.
* stm32l4xx_it.h/,c: This files contains the STM32 interruptions and callbacks.

## Usage Guide
The user have to make a STM32 project using CubeMx, add board_prod.h/.c to the project and adapt the usart / interruptions files gererated by the CubeMX. 
Add usart_init to Main loop and then call at_prod_init to start the parser API:
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
  reset_t rst = get_reset_type();
  build_fw_version(&fw_version);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  usart_init(p_huart1);

  /* Enter Production Mode */
  if((rst != RESET_CAUSE_IWDG) && (rst != RESET_CAUSE_WWDG)) at_prod_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
	  printf("while loop\r\n");
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```
