/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f4xx.h"

#define Red_LED_PIN     14
#define Orange_LED_PIN     13
#define GPIOD_CLK_EN()  (RCC->AHB1ENR |= (1 << 3))

volatile uint32_t millis = 0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
///////////void SystemClock_Config(void);
void SysTick_Handler(void) {
    millis++;  // Increment every 1ms (assuming 1ms SysTick configured)
}
void delay_ms(uint32_t delay) {
    uint32_t start = millis;
    while ((millis - start) < delay);  // Wait until delay time is reached
}
void GPIO_Config(void) {
    GPIOD_CLK_EN();  // Enable clock for GPIOD

    GPIOD->MODER &= ~(3 << (Red_LED_PIN * 2));  // Clear mode bits
    GPIOD->MODER |= (1 << (Red_LED_PIN * 2));   // Set as output mode
    GPIOD->OTYPER &= ~(1 << Red_LED_PIN);       // Push-pull output
    GPIOD->OSPEEDR |= (2 << (Red_LED_PIN * 2)); // Medium speed
    GPIOD->PUPDR &= ~(3 << (Red_LED_PIN * 2));  // No pull-up/pull-down

    GPIOD->MODER &= ~(3 << (Orange_LED_PIN * 2));  // Clear mode bits
    GPIOD->MODER |= (1 << (Orange_LED_PIN * 2));   // Set as output mode
    GPIOD->OTYPER &= ~(1 << Orange_LED_PIN);       // Push-pull output
    GPIOD->OSPEEDR |= (2 << (Orange_LED_PIN * 2)); // Medium speed
    GPIOD->PUPDR &= ~(3 << (Orange_LED_PIN * 2));  // No pull-up/pull-down
}
void SysTick_Config_Custom(void) {
    SysTick->LOAD = (16000 - 1);  // Assuming 16 MHz clock => 1ms tick
    SysTick->VAL = 0;
    SysTick->CTRL = 7;  // Enable SysTick, use system clock, enable interrupt
}



////////static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //////HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //////SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //////MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  SysTick_Config_Custom();  // Configure SysTick for 1ms
  GPIO_Config();            // Configure LED pin

  uint32_t prevMillis14 = 0;
  uint32_t prevMillis13 = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if ((millis - prevMillis14) >= 1000) {  // Check if 1000ms elapsed
	              GPIOD->ODR ^= (1 << Red_LED_PIN);  // Toggle LED
	              prevMillis14 = millis;
	          }
	  if ((millis - prevMillis13) >= 2000) {  // Check if 1000ms elapsed
	     GPIOD->ODR ^= (1 << Orange_LED_PIN);  // Toggle LED
	  	    prevMillis13 = millis;
	  	 }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

//static void MX_GPIO_Init(void)
//{
  //GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  //__HAL_RCC_GPIOA_CLK_ENABLE();
 // __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOD, Green_LED_Pin|Orange_LED_Pin|Red_LED_Pin|Blue_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  //GPIO_InitStruct.Pin = User_Button_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED_Pin Orange_LED_Pin Red_LED_Pin Blue_LED_Pin */
  //GPIO_InitStruct.Pin = Green_LED_Pin|Orange_LED_Pin|Red_LED_Pin|Blue_LED_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
