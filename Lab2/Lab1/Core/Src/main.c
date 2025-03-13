/* USER CODE BEGIN Header */
/**
  **************************************************************************
  * @file : main.c
  * @brief : Main program body
  **************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
uint32_t lastToggleTime = 0;
uint32_t lastButtonPress = 0;
uint32_t orangeBlinkPeriod = 500; // Start with 1 sec
uint8_t IncreaseLedState = 1; // Toggle state

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
  * @brief The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Time tracking variables */
  uint32_t prevMillis = HAL_GetTick();
  uint32_t prevOrangeMillis = HAL_GetTick();
  uint32_t prevRedMillis = HAL_GetTick();

  /* Infinite loop */
  while (1)
  {
    uint32_t currentMillis = HAL_GetTick();
    /*
    //Task 1
    // Binary Counting using Orange (PD13) and Red (PD14) LEDs every 1 sec
    if (currentMillis - prevMillis <= 1000) {

     	HAL_GPIO_WritePin(GPIOD, Green_LED_Pin,0);
     	HAL_GPIO_WritePin(GPIOD, Blue_LED_Pin,0);

     }
    if ((currentMillis - prevMillis >= 1000) && (currentMillis - prevMillis <= 2000)) {

    	HAL_GPIO_WritePin(GPIOD, Green_LED_Pin,1);
    	HAL_GPIO_WritePin(GPIOD, Blue_LED_Pin,1);

    }
    if (currentMillis - prevMillis >= 2000) {

     	HAL_GPIO_WritePin(GPIOD, Green_LED_Pin,0);
     	HAL_GPIO_WritePin(GPIOD, Blue_LED_Pin,1);

     }
    if (currentMillis - prevMillis >= 3000) {

         prevMillis = currentMillis;
     }
*/
    /*
    //Task 2
    // Red LED (PD14) Blinking Every 1 Sec
    // Orange LED (PD13) Blinking with Variable Frequency
    if (currentMillis - prevRedMillis >= 500) {
        HAL_GPIO_TogglePin(GPIOD, Red_LED_Pin);
        prevRedMillis = currentMillis;
    }
    if (currentMillis - prevOrangeMillis >= 1500) {
        HAL_GPIO_TogglePin(GPIOD, Orange_LED_Pin);
        prevOrangeMillis = currentMillis;
    }

    */
    /*
    //Task 3
    // Button Handling for Green LED Toggle
    if (HAL_GPIO_ReadPin(GPIOA, User_Button_Pin) == GPIO_PIN_SET) {
    		HAL_GPIO_WritePin(GPIOD, Green_LED_Pin, 1);
    }
    if (HAL_GPIO_ReadPin(GPIOA, User_Button_Pin) != GPIO_PIN_SET) {
    	HAL_GPIO_WritePin(GPIOD, Green_LED_Pin, 0);
    }
     */

    //Task 4
    // Button Handling to Increase Orange LED Blink Period
	if (currentMillis - lastToggleTime >= orangeBlinkPeriod) {
		HAL_GPIO_TogglePin(GPIOD, Orange_LED_Pin);
	    lastToggleTime = currentMillis;
	}
	if (HAL_GPIO_ReadPin(GPIOA, User_Button_Pin) == GPIO_PIN_SET) {
		if (IncreaseLedState == 1){
			orangeBlinkPeriod = orangeBlinkPeriod + 250; // Increase by 1 sec
			IncreaseLedState = 0;
		}
	}
	if (HAL_GPIO_ReadPin(GPIOA, User_Button_Pin) != GPIO_PIN_SET) {
		IncreaseLedState = 1;
	}

  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure GPIO pins : Green (PD12), Orange (PD13), Red (PD14) */
  GPIO_InitStruct.Pin = Green_LED_Pin | Orange_LED_Pin | Red_LED_Pin | Blue_LED_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure GPIO pin : USER_BUTTON_Pin (PA0) */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
