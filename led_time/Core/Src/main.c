/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention		: 邵东成
  * @data			: 20220530
  * @version		:当前文件版本号(V1.0.0)
  * @copyright		: USR-IOT
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
volatile uint32_t led_yellow_stat = LED_YELLOW_1;  //初始化黄灯状态标志位
extern uint32_t new_time;
extern uint32_t old_time;
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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @fn S_XgreenYred_func
  * @brief 这是一个交通灯状态0的函数
  * @details 直接调用
  * @param void
  * @return 无
  * @retval 无
  * @see 扇入：HAL库相关写引脚函数
  * @see 无
  * @note 
  * @attention 
  * @par example:
  * @code
  * @endcode
  */
void S_XgreenYred_func()
{
	HAL_GPIO_WritePin(GPIOA,Y_R_Pin|Y_Y_Pin|Y_G_Pin,LED_OFF);
	HAL_GPIO_WritePin(GPIOC,X_R_Pin|X_Y_Pin|X_G_Pin,LED_OFF);
	HAL_GPIO_WritePin(GPIOC,X_G_Pin,LED_ON);
	HAL_GPIO_WritePin(GPIOA,Y_R_Pin,LED_ON);
}

/**
  * @fn S_XyellowYred_func
  * @brief 这是一个交通灯状态1的函数
  * @details 直接调用
  * @param void
  * @return 无
  * @retval 无
  * @see 扇入：HAL库相关写引脚函数
  * @see 无
  * @note 
  * @attention 
  * @par example:
  * @code
  * @endcode
  */
void S_XyellowYred_func()
{
	HAL_GPIO_WritePin(GPIOC,X_G_Pin,LED_OFF);
	led_yellow_stat = new_time;
	switch(led_yellow_stat)
	{
		case LED_YELLOW_1:
			HAL_GPIO_WritePin(GPIOC,X_Y_Pin,LED_ON);
			break;
		case LED_YELLOW_2:
			HAL_GPIO_WritePin(GPIOC,X_Y_Pin,LED_OFF);
			break;
		case LED_YELLOW_3:
			HAL_GPIO_WritePin(GPIOC,X_Y_Pin,LED_ON);
			break;
	}
}

/**
  * @fn S_XredYgreen_func
  * @brief 这是一个交通灯状态2的函数
  * @details 直接调用
  * @param void
  * @return 无
  * @retval 无
  * @see 扇入：HAL库相关写引脚函数
  * @see 无
  * @note 
  * @attention 
  * @par example:
  * @code
  * @endcode
  */
void S_XredYgreen_func()
{
	HAL_GPIO_WritePin(GPIOA,Y_R_Pin|Y_Y_Pin|Y_G_Pin,LED_OFF);
	HAL_GPIO_WritePin(GPIOC,X_R_Pin|X_Y_Pin|X_G_Pin,LED_OFF);
	HAL_GPIO_WritePin(GPIOC,X_R_Pin,LED_ON);
	HAL_GPIO_WritePin(GPIOA,Y_G_Pin,LED_ON);
}

/**
  * @fn S_XredYyellow_func
  * @brief 这是一个交通灯状态3的函数
  * @details 直接调用
  * @param void
  * @return 无
  * @retval 无
  * @see 扇入：HAL库相关写引脚函数
  * @see 无
  * @note 
  * @attention 
  * @par example:
  * @code
  * @endcode
  */
void S_XredYyellow_func()
{
	HAL_GPIO_WritePin(GPIOA,Y_G_Pin,LED_OFF);
	led_yellow_stat = new_time;
	switch(led_yellow_stat)
	{
		case LED_YELLOW_1:
			HAL_GPIO_WritePin(GPIOA,Y_Y_Pin,LED_ON);
			break;
		case LED_YELLOW_2:
			HAL_GPIO_WritePin(GPIOA,Y_Y_Pin,LED_OFF);
			break;
		case LED_YELLOW_3:
			HAL_GPIO_WritePin(GPIOA,Y_Y_Pin,LED_ON);
			break;
	}
}
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
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	
	S_led_time_t S_led_time; 
	state_flag_t state_flag = S_XgreenYred;
	
	S_led_time.S_XgreenYred_time = S_LED_RED_GREEN_TIME * SECOND_COEFFICIENT;
	S_led_time.S_XyellowYred_time = S_LED_RED_YELLO_TIME * SECOND_COEFFICIENT;
	S_led_time.S_XredYgreen_time = S_LED_RED_GREEN_TIME * SECOND_COEFFICIENT;
	S_led_time.S_XredYyellow_time = S_LED_RED_YELLO_TIME * SECOND_COEFFICIENT;
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	switch(state_flag)
	{
		case S_XgreenYred:
			S_XgreenYred_func();
			if(new_time >= S_led_time.S_XgreenYred_time)
			{
				state_flag = S_XyellowYred;
				new_time = INIT_NEW_TIME;
			}
			break;
		case S_XyellowYred:
			S_XyellowYred_func();
			if(new_time >= S_led_time.S_XyellowYred_time)
			{
				state_flag = S_XredYgreen;
				new_time = INIT_NEW_TIME;
			}
			break;
		case S_XredYgreen:
			S_XredYgreen_func();
			if(new_time >= S_led_time.S_XredYgreen_time)
			{
				state_flag = S_XredYyellow;
				new_time = INIT_NEW_TIME;
			}
			break;
		case S_XredYyellow:
			S_XredYyellow_func();
			if(new_time >= S_led_time.S_XredYyellow_time)
			{
				state_flag = S_XgreenYred;
				new_time = INIT_NEW_TIME;
			}
			break;
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
