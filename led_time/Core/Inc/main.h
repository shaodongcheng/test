/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define Y_R_Pin GPIO_PIN_5
#define Y_R_GPIO_Port GPIOA
#define Y_Y_Pin GPIO_PIN_6
#define Y_Y_GPIO_Port GPIOA
#define Y_G_Pin GPIO_PIN_7
#define Y_G_GPIO_Port GPIOA
#define X_R_Pin GPIO_PIN_4
#define X_R_GPIO_Port GPIOC
#define X_Y_Pin GPIO_PIN_5
#define X_Y_GPIO_Port GPIOC
#define L_G_Pin GPIO_PIN_0
#define L_G_GPIO_Port GPIOB
#define L_B_Pin GPIO_PIN_1
#define L_B_GPIO_Port GPIOB
#define X_G_Pin GPIO_PIN_6
#define X_G_GPIO_Port GPIOC
#define L_R_Pin GPIO_PIN_5
#define L_R_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LED_OFF 1  //关灯
#define LED_ON 0  //开灯
#define SECOND_COEFFICIENT 1  //时间系数（进入一次定时器为1S则为1，500ms则为2,以此类推）
#define S_LED_RED_GREEN_TIME 5  //红绿灯状态时间,s
#define S_LED_RED_YELLO_TIME 3  //黄灯和红灯状态时间,s

#define LED_YELLOW_1 0  //黄灯状态1
#define LED_YELLOW_2 1  //黄灯状态2
#define LED_YELLOW_3 2  //黄灯状态3

typedef enum
{
	S_XgreenYred=0,  //整体状态状态0
	S_XyellowYred,
	S_XredYgreen,
	S_XredYyellow,
}state_flag_t;

typedef struct
{
	uint16_t S_XgreenYred_time;  //状态零持续的时间
	uint16_t S_XyellowYred_time;
	uint16_t S_XredYgreen_time;
	uint16_t S_XredYyellow_time;
}S_led_time_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
