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
#define PREAMBLE_LEN 5  //数据头长度
#define PREAMBLE_BUF_INIT 0XAA,0XAA,0XAA,0X96,0X69  //数据头固定内容
#define MAX_DATA_LEN 64  //数据最大长度
#define PARA_HIGH 115200u  //较高波特率
#define PARA_LOW 9600u  //较低波特率

#define CMD_PARA_HIGH 0x00  //较高波特率
#define CMD_PARA_LOW 0x04  //较低波特率

#define SW1_INIT 0X00  //固定状态字1
#define SW2_INIT 0X00  //固定状态字2
#define SW3_INIT 0X90  //固定状态字3
#define MAX_SEND_RECV_LEN 128  //发送接收最大长度  
#define TIMEOUT 1000  //发送接收超时单位ms
#define CMD_NONE 0X10  //无实际操作的命令
#define CMD_BPS 0X60  //设置波特率的命令
#define BPS_ADDR 0x0801FC00ul //存放波特率的地址

#define ZERO_INIT 0  //清零宏定义
#define DATA_LEN_BIT 2 //数据长度位的位数，目前为两位
#define FIXED_LEN 3 //除了数据头和len位和data以外的位置的长度
#define CHECK_LEN 1 //校验位长度
#define SW3_LEN 3
#define SEND_LEN DATA_LEN_BIT + CHECK_LEN + PREAMBLE_LEN + SW3_LEN //每次要发送的长度

#define FLAG_ON 1
#define RX_BYTE 1 //中断每次接受的字节数
#define FRAME_SPACE_TIME 2 //帧间隔超时时间，当前定时器延时为5毫秒，所以超时时间为5*2=10ms
#define RESET 0  //修改完成波特率时，是否自动重启0：手动按下复位键重启   1：修改完成自动重启
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
/* USER CODE BEGIN Private defines */
typedef struct sw  //状态字结构体
{
	uint8_t sw1;  //状态字1
	uint8_t sw2;  //状态字2
	uint8_t sw3;  //状态字3
}sw_t;
typedef struct buff  //协议结构体
{
	uint8_t preamble_buf[PREAMBLE_LEN];  //数据头
	uint8_t len1_buf;
	uint8_t len2_buf;
	uint8_t cmd_buf;  //命令
	uint8_t para_buf;  //命令参数
	uint8_t data_buf[MAX_DATA_LEN];  //数据
	uint8_t check_buf[CHECK_LEN];  //校验位
	sw_t sw_buf;  //状态字
}command_t; 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
