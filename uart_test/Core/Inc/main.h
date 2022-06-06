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
#define PREAMBLE_LEN 5  //����ͷ����
#define PREAMBLE_BUF_INIT 0XAA,0XAA,0XAA,0X96,0X69  //����ͷ�̶�����
#define MAX_DATA_LEN 64  //������󳤶�
#define PARA_HIGH 115200u  //�ϸ߲�����
#define PARA_LOW 9600u  //�ϵͲ�����

#define CMD_PARA_HIGH 0x00  //�ϸ߲�����
#define CMD_PARA_LOW 0x04  //�ϵͲ�����

#define SW1_INIT 0X00  //�̶�״̬��1
#define SW2_INIT 0X00  //�̶�״̬��2
#define SW3_INIT 0X90  //�̶�״̬��3
#define MAX_SEND_RECV_LEN 128  //���ͽ�����󳤶�  
#define TIMEOUT 1000  //���ͽ��ճ�ʱ��λms
#define CMD_NONE 0X10  //��ʵ�ʲ���������
#define CMD_BPS 0X60  //���ò����ʵ�����
#define BPS_ADDR 0x0801FC00ul //��Ų����ʵĵ�ַ

#define ZERO_INIT 0  //����궨��
#define DATA_LEN_BIT 2 //���ݳ���λ��λ����ĿǰΪ��λ
#define FIXED_LEN 3 //��������ͷ��lenλ��data�����λ�õĳ���
#define CHECK_LEN 1 //У��λ����
#define SW3_LEN 3
#define SEND_LEN DATA_LEN_BIT + CHECK_LEN + PREAMBLE_LEN + SW3_LEN //ÿ��Ҫ���͵ĳ���

#define FLAG_ON 1
#define RX_BYTE 1 //�ж�ÿ�ν��ܵ��ֽ���
#define FRAME_SPACE_TIME 2 //֡�����ʱʱ�䣬��ǰ��ʱ����ʱΪ5���룬���Գ�ʱʱ��Ϊ5*2=10ms
#define RESET 0  //�޸���ɲ�����ʱ���Ƿ��Զ�����0���ֶ����¸�λ������   1���޸�����Զ�����
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
typedef struct sw  //״̬�ֽṹ��
{
	uint8_t sw1;  //״̬��1
	uint8_t sw2;  //״̬��2
	uint8_t sw3;  //״̬��3
}sw_t;
typedef struct buff  //Э��ṹ��
{
	uint8_t preamble_buf[PREAMBLE_LEN];  //����ͷ
	uint8_t len1_buf;
	uint8_t len2_buf;
	uint8_t cmd_buf;  //����
	uint8_t para_buf;  //�������
	uint8_t data_buf[MAX_DATA_LEN];  //����
	uint8_t check_buf[CHECK_LEN];  //У��λ
	sw_t sw_buf;  //״̬��
}command_t; 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
