/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <iostream>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t preamble_buf_init[PREAMBLE_LEN]={PREAMBLE_BUF_INIT};  //固定数据头

uint8_t send_buf[MAX_SEND_RECV_LEN]={0};  //发送（向PC发送响应的回复）缓存区
uint8_t receive_buf[MAX_SEND_RECV_LEN]={0};  //接收（接收来自PC的命令）缓存区

volatile uint32_t rx_len = ZERO_INIT;

uint8_t rx_buffer = ZERO_INIT;

volatile uint32_t rx_time = ZERO_INIT;
volatile uint32_t rx_time_flag = ZERO_INIT;
volatile uint32_t rx_flag = ZERO_INIT;

uint8_t reset_flag = 0;
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
  * @brief  这是用于把uint16_t字节序倒置的函数
  * @param[in] low_val 要转换字节序的uint16_t数据
  * @retval uint16_t 返回字节序倒置后的数据
  */
uint16_t ltoh(uint16_t low_val)
{
	uint16_t high_val = 0;
	uint8_t p[2] = {0};
	p[0] = low_val >> 8;
	p[1] = low_val;
	return *(uint16_t *)p;
}

//读取指定地址的半字(16 位数据)
//faddr:读地址
//返回值:对应数据.
uint32_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
	return *(uint32_t*)faddr;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*  定义并初始化协议内容 */
	command_t command;
	strcpy((char *)command.preamble_buf, (char *)preamble_buf_init);
	command.sw_buf.sw1 = SW1_INIT;
	command.sw_buf.sw2 = SW2_INIT;
	command.sw_buf.sw3 = SW3_INIT;
	uint16_t data_len = 0;	//数据长度变量
	
	FLASH_EraseInitTypeDef clean;	//清除用的结构体
	clean.NbPages = 1;
	clean.TypeErase = FLASH_TYPEERASE_PAGES;
	clean.PageAddress = BPS_ADDR;
	uint32_t PageError = 0;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  
  HAL_FLASH_Unlock();//解锁函数
  if(STMFLASH_ReadHalfWord(BPS_ADDR) == PARA_HIGH || STMFLASH_ReadHalfWord(BPS_ADDR) == PARA_LOW)
  {
	huart2.Init.BaudRate = STMFLASH_ReadHalfWord(BPS_ADDR); //设置波特率
  }
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_FLASH_Lock();
  //huart2.Init.BaudRate = 9600; //设置波特率（测试用代码，已经被注释）
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart2, (uint8_t *)"12138", 6, TIMEOUT);
  HAL_UART_Receive_IT(&huart2, &rx_buffer, RX_BYTE);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if(rx_time >= FRAME_SPACE_TIME)
        {
            rx_flag = FLAG_ON;
            rx_time = ZERO_INIT;
        }
		if(strncmp((char *)receive_buf, (char *)command.preamble_buf, PREAMBLE_LEN) == 0 && rx_flag)
		{
             /*------- 计算data部分的长度 -------*/
            data_len = receive_buf[PREAMBLE_LEN] * (1 << 8) + receive_buf[PREAMBLE_LEN + sizeof(command.len1_buf)] - FIXED_LEN;
            if(data_len >= MAX_SEND_RECV_LEN || data_len <= ZERO_INIT)
            data_len = ZERO_INIT;
            if(rx_len != (receive_buf[PREAMBLE_LEN] * (1 << 8) + receive_buf[PREAMBLE_LEN + sizeof(command.len1_buf)] + PREAMBLE_LEN) + DATA_LEN_BIT)
            {
                HAL_UART_Transmit(&huart2, "len err", 10, TIMEOUT);
                rx_len = ZERO_INIT;
                /*------- 产生错误，缓冲区清零准备下次接收和发送 ---------*/
                memset(send_buf, ZERO_INIT, MAX_SEND_RECV_LEN);
                memset(receive_buf, ZERO_INIT, MAX_SEND_RECV_LEN);
                /*------- 由于该位需要异或所以每次都需要清零 ---------*/
                memset( command.check_buf, ZERO_INIT, CHECK_LEN);
                continue;
            }
            //校验操作
            for(int i = PREAMBLE_LEN + DATA_LEN_BIT; i<PREAMBLE_LEN + DATA_LEN_BIT + sizeof(command.cmd_buf) + sizeof(command.para_buf) + data_len; i++)
			{
				(command.check_buf[ZERO_INIT]) = (command.check_buf[ZERO_INIT]) ^ receive_buf[i];
			}
            if(command.check_buf[ZERO_INIT] != receive_buf[PREAMBLE_LEN + DATA_LEN_BIT + sizeof(command.cmd_buf) + sizeof(command.para_buf) + data_len])
            {
                HAL_UART_Transmit(&huart2,"check err",10,TIMEOUT);
                rx_len = ZERO_INIT;
                /*------- 产生错误，缓冲区清零准备下次接收和发送 ---------*/
                memset(send_buf, ZERO_INIT, MAX_SEND_RECV_LEN);
                memset(receive_buf, ZERO_INIT, MAX_SEND_RECV_LEN);
                /*------- 由于该位需要异或所以每次都需要清零 ---------*/
                memset( command.check_buf, ZERO_INIT, CHECK_LEN);
                continue;
            }
            //判断命令
			switch(receive_buf[PREAMBLE_LEN + sizeof(command.len1_buf) * DATA_LEN_BIT])
			{
				case CMD_NONE:
                    //空操作
					break;
				case CMD_BPS:
					//内存操作相关代码
					if(receive_buf[PREAMBLE_LEN + sizeof(command.len1_buf) * DATA_LEN_BIT + sizeof(command.cmd_buf)] == CMD_PARA_HIGH)
					{
                        HAL_FLASH_Unlock();
						HAL_FLASHEx_Erase(&clean, &PageError);
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BPS_ADDR, PARA_HIGH);//FLASH 写操作函数（写入115200）
                        HAL_FLASH_Lock();
                        reset_flag = 1;
					}
					else if(receive_buf[PREAMBLE_LEN + sizeof(command.len1_buf) * DATA_LEN_BIT + sizeof(command.cmd_buf)] == CMD_PARA_LOW)
					{
                        HAL_FLASH_Unlock();
						HAL_FLASHEx_Erase(&clean, &PageError);
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, BPS_ADDR, PARA_LOW);//FLASH 写操作函数（写入9600）
                        HAL_FLASH_Lock();
                        reset_flag = 1;
					}
					break;
			}
			/*------- 起始的五位 -------*/
			strncpy((char *)send_buf, (char *)command.preamble_buf, PREAMBLE_LEN);
			/*------- 三位sw状态位 ---------*/
			send_buf[PREAMBLE_LEN + DATA_LEN_BIT] = SW1_INIT;
			send_buf[PREAMBLE_LEN + DATA_LEN_BIT + sizeof(command.sw_buf.sw1)] = SW2_INIT;
			send_buf[PREAMBLE_LEN + DATA_LEN_BIT + sizeof(command.sw_buf.sw1)+ sizeof(command.sw_buf.sw2)] = SW3_INIT;
			/*------- 一位校验位 ---------*/
			for(int i = PREAMBLE_LEN + DATA_LEN_BIT; i<PREAMBLE_LEN + sizeof(command.len1_buf) * DATA_LEN_BIT + sizeof(sw_t) + data_len; i++)
			{
				send_buf[PREAMBLE_LEN + DATA_LEN_BIT + sizeof(sw_t) \
					+ data_len] = send_buf[PREAMBLE_LEN +  DATA_LEN_BIT + sizeof(sw_t) + data_len] ^ send_buf[i];
			}
			
			/*------- 两位长度（len位）位 ---------*/
			*((uint16_t *)(send_buf + PREAMBLE_LEN)) = ltoh(sizeof(command.check_buf) + sizeof(sw_t) + data_len);
			/*------- 发送 ---------*/
			HAL_UART_Transmit(&huart2, send_buf, SEND_LEN + data_len, TIMEOUT);
            /*------- 发送完成，缓冲区清零准备下次接收和发送 ---------*/
			memset(send_buf, ZERO_INIT, MAX_SEND_RECV_LEN);
			memset(receive_buf, ZERO_INIT, MAX_SEND_RECV_LEN);
            /*------- 由于该位需要异或所以每次都需要清零 ---------*/
            memset( command.check_buf, ZERO_INIT, CHECK_LEN);
            rx_len = ZERO_INIT;
#if RESET
            if(reset_flag == 1)
            {
                __ASM volatile ("cpsid i"); /* 关闭所有中断 ENABLE_INT*/
                HAL_NVIC_SystemReset();          	/* 重启 */
            }
#endif
		}
        /*------- 延时100ms，避免过快读写导致数据覆盖等问题 ---------*/
	    //HAL_Delay(10);
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
/*
 *测试用重写printf（并未使用）
*/
int fputc(int ch ,FILE *f)
{
	while(((USART2->SR)&0x40)==0);
	USART2->DR=ch;
	return ch;
}
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
