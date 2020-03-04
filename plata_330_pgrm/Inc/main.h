/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

#define PB5_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET)
#define PB5_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET)

#define PC13_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET)
#define PC13_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET)

#define PC14_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET)
#define PC14_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET)

#define PC15_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_RESET)
#define PC15_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET)

#define PD0_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_RESET)
#define PD0_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_SET)


#define LED1(a) ((a!=0)?PC13_1 : PC13_0)
#define LED2(a) ((a!=0)?PC14_1 : PC14_0)
#define LED3(a) ((a!=0)?PC15_1 : PC15_0)

#define CS(a) 	((a==1)?PD0_1 : PD0_0)
#define WDOG(a) ((a!=0)?PB5_1 : PB5_0)

#define u64 unsigned long long
#define u32 unsigned int
#define u16    unsigned short
#define u8     uint8_t
#define uint8  uint8_t
//---------------------------------------------------------------------

// USART1 Receiver buffer
#define Bufer_size   8192u   //16384
#define RX_BUFFER_SIZE1 64u

#define buf_IO   		32u 
#define buf_Word 		32u 
#define buf_DATA_Word 	200u 
#define BUFFER_SR 		200u
#define BUF_STR 		64
#define MAX_PL 			157u

//------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
