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

#define PB12_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET)
#define PB12_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET)

#define PB13_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET)
#define PB13_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET)

#define PB15_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET)
#define PB15_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET)

#define PC13_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET)
#define PC13_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET)

#define PC14_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET)
#define PC14_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET)

#define PC15_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_RESET)
#define PC15_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET)

#define PD0_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_RESET)
#define PD0_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_SET)

#define TDO()  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
#define TDI(a) 	((a==1)?PB15_1 : PB15_0)
#define TCK(a) 	((a==1)?PB13_1 : PB13_0)
#define TMS(a) 	((a==1)?PB12_1 : PB12_0)

#define LED1(a) ((a!=0)?PC13_1 : PC13_0)
#define LED2(a) ((a!=0)?PC14_1 : PC14_0)
#define LED3(a) ((a!=0)?PC15_1 : PC15_0)

#define CS(a) 	((a==1)?PD0_1  : PD0_0)
#define WDOG(a) ((a!=0)?PB5_1  : PB5_0)


#define u64 unsigned long long
#define u32 unsigned int
#define u16    unsigned short
#define u8     uint8_t
#define uint8  uint8_t
#define UART 	1
#define ETH		2

typedef enum bool {
	true    = 0x1,
	false   = 0x0
}bool;
//---------------------------------------------------------------------

// USART1 Receiver buffer
#define Bufer_size   8192u   //16384
#define RX_BUFFER_SIZE1 64u

#define buf_IO   		32u 
#define buf_Word 		32u 
#define buf_DATA_Word 	256u 
#define BUFFER_SR 		200u
#define BUF_STR 		64
#define MAX_PL 			157u

//------------------------------------------------
/*
#define WRITE_ENABLE	0b00000110
#define WRITE_DISABLE	0b00000100
#define READ_STATUS		0b00000101
#define READ_BYTES		0b00000011   //address bytes:3 (25 MHz)
#define READ_ID			0b10101011	 //Dummy bytes  :3
#define FAST_READ		0b00001011   //address bytes:3 , Dummy bytes:1        (40 MHz)
#define WRITE_STATUS	0b00000001
#define WRITE_BYTES		0b00000010	 //address bytes:3 , Data bytes :1 to 256 (25 MHz)
#define ERASE_BULK		0b11000111
#define ERASE_SECTOR	0b11011000 	 //address bytes:3 (25 MHz)
#define READ_DEV_ID		0b10011111   //Dummy bytes  :2
*/
#define WRITE_ENABLE	0x6
#define WRITE_DISABLE	0x4
#define READ_STATUS		0x5
#define READ_BYTES		0x3    
#define READ_ID			0xab	  
#define FAST_READ		0x0b    
#define WRITE_STATUS	0x01
#define WRITE_BYTES		0x02	 
#define ERASE_BULK		0xc7
#define ERASE_SECTOR	0xd8	  
#define READ_DEV_ID		0x9f    

//------------------------------------------------

u8 PIN_control_PD13 (void);
u8 PIN_control_PD15 (void);
u32 IO    (char* );
void Menu1(char);
void Delay( unsigned int );
void Transf(char* ); 
void LED_OFF (void);
void spi_EPCS_rd    (u8 ,u8 *,     u32 );		//чтение статуса, без записи адреса но с dummy байтами
void spi_EPCS_read  (u8 ,u32 ,u8 *,u32 );		//чтение с адресом
void spi_EPCS_write (u8 ,u32 ,u8 *,u32 );
void spi_EPCS_wr_ENABLE (void);
u8 EPCS_WR (char *,u8);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
