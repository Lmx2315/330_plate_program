/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

u32 TIME_SYS;
u32 TIME_TEST;
u32 FLAG_T1;
u32 FLAG_T2;
uint32_t TIMER1;
uint32_t TIMER2;
volatile u32  SysTickDelay; 

uint8_t RX_uBUF[1];
unsigned int timer_DMA2;
u8 flag_pachka_TXT; //
uint16_t  text_lengh;
uint8_t text_buffer[Bufer_size];

volatile char          rx_buffer1[RX_BUFFER_SIZE1];
volatile unsigned int rx_wr_index1,rx_rd_index1,rx_counter1;
volatile u8  rx_buffer_overflow1;


char sr[BUFFER_SR+1];
unsigned char lsr;
unsigned char lk;
unsigned int led_tick;


char  strng[buf_IO];
char      InOut[BUFFER_SR+1];
char      Word [buf_Word];         //
char DATA_Word [buf_DATA_Word];    //
char DATA_Word2[buf_DATA_Word];    //
 
char Master_flag; // 
char lsym;
char  sym;
char flag;
char    NB;
char Adress;  //
char packet_sum;
char crc,comanda;
      
char In_data[BUF_STR];
char ink1; //
char data_in;

u16 lenght;
u16 SCH_LENGHT_PACKET;
	
unsigned     int index1;
unsigned     char crc_ok;
unsigned     char packet_ok;
unsigned     char packet_flag;
unsigned     int indexZ; 
unsigned     int index_word;
unsigned     int index_data_word;
unsigned     int index_data_word2;
unsigned     int lenght_data;//
unsigned     char data_flag;
unsigned     char data_flag2;
unsigned     char FLAG_lenght;//
unsigned     int sch_lenght_data;
unsigned     char FLAG_DATA;
unsigned char FLAG_CW;
u16 time_uart=0; //
unsigned char flag_pcf;
char lsym1;
char pack_ok1;
char pack_sum1;
char sym1;


u8 EVENT_INT0=0;
u8 EVENT_INT1=0;
u8 EVENT_INT2=0;
u8 EVENT_INT3=0;
u8 EVENT_INT4=0;
u8 EVENT_INT5=0;
u8 EVENT_INT6=0;
u8 EVENT_INT7=0;
u8 EVENT_INT8=0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1999;//даёт период 500мкс * 2000 = 1 сек
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;//на частоте 100 МГц даёт период 500 мкс
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10000;//определяет длинну импульса - тут 100 мкс
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;//определяет частоту импульса - тут 100 мс
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}


/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
    /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5, PB12,PB13,PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
    /*Configure GPIO pins : PD13 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
//----------------------------------------------
u8 spi2send8 (u8 d) //8 бит
{
	u8 a1;
	u8  b;
  a1 = (d)      &0xff;
  HAL_SPI_TransmitReceive(&hspi2, &a1, &b,1, 5000); 
  return b; 
}
//----------------------------------------------
u8 spisend8 (u8 d) //8 бит
{
	u8 a1;
	u8  b;
  //HAL_SPI_TransmitReceive(&hspi3, &address, &data, sizeof(data), 5000);

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000); 
  return b; 
}

//----------------------------------------------

u8 spi_EPCS_STATUS (void)			//счтывает статусный байт во флеш , нулевой бит - бит записи, его проверЯем
{
	u8 m[1];
	spi_EPCS_rd(READ_STATUS,m,1);
	return m[0];
}

void spi_EPCS_rd (u8 cmd,u8 d[],u32 n) //чтение данных
{  
   u32 i=0;
   CS(0);
   spisend8(cmd);//
   for (i=0;i<n;i++)  
   {
//	   Transf(".");
	   d[i]=spisend8(0);  
   }
   CS(1);
}

void spi_EPCS_read (u8 cmd,u32 adr,u8 d[],u32 n) //чтение данных
{  
   u32 i=0;
   CS(0);
   spisend8(cmd);//
   spisend8((adr>>16)&0xff);//
   spisend8((adr>> 8)&0xff);//
   spisend8( adr     &0xff);//
   for (i=0;i<n;i++)  
   {
	   Transf(".");
	   d[i]=spisend8(0);  
   }
   CS(1);
}

void spi_EPCS_write (u8 cmd,u32 adr,u8 d[],u32 n) //запись данных в один сектор - 256 байт!!!
{  
   u32 i=0;
   CS(0);  
   spisend8(cmd);//
   spisend8((adr>>16)&0xff);//
   spisend8((adr>> 8)&0xff);//
   spisend8( adr     &0xff);//
   
   for (i=0;i<n;i++)  
   {
	   Transf(".");
	   spisend8(d[i]);  
   }
   CS(1);
}
void spi_EPCS_ERASE_BULK (void) //разрешение записи во флеш
{  
   CS(0);  
   spisend8(ERASE_BULK);//
   CS(1);
}

void spi_EPCS_wr_ENABLE (void) //разрешение записи во флеш
{  
   CS(0);  
   spisend8(WRITE_ENABLE);//
   CS(1);
}

void spi_EPCS_wr_DISABLE (void) //разрешение записи во флеш
{  
   CS(0);  
   spisend8(WRITE_DISABLE);//
   CS(1);
}
//------------------------------------------------
void Delay( unsigned int Val)  
{  
   SysTickDelay = Val;  
   while (SysTickDelay != 0) {};  
}
//----------------------------------------------
volatile void delay_us( uint32_t time_delay)
{	
	time_delay=time_delay*10;
    while(time_delay--)	;
} 
//-------------------------------------------
unsigned int leng ( char *s)
{
  unsigned  char i=0;
  while ((s[i]!='\0')&&(i<120)) { i++;}
  return i;
}

void Transf(const char* s)  // процедура отправки строки символов в порт
{
  unsigned  short l=0;
  unsigned  short i=0;
         
  if ((flag_pachka_TXT==0) )
  {
    l=strlen(s);
    if ((text_lengh+l)>Bufer_size-5) text_lengh=0u;
    for (i=text_lengh;i<(text_lengh+l);i++) text_buffer[i]=s[i-text_lengh];
    text_lengh=text_lengh+l;
  } 
}

void itoa(int val,  char *bufstr, int base) //
{
    u8 buf[32] = {0};
    int i = 30;
    int j;
    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];
    i++; j=0;
    while (buf[i]!=0){ bufstr[j]=buf[i]; i++; j++;}
}

void f_out (char s[],double a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%.2f",a);
   Transf(strng);
   Transf ("\r\n");
}

void d_out (char s[],int a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%d",a);
   Transf(strng);
   Transf ("\r");
}

void u_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   Transf ("\r");
}

void un_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
}

void nu_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   //Transf ("\r\n");
}

void x32_out (char s[],u32 a)
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void x_out (char s[],u32 a)//было u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
// if (a<10) Transf("0x0"); else Transf("0x");
   Transf(strng);
   Transf ("\r\n");
}

void xn_out (char s[],u32 a)//было u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   if (a<10) Transf("0");
   Transf(strng);   
}

void xnz_out (char s[],u32 a)// 
{
   Transf (s);
   sprintf (strng,"%X",a);
   if (a<0x0f) Transf("0");
   Transf(strng);   
}

void xn4z_out (char s[],u32 a)// 
{
   Transf (s);
   sprintf (strng,"%X",a);
   if (a<0x00f)  Transf("000");else
   if (a<0x0ff)  Transf("00") ;else
   if (a<0xfff)  Transf("0")  ;
   Transf(strng);   
}

void in_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);   
}

void i_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);
   Transf ("\r\n");   
}

void hn_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);   
}

void h_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);  
   Transf ("\r\n");   
}

void UART_DMA_TX (void)
{
 uint16_t k;

if (HAL_UART_GetState(&huart1)!=HAL_UART_STATE_BUSY_TX )
{
	if ((flag_pachka_TXT==0)&&(text_lengh>1u)&(timer_DMA2>250))
	 {
		k = text_lengh;
		HAL_UART_Transmit_DMA(&huart1,text_buffer,k);
		text_lengh=0u;  //обнуление счётчика буфера 
		flag_pachka_TXT=1; //устанавливаем флаг передачи
	  }
  }	
} 

char getchar1(void)
{
   uint8_t data;
   while (rx_counter1 == 0);
   data = rx_buffer1[ rx_rd_index1++ ];
   if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1 = 0;
    --rx_counter1;
    return data;
}
      
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 //-------------------------  
   rx_buffer1[rx_wr_index1++]= (uint8_t) (RX_uBUF[0]& 0xFF); //считываем данные в буфер, инкрементируя хвост буфера
   if ( rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0; //идем по кругу
   	 
	  if (++rx_counter1 == RX_BUFFER_SIZE1) //переполнение буфера
      {
        rx_counter1=0; //начинаем сначала (удаляем все данные)
        rx_buffer_overflow1=1;  //сообщаем о переполнении
      }
 //--------------------------  
   HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
}

//----------------------------------------------
//обаработка пришедшей строки интел-HEX

u32 HIGH_ADDRES_FLASH=0;
u8 EPCS_WR (char m[],u8 port)
{
	u16 j   =0;
	u16 i   =0;
	u16 LL  =0;	//количество байт данных в строке (надо удваивать это значение потому что каждый полубайт передаЮтсЯ байтом)
	u16 AAAA=0;	//поле адреса 
	u16 TT  =0; //поле типа 00-двоичные данные,01-запись ЯвлЯетсЯ концом файла,02-запись адреса сегмента,03-Start Segment Address Record,04-запись расширенного адреса,05-Start Linear Address Record
	u8  DD  =0;
	u8  CC  =0; //контрольнаЯ сумма расчЮтнаЯ
	u8 error=0;
	u32 temp=0;
	u32 ADDRES_FLASH=0;
	char a[3]     ={0};
	char b[5]     ={0};
	u8   DATA[128]={0};
	u8 z  =0; //контрольнаЯ сумма
	char *ch;	
	
//	u8 CC1=0;
//	u8 CC2=0;
	
	if ((m[0]!=':')&&(port==2)) error=1;//проверЯем что строка начинаетсЯ с ":" , если источник PORT - ETH
	else
	{
		Transf(":");
		a[0]=m[1];//LL
		a[1]=m[2];		
		LL=strtol(a,&ch,16);			//длинна строки в HEX файле
		CC=CC+LL;		
//		xnz_out("",LL);
		
		b[0]=m[3];//AAAA
		b[1]=m[4];		
		b[2]=m[5];
		b[3]=m[6];
		
		AAAA=	strtol(b,&ch,16);		//адрес куда поместить двоичные данные из полЯ DDDD, если TT=00
		CC=CC+(AAAA>>8)+(AAAA&0xff);
//		xn4z_out("",AAAA);
		
		a[0]=m[7];//TT
		a[1]=m[8];		
		TT=	strtol(a,&ch,16);			//поле типа	00-двоичные данные,04-запись расширенного адреса
		CC=CC+TT;
//		xnz_out("",TT);
//		CC1=CC;
		if (TT==0x04)  //если пришла строка с расширением адреса, тут просто запоминаем адрес
		{
			b[0]=m[ 9];//DDDD ‘таршее слово смещениЯ адреса (0xDDDD0000)
			b[1]=m[10];		
			b[2]=m[11];
			b[3]=m[12];
	
			temp=	strtol(b,&ch,16);//адрес
			CC=CC+(temp>>8)+(temp&0xff);
//			xn4z_out("",temp);
			
			a[0]=m[13];//‘‘ контрольнаЯ сумма в конце строки
			a[1]=m[14];		
			z=strtol(a,&ch,16);	//
//			xnz_out("",z);
			
			CC=0-CC;
			
//			Transf("\r\n");
//			x_out(" CC:",CC);
//			x_out("CRC:",z);
			if (CC!=z) error=2; else HIGH_ADDRES_FLASH=temp<<16;			
		} 
		else 
		{
			if (LL<512) //проверка на выход за границу массива
			{
				temp=9+(2*LL);
				
				for (i=9;i<temp;i++)//собираем данные 
				{
					a[0]=m[i  ];	//TT
					a[1]=m[++i];		
					DD=	strtol(a,&ch,16);
					CC=CC+DD;    	//расчЮт контрольной суммы
					DATA[j++]=DD;	//переносим данные в промежуточный массив
//					xnz_out("",DD);
				}
			
				a[0]=m[  i];		//‘‘ контрольнаЯ сумма в конце строки
				a[1]=m[++i];		
				z=strtol(a,&ch,16);	//
//				xnz_out("",z);
			} else error=3;
//			CC2=CC;
			CC=0-CC;
			
//			Transf("\r\n");
//			u_out("  j:",j);
//			x_out("CC1:",CC1);
//			x_out("CC2:",CC2);
//			x_out(" CC:",CC);
//			x_out("CRC:",z);			
		
		
			if (CC!=z) error=4; 
			else 
			{
//				Transf("\r\nпрограмируем флеш:\r\n");
				ADDRES_FLASH=HIGH_ADDRES_FLASH+AAAA;				//формируем полный 32-битный адрес записи
				x_out("ADDRES_FLASH:",ADDRES_FLASH);
				
				while (spi_EPCS_STATUS()==1){};	//проверЯем что флеш не занЯта
				spi_EPCS_wr_ENABLE(); 			//разрешаем запись во флеш
				spi_EPCS_write(WRITE_BYTES,ADDRES_FLASH,DATA,LL);
				spi_EPCS_wr_DISABLE();			//запрещаем запись во флеш
//				Transf("\r\n");				
			}
			
		}	
	}
	
	return error;
}

void UART_conrol (void)
{
 u16 i=0;
 u16 j=0;

  if (rx_counter1!=0u)
    {  
//		Transf("*");
      if (rx_counter1<BUFFER_SR) j = rx_counter1; else j=BUFFER_SR;

      for (i=0u;i<j; i++) 
         {
           sr[i]=getchar1();
           lenght=i+1;  
           if (sr[i]==';') {break;}
          }
            sr[lenght]=0x00;
		//	Transf(sr);
            IO (sr);
    };
}

u32 crc_input=0u; 
u32 crc_comp=0u;
u8 mas[300];

u32 IO ( char* str)      // функция обработки протокола обмена
{
//	  u32 ccc;
//	  char *ch;
 unsigned int i=0;
// u8 tmp1;
// u32 lb;
// u64 v1;
// u32 z1,z2;

  i = lenght;//длинна принятой пачки
  if (lenght==0) i = leng(str);
  lenght = 0;
 
  indexZ = 0;
  
  if ((time_uart>50u)||(SCH_LENGHT_PACKET>MAX_PL))
  {
	  //-------------------
		packet_flag=0; 
		//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;
  }
  
  while (i>0u)   //перегрузка принятого пакета в массив обработки
  
  {  

	if ((str[indexZ]==0x7e)&&(packet_flag==0))// обнаружено начало пакета
	  {  
		//-------------------
		packet_flag=1; 
		//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;		
	  }

	 InOut[index1]=str[indexZ];
	 SCH_LENGHT_PACKET++;//подсчитываем длинну пакета
		 
	if (( InOut[index1]==';')&&(FLAG_DATA==0u)&&(packet_flag==1))  {packet_flag=0;packet_ok=1u;FLAG_CW=1u;}
    
	if (((InOut[index1]=='=')||(InOut[index1]==':'))&&(data_flag==0)) {data_flag=1u;FLAG_CW=1u;}

	if (( InOut[index1]=='.')&&(data_flag2==0)&&(FLAG_DATA==0))   {data_flag2=1u; FLAG_CW=1u;}
	
	if (( InOut[index1]=='$')&&(FLAG_lenght==0u)) {FLAG_lenght=2u;FLAG_CW=1u;}
    
	if ((index1>2u)&&(InOut[2]==' ')&&(FLAG_CW==0u)&&(FLAG_lenght<2u))  
            {
                             if   (data_flag!=1u) {Word[index_word]=InOut[index1];} // заполняем командное слово
                      
                             if  ((data_flag==1u)&&(data_flag2==0u))     DATA_Word[index_data_word]=InOut[index1];// заполняем  слово данных1
                             if  ((data_flag==1u)&&(data_flag2==1u))     DATA_Word2[index_data_word2]=InOut[index1]; // заполняем  слово данных2
                    
                             if  (data_flag!=1u)
                                     {if (index_word<buf_Word) index_word++;} 
                                   else 
                                            {
                                             if ((data_flag==1u)&&(data_flag2==0u)) if (index_data_word<buf_DATA_Word)  {index_data_word++;sch_lenght_data++;}
                                            
                                             if ((data_flag==1u)&&(data_flag2==1u)) if (index_data_word2<buf_DATA_Word) index_data_word2++;
                                            }
			}
	
		if ((FLAG_lenght==2u)&&(FLAG_CW==0u)) {lenght_data = (u8)(InOut[index1]);FLAG_lenght=1u;} //запоминаем длинну пакета данных после ":"
	
		if ((sch_lenght_data<lenght_data)&&(FLAG_lenght==1u)) FLAG_DATA = 1u; else {FLAG_DATA = 0u;}
	 
		if (index1<BUFFER_SR)  index1++;
		if (indexZ <BUFFER_SR)  indexZ ++;
		i--;
		FLAG_CW=0u;
	
  }
 

if (packet_ok==1u) 
  {    
      if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // проверка первого условия пакета - начало пакета
      if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // проверка второго условия пакета - адресат назначения
 
if (crc_ok==0x3)  //обработка команд адресатом которых является хозяин 
{

if (strcmp(Word,"JTAG_TST")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_TST\r"    );
     Transf("\r"); 
	 TST ();
   } else	
if (strcmp(Word,"JTAG2_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_SCAN\r"    );
     Transf("\r"); 
	 crc_comp=jtag_scan(NULL); 
//	 u_out("N:",crc_comp);
//   JTAG_SCAN();
   } else
if (strcmp(Word,"JTAG1_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_SCAN\r"    );
     Transf("\r"); 
     JTAG_SCAN();
   } else
if (strcmp(Word,"JTAG_ID")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_ID\r"    );
     Transf("\r");  
     ID_recive (crc_comp);
   } else	
if (strcmp(Word,"help")==0)                     
   {
     Transf ("принял help\r"    );
     Transf("\r");  
     Menu1(0);
   } else
if (strcmp(Word,"EPCS_WR")==0)                     
   {
//   Transf ("принЯл EPCS_WR\r");
//   Transf("\r");  
     x_out("err:",EPCS_WR(DATA_Word,UART));//UART или ETH
   } else	
if (strcmp(Word,"EPCS_ID")==0) //не поддерживаетсЯ EPCS128 !!!
   {
		Transf ("\r\nпринял EPCS_ID:\r\n");
		spi_EPCS_rd(READ_ID,mas,4);
		Transf ("\r\n");
		x_out("mas[3]:",mas[3]);
    } else
if (strcmp(Word,"EPCS_DEV_ID")==0) //only for EPCS128 !!!
   {
		Transf ("\r\nпринял EPCS_DEV_ID:\r\n");
		spi_EPCS_rd(READ_DEV_ID,mas,3);
		Transf ("\r\n");
		if (mas[2]==0x18) Transf("recive: EPCS128\r\n");
		x_out("mas[2]:",mas[2]);
    } else
if (strcmp(Word,"EPCS_STATUS")==0) //
   {
		crc_comp =atoi(DATA_Word);
		crc_input=atoi(DATA_Word2);
		Transf ("\r\nпринял EPCS_STATUS:\r\n");
		spi_EPCS_rd(READ_STATUS,mas,4);
		Transf ("\r\n");
		x_out("mas[0]:",mas[0]);
		x_out("mas[1]:",mas[1]);
		x_out("mas[2]:",mas[2]);
		x_out("mas[3]:",mas[3]);
    }  else
if (strcmp(Word,"EPCS_READ")==0) //
   {
		crc_comp =atoi(DATA_Word);
		crc_input=atoi(DATA_Word2);
		x_out ("\r\nпринял EPCS_READ:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
		
		spi_EPCS_read(READ_BYTES,crc_comp,mas,256);//чтение 256 байт данных
		Transf("\r\n---------------------------\r\n");
		for (i=0;i<256;i++)
		{
			hn_out (mas[i+0],0);hn_out (mas[i+1],0);hn_out (mas[i+2],0);hn_out (mas[i+3],0);
			i=i+3;
			Transf("\r\n");	
		}	
    }else
if (strcmp(Word,"EPCS_WRITE_TEST")==0) //
   {
		crc_comp =atoi(DATA_Word);
		crc_input=atoi(DATA_Word2);
		x_out ("\r\nпринял EPCS_WRITE_TEST:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
		Transf("\r\n---------------------------\r\n");
		
		for (i=0;i<256;i++) mas[i]=i;
			
		spi_EPCS_wr_ENABLE(); //разрешаем запись во флеш
		spi_EPCS_write(WRITE_BYTES,crc_comp,mas,256);
		spi_EPCS_wr_DISABLE();//запрещаем запись во флеш
    }else
if (strcmp(Word,"EPCS_ERASE_SECTOR")==0) //
   {
		crc_comp =atoi(DATA_Word);
		x_out ("\r\nпринял EPCS_ERASE_SECTOR:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ

		spi_EPCS_wr_ENABLE();//разрешаем запись во флеш
		spi_EPCS_write(ERASE_SECTOR,crc_comp,mas,0);
		spi_EPCS_wr_DISABLE();//запрещаем запись во флеш
    }else
if (strcmp(Word,"EPCS_ERASE_ALL")==0) //
   {
		crc_comp =atoi(DATA_Word);
		x_out ("\r\nпринял EPCS_ERASE_ALL:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ

		spi_EPCS_wr_ENABLE ();//разрешаем запись во флеш
		spi_EPCS_ERASE_BULK();//стираем всЮ во флеш
		spi_EPCS_wr_DISABLE();//запрещаем запись во флеш
    }else
if (strcmp(Word,"spi2")==0) //
   {
		crc_comp =atoi(DATA_Word);
		x_out("\r\nпринял spi2:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
		TMS(0);
		x_out("spi2_read:",spi2send8(crc_comp));
		TMS(1);
    }else
if (strcmp(Word,"TIM1")==0) //
   {
		crc_comp =atoi(DATA_Word);
		crc_input=atoi(DATA_Word2);
		un_out("\r\nпринял TIM1:",crc_comp);
		u_out(".",crc_input);
		 htim1.Instance->CCR1=crc_comp;
    }else
if (strcmp(Word,"TIM2")==0) //
   {
		crc_comp =atoi(DATA_Word);
		crc_input=atoi(DATA_Word2);
		un_out("\r\nпринял TIM2:",crc_comp);
		u_out(".",crc_input);
		 htim2.Instance->CCR1=crc_comp;
		 htim2.Instance->ARR =crc_input;
    } 


	
   } 
	  for (i=0u;i<buf_Word;i++)               Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)   DATA_Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)  DATA_Word2[i]     =0x0;  
      for (i=0u;i<BUFFER_SR;i++)  
      {
        InOut[i]     =0x0;
      }  
      
	  time_uart=0;  //обнуление счётчика тайм аута
      packet_flag=0; 
      index1=0u; 
      crc_ok=0; 
      i=0;
      packet_ok=0; 
      index_word=0u; 
      index_data_word=0u;
      data_flag=0;
      index_data_word2=0u;
      data_flag2	 =0;
      indexZ 		 =0u;
      FLAG_lenght    =0u;
      lenght_data    =0u;
      sch_lenght_data=0u;
      FLAG_CW   = 0u; //флаг управляющего байта, снимается сразу после исполнения
      FLAG_DATA = 0u;	  
      	  
      DATA_Word [0]=' ';
      DATA_Word2[0]=' ';
	  SCH_LENGHT_PACKET=0;
  }

  if ((packet_ok==1)&&(crc_ok==0x1))     //обработка команд адресатом которых является слейв

  {
    
    if (Master_flag==0)

      {            
         
      }
  }  
  return  crc_input;         
} 

void LED (void)
{
	static u8 z=0;
	if (TIME_TEST>1000)	
	{
		z=~z;
		LED1(z);
		TIME_TEST=0;
	}
}

void LED_OFF (void)
{
	if (TIMER1>1000) LED2(1);
	if (TIMER2>1000) LED3(1);
}

void BTN1 (void)
{
	if (PIN_control_PD13()!=0) 
	{
		LED2(0);
		TIMER1=0;
		Transf("BTN_1 !!!\r\n");
	}
}

void BTN2 (void)
{
	if (PIN_control_PD15()!=0) 
	{
		LED3(0);
		TIMER2=0;
		Transf("BTN_2 !!!\r\n");
	}
}

u8 PIN_control_PD13 (void)
{
  static u8 pn_old;
  u8 pn;
  u8 flag=0;
  pn=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13);
  if (pn_old!=pn) 
  {
	  pn_old=pn;
	  if (pn==0) flag=1;
	  } else flag=0;
  
  return flag;
}

u8 PIN_control_PD15 (void)
{
  static u8 pn_old;
  u8 pn;
  u8 flag=0;
  pn=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15);
  if (pn_old!=pn) 
  {
	  pn_old=pn;
	  if (pn==0) flag=1;
	  } else flag=0;
  return flag;
}

void Menu1(char a) 
 {
//***************************************************************************
    int i;  
 
  for (i=0;i<20;i++) Transf("\r");    // очистка терминала
  for (i=0; i<20; i++) Transf ("-");  // вывод приветствия
  Transf("\r");
  Transf("..........Terminal Тестовой платы.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("Расшифровка структуры команды:\r");
  Transf("~ - стартовый байт\r");
  Transf("1 - адрес абонента\r");
  Transf(";- конец пачки \r");
  Transf(".............. \r");
  Transf("---------------------------------------------\r\n");
  Transf("IP  :192.168.1.163 - IP адрес    блока\r");
  Transf("PORT:2054          - номер порта блока\r");
  Transf("~0 help; - текущее меню\r");
  Transf("~0 info; - информация \r");
  Transf("~0 dac1_init; - \r");
  Transf("~0 dac1_r:0;   - чтение регистра\r");
  Transf("~0 dac1_w:0.0; - запись регистра\r");
  Transf("~0 dac1_serdes_pll:1; - очистка регистра сигнала захвата PLL Serdes\r");
  Transf("~0 dac1_info:0; \r");
  Transf("~0 dac1_init:0; \r");
  Transf("~0 dac1_phy_wr:0; \r");
  Transf("~0 dac1_phy_info; \r");
  Transf("~0 lmk_sync; - sync на LMK\r");
  Transf("~0 init_lmk; - init на LMK\r");
  Transf("-------------------------------------------\r");
  Transf("\r");
  Transf("\r");
  Transf("++++++++++++++++++++\r");
  Transf("\r");
  Transf("\r");
  //for (i=0; i<64; i++) zputs ("*",1);  // вывод приветствия
  //for (i=0;i<10;i++) puts("\r",1);  // очистка терминала
  Transf("\r");
  //*******************************************************************************
}

int main(void)
{
 
  u8 w_var=0;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
//MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  
  HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  
  Transf("\r\n");
  Transf("-------------\r\n");
  Transf("    prg 330  \r\n");
  Transf("-------------\r\n");
  
  Adress=0x30; //адресс кассеты
  WDOG(1);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  w_var=~w_var;
	  WDOG(w_var);
	  LED();
	  BTN1();
	  BTN2();
	  LED_OFF();
	  UART_conrol();
	  UART_DMA_TX();
  }

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
