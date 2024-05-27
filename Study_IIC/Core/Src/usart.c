/* USER CODE BEGIN Header */
#include <stdio.h>
#include <string.h>
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t crc8_chk_value(uint8_t *message, uint8_t len)
{
	uint8_t crc;
    crc = 0;
    while(len--)
        crc ^= *message++;
    return crc;
}


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}

uint8_t RxBuffer = 0;
uint8_t RxData[16] = {0};
uint8_t RxLenght = 0;
uint8_t RxIndex = 0;
uint8_t RxFlag = 0;		//0：none  1:completion head 2:finish Data Length 3:finish Read data  4:completion tail  5:completion Calibration in progress
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
  uint8_t uwCRCValue = 0;
  switch(RxFlag)
  {
  	  case 0:
  		  if(RxBuffer == 0x55)
  		  {
  			RxIndex = 0;
  			memset(RxData,0,sizeof(RxBuffer));
		    RxFlag = 1;
  		  }
		  break;
  	  case 1:
  		  RxLenght = RxBuffer;
  		  RxData[RxIndex++] = RxBuffer;
  		  RxFlag = 2;
		  break;
  	  case 2:
  		  RxData[RxIndex++] = RxBuffer;
  		  RxLenght--;
  		  if(RxLenght == 0)
  			RxFlag = 3;
  		  break;
  	  case 3:
  		  RxIndex = 0;
  		  if(RxBuffer == 0x33)
		    RxFlag = 4;
  		  else
  			RxFlag = 0;
		  break;
  	  case 4:
  		  uwCRCValue = crc8_chk_value(&RxData[0],RxData[0]+1);
  		  if(uwCRCValue == RxBuffer)
		    RxFlag = 5;
  		  else
  			RxFlag = 0;
		  break;
  }

  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);
}

void SwitchPCA9548A(uint8_t i)
{
	HAL_GPIO_WritePin(GPIOC, RST3_Pin|RST4_Pin, GPIO_PIN_RESET);
	  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, RST1_Pin|RST2_Pin, GPIO_PIN_RESET);

	switch(i)
	{
		case 0x70:
			HAL_GPIO_WritePin(GPIOB, RST1_Pin, GPIO_PIN_SET);
			break;
		case 0x71:
			HAL_GPIO_WritePin(GPIOB, RST2_Pin, GPIO_PIN_SET);
			break;
		case 0x72:
			HAL_GPIO_WritePin(GPIOC, RST3_Pin, GPIO_PIN_SET);
			break;
		case 0x73:
			HAL_GPIO_WritePin(GPIOC, RST4_Pin, GPIO_PIN_SET);
			break;
	}
}


char TxData[128] = {0};
uint8_t FunctionFlag = 0; //0 none
void ProtocolAnalysis(void)
{
	uint8_t i = 0,j = 0;
	uint8_t ADDR_TCA9548A_TEMP = 0;
	uint32_t press = 0,temp = 0;
	uint8_t read_buff[3] = {0};
	if(RxFlag == 5)
	{
		switch(RxData[1])
		{
			case 0x01:
				ADDR_TCA9548A = 0x70 + RxData[2];
				SwitchPCA9548A(ADDR_TCA9548A);
				chn = RxData[3];
				if(func_tca9548a_select(chn) == 0)
				{
					HAL_msDelay(2);
//					M5803_get_cal(&press,&temp);	//非NSA2300
					M5803_get_cal_NSA(&press,&temp);
					memset(TxData,0,sizeof(TxData));
					sprintf(TxData,"55<Press=%ld,Temper=%ld>33",press,temp);
					printf("%s\r\n",TxData);
				}
				else
					printf("55<Device Loss>33\r\n");
				break;
			case 0x02:
				ADDR_TCA9548A_TEMP = ADDR_TCA9548A;
				for(i = 0x70;i <= 0x77;i++)//eight IIC Virtual Devices
				{
					ADDR_TCA9548A = i;
					SwitchPCA9548A(ADDR_TCA9548A);
					for(j = 0;j < 8;j++)//eight channel
					{
						if(func_tca9548a_select(j) == 0)
						{
//							TxData[0]=0xAC;//非NSA2300
//							HAL_msDelay(2);//非NSA2300
//							if(IIC_WriteNumByte(Device_Address, (uint8_t *)TxData, 1) == 0)//非NSA2300
							if(IIC_ReadNumByte_NSA(Device_Address,0X01,read_buff,1) == 0)
							{
								memset(TxData,0,sizeof(TxData));
								sprintf(TxData,"55<ADDR_TCA9548A=%#x,Channel=%d>33",ADDR_TCA9548A,j);
								printf("%s\r\n",TxData);
							}
						}
						else
							break;
					}
				}
				ADDR_TCA9548A = ADDR_TCA9548A_TEMP;
				break;
			case 0x03:
//				ADDR_TCA9548A_TEMP = ADDR_TCA9548A;
//				for(i = 0x70;i <= 0x77;i++)//eight IIC Virtual Devices
//				{
//					ADDR_TCA9548A = i;
//					SwitchPCA9548A(ADDR_TCA9548A);
//					for(j = 0;j < 8;j++)//eight channel
//					{
//						if(func_tca9548a_select(j) == 0)
//						{
//							HAL_msDelay(2);
//							if(IIC_WriteNumByte(Device_Address, (uint8_t *)TxData, 1) == 0)//非NSA2300
							if(IIC_ReadNumByte_NSA(Device_Address,0X01,read_buff,1) == 0)
							{
								HAL_msDelay(2);
//								M5803_get_cal(&press,&temp);//非NSA2300
								M5803_get_cal_NSA(&press,&temp);
								memset(TxData,0,sizeof(TxData));
								sprintf(TxData,"55<Press=%ld,Temper=%ld>33",press,temp);
								printf("%s\r\n",TxData);
							}
//						}
//						else
//							break;
//					}
//				}
				ADDR_TCA9548A = ADDR_TCA9548A_TEMP;
				break;
				break;
			case 0x04:
				PRODUCE_RANGE = (uint16_t)(RxData[2] << 8) | (uint16_t)RxData[3];
				ADC_OVERSAMP = RxData[4];
				if(PRODUCE_RANGE == PRODUCE_RANGE_2)
				{
					CAL_L_XKpa = CAL_L_XKpa_2;
					CAL_H_XKpa = CAL_H_XKpa_2;
				}
				else if(PRODUCE_RANGE == PRODUCE_RANGE_10)
				{
					CAL_L_XKpa = CAL_L_XKpa_10;
					CAL_H_XKpa = CAL_H_XKpa_10;
				}
				else if(PRODUCE_RANGE == PRODUCE_RANGE_400)
				{
					CAL_L_XKpa = CAL_L_XKpa_400;
					CAL_H_XKpa = CAL_H_XKpa_400;
				}
				else if(PRODUCE_RANGE == PRODUCE_RANGE_700)
				{
					CAL_L_XKpa = CAL_L_XKpa_700;
					CAL_H_XKpa = CAL_H_XKpa_700;
				}
				break;
			case 0x05:
				memset(TxData,0,sizeof(TxData));
				sprintf(TxData,"55<PRODUCE_RANGE=%d,ADC_OVERSAMP=%#x>33",PRODUCE_RANGE,ADC_OVERSAMP);
				printf("%s\r\n",TxData);
				break;
			case 0x06:
				memset(TxData,0,sizeof(TxData));
				sprintf(TxData,"55<ADDR_TCA9548A=%#x,Channel=%#x>33",ADDR_TCA9548A,chn);
				printf("%s\r\n",TxData);
				break;
		}
		RxFlag = 0;
	}
}
/* USER CODE END 1 */
