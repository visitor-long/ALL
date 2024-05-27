/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
extern uint8_t ADDR_TCA9548A; 	//0x70£¨A0 A1 A2 0£©  0x71£¨A0 1 A1 0 A2 0£© 0x72£¨A0 0 A1 1 A2 0£© 0x73£¨A0 1 A1 1 A2 0£©
extern uint8_t chn;

extern uint16_t PRODUCE_RANGE;
extern double CAL_L_XKpa;
extern double CAL_H_XKpa;

extern uint8_t ADC_OVERSAMP;

void HAL_usDelay(uint32_t udelay);
void HAL_msDelay(uint32_t udelay);

uint8_t func_tca9548a_select(uint8_t chn);
uint8_t func_tca9548a_read_ch(void);
void M5803_get_cal(uint32_t *press,uint32_t *temp);

uint8_t IIC_WriteNumByte_TE(uint8_t address, uint8_t reg_addr,uint8_t *buf, uint8_t count);
uint8_t IIC_ReadNumByte_TE(uint8_t address,uint8_t *buf, uint8_t count);
void M5803_get_cal_TE(uint32_t *press,uint32_t *temp);
void XGZ_converted(float *P_converted,float *T_converted);
uint8_t IIC_WriteNumByte(uint8_t address, uint8_t *buf, uint8_t count);
uint8_t test_TE(uint8_t address);
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
#define RST3_Pin GPIO_PIN_13
#define RST3_GPIO_Port GPIOC
#define RST4_Pin GPIO_PIN_14
#define RST4_GPIO_Port GPIOC
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOA
#define RST1_Pin GPIO_PIN_7
#define RST1_GPIO_Port GPIOB
#define RST2_Pin GPIO_PIN_8
#define RST2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOA

#define PRODUCE_RANGE_700 700
#define CAL_L_XKpa_700   -50
#define CAL_H_XKpa_700  450

#define PRODUCE_RANGE_400 400
#define CAL_L_XKpa_400   -50
#define CAL_H_XKpa_400  450

#define PRODUCE_RANGE_10 10
#define CAL_L_XKpa_10   -0.5556
#define CAL_H_XKpa_10  10.5556

#define PRODUCE_RANGE_2 2
#define CAL_L_XKpa_2	-0.1111
#define CAL_H_XKpa_2  2.1111

#define  Device_Address 0x28<<1    //0X78   0x6D
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
