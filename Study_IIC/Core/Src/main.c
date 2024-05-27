/* USER CODE BEGIN Header */
#include <string.h>
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FALSE 0
#define TRUE 1

//设置SCL高电平低电平
#define SCL_H HAL_GPIO_WritePin(GPIOA, SCL_Pin, GPIO_PIN_SET)
#define SCL_L HAL_GPIO_WritePin(GPIOA, SCL_Pin, GPIO_PIN_RESET)

//设置SDA高电平低电平
#define SDA_H HAL_GPIO_WritePin(GPIOA, SDA_Pin, GPIO_PIN_SET)
#define SDA_L HAL_GPIO_WritePin(GPIOA, SDA_Pin, GPIO_PIN_RESET)

//调用my_SDA_IN()和my_SDA_OUT()函数
#define SDA_IN()  my_SDA_IN()
#define SDA_OUT() my_SDA_OUT()

//读取SDA线的状态
#define SDA_read HAL_GPIO_ReadPin(GPIOA, SDA_Pin)

//一个8位无符号整数，可能用于表示ADC（模数转换器）的过采样值
uint8_t ADC_OVERSAMP=0xAC;//0xAC 0xB8

//一个16位无符号整数，可能与某种生产范围或量程有关。
uint16_t PRODUCE_RANGE=2;

//双精度浮点数，可能用于表示压力范围
double CAL_L_XKpa=-0.1111;
double CAL_H_XKpa=2.1111;

//一个8位无符号整数，I2C地址，用于与TCA9548A设备通信。
uint8_t ADDR_TCA9548A=0x72; 	//0x70（A0 A1 A2 0）  0x71（A0 1 A1 0 A2 0） 0x72（A0 0 A1 1 A2 0） 0x73（A0 1 A1 1 A2 0）
//一个8位无符号整数，可能用于表示通道号或其他类似的信息
uint8_t chn = 0;
char TxData[128] = {0};
double P_converted = 0,T_converted = 0;


//枚举定义了NSA2300寄存器地址。
enum REG_ADDR
{
	/*Status状态,Data_out1,Data_out2,Data_out3三个数据输出寄存器,
	Temp_out1,Temp_out2两个温度值输出寄存器,CMD,OTP_CMD,Part_ID,
	Sys_config,P_config,T_config1,T_config2,
	DAC_limit,CAL_OTP,Redundancy*/
	Status = 0x02,Data_out1 = 0x06,Data_out2 = 0x07,Data_out3 = 0x08,
	Temp_out1 = 0x09,Temp_out2 = 0x0A,CMD = 0x30,OTP_CMD = 0x6C,Part_ID = 0xA4,
	Sys_config = 0xA5,P_config = 0xA6,T_config1 = 0xA7,T_config2 = 0xA8,
	DAC_limit = 0xA9,CAL_OTP = 0xAA,Redundancy = 0xBC
}reg_addr;
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
//I2C通信相关函数
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);

//tca9548a相关函数
uint8_t func_tca9548a_select(uint8_t chn);
uint8_t func_tca9548a_read_ch(void);
uint8_t func_tca9548a_live(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //初始化I2C接口并设置了UART1的中断接收
  IIC_Init();
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Init Success\r\n");
  while (1)
  {
	  printf("1\r\n");
	  HAL_msDelay(1000);
	  printf("2\r\n");
	  WE_converted(&P_converted,&T_converted);
	  memset(TxData,0,sizeof(TxData));
	  sprintf(TxData,"55<Press=%.2f,Temper=%.2f>33",P_converted,T_converted);
	  printf("%s\r\n",TxData);
//	  ProtocolAnalysis();//串口协议

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
void HAL_usDelay(uint32_t udelay)
{
  uint32_t startval,tickn,delays,wait;

  startval = SysTick->VAL;
  tickn = HAL_GetTick();

  delays =udelay * 72;
  if(delays > startval)
  {
    while(HAL_GetTick() == tickn);
    wait = 72000 + startval - delays;
    while(wait < SysTick->VAL);
  }
  else
  {
    wait = startval - delays;
    while(wait < SysTick->VAL && HAL_GetTick() == tickn);
  }
}

void HAL_msDelay(uint32_t udelay)
{
	uint16_t i = 0;
	for(i = 0;i < udelay;i++)
	{
		HAL_usDelay(1000);
	}
}

//I2C通信
void my_SDA_IN(void)
{
	//
	GPIO_InitTypeDef GPIO_InitStruct= {0};
	GPIO_InitStruct.Pin = SDA_Pin;//PA7
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;//输入模式
	GPIO_InitStruct.Pull = GPIO_PULLUP;//上拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void my_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
	GPIO_InitStruct.Pull = GPIO_NOPULL;//禁用上下拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void IIC_Init(void)
{
	SDA_L;
	SCL_L;
//	SDA_H;
//	SCL_H;
}

void IIC_Start(void)
{
	SDA_OUT();
	SDA_H;
	SCL_H;
	HAL_usDelay(5);
 	SDA_L;
 	HAL_usDelay(5);
	SCL_L;
}

void IIC_Stop(void)
{
	SDA_OUT();
	SCL_L;
	SDA_L;
	HAL_usDelay(4);
	SCL_H;
	SDA_H;
	HAL_usDelay(4);
}

uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();
	SDA_H;HAL_usDelay(1);
	SCL_H;HAL_usDelay(1);
	while(SDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;
	return 0;
}

void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
	SDA_L;
	HAL_usDelay(2);
	SCL_H;
	HAL_usDelay(2);
	SCL_L;
}

void IIC_NAck(void)
{
	//发送一个NACK（Not Acknowledge）信号
	SCL_L;
	SDA_OUT();
	SDA_H;
	HAL_usDelay(2);
	SCL_H;
	HAL_usDelay(2);
	SCL_L;
}

void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	SDA_OUT();
    SCL_L;
    for(t=0;t<8;t++)//对于要发送字节的每一位执行循环内的操作
    {
		if((txd&0x80)>>7)//与操作检查txd最高位是否为1，将结果右移七位
			SDA_H;//txd最高位是1则拉高
		else
			SDA_L;//txd最低位是0则拉低
		txd<<=1;//左移一位 即移动到下一位
		HAL_usDelay(2);//delay2微秒，确保SDA线在SCL线上升沿之前保持稳定
		SCL_H;
		HAL_usDelay(2);
		SCL_L;
		HAL_usDelay(2);
    }
}

uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        SCL_L;
        HAL_usDelay(2);
		SCL_H;
        receive<<=1;
        if(SDA_read)receive++;
        HAL_usDelay(1);
    }
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}

uint8_t IIC_WriteNumByte(uint8_t address, uint8_t *buf, uint8_t count)
{
	unsigned char ack;
	address &=0xFE;
	IIC_Start();
	HAL_usDelay(2);
	IIC_Send_Byte(address);
	HAL_usDelay(2);
	ack=IIC_Wait_Ack();
	if(ack)
	{
		IIC_Stop();
		return 1;
	}

	while(count)
	{
		IIC_Send_Byte(*buf);
		HAL_usDelay(2);

		ack=IIC_Wait_Ack();
		if(ack)
		{
			IIC_Stop();
			return 1;
		}
		buf++;
		count--;
	}
	IIC_Stop();
	return 0;
}

uint8_t IIC_ReadNumByte(uint8_t address, uint8_t *buf, uint8_t count)
{
	unsigned char ack;

	address |=0x01;
	IIC_Start();
	IIC_Send_Byte(address);
	HAL_usDelay(2);

	ack=IIC_Wait_Ack();
	if(ack)
	{
		IIC_Stop();
		return 1;
	}

	while(count)
	{
		if(count!=1) *buf=IIC_Read_Byte(1);
		else *buf=IIC_Read_Byte(0);
		buf++;
		count--;
	}
	IIC_Stop();
	return 0;
}

uint8_t IIC_WriteNumByte_NSA(uint8_t address, uint8_t reg_addr,uint8_t *buf, uint8_t count)
{
	unsigned char ack;
	address &=0xFE;
	IIC_Start();
	HAL_usDelay(2);
	IIC_Send_Byte(address);
	HAL_usDelay(2);
	ack=IIC_Wait_Ack();
	if(ack)
	{
		IIC_Stop();
		return 1;
	}

	IIC_Send_Byte(reg_addr);
	HAL_usDelay(2);
	ack=IIC_Wait_Ack();
	if(ack)
	{
		IIC_Stop();
		return 1;
	}

	while(count)
	{
		IIC_Send_Byte(*buf);
		HAL_usDelay(2);

		ack=IIC_Wait_Ack();
		if(ack)
		{
			IIC_Stop();
			return 1;
		}
		buf++;
		count--;
	}
	IIC_Stop();
	return 0;
}

uint8_t IIC_ReadNumByte_NSA(uint8_t address, uint8_t reg_addr,uint8_t *buf, uint8_t count)
{
	unsigned char ack;

	address &= 0xFE;
	IIC_Start();
	IIC_Send_Byte(address);
	HAL_usDelay(2);

	ack=IIC_Wait_Ack();
	if(ack)
	{
		IIC_Stop();
		return 1;
	}

	IIC_Send_Byte(reg_addr);
	HAL_usDelay(2);

	ack=IIC_Wait_Ack();
	if(ack)
	{
		IIC_Stop();
		return 1;
	}

	IIC_Start();
	address |= 0x01;
	IIC_Send_Byte(address);
	HAL_usDelay(2);

	ack=IIC_Wait_Ack();
	if(ack)
	{
		IIC_Stop();
		return 1;
	}

	while(count)
	{
		if(count!=1) *buf=IIC_Read_Byte(1);
		else *buf=IIC_Read_Byte(0);
		buf++;
		count--;
	}
	IIC_Stop();

	return 0;
}

uint8_t M5803_IsBusy(void)
{
	uint8_t status;
	IIC_ReadNumByte(Device_Address,&status,1);
	status= (status>>5)&0x01;
	return status;
}

void M5803_get_cal(uint32_t *press,uint32_t *temp)
{
	uint8_t buffer[6] = {0},k = 0;

	buffer[0]=ADC_OVERSAMP;//0xAC 0xB8;

	IIC_WriteNumByte(Device_Address,buffer,1);

	while(1)
	{
		if(M5803_IsBusy() && k < 250)
		{
			HAL_msDelay(1);
			k++;
		}
		else
			break;
	}
	IIC_ReadNumByte(Device_Address,buffer,6);

	*press =((uint32_t)buffer[1]<<16)|((uint32_t)buffer[2]<<8) | buffer[3];
	*temp = ((uint32_t)buffer[4]<<8) | (buffer[5]<<0);

//	*temp=(double)temp_raw/65536;
//	*temp=((*temp)*19000-4000)/100;
//
//	*press=(double)press_raw/16777216;
//	*press=(*press)*(CAL_H_XKpa-CAL_L_XKpa)+CAL_L_XKpa;
}

void M5803_get_cal_NSA(uint32_t *press,uint32_t *temp)
{
	uint8_t buffer[6] = {0};

	buffer[0]= 0X0A;	//测试开始 读取压力值
	IIC_WriteNumByte_NSA(Device_Address,0X30,buffer,1);
	HAL_msDelay(10);

	while((IIC_ReadNumByte_NSA(Device_Address,0X30,buffer,1) & 0x08) != 0);

	IIC_ReadNumByte_NSA(Device_Address,0X06,buffer,1);
	IIC_ReadNumByte_NSA(Device_Address,0X07,&buffer[1],1);
	IIC_ReadNumByte_NSA(Device_Address,0X08,&buffer[2],1);

	*press = ((uint32_t)buffer[0] << 16) + ((uint32_t)buffer[1] << 8)+ ((uint32_t)buffer[2] << 0);

	IIC_ReadNumByte_NSA(Device_Address,0X09,buffer,1);
	IIC_ReadNumByte_NSA(Device_Address,0X0a,&buffer[1],1);

	*temp = (((uint32_t)buffer[0] << 8) + ((uint32_t)buffer[1]));

}


void WE_converted(double *P_converted,double *T_converted)
{
	uint32_t press_raw = 0,temp_raw = 0;
	// 读取传感器数据
	M5803_get_cal_NSA(&press_raw, &temp_raw);

	// 转换数据
	if(press_raw > 8388608)////超过 8388606 为负压值
	{
		press_raw = (double)(press_raw - 16777216) / 4096000;
	}
	else
	{
		press_raw = (double)press_raw / 4096000;   // 单位为Pa
	}
	*P_converted = press_raw;

	// 温度计算公式
	if(temp_raw > 32768)
	{
		temp_raw = temp_raw - 65536;
	}
	temp_raw = (double)temp_raw / 256;                  // 单位为℃
	*T_converted = temp_raw;

}

uint8_t func_tca9548a_select(uint8_t chn)//0~7
{
	IIC_Start();
	IIC_Send_Byte((ADDR_TCA9548A << 1) | 0x00);

	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;
	}

	IIC_Send_Byte(1 << chn);

	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;
	}

	IIC_Stop();
	return 0;
	cmd_fail:
		IIC_Stop();
		return 1;
}

uint8_t func_tca9548a_read_ch(void)
{
	uint8_t res = 0;

	IIC_Start();
	IIC_Send_Byte((ADDR_TCA9548A << 1) | 0x01);

	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;
	}
	res = IIC_Read_Byte(0);

	IIC_NAck();

	IIC_Stop();
	cmd_fail:
		IIC_Stop();
	return res;
}

uint8_t func_tca9548a_live(void)
{
	IIC_Start();
	IIC_Send_Byte((ADDR_TCA9548A << 1) | 0x01);

	if (IIC_Wait_Ack() != 0)
	{
		return 1;
	}
	return 0;
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
