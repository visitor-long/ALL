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
void IIC_Init(void);//初始化IIC的IO口
void IIC_Start(void);//发送IIC开始信号
void IIC_Stop(void);//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void);//IIC等待ACK信号
void IIC_Ack(void);//IIC发送ACK信号
void IIC_NAck(void);//IIC不发送ACK信号

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
	  ProtocolAnalysis();//串口协议
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

//发送数据后，等待应答信号到来
//返回值：1，接收应答失败，IIC直接退出
//        0，接收应答成功，什么都不做
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

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
/**
 * @brief 从I²C设备读取一个字节
 *
 * 此函数用于从I²C设备读取一个字节的数据，并根据传入的ack参数决定在读取完成后是否发送应答信号（ACK或NACK）。
 *
 * @param ack 是否在读取后发送应答信号。如果为0，则发送NACK；否则发送ACK。
 *
 * @return 读取到的字节值
 */
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

/**
 * @brief 向I²C设备写入多个字节
 *
 * 此函数用于向I²C设备写入指定数量的字节数据。首先发送设备地址（最低位清零以表示写操作），
 * 然后发送要写入的数据字节，每个字节后都等待设备的应答信号（ACK）。
 * 如果在任何时候收到NACK，则停止写入并返回错误代码。
 *
 * @param address I²C设备的地址（最低位清零表示写操作）
 * @param buf 要写入数据的指针
 * @param count 要写入的字节数
 *
 * @return 成功写入返回0，发生错误返回1
 */
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

/**
 * @brief 从I²C设备读取多个字节
 *
 * 此函数用于从I²C设备读取指定数量的字节数据。首先发送设备地址（最低位置1表示读操作），
 * 然后循环读取每个字节，并在每个字节读取后（除了最后一个字节）发送ACK应答信号。
 * 读取最后一个字节后，发送NACK信号以表示读取完成。
 *
 * @param address I²C设备的地址（最低位置1表示读操作）
 * @param buf 存储读取数据的缓冲区指针
 * @param count 要读取的字节数
 *
 * @return 成功读取返回0，发生错误返回1
 */
uint8_t IIC_ReadNumByte(uint8_t address, uint8_t *buf, uint8_t count)
{
	unsigned char ack;

	address |= 0x01;
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


/**
 * @brief 向XGZ芯片的指定寄存器写入多个字节
 *
 * 用于向XGZ芯片的某个特定寄存器地址写入多个字节数据。首先发送设备地址（最低位清零表示写操作），
 * 然后发送要写入数据的寄存器地址，接着循环发送要写入的数据字节，并在每个字节后等待设备的应答信号（ACK）。
 * 如果在任何时候收到NACK，则停止写入并返回错误代码。
 *
 * @param address XGZ芯片的地址（最低位清零表示写操作）
 * @param reg_addr 要写入的寄存器地址
 * @param buf 要写入数据的指针
 * @param count 要写入的字节数
 *
 * @return 成功写入返回0，发生错误返回1
 */
uint8_t IIC_WriteNumByte_TE(uint8_t address, uint8_t reg_addr,uint8_t *buf, uint8_t count)
{
	unsigned char ack;

	address &=0xFE;//最后一位

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


/**
 * @brief 从XGZ芯片的指定寄存器读取多个字节
 *
 * 此函数用于从XGZ芯片的指定寄存器地址中读取多个字节的数据。
 * 它首先发送设备的写地址（最低位清零表示写操作），然后发送要读取数据的寄存器地址。
 * 随后，它再次发送设备的读地址（最低位置1表示读操作）来开始数据读取。
 * 接着，它会循环读取指定数量的字节，并在每个字节读取后（除了最后一个字节）发送ACK应答信号。
 * 读取最后一个字节后，发送NACK信号以表示读取完成。
 *
 * @param address XGZ芯片的地址（在发送写请求时最低位清零，在发送读请求时最低位置1）
 * @param reg_addr 要读取的寄存器地址
 * @param buf 存储读取数据的缓冲区指针
 * @param count 要读取的字节数
 *
 * @return 成功读取返回0，发生错误返回1
 */
uint8_t test_TE(uint8_t address)
{
	unsigned char ack;

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

	IIC_Stop();
	return 0;
}


uint8_t IIC_ReadNumByte_TE(uint8_t address,uint8_t *buf, uint8_t count)
{
	unsigned char ack;

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
		if(count!=1) *buf = IIC_Read_Byte(1);
		else *buf=IIC_Read_Byte(0);
		buf++;
		count--;
	}

	IIC_Stop();
	return 0;
}


/**
 * @brief 从设备获取压力和温度数据
 * 从设备读取校准后的压力和温度数据，并将它们存储在提供的指针所指向的变量中。
 * @param press 指向存储压力数据的uint32_t类型变量的指针
 * @param temp 指向存储温度数据的uint32_t类型变量的指针
 * @return 无返回值，但会更新传入的压力和温度指针所指向的变量的值
 */
void M5803_get_cal_TE(uint32_t *press,uint32_t *temp)
{
	uint8_t buffer[4] = {0,0};

	IIC_ReadNumByte_TE(Device_Address,buffer,4);

	// 组合压力数据 高两位status bits设置为00 Normal operation
	*press = ((((uint32_t)buffer[0] << 8) + (uint32_t)buffer[1])) & 0x3fff ;

	// 组合温度数据
	*temp = ((((uint32_t)buffer[2] << 8) + ((uint32_t)buffer[3])) & 0xffe0) >> 5;


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
