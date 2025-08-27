/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD9959.h"
#include "G2B.h"
#include "stdio.h"
#include "key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUF_SIZE 64

uint8_t rx1_buf[RX_BUF_SIZE];
uint8_t rx1_tmp;
uint8_t rx1_index = 0;

uint8_t rx2_buf[RX_BUF_SIZE];
uint8_t rx2_tmp;
uint8_t rx2_index = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t fre=0;
uint16_t flag=0;
uint16_t fremax=0;
uint16_t fremin=0;
uint16_t mode=0;
uint16_t count=0;
uint16_t adc_buff[1024];//存放ADC采集的数据
extern int _estack; // 定义在链接脚本中
extern int32_t result; 
int s1=0;
int s2=0;
/* 
AdcConvEnd用来检测ADC是否采集完毕
0：没有采集完毕
1：采集完毕，在stm32f1xx_it里的DMA完成中断进行修改
 */
__IO uint8_t AdcConvEnd = 0;
volatile uint16_t sensor_data = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AD9959_sweep(uint32_t fre,uint32_t mode,uint32_t fremax,uint32_t fremin,uint32_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	result=100;
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
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &rx1_tmp, 1);
	HAL_UART_Receive_IT(&huart2, &rx2_tmp, 1);
	HAL_TIM_Base_Start(&htim3);   //开启定时器3
	AD9959_Init();								//初始化控制AD9959需要用到的IO口,及寄存器
  HAL_Delay(10); 								// 添加10ms延时确保初始化完成
	IO_Update();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(result==-1)
	{s1=-1;}//start
	else if(result==-2)
	{s1=-2;}//stop
	else if(result==-3)
	{s1=-3;}
	 AD9959_Set_Fre(CH0,1000000);
	 G2B_Set_Vpp(CH0,1000000,100);
    // 频率扫描示例（取消注释使用）
 if(s1==-1 ) 
 {
	 {
				AD9959_sweep(
        1000000,    // 起始频率
        3,        
        40000000,    // 最大频率
        1000000,    // 最小频率
        1000000);    // 步进速度
				HAL_Delay(100);
	 }
 }  
else if(s1==-3)
 {
	 AD9959_Set_Fre(CH0,1000000);
	 G2B_Set_Vpp(CH0,1000000,100);
	 IO_Update();
 } 
else if(result<0)
{}	
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) AdcConvEnd = 1;
}
void AD9959_sweep(uint32_t fre,uint32_t mode,uint32_t fremax,uint32_t fremin,uint32_t speed)
{

	if(mode==1)
	{
		fre=fremax+count*speed;
		count--;
			AD9959_Set_Fre(CH0,fre);	
		IO_Update();	
	}
	else if(mode==2)
	{
		fre=fremin+count*speed;
		count++;
			AD9959_Set_Fre(CH0,fre);	
		IO_Update();	
	}
else if (mode == 3) 
{
    // 1. 先计算新的 fre
    if (flag == 0) {
        fre = fremin + count * speed;  // 上升扫描
    } else {
        fre = fremax - count * speed;  // 下降扫描（从 fremax 开始减）        
    }

    // 2. 检查边界，更新 flag
    if (fre >= fremax) {  // 超过上限 → 改为下降扫描
        flag = 1;
        fre = fremax;    // 防止超出
        count = 0;       // 重置 count
    } else if (fre <= fremin) {  // 低于下限 → 改为上升扫描
        flag = 0;
        fre = fremin;    // 防止低于最小值
        count = 0;       // 重置 count
    }
		count++;
    printf("count=%d, fre=%d, flag=%d\n", count, fre, flag);
    AD9959_Set_Fre(CH0, fre);
    G2B_Set_Vpp(CH0, fre, result);
    IO_Update();
}

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
