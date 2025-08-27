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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  FFT_LENGTH 1024  
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buff1[FFT_LENGTH];
uint16_t adc_buff2[FFT_LENGTH];

int frq1=0,frq2=0;//最大值对应的频率下标
int wave_frq1,wave_frq2;//计算得出频率

__IO uint8_t AdcConvEnd = 0;

float adcBuff1[FFT_LENGTH];//ADC转化后的值
float fft_inputbuf1[FFT_LENGTH * 2];  
float fft_outputbuf1[FFT_LENGTH];
double fft_phase1;

float adcBuff2[FFT_LENGTH];//ADC转化后的值
float fft_inputbuf2[FFT_LENGTH * 2];  
float fft_outputbuf2[FFT_LENGTH];
double fft_phase2;

float max_value1,max_value2;//谐波幅值最大值

double phase;
double First_a,First_b;    //相位实部
double Second_a,Second_b;    //相位虚部
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*********ADC采集*********/
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff1, FFT_LENGTH);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buff2, FFT_LENGTH);
    HAL_TIM_Base_Start(&htim3);
    
/*********ADC转化值*********/    
    for (int i = 0; i < FFT_LENGTH; i++)
    {
        adcBuff1[i] = adc_buff1[i] * 3.3 / 4096;//将整型转化为浮点型
    }
    
    for (int i = 0; i < FFT_LENGTH; i++)
    {
        adcBuff2[i] = adc_buff2[i] * 3.3 / 4096;//将整型转化为浮点型
    }
      
/*********傅里叶变换*********/
    while (!AdcConvEnd);       //等待转换完毕
    
    for (int i = 0; i < FFT_LENGTH; i++)
    {
        fft_inputbuf1[i * 2] = adcBuff1[i];//实部赋值
        fft_inputbuf1[i * 2 + 1] = 0;//虚部赋值，固定为0.
    }
    for (int i = 0; i < FFT_LENGTH; i++)
    {
        fft_inputbuf2[i * 2] = adcBuff2[i];//实部赋值
        fft_inputbuf2[i * 2 + 1] = 0;//虚部赋值，固定为0.
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf1, 0, 1);
    arm_cmplx_mag_f32(fft_inputbuf1, fft_outputbuf1, FFT_LENGTH);
    
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf2, 0, 1);
    arm_cmplx_mag_f32(fft_inputbuf2, fft_outputbuf2, FFT_LENGTH);
    
    //计算谐波幅值
    fft_outputbuf1[0] /= 1024;
    for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
    {
        fft_outputbuf1[i] /= 512;
    }
    
    fft_outputbuf2[0] /= 1024;
    for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
    {
        fft_outputbuf2[i] /= 512;
    }
    
    /********寻找最大值********/
    max_value1=fft_outputbuf1[1];
    for(int i=1;i<FFT_LENGTH/2-1;i++)
    {
        if(fft_outputbuf1[i]>=max_value1)
        {
            max_value1=fft_outputbuf1[i];
            frq1=i;
        }
        else
            max_value1=max_value1;
    }
    
    max_value2=fft_outputbuf2[1];
    for(int i=1;i<FFT_LENGTH/2-1;i++)
    {
        if(fft_outputbuf2[i]>=max_value2)
        {
            max_value2=fft_outputbuf2[i];
            frq2=i;
        }
        else
            max_value2=max_value2;
    }
    
    wave_frq1=frq1*1024000/1024;
    wave_frq2=frq2*1024000/1024;
    
    First_a = fft_inputbuf1[frq1*2];
    First_b = fft_inputbuf1[frq1*2+1];
    Second_a = fft_inputbuf2[frq2*2];
    Second_b = fft_inputbuf2[frq2*2+1];
    
    fft_phase1 = atan2(First_b,First_a);
    fft_phase2 = atan2(Second_b,Second_a);
    phase=fft_phase1-fft_phase2;

    printf("%.2f\n",(phase*180.0/3.1415926)+(7*(wave_frq1/10000)+1));
    printf("%d  %d\n",frq1,frq2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
