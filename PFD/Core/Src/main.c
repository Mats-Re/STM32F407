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
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define FFT_SIZE 1024
#define SAMPLE_RATE 20000  // 采样率，单位 Hz

uint16_t adc_buffer[FFT_SIZE];
float32_t fft_input[2 * FFT_SIZE];  // 实部/虚部交错
float32_t fft_mag[FFT_SIZE];
uint32_t captureA = 0;
uint32_t captureB = 0;
uint8_t A_ready = 0, B_ready = 0;
__IO uint8_t AdcConvEnd = 0;
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
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_SIZE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
 HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
// printf("PFD系统启动\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		 if (A_ready && B_ready)
//        {
//            int32_t delta = (int32_t)captureA - (int32_t)captureB;
//            if (delta < 0) delta += 0xFFFF;

//            float T_us = 1000.0f;  // 假设周期为 1000us（需根据实际频率计算）
//            float delta_t = delta * 1.0f;  // 每个计数单位 = 1us
//            float phase = (delta_t / T_us) * 360.0f;

//            printf("Δt = %.0f us, 相位差 = %.2f°\r\n", delta_t, phase);

//            A_ready = 0;
//            B_ready = 0;
//        }
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
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim->Instance == TIM1)
//    {
//        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//        {
//            captureA = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//            A_ready = 1;
//        }
//        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//        {
//            captureB = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//            B_ready = 1;
//        }
//    }
//}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	while (!AdcConvEnd)                                   //等待转换完毕
    ;
    for (int i = 0; i < FFT_SIZE; i++)
    {
        fft_input[2*i]   = ((float32_t)adc_buffer[i] - 2048.0f);  // 实部（去直流）
        fft_input[2*i+1] = 0.0f;                                  // 虚部
    }

    // 计算 FFT
    arm_cfft_radix4_instance_f32 S;
    arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&S, fft_input);

    // 计算幅度
    for (int i = 0; i < FFT_SIZE; i++)
    {
        float real = fft_input[2*i];
        float imag = fft_input[2*i+1];
        fft_mag[i] = sqrtf(real*real + imag*imag);
    }

    // 寻找主频点
    uint32_t maxIndex = 0;
    float32_t maxValue = 0;
    arm_max_f32(fft_mag, FFT_SIZE / 2, &maxValue, &maxIndex);

    // 相位角（单位：弧度）
    float32_t phase_rad = atan2f(fft_input[2*maxIndex+1], fft_input[2*maxIndex]);
    float32_t freq = ((float)SAMPLE_RATE / FFT_SIZE) * maxIndex;

    printf("Freq: %.2f Hz, Phase: %.2f deg\n", freq, phase_rad * 180.0f / PI);
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
