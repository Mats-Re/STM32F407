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
#include "arm_math.h"		// // CMSIS DSP ��ѧ��
#include "arm_const_structs.h" // FFT �ṹ�嶨��
#include <stdio.h>// ��׼���������
#include "AD9959.h"// AD9959 DDS ����ͷ�ļ�
#include "hmi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PI 3.141592653589

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_LENGTH 1024				// FFT ���ȣ�����Ϊ2����������
#define SAMPLE_RATE 1024400   // <<== �޸�Ϊ��ʵ�ʵ�ADC�����ʣ���λ Hz��
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HMI_UpdateTwoFrequencies(float freq1, float freq2);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_buff[200];
__IO uint8_t AdcConvEnd = 0;		// ADCת����ɱ�־���ɻص�������λ
uint16_t adcBuff[FFT_LENGTH];		// ���ADC������������ͣ�
void HMI_SendWaveData(float freq);
float fft_inputbuf[FFT_LENGTH * 2];		// FFT�������飬������ʽ��ʵ��+�鲿����洢
float fft_outputbuf[FFT_LENGTH];			// FFT�����ֵ����
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Hanning������������Ƶ�׷ֱ��ʣ������԰�й©
void apply_hanning_window(float* input, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        float window = 0.5f * (1.0f - arm_cos_f32(2 * PI * i / (length - 1)));
        input[i] *= window;
    }
}
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
	AD9959_Init();			// ��ʼ�� AD9959 ����IO�ͼĴ���
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);			 // ������ʱ������ADC����
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, FFT_LENGTH);		// ����ADC DMA����

  while (!AdcConvEnd);		 // �ȴ�ADC���ת��
  // === ��������ת��Ϊ��������׼��FFT���� ===
  for (int i = 0; i < FFT_LENGTH; i++) {
      fft_inputbuf[i * 2]     = adcBuff[i] * 3.3f / 4096.0f;		// ʵ������ѹֵ
      fft_inputbuf[i * 2 + 1] = 0.0f;														// �鲿Ϊ0
  }

  arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);		// ����FFT��δλ��ת������
  arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH);		// ���㸴��ģ����Ϊ��ֵ
	 // === ��ֵ��һ������ ===
  fft_outputbuf[0] /= 1024.0f;																// ֱ����������N
  for (int i = 1; i < FFT_LENGTH; i++) {
      fft_outputbuf[i] /= 512.0f;															// ��ֱ����������N/2
  }
	  // === �ٴμӴ�����ѡ���������봦�ӹ�����===
for (int i = 0; i < FFT_LENGTH; i++) {
    float window = 0.5f * (1.0f - arm_cos_f32(2 * PI * i / (FFT_LENGTH - 1)));
    fft_inputbuf[i * 2] *= window;
}

  // === ��ӡFFT��� ===
  printf("FFT Result:\r\n");
  for (int i = 0; i < FFT_LENGTH; i++) {
      printf("%d:\t%.2f\r\n", i, fft_outputbuf[i]);
  }

  // ����������Ƶ ===
  uint32_t maxIndex1 = 0, maxIndex2 = 0;
  float maxValue1 = 0.0f, maxValue2 = 0.0f;
	  // === �ؽ������������ݣ�Ϊ��ֵ���׼����===
	for (int i = 0; i < FFT_LENGTH; i++) {
    float window = 0.5f - 0.5f * arm_cos_f32(2 * PI * i / (FFT_LENGTH - 1));
    fft_inputbuf[i * 2] = (adcBuff[i] * 3.3f / 4096.0f) * window;
    fft_inputbuf[i * 2 + 1] = 0.0f;
}
	 // === �ҳ���ֵ��������Ƶ��bin ===
  for (int i = 1; i < FFT_LENGTH / 2; i++) {
      float val = fft_outputbuf[i];
      if (val > maxValue1) {
          maxValue2 = maxValue1;
          maxIndex2 = maxIndex1;

          maxValue1 = val;
          maxIndex1 = i;
      } else if (val > maxValue2) {
          maxValue2 = val;
          maxIndex2 = i;
      }
  }
  // === ����Ƶ��ֵ��Hz�� ===
  float freq1 = ((float)maxIndex1 * SAMPLE_RATE) / FFT_LENGTH;
  float freq2 = ((float)maxIndex2 * SAMPLE_RATE) / FFT_LENGTH; 

  printf("Detected Frequencies:\r\n");
  printf("Freq1 = %d Hz (Bin %lu)\r\n", (int)freq1, maxIndex1);
  printf("Freq2 = %d Hz (Bin %lu)\r\n", (int)freq2, maxIndex2);
AD9959_Init();								//��ʼ������AD9959��Ҫ�õ���IO��,���Ĵ���
	AD9959_Set_Fre(CH0,freq1);	
	AD9959_Set_Fre(CH1,freq2);	
	AD9959_Set_Fre(CH2, 100000);	//����ͨ��2Ƶ��100000Hz
	AD9959_Set_Fre(CH3, 100000);	//����ͨ��3Ƶ��100000Hz
		
	AD9959_Set_Amp(CH0, 1023); 		//����ͨ��0���ȿ���ֵ1023����Χ0~1023
	AD9959_Set_Amp(CH1, 1023); 		//����ͨ��1���ȿ���ֵ1023����Χ0~1023
	AD9959_Set_Amp(CH2, 1023); 		//����ͨ��2���ȿ���ֵ1023����Χ0~1023
	AD9959_Set_Amp(CH3, 1023); 		//����ͨ��3���ȿ���ֵ1023����Χ0~1023

	AD9959_Set_Phase(CH0, 0);			//����ͨ��0��λ����ֵ0(0��)����Χ0~16383
	AD9959_Set_Phase(CH1, 0);	//����ͨ��1��λ����ֵ4096(90��)����Χ0~16383
	AD9959_Set_Phase(CH2, 0);	//����ͨ��2��λ����ֵ8192(180��)����Χ0~16383
	AD9959_Set_Phase(CH3, 0);	//����ͨ��3��λ����ֵ12288(270��)����Χ0~16383
	IO_Update();	//AD9959��������
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
HMI_UpdateTwoFrequencies((int)freq1,(int)freq2);	
sendWaveData(freq1);

    HAL_Delay(1000);  // 200ms����һ��
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
        AdcConvEnd = 1;
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
