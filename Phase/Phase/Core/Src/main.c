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
// ����У׼���ݽṹ
// ����У׼���ݽṹ
typedef struct {
    float freq;      // Ƶ�ʵ�(Hz)
    float offset;    // ��Ƶ�ʵ����λƫ��(��)
} PhaseCalibrationPoint;

#define CALIBRATION_POINTS 10  // У׼������
static PhaseCalibrationPoint calibration_table[CALIBRATION_POINTS] = {
    {1000,   73.5f},   // 1kHzʱ��У׼ֵ
    {5000,   72.8f},   // 5kHz
    {10000,  71.2f},   // 10kHz
    {20000,  69.5f},   // 20kHz
    {50000,  65.0f},   // 50kHz
    {100000, 60.0f},   // 100kHz
    {200000, 55.0f},   // 200kHz
    {300000, 50.0f},   // 300kHz
    {400000, 45.0f},   // 400kHz
    {500000, 40.0f}    // 500kHz
};
static float last_calibrated_freq = 0;
static float current_offset = 73.5f;  // Ĭ��1kHzУ׼ֵ
#define SAMPLE_LEN (2*N-1)  // 127��������
#define FFT_LENGTH 1024
static float hanning_window[FFT_LENGTH];  // Ԥ���㺺����
uint8_t hanning_initialized = 0;         // ��������ʼ����־
/* 
AdcConvEnd�������ADC�Ƿ�ɼ����
0��û�вɼ����
1���ɼ���ϣ���stm32f1xx_it���DMA����жϽ����޸�
 */
__IO uint8_t AdcConvEnd1 = 0;
__IO uint8_t AdcConvEnd2 = 0;
uint16_t adcBuff1[FFT_LENGTH];
uint16_t adcBuff2[FFT_LENGTH];
float fft_inputbuf[FFT_LENGTH*2];  
float fft_outputbuf[FFT_LENGTH];  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float Calculate_Frequency(uint16_t *adc_data, uint32_t fft_size, float sampling_freq);
float Calculate_Phase_Difference_Corrected(uint16_t *adc_data1, uint16_t *adc_data2, uint32_t fft_size, float sampling_freq);
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

  float sampling_freq = 1024280.f; // �޸�Ϊ1MHz
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff1, FFT_LENGTH);
HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcBuff2, FFT_LENGTH);
HAL_TIM_Base_Start(&htim3);


while (!AdcConvEnd1 && AdcConvEnd2)       //�ȴ�ת�����
	;
///////////////////////////////////////////////////////////
for (int i = 0; i < FFT_LENGTH; i++)
{
    fft_inputbuf[i * 2] = adcBuff1[i] * 3.3 / 4096;//ʵ����ֵ��* 3.3 / 4096��Ϊ�˽�ADC�ɼ�����ֵת����ʵ�ʵ�ѹ
    fft_inputbuf[i * 2 + 1] = 0;//�鲿��ֵ���̶�Ϊ0.
}
for (int i = 0; i < FFT_LENGTH; i++)
{
    fft_inputbuf[i * 2] = adcBuff2[i] * 3.3 / 4096;//ʵ����ֵ��* 3.3 / 4096��Ϊ�˽�ADC�ɼ�����ֵת����ʵ�ʵ�ѹ
    fft_inputbuf[i * 2 + 1] = 0;//�鲿��ֵ���̶�Ϊ0.
}
////////////////////////////////////////////////////////////
arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);
arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH); 
fft_outputbuf[0] /= 1024;

for (int i = 1; i < FFT_LENGTH; i++)//�������г����ֵ
{
    fft_outputbuf[i] /= 512;
}
float freq1 = Calculate_Frequency(adcBuff1, FFT_LENGTH, sampling_freq);
float freq2 = Calculate_Frequency(adcBuff2, FFT_LENGTH, sampling_freq);
 printf("Measured Frequency1	: %.2f kHz\r\n", freq1/1000);
printf("Measured Frequency2	: %.2f kHz\r\n", freq2/1000);
float phase_diff = Calculate_Phase_Difference_Corrected(adcBuff1, adcBuff2, FFT_LENGTH, sampling_freq);
printf("Corrected phase difference: %.2f degrees\r\n", phase_diff);
AdcConvEnd1 = 0;AdcConvEnd2 = 0;
HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff1, FFT_LENGTH);
HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcBuff2, FFT_LENGTH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
float Calculate_Frequency(uint16_t *adc_data, uint32_t fft_size, float sampling_freq) 
	{
    // 1. ׼��FFT��������
    for (uint32_t i = 0; i < fft_size; i++) {
        fft_inputbuf[i * 2] = (float)adc_data[i] * 3.3f / 4096.0f;
        fft_inputbuf[i * 2 + 1] = 0.0f;
    }   
    // 2. ִ��FFT����
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);
    
    // 3. ���������
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, fft_size);    
    // 4. Ѱ�һ���Ƶ�ʣ�����ֵ��Ӧ��Ƶ�ʣ�
    float max_magnitude = 0.0f;
    uint32_t max_idx = 0;
    for (uint32_t i = 1; i < fft_size / 2; i++) {
        if (fft_outputbuf[i] > max_magnitude) {
            max_magnitude = fft_outputbuf[i];
            max_idx = i;
        }
    }
    // 5. ʹ�������ֵ�����Ƶ�ʾ���
    float y1 = fft_outputbuf[max_idx - 1];
    float y2 = fft_outputbuf[max_idx];
    float y3 = fft_outputbuf[max_idx + 1]; 
    // �����߲�ֵ��ʽ
    float delta = 0.5f * (y3 - y1) / (2.0f * y2 - y1 - y3);
    float interpolated_idx = (float)max_idx + delta;  
    // 6. ����ʵ��Ƶ��
    float frequency = interpolated_idx * (sampling_freq / fft_size);   
    // 7. ����С�������λ
    return roundf(frequency * 100.0f) / 100.0f;
}
float Get_Frequency_Dependent_Offset(float measured_freq) {
    // 1. �߽���
    if (measured_freq <= calibration_table[0].freq) {
        return calibration_table[0].offset;
    }
    if (measured_freq >= calibration_table[CALIBRATION_POINTS-1].freq) {
        return calibration_table[CALIBRATION_POINTS-1].offset;
    }

    // 2. ���������У׼��
    uint8_t i = 0;
    while (i < CALIBRATION_POINTS-1 && measured_freq > calibration_table[i+1].freq) {
        i++;
    }

    // 3. ���Բ�ֵ����
    float x0 = calibration_table[i].freq;
    float x1 = calibration_table[i+1].freq;
    float y0 = calibration_table[i].offset;
    float y1 = calibration_table[i+1].offset;
    
    return y0 + ( (y1-y0) * (measured_freq - x0) / (x1 - x0) );
}
float Calculate_Phase_Difference_Corrected(uint16_t *adc_data1, uint16_t *adc_data2, uint32_t fft_size, float sampling_freq) 
{
    static float fft_inputbuf1[FFT_LENGTH * 2];
    static float fft_inputbuf2[FFT_LENGTH * 2];
    
    // 1. Ԥ���㺺����(ֻ��һ��)
    if (!hanning_initialized) {
        for (uint32_t i = 0; i < FFT_LENGTH; i++) {
            hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2 * PI * i / (FFT_LENGTH - 1)));
        }
        hanning_initialized = 1;
    }

    // 2. ������ȥֱ������
    float32_t mean1, mean2;
    arm_mean_f32((float32_t *)adc_data1, fft_size, &mean1);
    arm_mean_f32((float32_t *)adc_data2, fft_size, &mean2);
    
    // 3. Ӧ�ú�������׼��FFT����(������)
    for (uint32_t i = 0; i < fft_size; i++) {
        float sample1 = ((float)adc_data1[i] - mean1) * hanning_window[i];
        float sample2 = ((float)adc_data2[i] - mean2) * hanning_window[i];
        
        fft_inputbuf1[i * 2] = sample1;
        fft_inputbuf1[i * 2 + 1] = 0;
        fft_inputbuf2[i * 2] = sample2;
        fft_inputbuf2[i * 2 + 1] = 0;
    }

    // 4. ִ��FFT����
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf1, 0, 1);
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf2, 0, 1);

    // 5. ʹ��ƽ���Ƚ��Ż���������
    float max_sq_mag = 0.0f;
    uint32_t max_idx = 0;
    const uint32_t search_end = fft_size / 4;
    
    for (uint32_t i = 5; i < search_end; i++) {
        // ʹ��ƽ�����⿪��
        float real1 = fft_inputbuf1[i*2];
        float imag1 = fft_inputbuf1[i*2+1];
        float sq_mag = real1*real1 + imag1*imag1;
        
        if (sq_mag > max_sq_mag) {
            max_sq_mag = sq_mag;
            max_idx = i;
        }
    }

    // 6. ��̬У׼����
    float current_freq = (max_idx * sampling_freq) / fft_size;
    
    // Ƶ�ʱ仯����1%ʱ����У׼����
    if (fabsf(current_freq - last_calibrated_freq) > (0.01f * last_calibrated_freq)) {
        current_offset = Get_Frequency_Dependent_Offset(current_freq);
        last_calibrated_freq = current_freq;
    }

    // 7. ֱ�Ӽ�����λ��
    float phase1 = atan2f(fft_inputbuf1[max_idx*2+1], fft_inputbuf1[max_idx*2]);
    float phase2 = atan2f(fft_inputbuf2[max_idx*2+1], fft_inputbuf2[max_idx*2]);
    float phase_diff = phase1 - phase2;

    // 8. Ӧ�ö�̬У׼
    phase_diff += (current_offset * PI / 180.0f);  // ת��Ϊ����
    
    // 9. ��λ��淶����[-��, ��]
    if (phase_diff > PI) phase_diff -= 2 * PI;
    else if (phase_diff < -PI) phase_diff += 2 * PI;

    return phase_diff * (180.0f / PI);  // ת��Ϊ�Ƕ�
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
