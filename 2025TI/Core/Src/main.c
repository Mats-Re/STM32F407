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
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD9959.h"
#include "G2B.h"
#include "stdio.h"
#include "sweep.h"
#include "interact.h"
#include "string.h"
#include "math.h"
#include "ads8688.h"
#include "DB.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUF_SIZE 64
#define FFT_LENGTH 1024
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
__IO uint8_t AdcConvEnd = 0;
uint16_t adcBuff[FFT_LENGTH];

float fft_inputbuf[FFT_LENGTH * 2];  
float fft_outputbuf[FFT_LENGTH];  
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t fre=0;
uint16_t flag=0;
uint16_t fremax=0;
uint16_t fremin=0;
uint16_t mode=0;
uint16_t count=0; 
float last_result; 
extern int _estack; // 定义在链接脚本中
float adc_values[FFT_LENGTH];
extern int32_t result; 
extern int mod;
int s1=0;
int s2=0;
int i=0;
volatile uint16_t sensor_data = 0; 
float vol=1;
float freq=0;
float amp=0;
float frequency;
float ampli;
char  channel;
extern const float data[30][11];
float att[998];
//////////////////////////////////////////////////////
uint16_t adc_buff[1024];//存放ADC采集的数据

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AD9959_sweep(uint32_t fre,uint32_t mode,uint32_t fremax,uint32_t fremin,uint32_t speed);
float Calculate_Frequency(uint16_t *adc_data, uint32_t fftsize, float sampling_freq);
double normalize(double value);
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


	result=0;
	float aimval;
	float sampling_freq = 500000; 
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc1); 
	HAL_UART_Receive_IT(&huart1, &rx1_tmp, 1);
	HAL_UART_Receive_IT(&huart2, &rx2_tmp, 1);
	HAL_TIM_Base_Start(&htim3);   //开启定时器3
	HAL_TIM_Base_Start(&htim2);   //开启定时器3
	AD9959_Init();								//初始化控制AD9959需要用到的IO口,及寄存器							// 添加10ms延时确保初始化完成
	ADS8688_IO_Init();
	ADS8688_Init(0xFF);
	Set_CH_Range(CHIR_1, ADS8688_IR_N2_5V);
	Set_CH_Range(CHIR_2, ADS8688_IR_N2_5V);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, FFT_LENGTH);
HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff, FFT_LENGTH);
HAL_TIM_Base_Start(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
{
if(mod==1)
{
if(result==-42)
{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);//RE1
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);//RE3
}
////////////////////////////////////
if(result==-43)
{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);//RE1
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);//RE3
}
////////////////////////////////////
if(result==-1)
{
AD9959_Init();
freq-=100;
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
printf("vol.txt=\"%.2fmV\"\xff\xff\xff",amp);
printf("result.txt=\"%d\"\xff\xff\xff",result);
result=0;
}
////////////////////////////////////
if(result==-2)
{
AD9959_Init();
freq+=100;
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
printf("vol.txt=\"%.2fmV\"\xff\xff\xff",amp);
printf("result.txt=\"%d\"\xff\xff\xff",result);
result=0;
}
////////////////////////////////////
if(result==-3)
{
AD9959_Init();
amp+=100;
aimval=amp;
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
printf("vol.txt=\"%.2fmV\"\xff\xff\xff",amp);
printf("result.txt=\"%d\"\xff\xff\xff",result);
result=0;
}
////////////////////////////////////
if(result==-4)
{
AD9959_Init();
amp-=100;
aimval=amp;
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
printf("vol.txt=\"%.2fmV\"\xff\xff\xff",amp);
printf("result.txt=\"%d\"\xff\xff\xff",result);
result=0;
}
////////////////////////////////////
if(result>0)
{
AD9959_Init();
last_result=result;
IO_Update();
result=0;
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
printf("vol.txt=\"%.2fmV\"\xff\xff\xff",amp);
printf("result.txt=\"%d\"\xff\xff\xff",result);
IO_Update();
}
////////////////////////////////////
    // 频率扫描示例（取消注释使用）
if(result==-6) 
{
	AD9959_Set_Amp(CH0,235 / 521.5 * 1024);IO_Update();
	AD9959_Set_Amp(CH1,240 / 521.5 * 1024);IO_Update();
	printf("result.txt=\"%d\n\"\xff\xff\xff",result);
	if(i<=998)
{
AD9959_sweep
	(
1000,    // 起始频率
3,          //1:正向  2:降频  3：反复
200600,   // 最大频率
1000,    // 最小频率
200		// 步进速度
	); 
		 MAN_CH_Mode(MAN_CH_0);
		 adc_values[0] = Get_MAN_CH_Mode_Data();  //获取此次循环中的adc值
		 float voltage_CH1 = (adc_values[0] - 32768) * (1.0f / 3196.0f);
		 float V1=voltage_CH1*0.7396;
		 printf("V1.txt=\"%.2f\"\xff\xff\xff",V1 );
		 printf("VOL1.txt=\"%.2f\"\xff\xff\xff",voltage_CH1 );
		 MAN_CH_Mode(MAN_CH_1);
		 adc_values[0] = Get_MAN_CH_Mode_Data(); 
		 float voltage_CH2 = (adc_values[0] - 32768) * (1.0f / 3196.0f);
		 float V2=voltage_CH2*0.7494;
		 printf("V2.txt=\"%.2f\"\xff\xff\xff",V2 );
		 printf("VOL2.txt=\"%.2f\"\xff\xff\xff",voltage_CH2 );
		 float diff=V1/V2;
		 printf("att.txt=\"%.2f\"\xff\xff\xff",20 * log10(diff));		 	  
		 att[i]=20 * log10(diff);	
}
printf("i.txt=\"%d\"\xff\xff\xff",i);
if(i<=998)
i++;	 
else
{
detect_filter_type(att);
result=0;
}
HAL_Delay(1);
}
if(result==-9)
{
AD9959_Init();	
freq=1000;
amp=91.74;	
AD9959_Set_Fre(CH0,freq);
AD9959_Set_Amp(CH0,amp/ 521.5 * 1024);
IO_Update();
result=0;
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
printf("vol.txt=\"%.2fmV\"\xff\xff\xff",amp);
printf("result.txt=\"%d\"\xff\xff\xff",result);
}
if(result==-8)//(4)查找表
{
AD9959_Init();
aimval=amp;
getValue(freq, amp/1000,&vol);
amp=vol*1000;
printf("aim.txt=\"%.4fmV\"\xff\xff\xff",amp);
printf("vol.txt=\"%.2fV\"\xff\xff\xff",aimval/1000);
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
printf("result.txt=\"%d\"\xff\xff\xff",result);
IO_Update();
result=0;
}
if(result==-14)
{
freq=last_result;
printf("freq.txt=\"%.2fHz\"\xff\xff\xff",freq);
result=0;
}
if(result==-13)
{
amp=last_result;
printf("vol.txt=\"%.2fmV\"\xff\xff\xff",amp);
result=0;
}
if(result==-11)
{
i=0;
AD9959_Set_Fre(CH1,freq+1000);
AD9959_Set_Fre(CH0,freq);
AD9959_Set_Amp(CH1,amp / 521.5 * 1024*0.93);
AD9959_Set_Amp(CH0,amp / 521.5 * 1024*0.93);
IO_Update();
HAL_Delay(40);
//AD9959_Set_Amp(CH1,amp / 521.5 * 1024/2);IO_Update();
amp=aimval;
printf("result.txt=\"%d\"\xff\xff\xff",result);
result=0;
}
if(result==-8688)//进行ads8688采样
{   
    // 打印原始值
		 MAN_CH_Mode(MAN_CH_0);
		 adc_values[0] = Get_MAN_CH_Mode_Data();  
		 float voltage_CH1 = (adc_values[0] - 32768) * (1.0f / 3196.0f);
		 float V1=voltage_CH1*0.7396;
		 printf("V1.txt=\"%.2f\"\xff\xff\xff",V1 );
		 printf("VOL1.txt=\"%.2f\"\xff\xff\xff",voltage_CH1 );
		 MAN_CH_Mode(MAN_CH_1);
		 adc_values[0] = Get_MAN_CH_Mode_Data(); 
		 float voltage_CH2 = (adc_values[0] - 32768) * (1.0f / 3196.0f);
		 float V2=voltage_CH2*0.7494;
		 printf("V2.txt=\"%.2f\"\xff\xff\xff",V2 );
		 printf("VOL2.txt=\"%.2f\"\xff\xff\xff",voltage_CH2 );
		 float diff=V1/V2;
		 printf("att.txt=\"%.2f\"\xff\xff\xff",20 * log10(diff ));
		 result=0;
}
if(result==-10)
{
printf("result.txt=\"%d\"\xff\xff\xff",result);
AD9959_Init();
result=0;
}
//if(result==-19)//freq
//{
//HAL_TIM_Base_Start(&htim3);
//uint16_t adc_buff[200];
//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuff,FFT_LENGTH);
//while (!AdcConvEnd)       //等待转换完毕
//	;
//for (int i = 0; i < FFT_LENGTH; i++)
//{
//    fft_inputbuf[i * 2] = adcBuff[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
//    fft_inputbuf[i * 2 + 1] = 0;//虚部赋值，固定为0.
//}
//arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);
//arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH); 
//fft_outputbuf[0] /= 1024;
//for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
//{
//    fft_outputbuf[i] /= 512;
//}
//float frequency = Calculate_Frequency(adcBuff, FFT_LENGTH, sampling_freq);
//float resfre=normalize(frequency/2000);
//printf("Measured Frequency: %.2f kHz\r\n", (frequency/2000));
//printf("resfre.txt=\"%.2fkhz\"\xff\xff\xff",(float)resfre);
//printf("resolved Frequency: %.2f\r\n", resfre);
//int n = (resfre*1000 - 1000) / 200;
//HAL_TIM_Base_Start(&htim2);//开启定时器2
//float V=3.3*fabs(att[n]/(-40)-1)/4095;
//const uint16_t Sine12bit[1]={V};
//HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)Sine12bit,1,DAC_ALIGN_12B_R);//开启输出
//HAL_TIM_Base_Stop(&htim3);
//result=0;
//}

}
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
    if (flag == 0) 
		{
        fre = fremin + count * speed;  // 上升扫描
    } 
		else 
		{
        fre = fremax - count * speed;  // 下降扫描（从 fremax 开始减）        
    }

    // 2. 检查边界，更新 flag
    if (fre >= fremax) 
		{  // 超过上限 → 改为下降扫描
        flag = 1;
        fre = fremax;    // 防止超出
        count = 0;       // 重置 count
    } 
		else if (fre <= fremin) 
		{  // 低于下限 → 改为上升扫描
        flag = 0;
        fre = fremin;    // 防止低于最小值
        count = 0;       // 重置 count
    }
		count++;
    AD9959_Set_Fre(CH1,fre);//1K接未知电路
		AD9959_Set_Fre(CH0,fre+1000);
    IO_Update(); 
    // 打印原始值
		float voltage_CH1 = (adc_values[0] - 32768) * (1.0f / 3196.0f);
		MAN_CH_Mode(MAN_CH_0);
		adc_values[0] = Get_MAN_CH_Mode_Data();     
    printf("DB1.txt=\"%.2f\"\xff\xff\xff",20 * log10(100*voltage_CH1 ));
		MAN_CH_Mode(MAN_CH_0);
		adc_values[0] = Get_MAN_CH_Mode_Data();     
    printf("DB1.txt=\"%.2f\"\xff\xff\xff",20 * log10(100*voltage_CH1 ));
}

}
////////////////////////////////////////////////////////
float Calculate_Frequency(uint16_t *adc_data, uint32_t fftsize, float sampling_freq)
	{

    // 1. 准备FFT输入数据
	for (int i = 0; i < FFT_LENGTH; i++)
{
fft_inputbuf[i * 2] = adcBuff[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
fft_inputbuf[i * 2 + 1] = 0;//虚部赋值，固定为0.
}
fft_outputbuf[0] /= 1024;
for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
{
fft_outputbuf[i] /= 512;
}
for (uint32_t i = 0; i < fftsize; i++) 
	{
 fft_inputbuf[i * 2] = (float)adc_data[i] * 3.3f / 4096.0f;
fft_inputbuf[i * 2 + 1] = 0.0f;
    }   
    // 2. 执行FFT计算
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);    
    // 3. 计算幅度谱
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, fftsize);
    
    // 4. 寻找基波频率（最大幅值对应的频率）
    float max_magnitude = 0.0f;
    uint32_t max_idx = 0;
    for (uint32_t i = 1; i < fftsize / 2; i++) 
	{
if (fft_outputbuf[i] > max_magnitude) 
	{
    max_magnitude = fft_outputbuf[i];
    max_idx = i;
  }
  }   
    // 5. 使用三点插值法提高频率精度
    float y1 = fft_outputbuf[max_idx - 1];
    float y2 = fft_outputbuf[max_idx];
    float y3 = fft_outputbuf[max_idx + 1];   
    // 抛物线插值公式
    float delta = 0.5f * (y3 - y1) / (2.0f * y2 - y1 - y3);
    float interpolated_idx = (float)max_idx + delta;    
    // 6. 计算实际频率
    float frequency = interpolated_idx * (sampling_freq / fftsize);   
    // 7. 保留小数点后两位
    return frequency * 100.0f / 100.0f;
}

double normalize(double value) {
    // 核心算法：四舍五入到最近的0.2倍数
    return round(value * 5.0) * 0.2;
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
