/* USER CODE BEGIN Header */
/*
PC3->D15	PA1->D14
PC1->D13	PC2->D12
PF10->D11	PC0->D10
PF8->D9	PF9->D8
PF6->D7(MISO)	PF7->D6
PF4->D5	PF5->D4
PF2->D3	PF3->D2
PF0->D1	PF1->D0
	
	
CONVST-->PG11	
OS1-->PB8	
OS0-->PB9	
RANGE-->PB6	
OS2-->PB7	
SCK-->PG15	
RST-->PB3	
CS-->PG14	
BUSY->PG12	
*/
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ad7606.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void GPIO_PrintStatus(void)
{
    // 读取关键引脚电平，打印到串口
    uint8_t convst = HAL_GPIO_ReadPin(GPIOG, CONVST_Pin);
    uint8_t cs     = HAL_GPIO_ReadPin(CS_GPIO_Port, CS_Pin);
    uint8_t sck    = HAL_GPIO_ReadPin(GPIOG, SCK_Pin);
    uint8_t busy   = HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin);
    
    // 读取部分数据线状态示例
    uint8_t d0 = HAL_GPIO_ReadPin(GPIOF, D0_Pin);
    uint8_t d1 = HAL_GPIO_ReadPin(GPIOF, D1_Pin);
    uint8_t d2 = HAL_GPIO_ReadPin(GPIOF, D2_Pin);
    uint8_t d3 = HAL_GPIO_ReadPin(GPIOF, D3_Pin);
    
    printf("CONVST=%d, CS=%d, SCK=%d, BUSY=%d, D0=%d, D1=%d, D2=%d, D3=%d\r\n",
        convst, cs, sck, busy, d0, d1, d2, d3);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint16_t datatemp[8];
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AD7606_EmergencyFix() {
    // 启用GPIO内部上拉补偿硬件问题
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);  // PF8强制上拉
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // PA1强制上拉
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DataBus_DebugPrint(void) {
    printf("Data Bus Status:\n");
    printf("PF0(D1):%d PF1(D0):%d PF2(D3):%d PF3(D2):%d\n", 
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3));
    printf("PF4(D5):%d PF5(D4):%d PF6(D7):%d PF7(D6):%d\n",
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7));
    printf("PF8(D9):%d PF9(D8):%d PF10(D11):%d PC0(D10):%d\n",
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9),
           HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10),
           HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0));
    printf("PC1(D13):%d PC2(D12):%d PA1(D14):%d PC3(D15):%d\n",
           HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1),
           HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2),
           HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1),
           HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3));
}
float AD7606_DataToVoltage(int16_t raw)
{
    return ((float)raw / 32768.0f) * 10000.0f;  // ±10V模式
}

void GPIO_ForceTest(void) {
    // 配置PF8和PA1为输出模式
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    // 测试PF8 (D9)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    // 测试高低电平切换
    printf("Testing PF8 (D9):\n");
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
    printf("RESET: %d\n", HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8));
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
    printf("SET: %d\n", HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8));
    HAL_Delay(100);

    HAL_Delay(100);
    
    // 测试PA1 (D14)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    printf("Testing PA1 (D14):\n");
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    printf("RESET: %d\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    printf("SET: %d\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
    HAL_Delay(100);

    
    // 恢复为输入模式（带内部上拉）
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
int16_t ad7606_raw[8];
int16_t ad_data[8];
		AD7606_GPIO_Init();
    AD7606_SetOS(0);
    AD7606_SetRange(1); 
    HAL_Delay(10);  // 新增延时，确保电平稳定

		    GPIO_ForceTest();
    DataBus_DebugPrint();
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
