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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD9959.h"
#include "hmi.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define freq1 CH0 
#define freq2 CH1 
#define freq3 CH2 
#define freq4 CH3 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_SIZE 8      // 完整帧大小 (55 00 xx xx xx xx 0D 0A)
#define MAX_DATA_LEN 32   // 接收缓冲区大小 (大于帧大小即可)

// 接收数据结构
typedef struct {
uint8_t buffer[MAX_DATA_LEN];  // 接收数据存储
volatile uint16_t index;       // 当前存储位置
volatile uint8_t ready;        // 完整帧接收标志
} UART_RxBuffer;

UART_RxBuffer rx_data = {0}; // 初始化接收结构体
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void hex_to_ascii(uint8_t *hex_data, int hex_len, char *ascii_buffer);
uint8_t *hex_data;
int hex_len;
char *ascii_buffer;

void hex_bytes_to_ascii(uint8_t *hex_data, int hex_len, char *ascii_buffer);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t convert_hex_sequence(const uint8_t *data);
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
  AD9959_Init(); // 初始化AD9959
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // 设置AD9959初始参数
  AD9959_Set_Amp(CH0, 1023); // 设置通道0幅度
  AD9959_Set_Amp(CH1, 512);  // 设置通道1幅度
  AD9959_Set_Amp(CH2, 256);  // 设置通道2幅度
  AD9959_Set_Amp(CH3, 128);  // 设置通道3幅度

  AD9959_Set_Phase(CH0, 0);     // 设置通道0相位
  AD9959_Set_Phase(CH1, 4096);  // 设置通道1相位
  AD9959_Set_Phase(CH2, 8192);  // 设置通道2相位
  AD9959_Set_Phase(CH3, 12288); // 设置通道3相位
  IO_Update(); // AD9959更新数据
  
  // 发送欢迎消息
  printf("AD9959 DDS控制器已启动\r\n");

  printf("等待串口数据...\r\n");
  printf("数据格式: 55 00 xx xx xx xx 0D 0A\r\n");

  // 启动串口接收中断
  HAL_UART_Receive_IT(&huart1, &rx_data.buffer[rx_data.index], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (rx_data.ready) {
      // 转换接收到的数据
			
      uint32_t value = convert_hex_sequence(rx_data.buffer);
			char ascii_buffer[MAX_DATA_LEN * 2 + 1];
      hex_bytes_to_ascii(rx_data.buffer,rx_data.index, ascii_buffer);
      if (value != 0xFFFFFFFF) {
        printf("\r\n");
        
        // 设置AD9959频率
        AD9959_Set_Fre(CH0,(int)value);
        IO_Update();
        printf("Freq: %lu Hz\r\n",(unsigned long)value);
      } 
			else {
        printf("Channel: %c\r\n", ascii_buffer[4]);  // 用 \r\n 避免多余换行
      }
			switch(ascii_buffer[4])
   {
   case '1' :
      AD9959_Set_Fre(CH0,(int)value);
      break;
   case '2' :
      AD9959_Set_Fre(CH1,(int)value);
      break;
   case '3' :
      AD9959_Set_Fre(CH2,(int)value);
      break;
   case '4' :
      AD9959_Set_Fre(CH3,(int)value);
      break;
   }
    IO_Update();
      // 重置接收状态
      rx_data.index = 0;
      rx_data.ready = 0;
      
      // 重新启动接收
      HAL_UART_Receive_IT(&huart1,&rx_data.buffer[rx_data.index], 1);
    }
    // 短延迟，减少CPU占用
    HAL_Delay(10);
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

/*接收数据并把数据储存到  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t current_byte = rx_data.buffer[rx_data.index];
        
        // 检测帧结束符 (0D 0A)
        if (rx_data.index >= 1 && 
            rx_data.buffer[rx_data.index - 1] == 0x0D && 
            current_byte == 0x0A) 
        {
            rx_data.ready = 1; // 标记帧接收完成
            return;
        }
        
        rx_data.index++;
        if (rx_data.index >= MAX_DATA_LEN) {
            // 缓冲区溢出保护
            rx_data.index = 0;
        }
        HAL_UART_Receive_IT(huart, &rx_data.buffer[rx_data.index], 1);
    }
}

/**
  * @brief 串口错误回调函数
  * @param huart 串口句柄
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        printf("串口错误! 重置接收状态\r\n");
        // 重置接收状态
        rx_data.index = 0;
        HAL_UART_Receive_IT(huart, &rx_data.buffer[rx_data.index], 1);
    }
}

/**
  * @brief 转换十六进制序列为32位整数
  * @param data 数据指针 (必须包含完整帧)
  * @retval 转换后的32位值 (0xFFFFFFFF 表示无效帧)
  */
/*  */
uint32_t convert_hex_sequence(const uint8_t *data) 
	{
    // 检查帧头 (55 00) 和帧尾 (0D 0A)
    if (data[0] != 0x55 || data[1] != 0x00 || 
        data[6] != 0x0D || data[7] != 0x0A) {
        return 0xFFFFFFFF;  // 无效帧
    }
    
    // 小端序组合32位值
    return (uint32_t)data[5] << 24 | 
           (uint32_t)data[4] << 16 | 
           (uint32_t)data[3] << 8  | 
           (uint32_t)data[2];
		
}



// 核心转换函数
void hex_to_ascii(uint8_t *hex_data, int hex_len, char *ascii_buffer) {
    for (int i = 0; i < hex_len; i++) {
        // 处理高4位
        uint8_t nibble_high = (hex_data[i] >> 4) & 0x0F;
        ascii_buffer[i * 2] = (nibble_high < 10) ? 
                             (nibble_high + '0') : 
                             (nibble_high - 10 + 'A');

        // 处理低4位
        uint8_t nibble_low = hex_data[i] & 0x0F;
        ascii_buffer[i * 2 + 1] = (nibble_low < 10) ? 
                                  (nibble_low + '0') : 
                                  (nibble_low - 10 + 'A');
    }
    ascii_buffer[hex_len * 2] = '\0';  // 添加字符串结束符
}
void hex_bytes_to_ascii(uint8_t *hex_data, int hex_len, char *ascii_buffer) {
    int j = 0;
    for (int i = 0; i < hex_len; i++) {
        if (hex_data[i] >= 32 && hex_data[i] <= 126) {  // 只保留可见ASCII字符
            ascii_buffer[j++] = (char)hex_data[i];
        }
    }
    ascii_buffer[j] = '\0';
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
  printf("断言失败: %s line %lu\r\n", file, line);
  while (1) {}
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
