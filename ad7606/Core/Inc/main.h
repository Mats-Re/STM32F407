/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOF
#define D0_Pin GPIO_PIN_1
#define D0_GPIO_Port GPIOF
#define D3_Pin GPIO_PIN_2
#define D3_GPIO_Port GPIOF
#define D2_Pin GPIO_PIN_3
#define D2_GPIO_Port GPIOF
#define D5_Pin GPIO_PIN_4
#define D5_GPIO_Port GPIOF
#define D4_Pin GPIO_PIN_5
#define D4_GPIO_Port GPIOF
#define D7_Pin GPIO_PIN_6
#define D7_GPIO_Port GPIOF
#define D6_Pin GPIO_PIN_7
#define D6_GPIO_Port GPIOF
#define D9_Pin GPIO_PIN_8
#define D9_GPIO_Port GPIOF
#define D8_Pin GPIO_PIN_9
#define D8_GPIO_Port GPIOF
#define D11_Pin GPIO_PIN_10
#define D11_GPIO_Port GPIOF
#define D10_Pin GPIO_PIN_0
#define D10_GPIO_Port GPIOC
#define D13_Pin GPIO_PIN_1
#define D13_GPIO_Port GPIOC
#define D12_Pin GPIO_PIN_2
#define D12_GPIO_Port GPIOC
#define D15_Pin GPIO_PIN_3
#define D15_GPIO_Port GPIOC
#define D14_Pin GPIO_PIN_1
#define D14_GPIO_Port GPIOA
#define CONVST_Pin GPIO_PIN_11
#define CONVST_GPIO_Port GPIOG
#define BUSY_Pin GPIO_PIN_12
#define BUSY_GPIO_Port GPIOG
#define CS_Pin GPIO_PIN_14
#define CS_GPIO_Port GPIOG
#define SCK_Pin GPIO_PIN_15
#define SCK_GPIO_Port GPIOG
#define RST_Pin GPIO_PIN_3
#define RST_GPIO_Port GPIOB
#define RANGE_Pin GPIO_PIN_6
#define RANGE_GPIO_Port GPIOB
#define OS2_Pin GPIO_PIN_7
#define OS2_GPIO_Port GPIOB
#define OS1_Pin GPIO_PIN_8
#define OS1_GPIO_Port GPIOB
#define OS0_Pin GPIO_PIN_9
#define OS0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
