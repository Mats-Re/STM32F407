/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint8_t TIM2CH1_Cap_State=0;
uint16_t TIM2CH1_Cap_Value=0;
uint8_t TIM2CH2_Cap_State=0;
uint16_t TIM2CH2_Cap_Value=0;
float phase_diff=0;
uint8_t phase_diff_ready=0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#define SIGNAL_FREQ 500        // 被测信号频率(Hz)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
if ((TIM2CH1_Cap_State & 0x80) == 0) {
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) && __HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE)) {
    if (TIM2CH1_Cap_State & 0x40) {
      if ((TIM2CH1_Cap_State & 0x3F) == 0x3F) {
        TIM2CH1_Cap_State |= 0x80;
        TIM2CH1_Cap_Value = 0xFFFF;
      } else {
        TIM2CH1_Cap_State++;
      }
    }
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  }

  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) && __HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC1)) {
    if (TIM2CH1_Cap_State & 0x40) {
      // 捕获到下降沿
      TIM2CH1_Cap_State |= 0x80;
      TIM2CH1_Cap_Value = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    } else {
      // 捕获到上升沿
      TIM2CH1_Cap_State = 0x40;
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
  }
}

if ((TIM2CH2_Cap_State & 0x80) == 0) {
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) && __HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE)) {
    if (TIM2CH2_Cap_State & 0x40) {
      if ((TIM2CH2_Cap_State & 0x3F) == 0x3F) {
        TIM2CH2_Cap_State |= 0x80;
        TIM2CH2_Cap_Value = 0xFFFF;
      } else {
        TIM2CH2_Cap_State++;
      }
    }
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  }

  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) && __HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC2)) {
    if (TIM2CH2_Cap_State & 0x40) {
      TIM2CH2_Cap_State |= 0x80;
      TIM2CH2_Cap_Value = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
    } else {
      TIM2CH2_Cap_State = 0x40;
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);
  }
}

// 两路都捕获完成，计算相位差
if ((TIM2CH1_Cap_State & 0x80) && (TIM2CH2_Cap_State & 0x80)) {
  uint32_t timer1_temp, timer2_temp, time_diff;

  timer1_temp = ((TIM2CH1_Cap_State & 0x3F) * 65536) + TIM2CH1_Cap_Value;
  timer2_temp = ((TIM2CH2_Cap_State & 0x3F) * 65536) + TIM2CH2_Cap_Value;

  if (timer1_temp > timer2_temp) {
    time_diff = timer1_temp - timer2_temp;
  } else {
    time_diff = timer2_temp - timer1_temp;
  }

  // 使用全局变量 phase_diff 而不是局部的
  phase_diff = 360.0f * SIGNAL_FREQ * time_diff / 84000000.0f;

  // 设置标志，供主循环读取
  phase_diff_ready = 1;

  // 清除状态准备下一轮捕获
  TIM2CH1_Cap_State = 0;
  TIM2CH2_Cap_State = 0;
}
	
  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
