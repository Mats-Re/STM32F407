#include "main.h"
#include "stdio.h"
#include "usart.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "arm_math.h"		// // CMSIS DSP 数学库
#include "arm_const_structs.h" // FFT 结构体定义
#include <stdio.h>// 标准输入输出库
#include "AD9959.h"// AD9959 DDS 控制头文件
/*--------------------------------------------------------------*/
#define WAVE_NAME "s1"   // 串口屏中波形图控件名称
#define MAX_FREQ  100000.0f    // 最大频率值（根据您的应用调整）
#define MAX_WAVE  1000      // 波形图Y轴最大值
/*--------------------------------------------------------------*/
void HMI_UpdateTwoFrequencies(float freq1, float freq2) {
    // 创建缓冲区（足够容纳两条指令）
    char buffer[100];
    int offset = 0;
    
    // 1. 构建第一条指令：更新freq2显示
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, 
                      "start.freq2.txt=\"%.2f\"", freq2);
    
    // 添加第一条指令的结束符
    memcpy(buffer + offset, "\xFF\xFF\xFF", 3);
    offset += 3;
    
    // 2. 构建第二条指令：更新freq1显示
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, 
                      "start.freq1.txt=\"%.2f\"", freq1);
    
    // 添加第二条指令的结束符
    memcpy(buffer + offset, "\xFF\xFF\xFF", 3);
    offset += 3;
    
    // 3. 一次性发送所有指令
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, offset, HAL_MAX_DELAY);
}
/*--------------------------------------------------------------*/
void sendWaveData(float freq) {
    // 转换频率为波形值 (0-1000)
    int wave_val = (int)((freq / MAX_FREQ) * MAX_WAVE);
    
    // 创建指令并发送
    char cmd[30];
    sprintf(cmd, "%s.add(0,%d)\xFF\xFF\xFF", WAVE_NAME, wave_val);
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
}
