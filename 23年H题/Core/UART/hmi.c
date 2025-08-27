#include "main.h"
#include "stdio.h"
#include "usart.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "arm_math.h"		// // CMSIS DSP ��ѧ��
#include "arm_const_structs.h" // FFT �ṹ�嶨��
#include <stdio.h>// ��׼���������
#include "AD9959.h"// AD9959 DDS ����ͷ�ļ�
/*--------------------------------------------------------------*/
#define WAVE_NAME "s1"   // �������в���ͼ�ؼ�����
#define MAX_FREQ  100000.0f    // ���Ƶ��ֵ����������Ӧ�õ�����
#define MAX_WAVE  1000      // ����ͼY�����ֵ
/*--------------------------------------------------------------*/
void HMI_UpdateTwoFrequencies(float freq1, float freq2) {
    // �������������㹻��������ָ�
    char buffer[100];
    int offset = 0;
    
    // 1. ������һ��ָ�����freq2��ʾ
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, 
                      "start.freq2.txt=\"%.2f\"", freq2);
    
    // ��ӵ�һ��ָ��Ľ�����
    memcpy(buffer + offset, "\xFF\xFF\xFF", 3);
    offset += 3;
    
    // 2. �����ڶ���ָ�����freq1��ʾ
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, 
                      "start.freq1.txt=\"%.2f\"", freq1);
    
    // ��ӵڶ���ָ��Ľ�����
    memcpy(buffer + offset, "\xFF\xFF\xFF", 3);
    offset += 3;
    
    // 3. һ���Է�������ָ��
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, offset, HAL_MAX_DELAY);
}
/*--------------------------------------------------------------*/
void sendWaveData(float freq) {
    // ת��Ƶ��Ϊ����ֵ (0-1000)
    int wave_val = (int)((freq / MAX_FREQ) * MAX_WAVE);
    
    // ����ָ�����
    char cmd[30];
    sprintf(cmd, "%s.add(0,%d)\xFF\xFF\xFF", WAVE_NAME, wave_val);
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
}
