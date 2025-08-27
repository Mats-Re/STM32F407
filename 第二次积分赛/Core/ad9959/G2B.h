#ifndef __G2B_H
#define __G2B_H

#include "AD9959.h"

// ��ѹ���Ʋ���
#define VPP_MIN 5     // ��С���ֵ5mV
#define VPP_MAX 100   // �����ֵ100mV
#define MAX_AMPLITUDE 1023 // 10λ���ȿ������ֵ

// Ƶ��-��ѹ�����ṹ
typedef struct {
    uint32_t freq_hz;    // Ƶ��(Hz)
    float max_vpp;       // ��Ƶ���µ������ֵ(mV)
} FreqVppCompensation;

// ��ѹ���ƺ���
void G2B_Set_Vpp(uint8_t Channel, uint32_t freq_hz, uint16_t Vpp_mV);
float CalculateVpp(uint16_t *adc_buffer, uint16_t length);
int parse_sensor_frame(const uint8_t* rx2_buf, uint16_t* value);
#endif
