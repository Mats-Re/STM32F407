#ifndef __G2B_H
#define __G2B_H

#include "AD9959.h"

// 电压控制参数
#define VPP_MIN 5     // 最小峰峰值5mV
#define VPP_MAX 100   // 最大峰峰值100mV
#define MAX_AMPLITUDE 1023 // 10位幅度控制最大值

// 频率-电压补偿结构
typedef struct {
    uint32_t freq_hz;    // 频率(Hz)
    float max_vpp;       // 该频率下的最大峰峰值(mV)
} FreqVppCompensation;

// 电压控制函数
void G2B_Set_Vpp(uint8_t Channel, uint32_t freq_hz, uint16_t Vpp_mV);
float CalculateVpp(uint16_t *adc_buffer, uint16_t length);
int parse_sensor_frame(const uint8_t* rx2_buf, uint16_t* value);
#endif
