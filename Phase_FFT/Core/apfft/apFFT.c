#include "apFFT.h"
#include "main.h"
#include "arm_math.h"
#include "stdio.h"
#define FFT_LENGTH 1024
#define SYSTEM_TAU      (42.17e-6f)    // 时间延迟 (42.17 μs)
#define SYSTEM_PHI0     (-404.99f)   
uint16_t adcBuff1[FFT_LENGTH];
uint16_t adcBuff2[FFT_LENGTH];
float fft_inputbuf[FFT_LENGTH*2];  
float fft_outputbuf[FFT_LENGTH];  
/* USER CODE BEGIN 4 */
/**
  * @brief  计算两个信号的相位差(度)
  * @param  adc_data1: 第一个信号的ADC数据
  * @param  adc_data2: 第二个信号的ADC数据
  * @param  fft_size: FFT大小
  * @param  sampling_freq: 采样频率
  * @retval 两个信号在基波频率处的相位差(度)
  */


float Calculate_PhaseDifference(uint16_t *adc_data1, uint16_t *adc_data2, 
                               uint32_t fft_size, float sampling_freq) 
{
    static float fft_inputbuf1[FFT_LENGTH * 2];
    static float fft_inputbuf2[FFT_LENGTH * 2];
    static float fft_mag1[FFT_LENGTH];

    // 1. 预处理：去直流 & 缩放
    float mean1 = 0.0f, mean2 = 0.0f;
    for (uint32_t i = 0; i < fft_size; i++) {
        mean1 += adc_data1[i];
        mean2 += adc_data2[i];
    }
    mean1 /= fft_size;
    mean2 /= fft_size;

    for (uint32_t i = 0; i < fft_size; i++) {
        fft_inputbuf1[2*i] = (adc_data1[i] - mean1) * 3.3f / 4096.0f;
        fft_inputbuf1[2*i+1] = 0.0f;
        fft_inputbuf2[2*i] = (adc_data2[i] - mean2) * 3.3f / 4096.0f;
        fft_inputbuf2[2*i+1] = 0.0f;
    }

    // 2. FFT计算
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf1, 0, 1);
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf2, 0, 1);
    
    // 3. 幅度谱 & 寻找主频
    arm_cmplx_mag_f32(fft_inputbuf1, fft_mag1, fft_size);
    
    uint32_t max_idx = 1;
    float max_mag = fft_mag1[1];
    for (uint32_t i = 2; i < fft_size/2 - 1; i++) {
        if (fft_mag1[i] > max_mag) {
            max_mag = fft_mag1[i];
            max_idx = i;
        }
    }
    
    // 4. 抛物线插值 - 精确频率定位
    float magL = fft_mag1[max_idx-1];
    float magM = fft_mag1[max_idx];
    float magR = fft_mag1[max_idx+1];
    float delta = 0.5f * (magL - magR) / (magL - 2.0f*magM + magR);
    float interp_bin = max_idx + delta;
    
    // 5. 复数插值 - 精确相位提取（改进点）
    float re1 = fft_inputbuf1[2*max_idx];
    float im1 = fft_inputbuf1[2*max_idx+1];
    float re1_next = fft_inputbuf1[2*(max_idx+1)];
    float im1_next = fft_inputbuf1[2*(max_idx+1)+1];
    float re1_interp = re1 + delta*(re1_next - re1);
    float im1_interp = im1 + delta*(im1_next - im1);

    float re2 = fft_inputbuf2[2*max_idx];
    float im2 = fft_inputbuf2[2*max_idx+1];
    float re2_next = fft_inputbuf2[2*(max_idx+1)];
    float im2_next = fft_inputbuf2[2*(max_idx+1)+1];
    float re2_interp = re2 + delta*(re2_next - re2);
    float im2_interp = im2 + delta*(im2_next - im2);
    
    // 6. 计算实际信号频率
    float signal_freq = (interp_bin * sampling_freq) / fft_size;
    
    // 7. 直接相位差计算（关键改进）
    float phase1_rad = atan2f(im1_interp, re1_interp);
    float phase2_rad = atan2f(im2_interp, re2_interp);
    float raw_phase_rad = phase1_rad - phase2_rad;
    
    // 规范化到[-π, π]
    if (raw_phase_rad > M_PI) raw_phase_rad -= 2*M_PI;
    else if (raw_phase_rad < -M_PI) raw_phase_rad += 2*M_PI;
    
    float raw_phase_deg = raw_phase_rad * 180.0f / M_PI;
    
    // 8. 系统延迟补偿
    float systematic_phase = 360.0f * signal_freq * SYSTEM_TAU + SYSTEM_PHI0;
    float compensated_phase = raw_phase_deg - systematic_phase;
    
    // 9. 相位范围规范化
    if (compensated_phase > 180.0f) compensated_phase -= 360.0f;
    else if (compensated_phase < -180.0f) compensated_phase += 360.0f;
    
    // 调试输出（增强版）
    printf("主频: %.2fHz | 通道1相位: %.2f° | 通道2相位: %.2f° | 原始相位差: %.2f°\n", 
           signal_freq, 
           phase1_rad * 180/M_PI, 
           phase2_rad * 180/M_PI,
           raw_phase_deg);
           
    printf("系统补偿: %.2f° | 最终相位: %.2f°\n", 
           systematic_phase, compensated_phase);
    
    return compensated_phase;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
float Calculate_Frequency(uint16_t *adc_data, uint32_t fft_size, float sampling_freq) 
{
    static float fft_inputbuf[FFT_LENGTH * 2];
    static float fft_outputbuf[FFT_LENGTH];
for (int i = 0; i < FFT_LENGTH; i++)
{
    fft_inputbuf[i * 2] = adcBuff1[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
    fft_inputbuf[i * 2 + 1] = 0;//虚部赋值，固定为0.
}
for (int i = 0; i < FFT_LENGTH; i++)
{
    fft_inputbuf[i * 2] = adcBuff2[i] * 3.3 / 4096;//实部赋值，* 3.3 / 4096是为了将ADC采集到的值转换成实际电压
    fft_inputbuf[i * 2 + 1] = 0;//虚部赋值，固定为0.
}
////////////////////////////////////////////////////////////
fft_outputbuf[0] /= 1024;
for (int i = 1; i < FFT_LENGTH; i++)//输出各次谐波幅值
{
    fft_outputbuf[i] /= 512;
}
    // 1. 准备FFT输入数据
    for (uint32_t i = 0; i < fft_size; i++) {
        fft_inputbuf[i * 2] = (float)adc_data[i] * 3.3f / 4096.0f;
        fft_inputbuf[i * 2 + 1] = 0.0f;
    }
    // 2. 执行FFT计算
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);
    
    // 3. 计算幅度谱
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, fft_size);
    
    // 4. 寻找基波频率（最大幅值对应的频率）
    float max_magnitude = 0.0f;
    uint32_t max_idx = 0;
    for (uint32_t i = 1; i < fft_size / 2; i++) {
        if (fft_outputbuf[i] > max_magnitude) {
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
    float frequency = interpolated_idx * (sampling_freq / fft_size);
    
    // 7. 保留小数点后两位
    return roundf(frequency * 100.0f) / 100.0f;
}
/* USER CODE END 4 */