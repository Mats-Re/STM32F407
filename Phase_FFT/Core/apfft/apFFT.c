#include "apFFT.h"
#include "main.h"
#include "arm_math.h"
#include "stdio.h"
#define FFT_LENGTH 1024
#define SYSTEM_TAU      (42.17e-6f)    // ʱ���ӳ� (42.17 ��s)
#define SYSTEM_PHI0     (-404.99f)   
uint16_t adcBuff1[FFT_LENGTH];
uint16_t adcBuff2[FFT_LENGTH];
float fft_inputbuf[FFT_LENGTH*2];  
float fft_outputbuf[FFT_LENGTH];  
/* USER CODE BEGIN 4 */
/**
  * @brief  ���������źŵ���λ��(��)
  * @param  adc_data1: ��һ���źŵ�ADC����
  * @param  adc_data2: �ڶ����źŵ�ADC����
  * @param  fft_size: FFT��С
  * @param  sampling_freq: ����Ƶ��
  * @retval �����ź��ڻ���Ƶ�ʴ�����λ��(��)
  */


float Calculate_PhaseDifference(uint16_t *adc_data1, uint16_t *adc_data2, 
                               uint32_t fft_size, float sampling_freq) 
{
    static float fft_inputbuf1[FFT_LENGTH * 2];
    static float fft_inputbuf2[FFT_LENGTH * 2];
    static float fft_mag1[FFT_LENGTH];

    // 1. Ԥ����ȥֱ�� & ����
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

    // 2. FFT����
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf1, 0, 1);
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf2, 0, 1);
    
    // 3. ������ & Ѱ����Ƶ
    arm_cmplx_mag_f32(fft_inputbuf1, fft_mag1, fft_size);
    
    uint32_t max_idx = 1;
    float max_mag = fft_mag1[1];
    for (uint32_t i = 2; i < fft_size/2 - 1; i++) {
        if (fft_mag1[i] > max_mag) {
            max_mag = fft_mag1[i];
            max_idx = i;
        }
    }
    
    // 4. �����߲�ֵ - ��ȷƵ�ʶ�λ
    float magL = fft_mag1[max_idx-1];
    float magM = fft_mag1[max_idx];
    float magR = fft_mag1[max_idx+1];
    float delta = 0.5f * (magL - magR) / (magL - 2.0f*magM + magR);
    float interp_bin = max_idx + delta;
    
    // 5. ������ֵ - ��ȷ��λ��ȡ���Ľ��㣩
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
    
    // 6. ����ʵ���ź�Ƶ��
    float signal_freq = (interp_bin * sampling_freq) / fft_size;
    
    // 7. ֱ����λ����㣨�ؼ��Ľ���
    float phase1_rad = atan2f(im1_interp, re1_interp);
    float phase2_rad = atan2f(im2_interp, re2_interp);
    float raw_phase_rad = phase1_rad - phase2_rad;
    
    // �淶����[-��, ��]
    if (raw_phase_rad > M_PI) raw_phase_rad -= 2*M_PI;
    else if (raw_phase_rad < -M_PI) raw_phase_rad += 2*M_PI;
    
    float raw_phase_deg = raw_phase_rad * 180.0f / M_PI;
    
    // 8. ϵͳ�ӳٲ���
    float systematic_phase = 360.0f * signal_freq * SYSTEM_TAU + SYSTEM_PHI0;
    float compensated_phase = raw_phase_deg - systematic_phase;
    
    // 9. ��λ��Χ�淶��
    if (compensated_phase > 180.0f) compensated_phase -= 360.0f;
    else if (compensated_phase < -180.0f) compensated_phase += 360.0f;
    
    // �����������ǿ�棩
    printf("��Ƶ: %.2fHz | ͨ��1��λ: %.2f�� | ͨ��2��λ: %.2f�� | ԭʼ��λ��: %.2f��\n", 
           signal_freq, 
           phase1_rad * 180/M_PI, 
           phase2_rad * 180/M_PI,
           raw_phase_deg);
           
    printf("ϵͳ����: %.2f�� | ������λ: %.2f��\n", 
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
    fft_inputbuf[i * 2] = adcBuff1[i] * 3.3 / 4096;//ʵ����ֵ��* 3.3 / 4096��Ϊ�˽�ADC�ɼ�����ֵת����ʵ�ʵ�ѹ
    fft_inputbuf[i * 2 + 1] = 0;//�鲿��ֵ���̶�Ϊ0.
}
for (int i = 0; i < FFT_LENGTH; i++)
{
    fft_inputbuf[i * 2] = adcBuff2[i] * 3.3 / 4096;//ʵ����ֵ��* 3.3 / 4096��Ϊ�˽�ADC�ɼ�����ֵת����ʵ�ʵ�ѹ
    fft_inputbuf[i * 2 + 1] = 0;//�鲿��ֵ���̶�Ϊ0.
}
////////////////////////////////////////////////////////////
fft_outputbuf[0] /= 1024;
for (int i = 1; i < FFT_LENGTH; i++)//�������г����ֵ
{
    fft_outputbuf[i] /= 512;
}
    // 1. ׼��FFT��������
    for (uint32_t i = 0; i < fft_size; i++) {
        fft_inputbuf[i * 2] = (float)adc_data[i] * 3.3f / 4096.0f;
        fft_inputbuf[i * 2 + 1] = 0.0f;
    }
    // 2. ִ��FFT����
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, fft_inputbuf, 0, 1);
    
    // 3. ���������
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, fft_size);
    
    // 4. Ѱ�һ���Ƶ�ʣ�����ֵ��Ӧ��Ƶ�ʣ�
    float max_magnitude = 0.0f;
    uint32_t max_idx = 0;
    for (uint32_t i = 1; i < fft_size / 2; i++) {
        if (fft_outputbuf[i] > max_magnitude) {
            max_magnitude = fft_outputbuf[i];
            max_idx = i;
        }
    }
    
    // 5. ʹ�������ֵ�����Ƶ�ʾ���
    float y1 = fft_outputbuf[max_idx - 1];
    float y2 = fft_outputbuf[max_idx];
    float y3 = fft_outputbuf[max_idx + 1];
    
    // �����߲�ֵ��ʽ
    float delta = 0.5f * (y3 - y1) / (2.0f * y2 - y1 - y3);
    float interpolated_idx = (float)max_idx + delta;
    
    // 6. ����ʵ��Ƶ��
    float frequency = interpolated_idx * (sampling_freq / fft_size);
    
    // 7. ����С�������λ
    return roundf(frequency * 100.0f) / 100.0f;
}
/* USER CODE END 4 */