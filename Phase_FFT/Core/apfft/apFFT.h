#ifndef _APFFT_H_
#define _APFFT_H_
#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"
float Calculate_PhaseDifference(uint16_t *adc_data1, uint16_t *adc_data2, uint32_t fft_size, float sampling_freq);
float Calculate_Frequency(uint16_t *adc_data, uint32_t fft_size, float sampling_freq);

#define M_PI 3.1415926
#endif