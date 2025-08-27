#ifndef _hmi_H_
#define _hmi_H_
#include "stm32f4xx_hal.h"
#include "stdint.h"

//º¯ÊıÉùÃ÷
void HMI_UpdateTwoFrequencies(float freq1, float freq2);
void HMI_SendWaveData(float freq);

void sendWaveData(float freq);












#endif



