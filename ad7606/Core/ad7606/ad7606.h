#ifndef __AD7606_H
#define __AD7606_H

#include "stm32f4xx_hal.h"

// �������ţ�D0-D15����ȡ�꣨��ѡ��λ����GPIO��ȡ��ʽ��
uint16_t AD7606_ReadData(void);

// ���ƺ�������
void AD7606_GPIO_Init(void);
void AD7606_Reset(void);
void AD7606_StartConv(void);
uint8_t AD7606_WaitBusy(uint32_t timeout);
void AD7606_SetOS(uint8_t os);
void AD7606_SetRange(uint8_t range);
void AD7606_ReadAllChannels(int16_t *buf);  // ��ȡ8ͨ������

#endif
