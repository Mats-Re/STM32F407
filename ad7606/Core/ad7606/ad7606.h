#ifndef __AD7606_H
#define __AD7606_H

#include "stm32f4xx_hal.h"

// 数据引脚（D0-D15）读取宏（可选择位带或GPIO读取方式）
uint16_t AD7606_ReadData(void);

// 控制函数声明
void AD7606_GPIO_Init(void);
void AD7606_Reset(void);
void AD7606_StartConv(void);
uint8_t AD7606_WaitBusy(uint32_t timeout);
void AD7606_SetOS(uint8_t os);
void AD7606_SetRange(uint8_t range);
void AD7606_ReadAllChannels(int16_t *buf);  // 读取8通道数据

#endif
