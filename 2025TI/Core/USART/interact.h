#ifndef _INTERACT_H_
#define _INTERACT_H_
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
int parse_sensor_frame(const uint8_t* rx2_buf, uint16_t* value) ;
void intArrayToString(int *array, int length, char *str);
int32_t rx2ToIntSafe(void);
#endif

