#include "stdio.h"
#include "ctype.h"
#include "string.h"
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "stdlib.h"
/*********************/
#define RX_BUF_SIZE 64
uint16_t extract_protocol_value(const uint8_t *frame);
//////////////////////////////////////////////////////////////////////////
extern uint8_t rx1_buf[RX_BUF_SIZE];
extern uint8_t rx1_tmp;
extern uint8_t rx1_index;

extern uint8_t rx2_buf[RX_BUF_SIZE];
extern uint8_t rx2_tmp;
extern uint8_t rx2_index;
/* 
 * UART接收完成中断回调函数
 * 当UART接收到一个字节并通过DMA或中断方式传输完成后，HAL库会调用此函数
 * 参数：huart - 指向触发中断的UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)// 检查触发中断的是否为USART1
    {
        if (rx1_tmp == '\r' || rx1_tmp == '\n')/* 检测到行结束符（回车或换行） */
        {
					 if (rx1_index > 0)// 确保缓冲区中有有效数据（避免空包）
					 {
            rx1_buf[rx1_index] = '\0';// 添加字符串终止符，形成完整字符串
            const char *msg = "USART1 RECV: ";// 通过USART1发送接收提示头
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);// 回传接收到的有效数据
            HAL_UART_Transmit(&huart1, rx1_buf, rx1_index, HAL_MAX_DELAY);// 添加换行符使输出更易读
            HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);// 重置缓冲区索引，准备接收新数据包
						memset(rx1_buf,'\0',sizeof(rx1_buf));
            rx1_index = 0;
					 }
        }
        else/* 非结束符的常规数据 */
        {
            if (rx1_index < RX_BUF_SIZE - 1)// 防止缓冲区溢出（保留1字节空间给终止符）
                rx1_buf[rx1_index++] = rx1_tmp;// 存入接收缓冲区并更新索引
        }

        HAL_UART_Receive_IT(&huart1, &rx1_tmp, 1);  // 重新启用中断接收，继续监听下一个字节
    }
    else if (huart->Instance == USART2)// 检查触发中断的是否为USART2
    {
        if (rx2_tmp == '\r' || rx2_tmp == '\n')/* 处理逻辑与USART1相同，仅变量名不同 */
        {
					if (rx2_index > 0)
					{
			
            rx2_buf[rx2_index] = '\0';
            const char *msg = "USART2 RECV: ";// 注意：此处仍使用huart1作为输出通道			
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart1, rx2_buf,sizeof(rx2_buf), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
						memset(rx2_buf,'\0',sizeof(rx2_buf));
            rx2_index = 0;
					}
        }
        else
        {
            if (rx2_index < RX_BUF_SIZE - 1)
                rx2_buf[rx2_index++] = rx2_tmp;
        }
        HAL_UART_Receive_IT(&huart2, &rx2_tmp, 1); // 重新启用USART2的中断接收
    }
		
}

