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
 * UART��������жϻص�����
 * ��UART���յ�һ���ֽڲ�ͨ��DMA���жϷ�ʽ������ɺ�HAL�����ô˺���
 * ������huart - ָ�򴥷��жϵ�UART���
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)// ��鴥���жϵ��Ƿ�ΪUSART1
    {
        if (rx1_tmp == '\r' || rx1_tmp == '\n')/* ��⵽�н��������س����У� */
        {
					 if (rx1_index > 0)// ȷ��������������Ч���ݣ�����հ���
					 {
            rx1_buf[rx1_index] = '\0';// ����ַ�����ֹ�����γ������ַ���
            const char *msg = "USART1 RECV: ";// ͨ��USART1���ͽ�����ʾͷ
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);// �ش����յ�����Ч����
            HAL_UART_Transmit(&huart1, rx1_buf, rx1_index, HAL_MAX_DELAY);// ��ӻ��з�ʹ������׶�
            HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);// ���û�����������׼�����������ݰ�
						memset(rx1_buf,'\0',sizeof(rx1_buf));
            rx1_index = 0;
					 }
        }
        else/* �ǽ������ĳ������� */
        {
            if (rx1_index < RX_BUF_SIZE - 1)// ��ֹ���������������1�ֽڿռ����ֹ����
                rx1_buf[rx1_index++] = rx1_tmp;// ������ջ���������������
        }

        HAL_UART_Receive_IT(&huart1, &rx1_tmp, 1);  // ���������жϽ��գ�����������һ���ֽ�
    }
    else if (huart->Instance == USART2)// ��鴥���жϵ��Ƿ�ΪUSART2
    {
        if (rx2_tmp == '\r' || rx2_tmp == '\n')/* �����߼���USART1��ͬ������������ͬ */
        {
					if (rx2_index > 0)
					{
			
            rx2_buf[rx2_index] = '\0';
            const char *msg = "USART2 RECV: ";// ע�⣺�˴���ʹ��huart1��Ϊ���ͨ��			
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
        HAL_UART_Receive_IT(&huart2, &rx2_tmp, 1); // ��������USART2���жϽ���
    }
		
}

