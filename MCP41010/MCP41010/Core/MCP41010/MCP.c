#include "MCP.h"

// ��ʱ����������168MHz��Ƶ��


void MCP410XXInit(void)
{
    // ʹ��GPIOʱ��
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ����CS1��CS2���ţ�PG4, PG2��
    GPIO_InitStruct.Pin = MCP41xx_CS1_PIN | MCP41xx_CS2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // ����CLK���ţ�PD12��- �������
    GPIO_InitStruct.Pin = MCP41xx_CLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // �ؼ��޸�������DAT����Ϊ��©���+������֧��˫��ͨ�ţ�
    GPIO_InitStruct.Pin = MCP41xx_DAT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // ��©���
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // ������������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ��ʼ״̬����
    MCP41xx_SPI_CS1_H();
    MCP41xx_SPI_CS2_H();
    MCP41xx_SPI_CLK_L();
    MCP41xx_SPI_DAT_H();  // ������DAT�߳�ʼ�ߵ�ƽ������״̬��
		HAL_Delay(10);  // ����10ms��Դ�ȶ�ʱ�� 
}

// ��λ��1д���ݣ�ʹ����ȷ�����֣�
void MCP41xx_1writedata(int dat1)
{
    uchar i;
    MCP41xx_SPI_CS2_H();
    MCP41xx_SPI_CS1_H();
    MCP41xx_SPI_CLK_L();
    MCP41xx_SPI_CS1_L();

    HAL_Delay(1);  // ����HAL��ʱ

    // �����������������
    for(i = 0; i < 8; i++) {
        (CMD_WRITE_POT0 & (0x80 >> i)) ? MCP41xx_SPI_DAT_H() : MCP41xx_SPI_DAT_L();
        HAL_Delay(1);
        MCP41xx_SPI_CLK_H();
        HAL_Delay(1);
        MCP41xx_SPI_CLK_L();
        HAL_Delay(1);
    }

    // ��������
    for(i = 0; i < 8; i++) {
        (dat1 & (0x80 >> i)) ? MCP41xx_SPI_DAT_H() : MCP41xx_SPI_DAT_L();
        HAL_Delay(1);
        MCP41xx_SPI_CLK_H();
        HAL_Delay(1);
        MCP41xx_SPI_CLK_L();
        HAL_Delay(1);
    }

    MCP41xx_SPI_CS1_H();
    HAL_Delay(1);
}

// ��λ��2д����
void MCP41xx_2writedata(int dat2)
{
    uchar i;
    MCP41xx_SPI_CS1_H();  // ȷ��CS1��Ч
    MCP41xx_SPI_CS2_H();  // ������CS2
    MCP41xx_SPI_CLK_L();  // ʱ�ӳ�ʼ�͵�ƽ
    
    MCP41xx_SPI_CS2_L();  // ѡ�е�λ��2
    HAL_Delay(1);         // ����ʱ��

    // �����������������(0x32)
    for(i = 0; i < 8; i++) {
        // �����λ(MSB)��ʼ����
        (CMD_WRITE_POT1 & (0x80 >> i)) ? MCP41xx_SPI_DAT_H() : MCP41xx_SPI_DAT_L();
        HAL_Delay(1);
        
        // ����ʱ��������
        MCP41xx_SPI_CLK_H();
        HAL_Delay(1);
        
        // ����ʱ���½���
        MCP41xx_SPI_CLK_L();
        HAL_Delay(1);
    }

    // ��������
    for(i = 0; i < 8; i++) {
        // �����λ(MSB)��ʼ����
        (dat2 & (0x80 >> i)) ? MCP41xx_SPI_DAT_H() : MCP41xx_SPI_DAT_L();
        HAL_Delay(1);
        
        MCP41xx_SPI_CLK_H();
        HAL_Delay(1);
        
        MCP41xx_SPI_CLK_L();
        HAL_Delay(1);
    }

    // ��������
    MCP41xx_SPI_CS2_H();  // ȡ��ѡ��
    HAL_Delay(1);         // ����ʱ��
}
