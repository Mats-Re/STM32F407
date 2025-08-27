#include "AD7606.h"

// �궨���������
#define AD7606_RST_HIGH()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define AD7606_RST_LOW()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

#define AD7606_CONVST_HIGH()   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET)
#define AD7606_CONVST_LOW()    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET)

#define AD7606_CS_LOW()        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET)
#define AD7606_CS_HIGH()       HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET)

#define AD7606_BUSY_READ()     HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12)

#define AD7606_OS0(x)          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define AD7606_OS1(x)          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define AD7606_OS2(x)          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define AD7606_RANGE_SET(x)    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define AD7606_RD_LOW()      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET)
#define AD7606_RD_HIGH()     HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET)
extern void Error_Handler(void);

// ��ʼ��GPIO
void AD7606_GPIO_Init(void) {
	    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. ����GPIO�˿�ʱ�ӣ�����δ�����貹�䣩
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    // 2. �������ţ�CONVST(PG11)��RESET(PB3)��CS(PG14)��BUSY(PG12)��SCK(PG15)
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    // CONVST
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);

    // RESET
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    // CS/RD
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);

    // SCK (����)
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);

    // 3. BUSY ��������Ϊ����ģʽ
    GPIO_InitStruct.Pin  = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // 4. ���� OS0~OS2��RANGE ����Ϊ���
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6; // OS0~2, RANGE
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Ĭ������
    AD7606_SetOS(0);     // Ĭ�ϲ�������
    AD7606_SetRange(1);  // Ĭ�� ��10V

    // 5. ������������ D0~D15 Ϊ����
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // �����������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    // PF0-PF10 (������)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                         GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                         GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    // PC0-PC3 (������)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // PA1 (������)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 6. Ӳ����λ
    AD7606_Reset();
    // ������������ǰ��CubeMX������Ϊ���ʵ�����/���ģʽ��8λ������Ϊ����
}

// ��λ
void AD7606_Reset(void) {
    AD7606_RST_LOW();
    HAL_Delay(1);
    AD7606_RST_HIGH();
    HAL_Delay(2);
}

// ��������
void AD7606_StartConv(void) {
    AD7606_CONVST_LOW();
    __NOP(); __NOP();  // Լ10ns������tCONVST��Сʱ��
    AD7606_CONVST_HIGH();
}

// �ȴ�BUSY��ͣ���ʱ����
uint8_t AD7606_WaitBusy(uint32_t timeout) {
    uint32_t tickStart = HAL_GetTick();
    while (AD7606_BUSY_READ() == GPIO_PIN_SET) {
        if ((HAL_GetTick() - tickStart) > timeout)
            return 1;  // timeout
    }
    return 0;
}

// ���ù�������
void AD7606_SetOS(uint8_t os) {
    AD7606_OS0(os & 0x01);
    AD7606_OS1((os >> 1) & 0x01);
    AD7606_OS2((os >> 2) & 0x01);
}

// ���������ѹ��Χ��0=��5V, 1=��10V��
void AD7606_SetRange(uint8_t range) {
    AD7606_RANGE_SET(range);
}

// ���ڶ�ȡ16λ����
uint16_t AD7606_ReadData(void) {
    uint16_t value = 0;

    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) << 0;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) << 1;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3) << 2;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2) << 3;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5) << 4;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4) << 5;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7) << 6;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) << 7;

    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9) << 8;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8) << 9;
    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) << 10;
    value |= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10) << 11;
    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) << 12;
    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) << 13;
    value |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) << 14;
    value |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) << 15;

    return value;
}

// ��ȡ����8ͨ����������8�ζ�ȡ��RD�ű��ֵ͵�ƽ��
void AD7606_ReadAllChannels(int16_t *buf) 
{
    AD7606_CS_LOW();
    AD7606_RD_HIGH();  // ��ʼ�ø�
    
    for (int i = 0; i < 8; i++) {
        // RD�½��أ��������������
        AD7606_RD_LOW();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); // 5��NOPԼ25ns
        
        // ��ȡ��ǰͨ������
        buf[i] = (int16_t)AD7606_ReadData();
        
        // RD�����أ��л�����һͨ����
        AD7606_RD_HIGH();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); // 5��NOPԼ25ns
    }
    AD7606_CS_HIGH();
}
