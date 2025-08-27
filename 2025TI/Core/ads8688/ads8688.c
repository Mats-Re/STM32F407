#include "ads8688.h"
#include "stm32f4xx_hal.h"

// �Զ���΢����ʱ����
void Delay_us(uint16_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t delay_ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < delay_ticks);
}

void ADS8688_IO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // ʹ��GPIOʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    // ����PA15 (DAISY) Ϊ���
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // ����PC11 (SD0) Ϊ����
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // ����PD2 (CLK), PD3 (CS), PD4 (SDI), PD5 (RST) Ϊ���
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // ��ʼ������״̬
    DAISY_L();
    CS_H();
    RST_H();
    CLK_L();
    
    // ����DWT���������ھ�ȷ��ʱ
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}



/**********************************************************************************************************
*	�� �� ��: ADS8688_Init
*	����˵��: ADS8688��ʼ��
*	��    ��: ch_en��Ҫʹ�ܵ�ͨ��
*	�� �� ֵ: ��
**********************************************************************************************************/
void ADS8688_Init(uint8_t ch_en)	   //ADS8688��ʼ��
{
	ADS8688_IO_Init();
	SOFT_RESET(); //��λ
	HAL_Delay(2);
  Set_Auto_Scan_Sequence(ch_en);//ʹ��ͨ��
	ADS8688_WriteReg(CH_PD,~ch_en);//ͨ���˳��͹���״̬ ͨ���ϵ�
	HAL_Delay(2);
	while(ADS8688_ReadReg(AUTO_SEQ_EN)!=ch_en)//�Ƿ�ʹ�ܳɹ�
	{
		Set_Auto_Scan_Sequence(ch_en);//ʹ��ͨ��
		ADS8688_WriteReg(CH_PD,~ch_en);//ͨ���˳��͹���״̬ ͨ���ϵ�
		HAL_Delay(2);
	}		
}

/**********************************************************************************************************
*	�� �� ��: ADS8688_SPI_Read8Bit
*	����˵��: ��SPI���߶�8��bit���ݡ� ����CS���ơ�
*	��    ��: ��
*	�� �� ֵ: ��
**********************************************************************************************************/
uint8_t ADS8688_SPI_Read8Bit(void)
{
	uint8_t i=0;
  uint8_t read=0;
  for(i=0;i<8;i++)
  {
    read<<=1;
    CLK_H();
		Delay_us(2);
    if(READ_SDO()) read++;
    CLK_L();
		Delay_us(2);
  }
  return read;
}
/**********************************************************************************************************
*	�� �� ��: ADS8688_SPI_Write8Bit
*	����˵��: ��SPI����д8��bit���ݡ� ����CS���ơ�
*	��    ��: data : ����
*	�� �� ֵ: ��
**********************************************************************************************************/
void ADS8688_SPI_Write8Bit(uint8_t data)
{
	uint8_t i=0;
  CS_L();
  for(i=0;i<8;i++)
  {
    if(data&0x80)	SDI_H();
    else	SDI_L();
		data<<=1;
    CLK_H();
		Delay_us(200);
    CLK_L();
		Delay_us(200);
  }
}
/**********************************************************************************************************
*	�� �� ��: ADS8688_WriteCmd
*	����˵��: д����Ĵ���
*	��    ��:  cmd : ����
*	�� �� ֵ: ��
**********************************************************************************************************/
void ADS8688_WriteCmd(uint16_t cmd)//дADS8688����Ĵ���
{
  CS_L();
  ADS8688_SPI_Write8Bit(cmd>>8 & 0XFF);
  ADS8688_SPI_Write8Bit(cmd & 0XFF);
	ADS8688_SPI_Write8Bit(0X00);
	ADS8688_SPI_Write8Bit(0X00);
  CS_H();
}

/*
*********************************************************************************************************
*	�� �� ��: SOFT_RESET
*	����˵��: �����λ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void SOFT_RESET(void)//�����λ
{
  ADS8688_WriteCmd(RST);
}

/**********************************************************************************************************
*	�� �� ��: AUTO_RST_Mode
*	����˵��: �Զ�ģʽ
*	��    ��: ��
*	�� �� ֵ: ��
**********************************************************************************************************/
void AUTO_RST_Mode(void)//�Զ�ɨ��ģʽ
{
  ADS8688_WriteCmd(AUTO_RST);
}
/**********************************************************************************************************
*	�� �� ��: MAN_CH_Mode
*	����˵��: �ֶ�����ɨ��ģʽͨ��ѡ��
*	��    ��:  ch : Ҫѡ���ͨ��
*	�� �� ֵ: ��
**********************************************************************************************************/
void MAN_CH_Mode(uint16_t ch)//�ֶ�����ɨ��ģʽ
{
  ADS8688_WriteCmd(ch);	
}

/**********************************************************************************************************
*	�� �� ��: ADS8688_ReadReg
*	����˵��: ��ָ���ļĴ���
*	��    ��:  addr : �Ĵ�����ַ
*	�� �� ֵ: data : �����ļĴ���ֵ��
**********************************************************************************************************/
uint8_t ADS8688_ReadReg(uint8_t addr)
{
  uint8_t data = 0;
  CS_L();
  ADS8688_SPI_Write8Bit(addr<<1);
  data = ADS8688_SPI_Read8Bit();
  data = ADS8688_SPI_Read8Bit();
  CS_H();
  return data;
}
/**********************************************************************************************************
*	�� �� ��: ADS8688_WriteReg
*	����˵��: дָ���ļĴ���
*	��    ��:  addr : �Ĵ�����ַ
*			  		data : �Ĵ���ֵ
*	�� �� ֵ: ��
**********************************************************************************************************/
void ADS8688_WriteReg(uint8_t addr,uint8_t data)
{
  CS_L();
  ADS8688_SPI_Write8Bit(addr<<1| 0X01);
  ADS8688_SPI_Write8Bit(data);
  CS_H();
}
/**********************************************************************************************************
*	�� �� ��: Set_Auto_Scan_Sequence
*	����˵��: �����Զ�ɨ������ͨ��
*	��    ��:  seq����Ҫ�����Զ�ɨ������ʹ�ܼĴ�����ֵ
*	�� �� ֵ: ��
**********************************************************************************************************/
void Set_Auto_Scan_Sequence(uint8_t seq)	
{
	ADS8688_WriteReg(AUTO_SEQ_EN, seq);
}
/**********************************************************************************************************
*	�� �� ��: Set_CH_Range
*	����˵��: ����ָ��ͨ��������Χ
*	��    ��:  ch : ͨ��
*			  		range : ������Χ
*	�� �� ֵ: ��
**********************************************************************************************************/
void Set_CH_Range(uint8_t ch,uint8_t range)
{
  ADS8688_WriteReg(ch,range);
}

/**********************************************************************************************************
*	�� �� ��: Get_AUTO_RST_Mode_Data
*	����˵��: ��ȡ�Զ�ɨ��ģʽADֵ
*	��    ��:data : ����ָ��
*			  	chnum : ͨ������
*	�� �� ֵ: ��
**********************************************************************************************************/
void Get_AUTO_RST_Mode_Data(uint16_t *data, uint8_t chnum)
{
  uint8_t i=0,datal=0,datah=0;
  for (i=0; i<chnum; i++)
  {
    CS_L();
    ADS8688_SPI_Write8Bit(0X00);
    ADS8688_SPI_Write8Bit(0X00);
    datah = ADS8688_SPI_Read8Bit();
    datal = ADS8688_SPI_Read8Bit();
		//delay_ms(1);
    CS_H();
    *(data+i) = datah<<8 | datal;
  }
}
/**********************************************************************************************************
*	�� �� ��: Get_MAN_CH_Mode_Data
*	����˵��: ��ȡ����ɨ��ģʽADֵ
*	��    ��: ��
*	�� �� ֵ: ��
**********************************************************************************************************/
uint16_t Get_MAN_CH_Mode_Data(void)
{
  uint8_t datah=0,datal=0;
  CS_L();
  ADS8688_SPI_Write8Bit(0X00);
  ADS8688_SPI_Write8Bit(0X00);
  datah = ADS8688_SPI_Read8Bit();
  datal = ADS8688_SPI_Read8Bit();
  CS_H();
  return (datah<<8 | datal);
}