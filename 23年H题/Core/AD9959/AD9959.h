#ifndef _AD9959_H_
#define _AD9959_H_
#include "stm32f4xx_hal.h"
#include "stdint.h"

// AD9959���������ض���
#define CS_Port       GPIOF
#define CS_Pin        GPIO_PIN_5

#define SCLK_Port     GPIOE
#define SCLK_Pin      GPIO_PIN_1

#define UPDATE_Port   GPIOE
#define UPDATE_Pin    GPIO_PIN_2

#define PS0_Port      GPIOE
#define PS0_Pin       GPIO_PIN_3

#define PS1_Port      GPIOE
#define PS1_Pin       GPIO_PIN_4

#define PS2_Port      GPIOE
#define PS2_Pin       GPIO_PIN_6

#define PS3_Port      GPIOE
#define PS3_Pin       GPIO_PIN_0

#define SDIO0_Port    GPIOC
#define SDIO0_Pin     GPIO_PIN_13

#define SDIO1_Port    GPIOF
#define SDIO1_Pin     GPIO_PIN_0

#define SDIO2_Port    GPIOF
#define SDIO2_Pin     GPIO_PIN_1

#define SDIO3_Port    GPIOF
#define SDIO3_Pin     GPIO_PIN_2

#define PWR_Port      GPIOF
#define PWR_Pin       GPIO_PIN_3

#define RST_Port      GPIOF
#define RST_Pin       GPIO_PIN_4

// �Ĵ�����ַ����
#define CSR_ADD       0x00   // ͨ��ѡ��Ĵ���
#define FR1_ADD       0x01   // ���ܼĴ���1
#define FR2_ADD       0x02   // ���ܼĴ���2
#define CFR_ADD       0x03   // ͨ�����ܼĴ���

#define CFTW0_ADD     0x04   // ͨ��0Ƶ��ת���ּĴ���
#define CPOW0_ADD     0x05   // ͨ��0��λת���ּĴ���
#define ACR_ADD       0x06   // ���ȿ��ƼĴ���

#define LSRR_ADD      0x07   // ����ɨ��б�ʼĴ���
#define RDW_ADD       0x08   // ���������Ĵ���
#define FDW_ADD       0x09   // �½������Ĵ���

#define PROFILE_ADDR_BASE 0x0A   // Profile�Ĵ�������ַ

// ͨ��ѡ��
#define CH0 0x10
#define CH1 0x20
#define CH2 0x40
#define CH3 0x80

// ���Ƶ�ƽѡ��
#define LEVEL_MOD_2    0x00  // 2��ƽ����
#define LEVEL_MOD_4    0x01  // 4��ƽ����
#define LEVEL_MOD_8    0x02  // 8��ƽ����
#define LEVEL_MOD_16   0x03  // 16��ƽ����

// ����ģʽ
#define DISABLE_Mod    0x00  // ���õ���
#define ASK            0x40  // ���ȵ���
#define FSK            0x80  // Ƶ�ʵ���
#define PSK            0xC0  // ��λ����

// ɨ��ʹ��
#define SWEEP_ENABLE   0x40  // ʹ��ɨ��
#define SWEEP_DISABLE  0x00  // ����ɨ��

// ��������
void delay_us(uint32_t us);
void AD9959_Reset(void);
void IO_Update(void);
void AD9959_Init(void);
void AD9959_WriteData(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData);
void Write_CFTW0(uint32_t fre);
void Write_ACR(uint16_t Ampli);
void Write_CPOW0(uint16_t Phase);
void Write_LSRR(uint8_t rsrr, uint8_t fsrr);
void Write_RDW(uint32_t r_delta);
void Write_FDW(uint32_t f_delta);
void Write_Profile_Fre(uint8_t profile, uint32_t data);
void Write_Profile_Ampli(uint8_t profile, uint16_t data);
void Write_Profile_Phase(uint8_t profile, uint16_t data);
void AD9959_Set_Fre(uint8_t Channel, uint32_t Freq);
void AD9959_Set_Amp(uint8_t Channel, uint16_t Ampli);
void AD9959_Set_Phase(uint8_t Channel, uint16_t Phase);
void AD9959_Modulation_Init(uint8_t Channel, uint8_t Modulation, uint8_t Sweep_en, uint8_t Nlevel);
void AD9959_SetFSK(uint8_t Channel, uint32_t *data, uint16_t Phase);
void AD9959_SetASK(uint8_t Channel, uint16_t *data, uint32_t fre, uint16_t Phase);
void AD9959_SetPSK(uint8_t Channel, uint16_t *data, uint32_t Freq);
void AD9959_SetFre_Sweep(uint8_t Channel, uint32_t s_data, uint32_t e_data, uint32_t r_delta, uint32_t f_delta, uint8_t rsrr, uint8_t fsrr, uint16_t Ampli, uint16_t Phase);
void AD9959_SetAmp_Sweep(uint8_t Channel, uint32_t s_Ampli, uint16_t e_Ampli, uint32_t r_delta, uint32_t f_delta, uint8_t rsrr, uint8_t fsrr, uint32_t fre, uint16_t Phase);
void AD9959_SetPhase_Sweep(uint8_t Channel, uint16_t s_data, uint16_t e_data, uint16_t r_delta, uint16_t f_delta, uint8_t rsrr, uint8_t fsrr, uint32_t fre, uint16_t Ampli);

#endif

