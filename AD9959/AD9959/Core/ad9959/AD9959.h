#ifndef _AD9959_H_
#define _AD9959_H_
#include "stm32f4xx_hal.h"
#include "stdint.h"

// AD9959控制引脚重定义
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

// 寄存器地址定义
#define CSR_ADD       0x00   // 通道选择寄存器
#define FR1_ADD       0x01   // 功能寄存器1
#define FR2_ADD       0x02   // 功能寄存器2
#define CFR_ADD       0x03   // 通道功能寄存器

#define CFTW0_ADD     0x04   // 通道0频率转换字寄存器
#define CPOW0_ADD     0x05   // 通道0相位转换字寄存器
#define ACR_ADD       0x06   // 幅度控制寄存器

#define LSRR_ADD      0x07   // 线性扫描斜率寄存器
#define RDW_ADD       0x08   // 上升步长寄存器
#define FDW_ADD       0x09   // 下降步长寄存器

#define PROFILE_ADDR_BASE 0x0A   // Profile寄存器基地址

// 通道选择
#define CH0 0x10
#define CH1 0x20
#define CH2 0x40
#define CH3 0x80

// 调制电平选择
#define LEVEL_MOD_2    0x00  // 2电平调制
#define LEVEL_MOD_4    0x01  // 4电平调制
#define LEVEL_MOD_8    0x02  // 8电平调制
#define LEVEL_MOD_16   0x03  // 16电平调制

// 调制模式
#define DISABLE_Mod    0x00  // 禁用调制
#define ASK            0x40  // 幅度调制
#define FSK            0x80  // 频率调制
#define PSK            0xC0  // 相位调制

// 扫描使能
#define SWEEP_ENABLE   0x40  // 使能扫描
#define SWEEP_DISABLE  0x00  // 禁用扫描

// 函数声明
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

