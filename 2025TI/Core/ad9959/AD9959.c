#include "AD9959.h"
#include "main.h"

// 全局变量
double ACC_FRE_FACTOR = 8.589934592;  // 频率因子 = (2^32)/500MHz
uint8_t FR1_DATA[3] = {0xD0, 0x00, 0x00};  // VCO增益控制，PLL 20倍频
uint8_t FR2_DATA[2] = {0x00, 0x00};         // 双向扫描模式
uint8_t CFR_DATA[3] = {0x00, 0x03, 0x02};   // 默认通道功能寄存器

// 微秒延时函数
void delay_us(uint32_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000) / 5;
    while(ticks--);
}

// 初始化IO口状态
void Init_IO_State(void) {
    HAL_GPIO_WritePin(PWR_Port, PWR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(UPDATE_Port, UPDATE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS0_Port, PS0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS1_Port, PS1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS2_Port, PS2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PS3_Port, PS3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SDIO0_Port, SDIO0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SDIO1_Port, SDIO1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SDIO2_Port, SDIO2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SDIO3_Port, SDIO3_Pin, GPIO_PIN_RESET);
}

// AD9959复位
void AD9959_Reset(void) {
    HAL_GPIO_WritePin(RST_Port, RST_Pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(RST_Port, RST_Pin, GPIO_PIN_SET);
    delay_us(30);
    HAL_GPIO_WritePin(RST_Port, RST_Pin, GPIO_PIN_RESET);
}

// 数据更新
void IO_Update(void) {
    HAL_GPIO_WritePin(UPDATE_Port, UPDATE_Pin, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(UPDATE_Port, UPDATE_Pin, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(UPDATE_Port, UPDATE_Pin, GPIO_PIN_RESET);
}

// AD9959初始化
void AD9959_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // 配置所有控制引脚为输出
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    // PE引脚配置
    GPIO_InitStruct.Pin = SCLK_Pin | UPDATE_Pin | PS0_Pin | PS1_Pin | PS2_Pin | PS3_Pin;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    // PC引脚配置
    GPIO_InitStruct.Pin = SDIO0_Pin ;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // PF引脚配置
    GPIO_InitStruct.Pin = CS_Pin | SDIO1_Pin | SDIO2_Pin| SDIO3_Pin| PWR_Pin|RST_Pin;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    // 初始化IO状态
    Init_IO_State();
    
    // AD9959复位
    AD9959_Reset();
    
    // 初始化功能寄存器
    AD9959_WriteData(FR1_ADD, 3, FR1_DATA);  // 写功能寄存器1
    AD9959_WriteData(FR2_ADD, 2, FR2_DATA);  // 写功能寄存器2
}

// 向AD9959写数据
void AD9959_WriteData(uint8_t RegisterAddress, uint8_t NumberofRegisters, uint8_t *RegisterData) {
    uint8_t ControlValue = RegisterAddress;
    uint8_t ValueToWrite = 0;
    uint8_t RegisterIndex = 0;
    uint8_t i = 0;
    
    // 写入地址
    HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
    
    for(i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_RESET);
        if(0x80 == (ControlValue & 0x80)) {
            HAL_GPIO_WritePin(SDIO0_Port, SDIO0_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(SDIO0_Port, SDIO0_Pin, GPIO_PIN_RESET);
        }
        HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_SET);
        ControlValue <<= 1;
        delay_us(1);
    }
    HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_RESET);
    
    // 写入数据
    for(RegisterIndex = 0; RegisterIndex < NumberofRegisters; RegisterIndex++) {
        ValueToWrite = RegisterData[RegisterIndex];
        for(i = 0; i < 8; i++) {
            HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_RESET);
            if(0x80 == (ValueToWrite & 0x80)) {
                HAL_GPIO_WritePin(SDIO0_Port, SDIO0_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(SDIO0_Port, SDIO0_Pin, GPIO_PIN_RESET);
            }
            HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_SET);
            ValueToWrite <<= 1;
            delay_us(1);
        }
        HAL_GPIO_WritePin(SCLK_Port, SCLK_Pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
}

// 以下函数实现保持不变（仅修改了GPIO操作）
void Write_CFTW0(uint32_t fre) {
    uint8_t CFTW0_DATA[4] = {0x00, 0x00, 0x00, 0x00};
    uint32_t Temp = (uint32_t)(fre * ACC_FRE_FACTOR);
    CFTW0_DATA[3] = (uint8_t)Temp;
    CFTW0_DATA[2] = (uint8_t)(Temp >> 8);
    CFTW0_DATA[1] = (uint8_t)(Temp >> 16);
    CFTW0_DATA[0] = (uint8_t)(Temp >> 24);
    AD9959_WriteData(CFTW0_ADD, 4, CFTW0_DATA);
}

void Write_ACR(uint16_t Ampli) {
    uint8_t ACR_DATA[3] = {0x00, 0x00, 0x00};
    uint32_t A_temp = Ampli | 0x1000;
    ACR_DATA[1] = (uint8_t)(A_temp >> 8);
    ACR_DATA[2] = (uint8_t)A_temp;
    AD9959_WriteData(ACR_ADD, 3, ACR_DATA);
}

void Write_CPOW0(uint16_t Phase) {
    uint8_t CPOW0_data[2] = {0x00, 0x00};
    CPOW0_data[1] = (uint8_t)Phase;
    CPOW0_data[0] = (uint8_t)(Phase >> 8);
    AD9959_WriteData(CPOW0_ADD, 2, CPOW0_data);
}

void Write_LSRR(uint8_t rsrr, uint8_t fsrr) {
    uint8_t LSRR_data[2] = {fsrr, rsrr}; // [0]=fsrr, [1]=rsrr
    AD9959_WriteData(LSRR_ADD, 2, LSRR_data);
}

void Write_RDW(uint32_t r_delta) {
    uint8_t RDW_data[4] = {0x00, 0x00, 0x00, 0x00};
    RDW_data[3] = (uint8_t)r_delta;
    RDW_data[2] = (uint8_t)(r_delta >> 8);
    RDW_data[1] = (uint8_t)(r_delta >> 16);
    RDW_data[0] = (uint8_t)(r_delta >> 24);
    AD9959_WriteData(RDW_ADD, 4, RDW_data);
}

void Write_FDW(uint32_t f_delta) {
    uint8_t FDW_data[4] = {0x00, 0x00, 0x00, 0x00};
    FDW_data[3] = (uint8_t)f_delta;
    FDW_data[2] = (uint8_t)(f_delta >> 8);
    FDW_data[1] = (uint8_t)(f_delta >> 16);
    FDW_data[0] = (uint8_t)(f_delta >> 24);
    AD9959_WriteData(FDW_ADD, 4, FDW_data);
}

void Write_Profile_Fre(uint8_t profile, uint32_t data) {
    uint8_t profileAddr = PROFILE_ADDR_BASE + profile;
    uint8_t Profile_data[4] = {0x00, 0x00, 0x00, 0x00};
    uint32_t Temp = (uint32_t)(data * ACC_FRE_FACTOR);
    Profile_data[3] = (uint8_t)Temp;
    Profile_data[2] = (uint8_t)(Temp >> 8);
    Profile_data[1] = (uint8_t)(Temp >> 16);
    Profile_data[0] = (uint8_t)(Temp >> 24);
    AD9959_WriteData(profileAddr, 4, Profile_data);
}

void Write_Profile_Ampli(uint8_t profile, uint16_t data) {
    uint8_t profileAddr = PROFILE_ADDR_BASE + profile;
    uint8_t Profile_data[4] = {0x00, 0x00, 0x00, 0x00};
    Profile_data[1] = (uint8_t)(data << 6);
    Profile_data[0] = (uint8_t)(data >> 2);
    AD9959_WriteData(profileAddr, 4, Profile_data);
}

void Write_Profile_Phase(uint8_t profile, uint16_t data) {
    uint8_t profileAddr = PROFILE_ADDR_BASE + profile;
    uint8_t Profile_data[4] = {0x00, 0x00, 0x00, 0x00};
    Profile_data[1] = (uint8_t)(data << 2);
    Profile_data[0] = (uint8_t)(data >> 6);
    AD9959_WriteData(profileAddr, 4, Profile_data);
}

void AD9959_Set_Fre(uint8_t Channel, uint32_t Freq) {
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CFTW0(Freq);
}

void AD9959_Set_Amp(uint8_t Channel, uint16_t Ampli) {
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_ACR(Ampli);
}

void AD9959_Set_Phase(uint8_t Channel, uint16_t Phase) {
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CPOW0(Phase);
}

// 以下调制和扫描函数实现保持不变（逻辑不变）
void AD9959_Modulation_Init(uint8_t Channel, uint8_t Modulation, uint8_t Sweep_en, uint8_t Nlevel) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {Channel};
    uint8_t FR1_data[3];
    uint8_t FR2_data[2];
    uint8_t CFR_data[3];
    
    for(i = 0; i < 3; i++) {
        FR1_data[i] = FR1_DATA[i];
        CFR_data[i] = CFR_DATA[i];
    }
    FR2_data[0] = FR2_DATA[0];
    FR2_data[1] = FR2_DATA[1];
    
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    
    FR1_data[1] = Nlevel;
    CFR_data[0] = Modulation;
    CFR_data[1] |= Sweep_en;
    CFR_data[2] = 0x00;
    
    if(Channel != 0) {
        AD9959_WriteData(FR1_ADD, 3, FR1_data);
        AD9959_WriteData(FR2_ADD, 2, FR2_data);
        AD9959_WriteData(CFR_ADD, 3, CFR_data);
    }
}

void AD9959_SetFSK(uint8_t Channel, uint32_t *data, uint16_t Phase) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CPOW0(Phase);
    Write_CFTW0(data[0]);
    for(i = 0; i < 15; i++)
        Write_Profile_Fre(i, data[i+1]);
}

void AD9959_SetASK(uint8_t Channel, uint16_t *data, uint32_t fre, uint16_t Phase) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CFTW0(fre);
    Write_CPOW0(Phase);
    Write_ACR(data[0]);
    for(i = 0; i < 15; i++)
        Write_Profile_Ampli(i, data[i+1]);
}

void AD9959_SetPSK(uint8_t Channel, uint16_t *data, uint32_t Freq) {
    uint8_t i = 0;
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CFTW0(Freq);
    Write_CPOW0(data[0]);
    for(i = 0; i < 15; i++)
        Write_Profile_Phase(i, data[i+1]);
}

void AD9959_SetFre_Sweep(uint8_t Channel, uint32_t s_data, uint32_t e_data, uint32_t r_delta, uint32_t f_delta, uint8_t rsrr, uint8_t fsrr, uint16_t Ampli, uint16_t Phase) {
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CPOW0(Phase);
    Write_ACR(Ampli);
    Write_LSRR(rsrr, fsrr);
    
    uint32_t Temp = (uint32_t)(r_delta * ACC_FRE_FACTOR);
    Write_RDW(Temp);
    
    Temp = (uint32_t)(f_delta * ACC_FRE_FACTOR);
    Write_FDW(Temp);
    
    Write_CFTW0(s_data);
    Write_Profile_Fre(0, e_data);
}

void AD9959_SetAmp_Sweep(uint8_t Channel, uint32_t s_Ampli, uint16_t e_Ampli, uint32_t r_delta, uint32_t f_delta, uint8_t rsrr, uint8_t fsrr, uint32_t fre, uint16_t Phase) {
    uint8_t CHANNEL[1] = {Channel};
    uint8_t ACR_data[3] = {0x00, 0x00, 0x00};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CFTW0(fre);
    Write_CPOW0(Phase);
    Write_LSRR(rsrr, fsrr);
    Write_RDW(r_delta << 22);
    Write_FDW(f_delta << 22);
    
    uint32_t A_temp = s_Ampli | 0x1000;
    ACR_data[1] = (uint8_t)(A_temp >> 8);
    ACR_data[2] = (uint8_t)A_temp;
    AD9959_WriteData(ACR_ADD, 3, ACR_data);
    Write_Profile_Ampli(0, e_Ampli);
}

void AD9959_SetPhase_Sweep(uint8_t Channel, uint16_t s_data, uint16_t e_data, uint16_t r_delta, uint16_t f_delta, uint8_t rsrr, uint8_t fsrr, uint32_t fre, uint16_t Ampli) {
    uint8_t CHANNEL[1] = {Channel};
    AD9959_WriteData(CSR_ADD, 1, CHANNEL);
    Write_CFTW0(fre);
    Write_ACR(Ampli);
    Write_LSRR(rsrr, fsrr);
    Write_RDW(r_delta << 18);
    Write_FDW(f_delta << 18);
    Write_CPOW0(s_data);
    Write_Profile_Phase(0, e_data);
}

