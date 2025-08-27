#include "MCP.h"

// 延时函数（适配168MHz主频）


void MCP410XXInit(void)
{
    // 使能GPIO时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 配置CS1和CS2引脚（PG4, PG2）
    GPIO_InitStruct.Pin = MCP41xx_CS1_PIN | MCP41xx_CS2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    // 配置CLK引脚（PD12）- 保持输出
    GPIO_InitStruct.Pin = MCP41xx_CLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 关键修复：配置DAT引脚为开漏输出+上拉（支持双向通信）
    GPIO_InitStruct.Pin = MCP41xx_DAT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // 启用上拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 初始状态设置
    MCP41xx_SPI_CS1_H();
    MCP41xx_SPI_CS2_H();
    MCP41xx_SPI_CLK_L();
    MCP41xx_SPI_DAT_H();  // 新增：DAT线初始高电平（空闲状态）
		HAL_Delay(10);  // 增加10ms电源稳定时间 
}

// 电位器1写数据（使用正确命令字）
void MCP41xx_1writedata(int dat1)
{
    uchar i;
    MCP41xx_SPI_CS2_H();
    MCP41xx_SPI_CS1_H();
    MCP41xx_SPI_CLK_L();
    MCP41xx_SPI_CS1_L();

    HAL_Delay(1);  // 改用HAL延时

    // 发送修正后的命令字
    for(i = 0; i < 8; i++) {
        (CMD_WRITE_POT0 & (0x80 >> i)) ? MCP41xx_SPI_DAT_H() : MCP41xx_SPI_DAT_L();
        HAL_Delay(1);
        MCP41xx_SPI_CLK_H();
        HAL_Delay(1);
        MCP41xx_SPI_CLK_L();
        HAL_Delay(1);
    }

    // 发送数据
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

// 电位器2写数据
void MCP41xx_2writedata(int dat2)
{
    uchar i;
    MCP41xx_SPI_CS1_H();  // 确保CS1无效
    MCP41xx_SPI_CS2_H();  // 先拉高CS2
    MCP41xx_SPI_CLK_L();  // 时钟初始低电平
    
    MCP41xx_SPI_CS2_L();  // 选中电位器2
    HAL_Delay(1);         // 建立时间

    // 发送修正后的命令字(0x32)
    for(i = 0; i < 8; i++) {
        // 从最高位(MSB)开始发送
        (CMD_WRITE_POT1 & (0x80 >> i)) ? MCP41xx_SPI_DAT_H() : MCP41xx_SPI_DAT_L();
        HAL_Delay(1);
        
        // 产生时钟上升沿
        MCP41xx_SPI_CLK_H();
        HAL_Delay(1);
        
        // 产生时钟下降沿
        MCP41xx_SPI_CLK_L();
        HAL_Delay(1);
    }

    // 发送数据
    for(i = 0; i < 8; i++) {
        // 从最高位(MSB)开始发送
        (dat2 & (0x80 >> i)) ? MCP41xx_SPI_DAT_H() : MCP41xx_SPI_DAT_L();
        HAL_Delay(1);
        
        MCP41xx_SPI_CLK_H();
        HAL_Delay(1);
        
        MCP41xx_SPI_CLK_L();
        HAL_Delay(1);
    }

    // 结束传输
    MCP41xx_SPI_CS2_H();  // 取消选中
    HAL_Delay(1);         // 保持时间
}
