#ifndef __MCP41xx_h
#define __MCP41xx_h
#include "stm32f4xx_hal.h"

#define uint unsigned int
#define uchar unsigned char

#define CMD_WRITE_POT0 0x11  // 00110001
#define CMD_WRITE_POT1 0x12  // 00110010

// 引脚重定义（根据实际硬件连接）
#define MCP41xx_CS1_PIN     GPIO_PIN_4
#define MCP41xx_CS1_PORT    GPIOG

#define MCP41xx_CS2_PIN     GPIO_PIN_2
#define MCP41xx_CS2_PORT    GPIOG

#define MCP41xx_CLK_PIN     GPIO_PIN_12
#define MCP41xx_CLK_PORT    GPIOD

#define MCP41xx_DAT_PIN     GPIO_PIN_15
#define MCP41xx_DAT_PORT    GPIOB

// HAL库GPIO操作宏
#define MCP41xx_SPI_CLK_H()     HAL_GPIO_WritePin(MCP41xx_CLK_PORT, MCP41xx_CLK_PIN, GPIO_PIN_SET)
#define MCP41xx_SPI_CLK_L()     HAL_GPIO_WritePin(MCP41xx_CLK_PORT, MCP41xx_CLK_PIN, GPIO_PIN_RESET)

#define MCP41xx_SPI_DAT_H()     HAL_GPIO_WritePin(MCP41xx_DAT_PORT, MCP41xx_DAT_PIN, GPIO_PIN_SET)
#define MCP41xx_SPI_DAT_L()     HAL_GPIO_WritePin(MCP41xx_DAT_PORT, MCP41xx_DAT_PIN, GPIO_PIN_RESET)

#define MCP41xx_SPI_CS1_H()     HAL_GPIO_WritePin(MCP41xx_CS1_PORT, MCP41xx_CS1_PIN, GPIO_PIN_SET)
#define MCP41xx_SPI_CS1_L()     HAL_GPIO_WritePin(MCP41xx_CS1_PORT, MCP41xx_CS1_PIN, GPIO_PIN_RESET)

#define MCP41xx_SPI_CS2_H()     HAL_GPIO_WritePin(MCP41xx_CS2_PORT, MCP41xx_CS2_PIN, GPIO_PIN_SET)
#define MCP41xx_SPI_CS2_L()     HAL_GPIO_WritePin(MCP41xx_CS2_PORT, MCP41xx_CS2_PIN, GPIO_PIN_RESET)

void MCP410XXInit(void);
void mcp_delay(uint n);
void MCP41xx_1writedata(int dat1);
void MCP41xx_2writedata(int dat2);

#endif
