/*
 * drvmanager-DAC8552.c
 *
 *  Created on: 2025年4月23日
 *      Author: 47015
 */

#include "drvmanager-dac8552.h"

#include "stm32f4xx.h"                 // STM32 标准库主头文件
#include "stm32f4xx_spi.h"               // SPI 外设驱动
#include "stm32f4xx_gpio.h"              // GPIO 外设驱动
#include "stm32f4xx_rcc.h"               // 时钟管理
#include "drvmanager.h"
#include "stm32f4xx_flash.h"             // Flash 操作
#include "drvmanager-ads1120.h"
#include "core_cmFunc.h"

#include "drvmanager.h"
#include "stm32f4xx.h"
#include "core_cmInstr.h"


void DAC8552_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    // 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

    // 配置 SCK
    GPIO_InitStruct.GPIO_Pin = SPI_SCK_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI_SCK_PORT, &GPIO_InitStruct);

    // 配置 MOSI
    GPIO_InitStruct.GPIO_Pin = SPI_MOSI_PIN;
    GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStruct);

    // 配置 CS
    GPIO_InitStruct.GPIO_Pin = SPI_CS_DAC_PIN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // 空闲高
    GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);

    // 空闲电平：SCK/HIGH, MOSI LOW, CS HIGH
    GPIO_SetBits(SPI_SCK_PORT, SPI_SCK_PIN);//PB10
    GPIO_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN);//PC3
    GPIO_SetBits(SPI_CS_PORT, SPI_CS_DAC_PIN); //PC4
}
// 软件 SPI 发送一个字节，先高位
void DAC8552_SPI_SendByte(uint8_t byte) {
	__disable_irq(); // 进入临界区，禁止中断


	DRVMGR_TimerDelayUs(1);
    for (int8_t i = 7; i >= 0; i--) {
        // 输出 MOSI 位
        if (byte & (1U << i))
            GPIO_SetBits(SPI_MOSI_PORT, SPI_MOSI_PIN);
        else
            GPIO_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN);
        // 拉低 SCK
        GPIO_ResetBits(SPI_SCK_PORT, SPI_SCK_PIN);
        DRVMGR_TimerDelayUs(1);
        // 拉高 SCK
        GPIO_SetBits(SPI_SCK_PORT, SPI_SCK_PIN);
        DRVMGR_TimerDelayUs(1);
    }

    __enable_irq(); // 退出临界区

}

// 写入指定通道数据
void DAC8552_WriteChannel(uint8_t channel, uint16_t data) {
    uint8_t cmd ;
    cmd = channel;

    __disable_irq();

    SPI_SetCS(DEVICE_DAC, 0);
    DRVMGR_TimerDelayUs(1);
    // 拉低 CS 开始
    GPIO_ResetBits(SPI_CS_PORT, SPI_CS_DAC_PIN);
    DRVMGR_TimerDelayUs(1);

    // 发送命令字
    DAC8552_SPI_SendByte(cmd);
    // 发送高 8 位数据
    DAC8552_SPI_SendByte((uint8_t)(data >> 8));
    // 发送低 8 位数据
    DAC8552_SPI_SendByte((uint8_t)(data & 0xFF));

    // 拉高 CS 结束，DAC 更新
    GPIO_SetBits(SPI_CS_PORT, SPI_CS_DAC_PIN);
    DRVMGR_TimerDelayUs(1);
    SPI_SetCS(DEVICE_DAC, 1);
    DRVMGR_TimerDelayUs(1);
    DRVMGR_TimerDelayUs(1);

    __enable_irq(); // 退出临界区

}








void DAC8552_Hand(void){
	DRVMGR_TimerDelayUs(1);
	DAC8552_WriteChannelA(DAC_CMD_CH_A , 0xFFFF);
	DRVMGR_TimerDelayUs(1);

	ADS1120_Gpio_Init();
	DAC8552_GPIO_Init();
    DRVMGR_TimerDelayUs(10);
	DAC8552_WriteChannelB(DAC_CMD_CH_B , 0xFFFF);

}

