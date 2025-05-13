/*
 * drvmanager-dac8552.h
 *
 *  Created on: 2025年4月23日
 *      Author: 47015
 */

#ifndef DRVMANAGER_DRVMANAGER_DAC8552_H_
#define DRVMANAGER_DRVMANAGER_DAC8552_H_

#include "drvmanager.h"



/* ===== DAC8552 Function Declarations ===== */
void DAC8552_GPIO_Init(void);
void DAC8552_Init(void);
void DAC8552_WriteChannels(uint16_t data_a, uint16_t data_b);
void DAC8552_Hand(void);
void DAC8552_Set_Channel(uint8_t cmd, uint16_t Data);
void DAC8552_WriteChannelA(uint8_t channel, uint16_t data);
void DAC8552_WriteChannelB(uint8_t channel, uint16_t data);
void DAC8552_SPI_SendByte(uint8_t byte);
uint8_t DAC8552_SPI_RW(uint8_t byte);

#endif /* DRVMANAGER_DRVMANAGER_DAC8552_H_ */
