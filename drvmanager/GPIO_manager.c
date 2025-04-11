/*
 * GPIO_manager.c
 *
 *  Created on: 2023年2月14日
 *      Author: liyan
 */
#include "drvmanager.h"
#include "stm32f4xx.h"

void GPIOInit(void);
extern unsigned char Data_Arrays[256][64];
static void DRVMGR_GPIOHwPinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;



	// IOprogramer --> PF5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);



	// WP# --> PD0
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// SPI3CS --> PD1
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// X1 --> PD3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// X2 --> PD4
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// X3 --> PD5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// X4 --> PD6
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// X5 --> PD7
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// X6 --> PG9
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X7 --> PG10
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X8 --> PG11
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X9 --> PG12
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X10 --> PG13
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X11 --> PG14
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X12 --> PG15
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// X13 --> PB3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// X14 --> PB4
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	// Y1 --> PB9
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Y2 --> PE0
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y3 --> PE1
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y4 --> PE2
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y5 --> PE3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y6 --> PE4
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y7 --> PE5
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Y8 --> PE6
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// OE1# --> PF2
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	// OE2# --> PF3
	GPIO_InitStructure.GPIO_Pin	  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}


void GPIOInit(void)
{
	DRVMGR_GPIOHwPinInit();
}
void Download_Mode(void)
{

	while((~GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_5))&0x1)
		{
			if(DRVMGR_MSTimerIsExpiration(MS_TMR_ID_RUN))
			{
				DRVMGR_MSTimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME);
				FEED();
			}
		 DownLoad_Data_Deal();
		}
}
void Scan_Digital_Input(void)
{
	unsigned char dataL=0,dataH=0;
	dataL=((GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)&0x1)<<0)| \
	     ((GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4)&0x1)<<1)| \
		 ((GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5)&0x1)<<2)| \
		 ((GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)&0x1)<<3)| \
		 ((GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_7)&0x1)<<4)| \
		 ((GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9)&0x1)<<5)| \
		 ((GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10)&0x1)<<6)| \
		 ((GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_11)&0x1)<<7) \
		 ;
	Data_Arrays[0][0]=~dataL;
	dataH=((GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_12)&0x1)<<0)| \
		 ((GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_13)&0x1)<<1)| \
		 ((GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_14)&0x1)<<2)| \
		 ((GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_15)&0x1)<<3)| \
		 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)&0x1)<<4)| \
		 ((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)&0x1)<<5)  \
	     ;
   Data_Arrays[0][1]=(~dataH)&0x3F;
}
void enable_PutOut(void)
{
	GPIO_SetBits(GPIOF, GPIO_Pin_2);
	GPIO_ResetBits(GPIOF, GPIO_Pin_3);

}
void  Digital_PutOut(void)
{
	unsigned char data;

	data=~Data_Arrays[1][0];
   //Y1
    if((data>>0)&0x1)
	  GPIO_SetBits(GPIOB, GPIO_Pin_9);
    else
      GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    //Y2
    if((data>>1)&0x1)
	  GPIO_SetBits(GPIOE, GPIO_Pin_0);
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_0);
   //Y3
    if((data>>2)&0x1)
	  GPIO_SetBits(GPIOE, GPIO_Pin_1);
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    //Y4
    if((data>>3)&0x1)
	  GPIO_SetBits(GPIOE, GPIO_Pin_2);
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_2);
    //Y5
    if((data>>4)&0x1)
	  GPIO_SetBits(GPIOE, GPIO_Pin_3);
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_3);
    //Y6
    if((data>>5)&0x1)
	  GPIO_SetBits(GPIOE, GPIO_Pin_4);
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_4);
    //Y7
    if((data>>6)&0x1)
	  GPIO_SetBits(GPIOE, GPIO_Pin_5);
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_5);
    //Y8
    if((data>>7)&0x1)
	  GPIO_SetBits(GPIOE, GPIO_Pin_6);
    else
      GPIO_ResetBits(GPIOE, GPIO_Pin_6);
}
