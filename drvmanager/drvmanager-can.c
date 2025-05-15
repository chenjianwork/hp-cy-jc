/*
 * drvmanager-can.c
 *
 *  Created on: 2023年2月16日
 *      Author: liyan
 */

#include "drvmanager/drvmanager.h"
#include "stm32f4xx.h"
#include "commanager/commanager.h"

extern unsigned int MASK_d;


static void DRVMGR_CANHwPinInit(void);


void DRVMGR_CANInit(void)
{
	DRVMGR_CANHwPinInit();
}
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_CANHwPinInit(void)
{
	CAN_InitTypeDef   CAN_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_FilterInitTypeDef      CAN_FilterInitStructure;
	NVIC_InitTypeDef                 NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //复用推挽
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉

    GPIO_Init(GPIOA, &GPIO_InitStructure);            //初始化IO

    //引脚复用映射配置
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOB13复用为CAN2
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOB12复用为CAN2

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    //复用推挽
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉

    GPIO_Init(GPIOB, &GPIO_InitStructure);            //初始化IO

    //CAN单元设置
    CAN_InitStructure.CAN_TTCM=DISABLE;            //非时间触发通信模式
    CAN_InitStructure.CAN_ABOM=ENABLE;            //软件自动离线管理
    CAN_InitStructure.CAN_AWUM=DISABLE;            //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART=ENABLE;            //使能报文自动传送
    CAN_InitStructure.CAN_RFLM=DISABLE;             //报文不锁定,新的覆盖旧的
    CAN_InitStructure.CAN_TXFP=DISABLE;            //优先级由报文标识符决定
    CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;            //模式设置： mode:0,普通模式;1,回环模式;
    //外设时钟36M
    //设置波特率500kbps
//    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;                //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq     CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//    CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;             //Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
//    CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;                //Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~    CAN_BS2_8tq

    //设置波特率250kbps
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;                //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq     CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;             //Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;                //Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~    CAN_BS2_8tq

    CAN_InitStructure.CAN_Prescaler=9;        //分频系数(Fdiv)为brp+1

    CAN_Init(CAN1, &CAN_InitStructure);            //初始化CAN1
    CAN_Init(CAN2, &CAN_InitStructure);            //初始化CAN2


    //滤波器配置
    CAN_FilterInitStructure.CAN_FilterNumber=0;    //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;     //屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;     //32位宽
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器0

    // 设置过滤器ID和掩码 - 使用0x2080作为基准ID
    CAN_FilterInitStructure.CAN_FilterIdHigh = (0x2080 << 3) >> 16;    // 高16位
    CAN_FilterInitStructure.CAN_FilterIdLow = ((0x2080 << 3) & 0xFFFF) | CAN_ID_EXT;  // 低16位

    // 设置掩码 - 只匹配后12位(0x080)，其他位不关心
    // 掩码为0xFFFFF000，表示高20位不关心，低12位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;  // 高16位掩码
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xF000 | CAN_ID_EXT;  // 低16位掩码

    CAN_FilterInit(&CAN_FilterInitStructure);            //滤波器初始化

    //CAN1 NVIC 中断配置
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1,CAN_IT_ERR,DISABLE);
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);                //FIFO0消息挂号中断允许.


    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);

    //滤波器配置 - CAN2
    CAN_FilterInitStructure.CAN_FilterNumber=15;    //过滤器15
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;     //屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;     //32位宽
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//激活过滤器

    // 使用相同的过滤器配置
    CAN_FilterInitStructure.CAN_FilterIdHigh = (0x2080 << 3) >> 16;    // 高16位
    CAN_FilterInitStructure.CAN_FilterIdLow = ((0x2080 << 3) & 0xFFFF) | CAN_ID_EXT;  // 低16位

    // 设置掩码 - 只匹配后12位(0x080)，其他位不关心
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;  // 高16位掩码
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xF000 | CAN_ID_EXT;  // 低16位掩码

    CAN_FilterInit(&CAN_FilterInitStructure);            //滤波器初始化

    //CAN2 NVIC 中断配置
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级为2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN2,CAN_IT_ERR,DISABLE);
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);                //FIFO0消息挂号中断允许.


    NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}
void CAN1_RX0_IRQHandler(void)
{
	CAN1_rx_data();
}
void CAN2_RX0_IRQHandler(void)
{
	CAN2_rx_data();
}
void CAN1_TX_IRQHandler(void)
{
	if(SET == CAN_GetITStatus(CAN1,CAN_IT_TME))
	{
	  CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}
void CAN2_TX_IRQHandler(void)
{
	if(SET == CAN_GetITStatus(CAN2,CAN_IT_TME))
	{
	  CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
	}
}
