/*!
****************************************************************************************************
* 文件名称：drvmanager-timer.c
* 功能简介：该文件是驱动管理器定时器驱动模块的实现源文件
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include "drvmanager/drvmanager.h"
#include "stm32f4xx.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _TIMER {
	uint8_t	 Enable;	 // 使能控制
	uint8_t	 Expiration; // 到期标识
	uint16_t Count;		 // 计数
	uint16_t Limit;		 // 计数上限
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/

static struct _TIMER G_TMR_TABLE[TMR_ID_NUMS];
static struct _TIMER G_MS_TMR_TABLE[MS_TMR_ID_NUMS];

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void DRVMGR_TimerUpdate(void);
static void DRVMGR_MSTimerUpdate(void);
static void DRVMGR_TimerHwInit(void);
static void DRVMGR_MSTimerHwInit(void);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化定时器驱动模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_TimerInit(void)
{
	int i;

	// 加上断言可以确保避免一些错误
	STATIC_ASSERT(TMR_ID_MAXIMUM <= TMR_ID_NUMS);
	STATIC_ASSERT(MS_TMR_ID_MAXIMUM <= MS_TMR_ID_NUMS);

	for (i = 0; i < ELEMENTS_OF(G_TMR_TABLE); i++) {
		G_TMR_TABLE[i].Enable	  = false;
		G_TMR_TABLE[i].Expiration = false;
		G_TMR_TABLE[i].Count	  = 0;
		G_TMR_TABLE[i].Limit	  = 0;
	}

	for (i = 0; i < ELEMENTS_OF(G_MS_TMR_TABLE); i++) {
		G_MS_TMR_TABLE[i].Enable	 = false;
		G_MS_TMR_TABLE[i].Expiration = false;
		G_MS_TMR_TABLE[i].Count		 = 0;
		G_MS_TMR_TABLE[i].Limit		 = 0;
	}

	DRVMGR_TimerHwInit();
	DRVMGR_MSTimerHwInit();
}

/*!
****************************************************************************************************
* 功能描述：该方法是定时器驱动模块的周期服务函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_TimerHandle(void)
{
	/* NOTHING TO DO */
}

/*!
****************************************************************************************************
* 功能描述：该方法用于延时指定的微秒数
* 注意事项：NA
* 输入参数：us -- 期待循环等待的微秒数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_TimerDelayUs(uint16_t us)
{
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	SysTick->LOAD = (SystemCoreClock / 1000000) * us - 1;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL = 0;
	while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
		;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于启动秒定时器运行
* 注意事项：NA
* 输入参数：idx -- 定时器ID
*           limit -- 到期时间
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_TimerStart(int idx, uint16_t limit)
{
	if (idx >= ELEMENTS_OF(G_TMR_TABLE)) {
		return;
	}

	G_TMR_TABLE[idx].Count		= 0;
	G_TMR_TABLE[idx].Limit		= limit;
	G_TMR_TABLE[idx].Expiration = false;
	G_TMR_TABLE[idx].Enable		= true;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于取消秒定时器运行
* 注意事项：NA
* 输入参数：idx -- 定时器ID
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_TimerCancel(int idx)
{
	if (idx >= ELEMENTS_OF(G_TMR_TABLE)) {
		return;
	}

	G_TMR_TABLE[idx].Enable		= false;
	G_TMR_TABLE[idx].Expiration = false;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断秒定时器是否到期
* 注意事项：如果定时器ID无效，将始终返回FALSE
* 输入参数：idx -- 定时器ID
* 输出参数：NA
* 返回参数：如果定时器到期返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DRVMGR_TimerIsExpiration(int idx)
{
	if (idx >= ELEMENTS_OF(G_TMR_TABLE)) {
		return false;
	}

	return G_TMR_TABLE[idx].Expiration;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于启动毫秒定时器运行
* 注意事项：NA
* 输入参数：idx -- 定时器ID
*           limit -- 到期时间
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_MSTimerStart(int idx, uint16_t limit)
{
	if (idx >= ELEMENTS_OF(G_MS_TMR_TABLE)) {
		return;
	}

	G_MS_TMR_TABLE[idx].Count	   = 0;
	G_MS_TMR_TABLE[idx].Limit	   = limit;
	G_MS_TMR_TABLE[idx].Expiration = false;
	G_MS_TMR_TABLE[idx].Enable	   = true;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于取消毫秒定时器运行
* 注意事项：NA
* 输入参数：idx -- 定时器ID
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DRVMGR_MSTimerCancel(int idx)
{
	if (idx >= ELEMENTS_OF(G_MS_TMR_TABLE)) {
		return;
	}

	G_MS_TMR_TABLE[idx].Enable	   = false;
	G_MS_TMR_TABLE[idx].Expiration = false;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断毫秒定时器是否到期
* 注意事项：如果定时器ID无效，将始终返回FALSE
* 输入参数：idx -- 定时器ID
* 输出参数：NA
* 返回参数：如果定时器到期返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DRVMGR_MSTimerIsExpiration(int idx)
{
	if (idx >= ELEMENTS_OF(G_MS_TMR_TABLE)) {
		return false;
	}

	return G_MS_TMR_TABLE[idx].Expiration;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于更新秒定时器计数执行超时处理
* 注意事项：该方法应在硬件定时器中周期调用
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_TimerUpdate(void)
{
	int i;

	for (i = 0; i < ELEMENTS_OF(G_TMR_TABLE); i++) {
		// 仅在使能并且未到期的情况下进行计数
		if ((G_TMR_TABLE[i].Enable == true) && (G_TMR_TABLE[i].Expiration == false)) {
			G_TMR_TABLE[i].Count++;
			if (G_TMR_TABLE[i].Count >= G_TMR_TABLE[i].Limit) {
				G_TMR_TABLE[i].Expiration = true;
			}
		}
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于更新毫秒定时器计数执行超时处理
* 注意事项：该方法应在硬件定时器中周期调用
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_MSTimerUpdate(void)
{
	int i;

	for (i = 0; i < ELEMENTS_OF(G_MS_TMR_TABLE); i++) {
		// 仅在使能并且未到期的情况下进行计数
		if ((G_MS_TMR_TABLE[i].Enable == true) && (G_MS_TMR_TABLE[i].Expiration == false)) {
			G_MS_TMR_TABLE[i].Count++;
			if (G_MS_TMR_TABLE[i].Count >= G_MS_TMR_TABLE[i].Limit) {
				G_MS_TMR_TABLE[i].Expiration = true;
			}
		}
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于初始化定时器外设模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_TimerHwInit(void)
{
	TIM_TimeBaseInitTypeDef timeBaseStructure;

	// 使能外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// 配置时间参数：APB1时钟为系统时钟的1/4，定时频率为1Hz
	timeBaseStructure.TIM_Period		= SystemCoreClock / 2 - 1;
	timeBaseStructure.TIM_Prescaler		= 1;
	timeBaseStructure.TIM_ClockDivision = 0;
	timeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &timeBaseStructure);

	// 使能超时中断
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);

	// 使能定时器
	TIM_Cmd(TIM2, ENABLE);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于初始化定时器外设模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DRVMGR_MSTimerHwInit(void)
{
	TIM_TimeBaseInitTypeDef timeBaseStructure;

	// 使能外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// 配置时间参数：APB1时钟为系统时钟的1/4，定时频率为1000Hz

	//计数器计数Counter=SystemCoreClock / 2000=36000次就发生更新事件（因为从0开始需要-1）
	timeBaseStructure.TIM_Period		= SystemCoreClock / 2000 - 1;
	//分频器设置2，那么系统时钟经过分频器就变为72000000/2=36000000,
	//那么计数一次的时间t=1/36000000,那么定时器定时时间=36000*1/36000000=1/1000=1ms
	timeBaseStructure.TIM_Prescaler		= 1;
	timeBaseStructure.TIM_ClockDivision = 0;
	timeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &timeBaseStructure);

	// 使能超时中断
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);

	// 使能定时器
	TIM_Cmd(TIM3, ENABLE);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		// 清除超时更新标志
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		// 处理秒定时器事件
		DRVMGR_TimerUpdate();
	}
	//FEED();
}

/*!
****************************************************************************************************
* 功能描述：该方法用于
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
		// 清除超时更新标志
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		// 处理毫秒定时器事件
		DRVMGR_MSTimerUpdate();
	}
	FEED();
}
