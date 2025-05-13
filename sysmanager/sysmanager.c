/*
 * sysmanager.c
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */
#include <hqhp/config.h>
#include <stdlib.h>
#include <string.h>
#include "sysmanager/sysmanager.h"
#include "stm32f4xx_flash.h"        // 包含Flash操作相关函数定义
#include "drvmanager/drvmanager.h"
#include "drvmanager/drvmanager-ads1120.h"
#include "drvmanager/drvmanager-dac8552.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define ERR_CHECK_PERIOD (1) // 故障检测周期，单位秒

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
struct MEOH_DATA MEOHData;
struct _RUN_INFO gRUNInfo;
struct _RUN_PARA gRUNPara;

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
static void SYSMGR_CtlHandle(void);

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于系统管理器模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void SYSMGR_Init(void)
{
	sys_stm32_clock_init(336, 8, 2, 7);   // 168MHz
	GPIOInit();
	SYSMGR_Init_Flag();
	memset(SpecialRamBlock, 0, SPECIAL_RAM_BLOCK_SIZE);
	Init_USER_Code();//初始化用户数据;
	init_edge();
//	Read_JZ_LCD_data();
	DRVMGR_Init();
//	ADC_Data_Deal();//模拟量采集
	DEVMGR_Init();
	//COMMGR_Init();	//未编写代码
	Init_Data_Arrays();
	ADS1120_Init();
	DAC8552_GPIO_Init();
//	DRVMGR_ADS1120_Init();
//	DRVMGR_DAC8552_Init();
//	SYSMGR_Params_Init(FLASH_PARA_SAVE_ADDR); //参数初始化
//	DRVMGR_TimerStart(SMS_TMR_ID_CAN_OnLine_1, RUN_CAN_OnLine_TIME);
	//DRVMGR_MSTimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME); // 启动运行指示灯定时器
//	DRVMGR_MSTimerStart(MS_TMR_ID_100MS, 100);
//	DRVMGR_MSTimerStart(MS_TMR_ID_PLC_RX_TIMEOUT, 100);

	FEED();
	Digital_PutOut();
	enable_PutOut();
	// 进入不可操作状态
	SYSMGR_SetRunState(RUNSTATE_IDLE);
	DRVMGR_TimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理系统管理器周期事务
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void SYSMGR_Handle(void)
{
	if (DRVMGR_TimerIsExpiration(MS_TMR_ID_RUN)) {
		// 首先重启定时器
		DRVMGR_TimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME);
		// 执行任务
		switch (SYSMGR_InqRunState()) {
		;
		default:
			break;
		}
	}
	SYSMGR_CtlHandle();
}

void SYSMGR_Init_Flag(void)
{
	DownLoad_Step=0;
	Arbitration_CAN=0;//默认CAN1 传输数据;
	Data_DownLoad_Complete_Flag=0;
	analog_zj_Complete_Flag=0;
	analog_zj_Mode=0;
	iAcnt=0;
	Stored_Data_Flag=0;//存储关键数据标识（触摸屏配置文件）
	RxBytes=0;
	analog_zj_Chn_Flag=0;
    V_4mA =Vref_adc/5;
    State_get_data=0;
    DATA_REC_COMPLETE1=0;
    DATA_REC_COMPLETE2=0;
}




/*!
****************************************************************************************************
* 功能描述：该方法用于查询系统运行模式
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：RUNMODE_DEBUG  -- 调试模式
*           RUNMODE_NORMAL -- 正常模式
****************************************************************************************************
*/
uint8_t SYSMGR_InqRunMode(void)
{
	uint8_t runMode;

	runMode = gRUNInfo.RunMode;

	return runMode;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置系统运行模式
* 注意事项：NA
* 输入参数：runMode -- 运行模式
* 			 @arg RUNMODE_DEBUG      -- 调试模式
* 			 @arg RUNMODE_NORMAL     -- 正常模式
* 			 @arg RUNMODE_STANDALONE -- 脱机模式
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void SYSMGR_SetRunMode(uint8_t runMode)
{
	gRUNInfo.RunMode = runMode;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询上一次系统运行状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：当前系统运行状态
****************************************************************************************************
*/
uint8_t SYSMGR_InqLastRunState(void)
{
	uint8_t runState;

	runState = gRUNInfo.LastRunState;

	return runState;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询系统运行状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：当前系统运行状态
****************************************************************************************************
*/
uint8_t SYSMGR_InqRunState(void)
{
	uint8_t runState;

	runState = gRUNInfo.RunState;

	return runState;
}



/*!
****************************************************************************************************
* 功能描述：该方法用于设置系统运行状态
* 注意事项：NA
* 输入参数：runState -- 运行状态
* 输出参数：NA
* 返回参数：如果设置成功，返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool SYSMGR_SetRunState(uint8_t runState)
{
	bool isOk = true;


	switch (runState) {

	// 空闲状态
	case RUNSTATE_IDLE:
		isOk = SYSMGR_IdleInit();
		break;

	// 备机运行状态
	case RUNSTATE_RUNNING:
		isOk = SYSMGR_RunningInit();
		break;
	// 吹扫状态
	case RUNSTATE_BLAST:
		isOk = SYSMGR_BlastInit();
		break;

	default:
		break;
	}
	if (isOk) {
		// 保存上次工作状态
		gRUNInfo.LastRunState = gRUNInfo.RunState;
		// 更新新状态
		gRUNInfo.RunState = runState;
	}
	return isOk;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于执行流程处理
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void SYSMGR_CtlHandle(void)
{
	switch (SYSMGR_InqRunState()) {

	// 空闲状态
	case RUNSTATE_IDLE:
		SYSMGR_IdleHandle();
		if (ERRMGR_MajorErrorIsExist()) {
		//	SYSMGR_SetRunState(RUNSTATE_INOPERATIVE);
			;
		}
		break;

	// 备机运行状态
	case RUNSTATE_RUNNING:
		SYSMGR_RunningHandle();
		 if (SYSMGR_RunningIsDone()) {
			//SYSMGR_SetRunState(RUNSTATE_LCYCLE);
			 ;
		} else if (ERRMGR_MajorErrorIsExist()) {
		//	SYSMGR_SetRunState(RUNSTATE_INOPERATIVE);
			;
		} else if (ERRMGR_MinorErrorIsExist()) {
		//	SYSMGR_SetRunState(RUNSTATE_IDLE);
			;
		}
		break;

	//  吹扫状态
	case RUNSTATE_BLAST:
		SYSMGR_BlastHandle();
		if (ERRMGR_MinorErrorIsExist()||ERRMGR_MajorErrorIsExist()||SYSMGR_BlastIsDone()) {
			SYSMGR_SetRunState(RUNSTATE_IDLE);
		} 
		break;

	default:
		break;
	}
}


uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq) {
    ErrorStatus status;
    // 1. 复位 RCC 配置
    RCC_DeInit();
    // 2. 打开 HSE
    RCC_HSEConfig(RCC_HSE_ON);
    status = RCC_WaitForHSEStartUp();
    if (status != SUCCESS) return 1;  // HSE 启动失败

    // 3. 配置 Flash 延时和预取
        FLASH_SetLatency(FLASH_Latency_5);       // 5 延时周期
    // 标准库中使用 ENABLE/DISABLE 控制预取缓冲
    FLASH_PrefetchBufferCmd(ENABLE);          // 使能 Flash 预取缓冲

    // 4. 配置分频：HCLK = SYSCLK / 1, PCLK1 = HCLK / 4, PCLK2 = HCLK / 2
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2);

    // 5. 配置 PLL: PLLCLK = HSE/pllm * plln / pllp, USBCLK = HSE/pllm * plln / pllq
    RCC_PLLConfig(RCC_PLLSource_HSE, pllm, plln, pllp, pllq);
    // 6. 启动 PLL
    RCC_PLLCmd(ENABLE);
    // 7. 等待 PLL 就绪
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    // 8. 切换系统时钟到 PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    // 9. 等待切换完成
    while (RCC_GetSYSCLKSource() != 0x08);
    return 0;
}
