/*
 * sysmanager-running.c
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */

#include "sysmanager/sysmanager.h"
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"

/**
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
// 时间相关常量
#define RUNSTATE_CHECK_ONLINE_TIME  1000    // 运行状态检查时间(ms)
#define PID_PERIOD                  100     // PID计算周期(ms)

// 压力控制相关常量
#define PRESSURE_SP                 0.8f    // 压力设定值(MPa)
#define PRESSURE_CHANGE_THRESHOLD   0.1f    // 压力变化阈值(MPa)

// 输出限制相关常量
#define VALVE_CURRENT_MIN          4.0f    // 阀门最小输出电流(mA)
#define VALVE_CURRENT_MAX          20.0f   // 阀门最大输出电流(mA)
#define VFD_FREQ_BASE              50.0f   // 变频器基础频率(Hz)
#define VFD_FREQ_MIN_HZ            0.0f    // 变频器最小频率(Hz)
#define VFD_FREQ_MAX_HZ            100.0f  // 变频器最大频率(Hz)

/**
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
/*!
* @brief 工作信息结构体
*/
struct _WORK_INFO {
    uint8_t         State;              // 工作状态
    struct _PID     OuterPID;           // 外环PID控制器(压力环)
    struct _PID     InnerPID;           // 内环PID控制器(频率环)
    struct _TIMER1  Tmr;                // PID周期定时器
    float           LastVfdOutput;      // 上一次变频器输出值(Hz)
    float           LastValveOutput;    // 上一次阀门输出值(mA)
    float           ValveSetPoint;      // 阀门开度设定值(%)
};

/**
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _WORK_INFO gWorkInfo;

/**
****************************************************************************************************
* 函数声明
****************************************************************************************************
*/
static void SYSMGR_RunningPIDCtl(void);

/**
****************************************************************************************************
* 函数实现
****************************************************************************************************
*/

/*!
****************************************************************************************************
* @brief 运行管理器初始化
* @return bool 初始化成功返回true，否则返回false
****************************************************************************************************
*/
bool SYSMGR_RunningInit(void)
{
	// 初始化工作状态
	gWorkInfo.State = WORK_INIT;

	gWorkInfo.LastVfdOutput = VFD_FREQ_BASE;  // 初始化为基础频率
	gWorkInfo.LastValveOutput = VALVE_CURRENT_MIN;  // 初始化为最小电流
	gWorkInfo.ValveSetPoint = 0.0f;

	// 初始化级联PID控制器
	PID_InitCascade(&gWorkInfo.OuterPID,    // 外环PID(压力环)
				   &gWorkInfo.InnerPID,     // 内环PID(频率环)
				   2.0f,   // 外环比例系数
				   0.1f,   // 外环积分系数
				   0.01f,  // 外环微分系数
				   1.5f,   // 内环比例系数
				   0.08f,  // 内环积分系数
				   0.02f); // 内环微分系数

	// 启动PID周期定时器
	DRVMGR_TimerStart(MS_TMR_ID_VDF_PID_TIMEOUT, PID_PERIOD);

	return true;
}

/*!
****************************************************************************************************
* @brief 检查运行是否完成
* @return bool 运行完成返回true，否则返回false
****************************************************************************************************
*/
bool SYSMGR_RunningIsDone(void)
{
	return (gWorkInfo.State == WORK_DONE);
}

/*!
****************************************************************************************************
* @brief 运行管理器主处理函数
****************************************************************************************************
*/
void SYSMGR_RunningHandle(void)
{
	switch (gWorkInfo.State) {
	case WORK_DONE:
		break;

	case WORK_INIT:
		DRVMGR_TimerStart(MS_TMR_ID_RUNSTATE_CHECK_ONLINE, RUNSTATE_CHECK_ONLINE_TIME);
		gWorkInfo.State = WORK_DOING;
		break;

	case WORK_DOING:
		// 检查运行状态
		if (COMMGR_GetRunStateCAN() == 0) {
			SYSMGR_SetRunState(RUNSTATE_IDLE);
			gWorkInfo.State = WORK_EXIT;
			break;
		}

		// 执行PID控制
		SYSMGR_RunningPIDCtl();
		break;

	case WORK_EXIT:
		gWorkInfo.State = WORK_DONE;
		SYSMGR_SetRunState(RUNSTATE_IDLE);
		break;

	default:
		gWorkInfo.State = WORK_EXIT;
		break;
	}
}


/*!
****************************************************************************************************
* @brief PID控制处理函数
* @note 该函数在PID_PERIOD周期内被调用
****************************************************************************************************
*/
static void SYSMGR_RunningPIDCtl(void)
{
	// 检查定时器是否到期
	if (!DRVMGR_TimerIsExpiration(MS_TMR_ID_VDF_PID_TIMEOUT)) {
		return;
	}

	// 重启PID周期计算定时器
	DRVMGR_TimerStart(MS_TMR_ID_VDF_PID_TIMEOUT, PID_PERIOD);

	// 获取当前压力值
	float currP = DEVMGR_PSInqValue();
	if (currP < 0) {
		// 压力传感器故障，保持当前输出
		return;
	}

	// 计算级联PID控制输出
	float valve_sp = PID_ComputeCascade(
		&gWorkInfo.OuterPID,    // 外环PID(压力环)
		&gWorkInfo.InnerPID,    // 内环PID(频率环)
		PRESSURE_SP,            // 压力设定值
		currP,                  // 压力反馈值
		&gWorkInfo.LastValveOutput,  // 阀门输出电流
		&gWorkInfo.LastVfdOutput     // 变频器输出频率
	);

	// 更新状态信息
	gWorkInfo.ValveSetPoint = valve_sp;
}

/*!
****************************************************************************************************
* @brief 查询PID设定值
* @return float PID设定值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDSP(void)
{
	return gWorkInfo.OuterPID.Sp;
}

/*!
****************************************************************************************************
* @brief 查询PID误差值
* @return float PID误差值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDErr(void)
{
	return gWorkInfo.OuterPID.Err[0];
}

/*!
****************************************************************************************************
* @brief 查询PID输出值
* @return float PID输出值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDOutput(void)
{
	return gWorkInfo.OuterPID.Out;
}

/*!
****************************************************************************************************
* @brief 查询PID积分值
* @return float PID积分值
****************************************************************************************************
*/
float SYSMGR_Running_InqPIDSum(void)
{
	return gWorkInfo.OuterPID.Sum;
}
