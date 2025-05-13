/*
 * devmananger-ps.c
 *
 *  Created on: 2025年4月9日
 *      Author: 47015
 */

#include <string.h>
#include <hqhp/config.h>
#include <hqhp/defs.h>
#include "stm32f4xx.h"
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"
#include "sysmanager/sysmanager.h"
#include "drvmanager/drvmanager-ads1120.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define ZERO_VOL (CONFIG_PS_ZERO_VOL_PT207) // 压力为零时输出电压值，单位mV
#define FULL_VOL (CONFIG_PS_FULL_VOL_PT206) // 压力满量程时输出电压值，单位mV
#define LOST_VOL (ZERO_VOL * 0.8f)	  // 压力传感器未接时电压值，单位mV
#define CHECK_PERIOD (100)
#define OVER_TIMES_LIMIT (3 * (1000 / CHECK_PERIOD))

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _PS_INFO {
	bool  IsLost;
	float Value;	// 压力值，单位MPa
	float VolValue; // 电压值，单位mV
};

struct _PS_MGR {
	struct _PS_INFO Info;
	uint8_t OVPSCnt;					   // 超压次数
	uint8_t OVPSEnable;					   // 超压使能
	uint8_t IsLimit;
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _PS_MGR gPS_MGR;

/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化压力传感器设备模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PSInit(void)
{
	gPS_MGR.Info.IsLost    = false;
	gPS_MGR.Info.Value     = 0;
	gPS_MGR.Info.VolValue  = 0;
	gPS_MGR.IsLimit = false;
	gPS_MGR.OVPSCnt = 0;
	
	// 启动过压检测定时器
	DRVMGR_MSTimerStart(MSEC_TMR_ID_PS_OVPS, CHECK_PERIOD);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理压力传感器设备模块的周期事务
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PSHandle(void)
{
	if (DEVMGR_PSIsLost()) {
		ERRMGR_MajorErrorSet(MAJOR_ERR_PS_LOST);
	} else {
		ERRMGR_MajorErrorClear(MAJOR_ERR_PS_LOST);
	}
	// 判断超压检测定时器是否到期
	if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_PS_OVPS)) {
		// 重启压力检测定时器
		DRVMGR_MSTimerStart(MSEC_TMR_ID_PS_OVPS, CHECK_PERIOD);
		// 过流检测使能的情况下检测压力是否过限
		if (DEVMGR_PSInqValue() < gRUNPara.Limit_PT207) {
			gPS_MGR.OVPSCnt = 0;
			gPS_MGR.IsLimit = false;
		} else {
			gPS_MGR.OVPSCnt += 1;
			if (gPS_MGR.OVPSCnt > OVER_TIMES_LIMIT) {
				gPS_MGR.OVPSCnt = OVER_TIMES_LIMIT;
				gPS_MGR.IsLimit = true;
			}
		}
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于重置压力变送器参数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：ERROR_NONE -- 执行成功
*           ERROR_FLASH_WR_FAILED -- 参数写入失败
****************************************************************************************************
*/
int DEVMGR_PSRestoreDefaultPara(void)
{
	gPS_MGR.Info.IsLost	  = false;
	gPS_MGR.Info.Value	  = 0;
	gPS_MGR.Info.VolValue = 0;
	
	// 设置默认值
	gRUNPara.Ratio_PT207 = 1.0f;
	gRUNPara.Delta_PT207 = 0.0f;
	gRUNPara.Range_PT207 = CONFIG_PS_DEFAULT_RANGE_PT207;	 // 量程，单位MPa
	gRUNPara.Limit_PT207 = CONFIG_PS_DEFAULT_LIMIT_PT207;	 // 限压值，单位MPa
	gRUNPara.OverTime_PT207 = CONFIG_PS_DEFAULT_OVER_TIME_PT207; // 过压时间，单位秒

	int err = SYSMGR_Write_Flash_Params(&gRUNPara, FLASH_PARA_SAVE_ADDR);
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_PS_PARA_WR_FAILED);
		return ERROR_FLASH_WR_FAILED;
	}

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断压力传感器是否连接
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果为TRUE则说明压力传感器未连接
****************************************************************************************************
*/
bool DEVMGR_PSIsLost(void)
{
	bool isLost = true;

	if (DRVMGR_ADCGetValue(2) > LOST_VOL) {
		isLost = false;
	}

	return isLost;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取压力传感器是否过压
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果过压返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_PSIsLimit(void)
{
	return gPS_MGR.IsLimit;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取压力传感器的量程
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：量程，单位MPa
****************************************************************************************************
*/
float DEVMGR_PSInqRange(void)
{
	return gRUNPara.Range_PT207;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取压力传感器的缩放系数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：缩放系数，无量纲
****************************************************************************************************
*/
float DEVMGR_PSInqRatio(void)
{
	return gRUNPara.Ratio_PT207;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取压力传感器的偏移参数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：偏移参数，单位MPa
****************************************************************************************************
*/
float DEVMGR_PSInqDelta(void)
{
	return gRUNPara.Delta_PT207;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取压力传感器的过压值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：过压值，单位MPa
****************************************************************************************************
*/
float DEVMGR_PSInqLimit(void)
{
	return gRUNPara.Limit_PT207;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置压力传感器的过压值
* 注意事项：NA
* 输入参数：limit -- 过压值，单位MPa
* 输出参数：NA
* 返回参数：ERROR_NONE -- 执行成功
*           ERROR_FLASH_WR_FAILED -- 参数写入失败
****************************************************************************************************
*/
int DEVMGR_PSSetLimit(float limit)
{
	if (memcmp(&gRUNPara.Limit_PT207, &limit, sizeof(limit)) == 0) {
		return ERROR_NONE;
	}

	gRUNPara.Limit_PT207 = limit;

	int err = SYSMGR_Write_Flash_Params(&gRUNPara, FLASH_PARA_SAVE_ADDR);
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_PS_PARA_WR_FAILED);
		return ERROR_FLASH_WR_FAILED;
	}

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询压力传感器的参数
* 注意事项：NA
* 输入参数：range -- 量程，单位MPa
* 输出参数：NA
* 返回参数：保存成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
int DEVMGR_PSGetPara(float *range, float *ratio, float *delta, float *limit)
{
	*range = gRUNPara.Range_PT207;
	*ratio = gRUNPara.Ratio_PT207;
	*delta = gRUNPara.Delta_PT207;
	*limit = gRUNPara.Limit_PT207;

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置压力传感器的参数
* 注意事项：NA
* 输入参数：range -- 量程，单位MPa
* 输出参数：NA
* 返回参数：ERROR_NONE -- 执行成功
*           ERROR_FLASH_WR_FAILED -- 参数写入失败
****************************************************************************************************
*/
int DEVMGR_PSSetPara(float range, float ratio, float delta, float limit)
{
	gRUNPara.Range_PT207 = range;
	gRUNPara.Ratio_PT207 = ratio;
	gRUNPara.Delta_PT207 = delta;
	gRUNPara.Limit_PT207 = limit;

	int err = SYSMGR_Write_Flash_Params(&gRUNPara, FLASH_PARA_SAVE_ADDR);
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_PS_PARA_WR_FAILED);
		return ERROR_FLASH_WR_FAILED;
	}

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取压力传感器的压力值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：压力值，单位MPa
****************************************************************************************************
*/
float DEVMGR_PSInqValue(void)
{
	float pressure;
	float volValue;

	// 获取压力传感器通道采样电压
	volValue = DRVMGR_ADCGetValue(2);

	// 根据传感器类型进行电压-压力变换
	pressure = (volValue - ZERO_VOL) * gRUNPara.Range_PT207 / (FULL_VOL - ZERO_VOL);
	pressure = pressure * gRUNPara.Ratio_PT207 + gRUNPara.Delta_PT207;
	if (pressure < 0) {
		pressure = 0;
	} else if (pressure > gRUNPara.Range_PT207) {
		pressure = gRUNPara.Range_PT207;
	}

	gPS_MGR.Info.Value = pressure;
	gPS_MGR.Info.VolValue = volValue;

	return gPS_MGR.Info.Value;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于使能过压检测
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PSOVPSChkEnable(void)
{
	gPS_MGR.OVPSEnable = 1;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于失能过压检测
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PSOVPSChkDisable(void)
{
	gPS_MGR.OVPSEnable = 0;
}
