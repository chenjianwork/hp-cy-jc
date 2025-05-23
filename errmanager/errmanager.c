/*
 * errmanager.c
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */
#include "errmanager.h"


/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
struct _ERR_MGR
{
	uint32_t MajorError;
	uint32_t MinorError;
};


/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _ERR_MGR gERRMGR;

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
* 功能描述：该方法用于初始化故障管理器
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void ERRMGR_Init(void)
{
	gERRMGR.MajorError = 0;
	gERRMGR.MinorError = 0;
}

/*!
****************************************************************************************************
* 功能描述：该方法是故障管理的周期服务函数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void ERRMGR_Handle(void)
{
	/* NOTHING TO DO */
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询主错误状态值
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：NA
* 输出参数：NA
* 返回参数：主错误状态值
****************************************************************************************************
*/
uint32_t ERRMGR_MajorErrorInqValue(void)
{
	return gERRMGR.MajorError;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否存在主错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果存在返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool ERRMGR_MajorErrorIsExist(void)
{
	return gERRMGR.MajorError != 0;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否存在指定的主错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：majorError -- 主错误标识
* 输出参数：NA
* 返回参数：如果存在返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool ERRMGR_MajorErrorIsSet(uint8_t majorError)
{
	return gERRMGR.MajorError & (1 << majorError);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否存在指定的次错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：minorError -- 次错误标识
* 输出参数：NA
* 返回参数：如果存在返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool ERRMGR_MinorErrorIsOnlySet(uint8_t minorError)
{
	return gERRMGR.MinorError == (1 << minorError);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置指定的主错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：majorError -- 主错误标识
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void ERRMGR_MajorErrorSet(uint8_t majorError)
{
	if (majorError > 31) {
		return;
	}
	gERRMGR.MajorError |= (1 << majorError);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于清除指定的主错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：majorError -- 主错误标识
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void ERRMGR_MajorErrorClear(uint8_t majorError)
{
	gERRMGR.MajorError &= ~(1 << majorError);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询次错误状态值
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：NA
* 输出参数：NA
* 返回参数：次错误状态值
****************************************************************************************************
*/
uint32_t ERRMGR_MinorErrorInqValue(void)
{
	return gERRMGR.MinorError;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否存在次错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果存在返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool ERRMGR_MinorErrorIsExist(void)
{
	return gERRMGR.MinorError != 0;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否存在指定的次错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：minorError -- 次错误标识
* 输出参数：NA
* 返回参数：如果存在返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool ERRMGR_MinorErrorIsSet(uint8_t minorError)
{
	return gERRMGR.MinorError & (1 << minorError);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置指定的次错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：minorError -- 次错误标识
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void ERRMGR_MinorErrorSet(uint8_t minorError)
{
	if (minorError > 31) {
		return;
	}
	gERRMGR.MinorError |= (1 << minorError);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于清除指定的次错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：minorError -- 次错误标识
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void ERRMGR_MinorErrorClear(uint8_t minorError)
{
	gERRMGR.MinorError &= ~(1 << minorError);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于清除所有的次错误
* 注意事项：调用该方法前需要先调用初始化方法
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void ERRMGR_MinorErrorClearAll(void)
{
	gERRMGR.MinorError = 0;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/



