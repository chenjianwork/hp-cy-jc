/*!
****************************************************************************************************
* 文件名称：devmanager-flow.c
* 功能简介：流量计管理模块
* 文件作者：Haotian
* 创建日期：2020-09-17
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <config.h>
#include <string.h>
#include <hqhp/crypto/crc.h>
#include <hqhp/drvmanager.h>
#include "commanager.h"
#include "devmanager.h"

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define FLOW_IDX_NUMS           (2)    // 流量计索引个数
#define MODE_WORK              (0)    // 工作模式
#define MODE_CLEAR             (1)    // 清零模式
#define CLR_CNT_LIMIT          (5)    // 清总累尝试次数，如果连续发送CLR_CNT_LIMIT次清总累命令未得到响应则认为失败

#define POLL_PERIOD            (50)   // 轮询间隔，单位毫秒
#define LINK_TIMEOUT           (5000) // 链路连接超时时间，单位毫秒
#define RX_BYTE_TIMEOUT        (100)  // 字节接收超时，单位毫秒
#define OFLOW_CHK_PERIOD       (1000) // 过流检测周期，单位毫秒
#define OFLOW_TIME_UPLIMIT     (5000) // 过流时间上限，单位毫秒
#define OFLOW_CNT_LIMIT        (OFLOW_TIME_UPLIMIT / OFLOW_CHK_PERIOD) // 过流次数上限

#define RX_FRAME_MIN_SIZE      (2)    // 最小帧长度
#define RX_FRAME_MAX_SIZE      (128)  // 最大帧长度
#define TX_BUF_SIZE            (128)  // 发送缓存大小，单位字节

// 浮点数字节序定义
#define ENDIAN_1234            (0)    // 浮点数字节序：1-2-3-4
#define ENDIAN_2143            (1)    // 浮点数字节序：2-1-4-3
#define ENDIAN_4321            (2)    // 浮点数字节序：4-3-2-1
#define ENDIAN_3412            (3)    // 浮点数字节序：3-4-1-2

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
// 流量计管理结构体
struct _FLOW_MGR {
	uint8_t Idx;                // 当前读的流量计索引
	uint8_t Mode;               // 工作模式
	uint8_t IsClrDone;          // 是否清除总累完成
	uint8_t IsClrSucceed;       // 是否清除总累成功
	uint8_t ClrCnt;             // 清除总累重试次数
	uint8_t OFlowCnt;           // 过流次数
	uint8_t OFlowEnable;        // 过流使能

	struct {
		uint8_t  Refresh;       // 接收刷新标志，当用户取走数据后，标志复位
		uint16_t Bytes;         // 接收计数标识
		uint8_t  Buffer[RX_FRAME_MAX_SIZE];    // 接收缓存
		uint16_t FrameLen;      // 帧长度
		uint8_t  FrameBody[RX_FRAME_MAX_SIZE]; // 帧缓存
	} Rx;

	uint8_t IsFuelOnline;       // 进气流量计是否在线
	uint8_t IsFuelRefresh;      // 进气流量计数据是否已更新
	
	struct {
		float Volume;           // 气量
		float Flow;             // 流量
		float Temperature;      // 温度
		float Density;          // 密度
		float Gain;             // 增益
	} JinQi;
};

// 流量计参数结构体
struct _FLOW_PARA {
	uint8_t Type;               // 流量计类型
	float   Limit;              // 流量上限
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _FLOW_MGR  gFLOW_MGR;
static struct _FLOW_PARA gFLOW_PARA;

// 流量计地址表：液相流量计地址 - 气相流量计地址
static const uint8_t gAddressTable[3][2] = {
	{ 0x01, 0x02 },  // 罗斯蒙特流量计
	{ 0x01, 0x02 },  // E+H流量计
	{ 0x03, 0x04 }   // ADS流量计
};

/*!
****************************************************************************************************
* 本地函数声明
****************************************************************************************************
*/
static void DEVMGR_FlowTxProcess(void);
static void DEVMGR_FlowRxByteCallback(uint8_t data);
static void DEVMGR_FlowRxProcess(uint8_t* data, uint16_t bytes);
static void DEVMGR_FlowBytesToSingle(uint8_t endian, float *value, const uint8_t *data, ssize_t *index);

static void DEVMGR_FlowMCTxProcess(void);
static void DEVMGR_FlowEHTxProcess(void);
static void DEVMGR_FlowADSTxProcess(void);
static void DEVMGR_FlowMCRxProcess(uint8_t *data, uint16_t bytes);
static void DEVMGR_FlowEHRxProcess(uint8_t *data, uint16_t bytes);
static void DEVMGR_FlowADSRxProcess(uint8_t *data, uint16_t bytes);

/*!
****************************************************************************************************
* 接口函数实现
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于初始化流量计设备模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_FlowInit(void)
{
	int i;

	gFLOW_MGR.Idx = 0;
	gFLOW_MGR.Mode = MODE_WORK;
	gFLOW_MGR.IsClrDone = 1;
	gFLOW_MGR.OFlowCnt = 0;
	gFLOW_MGR.OFlowEnable = 1;
	gFLOW_MGR.Rx.Refresh   = false;
	gFLOW_MGR.Rx.Bytes	   = 0;
	gFLOW_MGR.Rx.FrameLen  = 0;
	gFLOW_MGR.IsFuelOnline = false;

	// 读取配置参数
	int err = DEVMGR_45DB081RDMajorWithChk(MEM_ADDR_FS, &gFLOW_PARA, sizeof(gFLOW_PARA));
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_PARA_RD_FAILED);
		// 设置默认值
		gFLOW_PARA.Type	 = DEVID_FLOW_ADS;
		gFLOW_PARA.Limit = CONFIG_FS_DEFAULT_LIMIT;
	}

	DRVMGR_UARTInit(CONFIG_UART_FLOW, CONFIG_UART_FLOW_BAUD, CONFIG_UART_FLOW_PARITY);
	DRVMGR_UARTSetRxCallback(CONFIG_UART_FLOW, DEVMGR_FlowRxByteCallback);

	// 启动数据轮询定时器
	DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW, POLL_PERIOD);
	// 启动过流检测定时器
	DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW_OFLOW, OFLOW_CHK_PERIOD);
	// 启动进气流量计链路超时定时器
	DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW_FUEL_LINK_TIMEOUT, LINK_TIMEOUT);
}

/*!
****************************************************************************************************
* 功能描述：
* 注意事项：
* 输入参数：
* 输出参数：
* 返回参数：
****************************************************************************************************
*/
void DEVMGR_FlowHandle(void)
{
	if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_FLOW_RX_TIMEOUT)) {
		// 停止字节超时定时器
		DRVMGR_MSTimerCancel(MSEC_TMR_ID_FLOW_RX_TIMEOUT);
		gFLOW_MGR.Rx.Bytes = 0; // 清除已经接收的部分数据
	}

	// 等待新接收的帧数据
	if (gFLOW_MGR.Rx.Refresh) {
		DEVMGR_FlowRxProcess(gFLOW_MGR.Rx.FrameBody, gFLOW_MGR.Rx.FrameLen);
		gFLOW_MGR.Rx.Refresh = false;
	}

	// 发送待发送的帧数据
	if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_FLOW)) {
		DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW, POLL_PERIOD);
		if (gFLOW_MGR.Mode == MODE_CLEAR) {
			gFLOW_MGR.ClrCnt += 1;
			if (gFLOW_MGR.ClrCnt > CLR_CNT_LIMIT) {
				gFLOW_MGR.ClrCnt = CLR_CNT_LIMIT;
				gFLOW_MGR.IsClrDone = 1;
				gFLOW_MGR.IsClrSucceed = 0;
				gFLOW_MGR.Mode = MODE_WORK; // 失败后恢复为工作模式
			}
		}
		DEVMGR_FlowTxProcess();
	}

	// 判断过流检测定时器是否到期
	if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_FLOW_OFLOW)) {
		// 重启过流检测定时器
		DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW_OFLOW, OFLOW_CHK_PERIOD);
		// 过流检测使能的情况下检测流量是否过限
		if (!gFLOW_MGR.OFlowEnable) {
			gFLOW_MGR.OFlowCnt = 0;
		} else {
			if (gFLOW_MGR.JinQi.Flow < gFLOW_PARA.Limit) {
				gFLOW_MGR.OFlowCnt = 0;
			} else {
				gFLOW_MGR.OFlowCnt += 1;
				if (gFLOW_MGR.OFlowCnt >= OFLOW_CNT_LIMIT) {
					ERRMGR_MinorErrorSet(MINOR_ERR_OVER_FLOW);
				}
			}
		}
	}

	// 判断进气流量计是否链路超时
	if (DRVMGR_MSTimerIsExpiration(MSEC_TMR_ID_FLOW_FUEL_LINK_TIMEOUT)) {
		// 重启定时器
		DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW_FUEL_LINK_TIMEOUT, LINK_TIMEOUT);
		// 设置进气流量计链路故障
		gFLOW_MGR.IsFuelOnline = false;
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_FUEL_LOST);
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于重置流量传感器参数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：ERROR_NONE -- 执行成功
*           ERROR_FLASH_WR_FAILED -- 参数写入失败
****************************************************************************************************
*/
int DEVMGR_FlowRestoreDefaultPara(void)
{
	// 设置默认值
	gFLOW_PARA.Type	 = DEVID_FLOW_ADS;
	gFLOW_PARA.Limit = CONFIG_FS_DEFAULT_LIMIT;
	// 写入数据
	int err = DEVMGR_45DB081WRWithChk(MEM_ADDR_FS, &gFLOW_PARA, sizeof(gFLOW_PARA));
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_PARA_WR_FAILED);
		return ERROR_FLASH_WR_FAILED;
	}

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询流量传感器类型
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：流量传感器类型
****************************************************************************************************
*/
uint8_t DEVMGR_FlowInqType(void)
{
	return gFLOW_PARA.Type;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询流量传感器限流值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：流量传感器限流值
****************************************************************************************************
*/
float DEVMGR_FlowInqLimit(void)
{
	return gFLOW_PARA.Limit;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置流量传感器类型
* 注意事项：NA
* 输入参数：type -- 类型
* 输出参数：NA
* 返回参数：ERROR_NONE -- 执行成功
*           ERROR_PARA_INVALID -- 参数无效
*           ERROR_FLASH_WR_FAILED -- 参数写入失败
****************************************************************************************************
*/
int DEVMGR_FlowUpdateType(uint8_t type)
{
	// 检查数据有效性
	switch (type) {
	case DEVID_FLOW_ADS:
	case DEVID_FLOW_MICRO:
	case DEVID_FLOW_EH:
		break;
	default:
		return ERROR_PARA_INVALID;
	}

	// 比较数据
	if (gFLOW_PARA.Type == type) {
		return ERROR_NONE;
	}

	// 更新数据
	gFLOW_PARA.Type = type;
	// 写入数据
	int err = DEVMGR_45DB081WRWithChk(MEM_ADDR_FS, &gFLOW_PARA, sizeof(gFLOW_PARA));
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_PARA_WR_FAILED);
		return ERROR_FLASH_WR_FAILED;
	}

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置流量传感器限流值
* 注意事项：NA
* 输入参数：limit -- 限流值
* 输出参数：NA
* 返回参数：ERROR_NONE -- 执行成功
*           ERROR_FLASH_WR_FAILED -- 参数写入失败
****************************************************************************************************
*/
int DEVMGR_FlowUpdateLimit(float limit)
{
	// 比较数据
	if (memcmp(&gFLOW_PARA.Limit, &limit, sizeof(limit)) == 0) {
		return ERROR_NONE;
	}

	// 更新数据
	gFLOW_PARA.Limit = limit;
	// 写入数据
	int err = DEVMGR_45DB081WRWithChk(MEM_ADDR_FS, &gFLOW_PARA, sizeof(gFLOW_PARA));
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_PARA_WR_FAILED);
		return ERROR_FLASH_WR_FAILED;
	}

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置流量传感器类型和限流值
* 注意事项：NA
* 输入参数：type  -- 类型
*           limit -- 限流值
* 输出参数：NA
* 返回参数：ERROR_NONE -- 执行成功
*           ERROR_FLASH_WR_FAILED -- 参数写入失败
****************************************************************************************************
*/
int DEVMGR_FlowUpdatePara(uint8_t type, float limit)
{
	// 更新数据
	gFLOW_PARA.Type = type;
	gFLOW_PARA.Limit = limit;
	// 写入数据
	int err = DEVMGR_45DB081WRWithChk(MEM_ADDR_FS, &gFLOW_PARA, sizeof(gFLOW_PARA));
	if (err) {
		// 更新错误标志
		ERRMGR_MajorErrorSet(MAJOR_ERR_FS_PARA_WR_FAILED);
		return ERROR_FLASH_WR_FAILED;
	}

	return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于使能过流检测
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_FlowOFlowChkEnable(void)
{
	gFLOW_MGR.OFlowEnable = 1;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于失能过流检测
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_FlowOFlowChkDisable(void)
{
	gFLOW_MGR.OFlowEnable = 0;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询进气流量计是否在线
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果进气流量计在线返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_FlowIsFuelOnline(void)
{
	return gFLOW_MGR.IsFuelOnline;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于清除流量计内部总累信息
* 注意事项：仅清除液相流量计
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_FlowClearTotal(void)
{
	gFLOW_MGR.ClrCnt = 0;
	gFLOW_MGR.IsClrDone = 0;
	gFLOW_MGR.IsClrSucceed = 0;
	gFLOW_MGR.Mode = MODE_CLEAR;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否完成清除流量计内部总累信息
* 注意事项：仅清除液相流量计
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果清除完成返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_FlowIsClearDone(void)
{
	return gFLOW_MGR.IsClrDone;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于判断是否成功清除流量计内部总累信息
* 注意事项：仅清除液相流量计
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果清除成功返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_FlowIsClearSucceed(void)
{
	return gFLOW_MGR.IsClrSucceed;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于清除流量计数据刷新标志
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_FlowClrRefresh(void)
{
	gFLOW_MGR.IsFuelRefresh = false;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于查询进气流量计数据是否已刷新
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：如果进气流量计数据已刷新返回TRUE，否则返回FALSE
****************************************************************************************************
*/
bool DEVMGR_FlowIsFuelRefresh(void)
{
	return gFLOW_MGR.IsFuelRefresh;
}

/*!
****************************************************************************************************
* 本地函数
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 功能描述：该方法用于设置进气流量计在线，并重启离线检测定时器
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowSetFuelOnline(void)
{
	gFLOW_MGR.IsFuelOnline = true;
	// 清除故障
	ERRMGR_MajorErrorClear(MAJOR_ERR_FS_FUEL_LOST);
	// 重启定时器
	DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW_FUEL_LINK_TIMEOUT, LINK_TIMEOUT);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于从流量计串口中单字节接收数据
* 注意事项：NA
* 输入参数：data -- 串口接收到的单字节数据
* 输出参数：gFLOW_MGR.Rx -- 接收相关的参数
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowRxByteCallback(uint8_t data)
{
	uint8_t	 condition1; // 条件#1
	uint8_t	 condition2; // 条件#2
	uint16_t frameLen;
	uint16_t crc;
	uint8_t	 crcHiByte;
	uint8_t	 crcLoByte;

	if (gFLOW_MGR.Rx.Bytes < ELEMENTS_OF(gFLOW_MGR.Rx.Buffer)) {
		// 重启接收字节超时定时器
		DRVMGR_MSTimerStart(MSEC_TMR_ID_FLOW_RX_TIMEOUT, RX_BYTE_TIMEOUT);
		gFLOW_MGR.Rx.Buffer[gFLOW_MGR.Rx.Bytes++] = data;
	}

	// 不满足最小帧长度
	if (gFLOW_MGR.Rx.Bytes < RX_FRAME_MIN_SIZE) {
		return;
	}

	condition1 = 0;
	condition1 |= gFLOW_MGR.Rx.Buffer[0] == 0x01;
	condition1 |= gFLOW_MGR.Rx.Buffer[0] == 0x02;
	condition1 |= gFLOW_MGR.Rx.Buffer[0] == 0x03;
	condition1 |= gFLOW_MGR.Rx.Buffer[0] == 0x04;

	condition2 = 0;
	condition2 |= gFLOW_MGR.Rx.Buffer[1] == 0x03;
	condition2 |= gFLOW_MGR.Rx.Buffer[1] == 0x05;
	condition2 |= gFLOW_MGR.Rx.Buffer[1] == 0x06;
	if ((condition1 == 0) || (condition2 == 0)) {
		gFLOW_MGR.Rx.Bytes = 0;
		memset(&gFLOW_MGR.Rx.Buffer[0], 0, sizeof(gFLOW_MGR.Rx.Buffer));
		return;
	}

	// 获取帧长度
	if ((gFLOW_MGR.Rx.Buffer[1] == 0x06) || (gFLOW_MGR.Rx.Buffer[1] == 0x05)) {
		frameLen = 8;
	} else {
		frameLen = gFLOW_MGR.Rx.Buffer[2] + 5;
	}

	// 判断帧长度
	if (frameLen > RX_FRAME_MAX_SIZE) {
		// 长度错误，清零，准备重新接收
		gFLOW_MGR.Rx.Bytes = 0;
		memset(&gFLOW_MGR.Rx.Buffer[0], 0, sizeof(gFLOW_MGR.Rx.Buffer));
		return;
	}

	if (gFLOW_MGR.Rx.Bytes < frameLen) {
		return;
	}

	// 停止字节超时定时器
	DRVMGR_MSTimerCancel(MSEC_TMR_ID_FLOW_RX_TIMEOUT);

	// 计算校验
	crc		  = CRC_Compute(CRC16_MODBUS, gFLOW_MGR.Rx.Buffer, frameLen - 2);
	crc		  = CRC_ComputeComplete(CRC16_MODBUS, crc);
	crcHiByte = (uint8_t)((crc >> 8) & 0xFF);
	crcLoByte = (uint8_t)((crc >> 0) & 0xFF);

	// 判断校验
	if ((gFLOW_MGR.Rx.Buffer[frameLen - 2] != crcLoByte) || (gFLOW_MGR.Rx.Buffer[frameLen - 1] != crcHiByte)) {
		// 校验错误，清零，准备重新接收
		gFLOW_MGR.Rx.Bytes = 0;
		memset(&gFLOW_MGR.Rx.Buffer[0], 0, sizeof(gFLOW_MGR.Rx.Buffer));
		return;
	}

	// 判断上次数据是否已经被处理
	if (gFLOW_MGR.Rx.Refresh == false) {
		gFLOW_MGR.Rx.Refresh  = true;
		gFLOW_MGR.Rx.FrameLen = frameLen;
		memcpy(&gFLOW_MGR.Rx.FrameBody[0], &gFLOW_MGR.Rx.Buffer[0], frameLen);
	}

	// 清除本次接收数据
	gFLOW_MGR.Rx.Bytes = 0;
	memset(&gFLOW_MGR.Rx.Buffer[0], 0, sizeof(gFLOW_MGR.Rx.Buffer));
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowRxProcess(uint8_t* data, uint16_t bytes)
{
	switch (gFLOW_PARA.Type) {
	case DEVID_FLOW_MICRO:
		DEVMGR_FlowMCRxProcess(data, bytes);
		break;
	case DEVID_FLOW_EH:
		DEVMGR_FlowEHRxProcess(data, bytes);
		break;
	case DEVID_FLOW_ADS:
		DEVMGR_FlowADSRxProcess(data, bytes);
		break;
	default:
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowTxProcess(void)
{
	switch (gFLOW_PARA.Type) {
		case DEVID_FLOW_MICRO:
			DEVMGR_FlowMCTxProcess();
			break;
		case DEVID_FLOW_EH:
			DEVMGR_FlowEHTxProcess();
			break;
		case DEVID_FLOW_ADS:
			DEVMGR_FlowADSTxProcess();
			break;
		default:
			break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于将字节数据按照指定的字节序转换为浮点数
* 注意事项：NA
* 输入参数：endian -- 浮点数字节序
*           data   -- 字节数组
*           index  -- 开始转换的索引
* 输出参数：value  -- 转换完成的浮点数
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowBytesToSingle(uint8_t endian, float *value, const uint8_t *data, ssize_t *index)
{
	union {
		uint8_t Buf[4];
		float	Value;
	} u;

	switch (endian) {
	case ENDIAN_1234:
		u.Buf[0] = data[index[0] + 0];
		u.Buf[1] = data[index[0] + 1];
		u.Buf[2] = data[index[0] + 2];
		u.Buf[3] = data[index[0] + 3];
		break;
	case ENDIAN_2143:
		u.Buf[0] = data[index[0] + 1];
		u.Buf[1] = data[index[0] + 0];
		u.Buf[2] = data[index[0] + 3];
		u.Buf[3] = data[index[0] + 2];
		break;
	case ENDIAN_4321:
		u.Buf[0] = data[index[0] + 3];
		u.Buf[1] = data[index[0] + 2];
		u.Buf[2] = data[index[0] + 1];
		u.Buf[3] = data[index[0] + 0];
		break;
	case ENDIAN_3412:
		u.Buf[0] = data[index[0] + 2];
		u.Buf[1] = data[index[0] + 3];
		u.Buf[2] = data[index[0] + 0];
		u.Buf[3] = data[index[0] + 1];
		break;
	}

	value[0] = u.Value;
	index[0] += 4;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送MICRO流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowMCTxProcess(void)
{
	uint16_t txLen;
	uint8_t txBuf[TX_BUF_SIZE];
	uint32_t crcResult;

	switch (gFLOW_MGR.Mode) {
	// 工作模式 ------------------------------------------------------------------------------------
	case MODE_WORK:
		// 发送数据
		txLen = 0;
		txBuf[txLen++] = gAddressTable[DEVID_FLOW_MICRO - 1][gFLOW_MGR.Idx]; // 地址
		txBuf[txLen++] = 0x03;		  // 功能码
		txBuf[txLen++] = 0x00;		  // 起始地址高字节
		txBuf[txLen++] = 0xF6;		  // 起始地址低字节
		txBuf[txLen++] = 0x00;		  // 寄存器个数高字节
		txBuf[txLen++] = 0x2E;		  // 寄存器个数低字节
		crcResult = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
		crcResult = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult >> 8);
		DRVMGR_UARTSendBytes(CONFIG_UART_FLOW, txBuf, txLen);
		// 根据计量模式更新设备地址
		if (gRUNPara.machineType == 1) {
			gFLOW_MGR.Idx = 0;
		} else {
			gFLOW_MGR.Idx += 1;
			if (gFLOW_MGR.Idx >= FLOW_IDX_NUMS) {
				gFLOW_MGR.Idx = 0;
			}
		}
		break;

	// 清零模式 ------------------------------------------------------------------------------------
	case MODE_CLEAR:
		txLen = 0;
		txBuf[txLen++] = gAddressTable[DEVID_FLOW_MICRO - 1][0]; // 地址
		txBuf[txLen++] = 0x06;				 // 功能码
		txBuf[txLen++] = 0x00;				 // 起始地址高字节
		txBuf[txLen++] = 0x07;				 // 起始地址低字节
		txBuf[txLen++] = 0x00;				 // 寄存器个数高字节
		txBuf[txLen++] = 0x00;				 // 寄存器个数低字节
		crcResult = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
		crcResult = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult >> 8);
		DRVMGR_UARTSendBytes(CONFIG_UART_FLOW, txBuf, txLen);
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送E+H流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowEHTxProcess(void)
{
	uint16_t txLen;
	uint8_t txBuf[TX_BUF_SIZE];
	uint32_t crcResult;

	switch (gFLOW_MGR.Mode) {
	// 工作模式 ------------------------------------------------------------------------------------
	case MODE_WORK:
		// 发送数据
		txLen = 0;
		txBuf[txLen++] = gAddressTable[DEVID_FLOW_EH - 1][gFLOW_MGR.Idx]; // 地址
		txBuf[txLen++] = 0x03;		  // 功能码
		txBuf[txLen++] = 0x13;		  // 起始地址高字节
		txBuf[txLen++] = 0xBA;		  // 起始地址低字节
		txBuf[txLen++] = 0x00;		  // 寄存器个数高字节
		txBuf[txLen++] = 0x0C;		  // 寄存器个数低字节
		crcResult = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
		crcResult = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult >> 8);
		DRVMGR_UARTSendBytes(CONFIG_UART_FLOW, txBuf, txLen);
		// 根据计量模式更新设备地址
		if (gRUNPara.machineType == 1) {
			gFLOW_MGR.Idx = 0;
		} else {
			gFLOW_MGR.Idx += 1;
			if (gFLOW_MGR.Idx >= FLOW_IDX_NUMS) {
				gFLOW_MGR.Idx = 0;
			}
		}
		break;

	// 清零模式 ------------------------------------------------------------------------------------
	case MODE_CLEAR:
		txLen = 0;
		txBuf[txLen++] = gAddressTable[DEVID_FLOW_EH - 1][0]; // 地址
		txBuf[txLen++] = 0x06;				 // 功能码
		txBuf[txLen++] = 0x0A;				 // 起始地址高字节
		txBuf[txLen++] = 0x2F;				 // 起始地址低字节
		txBuf[txLen++] = 0x00;				 // 寄存器个数高字节
		txBuf[txLen++] = 0x01;				 // 寄存器个数低字节
		crcResult = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
		crcResult = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult >> 8);
		DRVMGR_UARTSendBytes(CONFIG_UART_FLOW, txBuf, txLen);
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理发送ADS流量计轮询请求命令
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowADSTxProcess(void)
{
	uint16_t txLen;
	uint8_t txBuf[TX_BUF_SIZE];
	uint32_t crcResult;

	switch (gFLOW_MGR.Mode) {
	// 工作模式 ------------------------------------------------------------------------------------
	case MODE_WORK:
		// 发送数据
		txLen = 0;
		txBuf[txLen++] = gAddressTable[DEVID_FLOW_ADS - 1][gFLOW_MGR.Idx]; // 地址
		txBuf[txLen++] = 0x03;		  // 功能码
		txBuf[txLen++] = 0x00;		  // 起始地址高字节
		txBuf[txLen++] = 0x1C;		  // 起始地址低字节
		txBuf[txLen++] = 0x00;		  // 寄存器个数高字节
		txBuf[txLen++] = 0x0A;		  // 寄存器个数低字节
		crcResult = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
		crcResult = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult >> 8);
		DRVMGR_UARTSendBytes(CONFIG_UART_FLOW, txBuf, txLen);
		// 根据计量模式更新设备地址
		if (gRUNPara.machineType == 1) {
			gFLOW_MGR.Idx = 0;
		} else {
			gFLOW_MGR.Idx += 1;
			if (gFLOW_MGR.Idx >= FLOW_IDX_NUMS) {
				gFLOW_MGR.Idx = 0;
			}
		}
		break;

	// 清零模式 ------------------------------------------------------------------------------------
	case MODE_CLEAR:
		txLen = 0;
		txBuf[txLen++] = gAddressTable[DEVID_FLOW_ADS - 1][0]; // 地址
		txBuf[txLen++] = 0x06;				 // 功能码
		txBuf[txLen++] = 0x07;				 // 起始地址高字节
		txBuf[txLen++] = 0xD6;				 // 起始地址低字节
		txBuf[txLen++] = 0x01;				 // 寄存器个数高字节
		txBuf[txLen++] = 0x00;				 // 寄存器个数低字节
		crcResult = CRC_Compute(CRC16_MODBUS, txBuf, txLen);
		crcResult = CRC_ComputeComplete(CRC16_MODBUS, crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult);
		txBuf[txLen++] = (uint8_t) (crcResult >> 8);
		DRVMGR_UARTSendBytes(CONFIG_UART_FLOW, txBuf, txLen);
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理艾默生流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowMCRxProcess(uint8_t *data, uint16_t bytes)
{
	struct {
		float Volume;
		float Flow;
		float Temperature;
		float Density;
		float Gain;
	} response;

	ssize_t index;
	uint8_t addr = data[0];		// 地址
	uint8_t funcCode = data[1];	// 功能码
	uint8_t dataLen  = data[2];	// 数据长度
	uint8_t *dataBody = &data[3];	// 数据内容

	if (addr == gAddressTable[DEVID_FLOW_MICRO - 1][0]) {
		// 更新液相流量计在线标志
		DEVMGR_FlowSetFuelOnline();
	}

	switch (funcCode) {
	case 3:
		if (dataLen > 20) {
			index = 0;
			// 流量
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Flow, dataBody, &index);
			// 密度
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Density, dataBody, &index);
			// 温度
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Temperature, dataBody, &index);
			index = 24;
			// 质量
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Volume, dataBody, &index);
			index = 88;
			// 增益
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Gain, dataBody, &index);

			// 更新液相流量计数据
			if (addr == gAddressTable[DEVID_FLOW_MICRO - 1][0]) {
				// 更新数据刷新标志
				gFLOW_MGR.IsFuelRefresh = true;
				// 更新数据内容
				gFLOW_MGR.JinQi.Volume = response.Volume;
				gFLOW_MGR.JinQi.Flow = response.Flow;
				gFLOW_MGR.JinQi.Temperature = response.Temperature;
				gFLOW_MGR.JinQi.Density = response.Density;
				gFLOW_MGR.JinQi.Gain = response.Gain;
			}
		}
		break;

	case 6: // 清零成功
		gFLOW_MGR.IsClrDone = 1;
		gFLOW_MGR.IsClrSucceed = 1;
		gFLOW_MGR.Mode = MODE_WORK;
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理E+H流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowEHRxProcess(uint8_t *data, uint16_t bytes)
{
	struct {
		float Volume;
		float Flow;
		float Temperature;
		float Density;
		float Gain;
	} response;

	ssize_t index;
	uint8_t addr = data[0];		// 地址
	uint8_t funcCode = data[1];	// 功能码
	uint8_t dataLen  = data[2];	// 数据长度
	uint8_t *dataBody = &data[3];	// 数据内容

	if (addr == gAddressTable[DEVID_FLOW_EH - 1][0]) {
		// 更新液相流量计在线标志
		DEVMGR_FlowSetFuelOnline();
	}

	switch (funcCode) {
	case 3:
		if (dataLen == 24) {
			index = 0;
			// 质量
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Volume, dataBody, &index);
			// 流量
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Flow, dataBody, &index);
			// 密度
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Density, dataBody, &index);
			// 温度
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Temperature, dataBody, &index);
			// 增益
			DEVMGR_FlowBytesToSingle(ENDIAN_2143, &response.Gain, dataBody, &index);

			// 更新液相流量计数据
			if (addr == gAddressTable[DEVID_FLOW_EH - 1][0]) {
				// 更新数据刷新标志
				gFLOW_MGR.IsFuelRefresh = true;
				// 更新数据内容
				gFLOW_MGR.JinQi.Volume = response.Volume;
				gFLOW_MGR.JinQi.Flow = response.Flow;
				gFLOW_MGR.JinQi.Temperature = response.Temperature;
				gFLOW_MGR.JinQi.Density = response.Density;
				gFLOW_MGR.JinQi.Gain = response.Gain;
			}
		}
		break;

	case 6: // 清零成功
		gFLOW_MGR.IsClrDone = 1;
		gFLOW_MGR.IsClrSucceed = 1;
		gFLOW_MGR.Mode = MODE_WORK;
		break;
	}
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理ADS流量计通信数据
* 注意事项：NA
* 输入参数：data  -- 数据缓存
*           bytes -- 数据缓存字节数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
static void DEVMGR_FlowADSRxProcess(uint8_t *data, uint16_t bytes)
{
	struct {
		float Volume;
		float Flow;
		float Temperature;
		float Density;
		float Gain;
	} response;

	ssize_t index;
	uint8_t addr = data[0];		// 地址
	uint8_t funcCode = data[1];	// 功能码
	uint8_t dataLen  = data[2];	// 数据长度
	uint8_t *dataBody = &data[3];	// 数据内容

	if (addr == gAddressTable[DEVID_FLOW_ADS - 1][0]) {
		// 更新液相流量计在线标志
		DEVMGR_FlowSetFuelOnline();
	}

	switch (funcCode) {
	case 3:
		if (dataLen == 20) {
			index = 0;
			// 流量
			DEVMGR_FlowBytesToSingle(ENDIAN_1234, &response.Flow, dataBody, &index);
			// 温度
			DEVMGR_FlowBytesToSingle(ENDIAN_1234, &response.Temperature, dataBody, &index);
			// 质量
			DEVMGR_FlowBytesToSingle(ENDIAN_1234, &response.Volume, dataBody, &index);
			// 密度
			DEVMGR_FlowBytesToSingle(ENDIAN_1234, &response.Density, dataBody, &index);
			// 增益
			DEVMGR_FlowBytesToSingle(ENDIAN_1234, &response.Gain, dataBody, &index);

			// 更新液相流量计数据
			if (addr == gAddressTable[DEVID_FLOW_ADS - 1][0]) {
				// 更新数据刷新标志
				gFLOW_MGR.IsFuelRefresh = true;
				// 更新数据内容
				gFLOW_MGR.JinQi.Volume = response.Volume;
				gFLOW_MGR.JinQi.Flow = response.Flow;
				gFLOW_MGR.JinQi.Temperature = response.Temperature;
				gFLOW_MGR.JinQi.Density = response.Density;
				gFLOW_MGR.JinQi.Gain = response.Gain;
			}
		}
		break;

	case 6: // 清零成功
		gFLOW_MGR.IsClrDone = 1;
		gFLOW_MGR.IsClrSucceed = 1;
		gFLOW_MGR.Mode = MODE_WORK;
		break;
	}
}
