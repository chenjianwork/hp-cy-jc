/*
 * sysmanager.h
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */

#ifndef SYSMANAGER_SYSMANAGER_H_
#define SYSMANAGER_SYSMANAGER_H_

#include <string.h>
#include <hqhp/config.h>
#include <hqhp/defs.h>
#include <hqhp/drvmanager.h>
#include <hqhp/devmanager.h>
#include <hqhp/dbgmanager.h>
#include <hqhp/taskmanager.h>
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"
#include "errmanager/errmanager.h"
#include "commanager/commanager.h"
#include "stm32f4xx.h"


// Flash扇区地址定义
#define ADDR_FLASH_SECTOR_0   ((u32)0x08000000)   // 扇区0起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_1   ((u32)0x08004000)   // 扇区1起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_2   ((u32)0x08008000)   // 扇区2起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_3   ((u32)0x0800C000)   // 扇区3起始地址, 16 Kbytes   flag
#define ADDR_FLASH_SECTOR_4   ((u32)0x08010000)   // 扇区4起始地址, 64 Kbytes   adc data
#define ADDR_FLASH_SECTOR_5   ((u32)0x08020000)   // 扇区5起始地址, 128 Kbytes  PLC code
#define ADDR_FLASH_SECTOR_6   ((u32)0x08040000)   // 扇区6起始地址, 128 Kbytes  APP
#define ADDR_FLASH_SECTOR_7   ((u32)0x08060000)   // 扇区7起始地址, 128 Kbytes  APP
#define ADDR_FLASH_SECTOR_8   ((u32)0x08080000)   // 扇区8起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9   ((u32)0x080A0000)   // 扇区9起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10  ((u32)0x080C0000)   // 扇区10起始地址,128 Kbytes
#define ADDR_FLASH_SECTOR_11  ((u32)0x080E0000)   // 扇区11起始地址,128 Kbytes


// Flash存储相关定义
#define DATA_FLASH_SAVE_NUM   (2)     // 存储数据个数
#define FLASH_SAVE_ADDR       ADDR_FLASH_SECTOR_4  // 扇区有64kb的大小，一般存几个数据已经足够
#define PARA_PID_SAVE_ADDR    ADDR_FLASH_SECTOR_6	//PID参数存储位置
#define PARA_MAIN_SAVE_ADDR   ADDR_FLASH_SECTOR_7	//其他主要参数存储


//#define PRESSURE_REDUCE_GAIN  (75)  //减压阀放大倍数

//#define PROPORT_VALVE_WORK_MAX  (VREFIN/SAMPLE_R)//比例阀最大工作电流,约20mA
//#define PROPORT_VALVE_WORK_MIN  (4)   //比例阀最小工作电流4mA

// 运行状态 ------------------------------------------------------------------------------------- //
enum {
	RUNSTATE_IDLE = 0, 			// 空闲状态
	RUNSTATE_RUNNING,		  // 备机状态
	RUNSTATE_BLAST, 		  // 吹扫状态

};

#define WORK_DONE  0x00 // 完成状态
#define WORK_INIT  0x10 // 初始状态
#define WORK_EXIT  0x20 // 退出状态
#define WORK_DOING 0x30 // 工作状态

// 此结构体存储变频器的各项参数，包括比例、积分、微分及其它相关参数
#pragma pack(4)  // 设置4字节对齐
struct _RUN_PARA {
	//PID参数
	struct _PID_PARA pid_para;  // PID参数结构体

	//压力变送器参数	PT207
	float	 Range_PT207;	   // 量程，单位MPa
	float	 Limit_PT207;	   // 过压保护，单位MPa
	float	 Ratio_PT207;	   // 原始采样值缩放系数，无量纲
	float	 Delta_PT207;	   // 原始采样值偏移系数，单位MPa
	uint32_t OverTime_PT207; // 过压时间，单位秒

	//压力变送器参数	PT206
	float	 Range_PT206;	   // 量程，单位MPa
	float	 Limit_PT206;	   // 过压保护，单位MPa
	float	 Ratio_PT206;	   // 原始采样值缩放系数，无量纲
	float	 Delta_PT206;	   // 原始采样值偏移系数，单位MPa
	uint32_t OverTime_PT206; // 过压时间，单位秒

	//流量计参数ONE
	uint32_t Type_ONE;               // 流量计类型
	float   Limit_ONE;              // 流量上限

	//流量计参数TWO	
	uint32_t Type_TWO;               // 流量计类型
	float   Limit_TWO;              // 流量上限
};
#pragma pack()   // 恢复默认对齐方式


/* 运行信息定义 --------------------------------------------------------------------------------- */
struct _RUN_INFO {
	uint8_t	 RunMode;			// 工作模式
	uint8_t	 RunState;			// 运行状态
	uint8_t	 LastRunState;		// 上次运行状态
};

//MEOH_DATA新增变量定义   甲醇流量计DATA
struct MEOH_DATA {
	float jinQi_WenDu;
	float jinQi_ZengYi;
	float jinQi_MiDu;
	float jinQi_LiuLiang;
	float jinQi_ZhiLiang;
	float jinQi_StartZhiLiang;
	float jinQi_EndZhiLiang;
};

uint16_t DATA_REC_COMPLETE1,DATA_REC_COMPLETE2;
unsigned int Digital_Input_Data;
extern void DRVMGR_TimerDelayUs(uint16_t us);
_Engine_DATA Engine_Parameter_Host;
_Engine_DATA Engine_Parameter_Auxiliary_1;


/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
extern struct _RUN_INFO 		gRUNInfo;
extern struct _RUN_PARA 		gRUNPara;
/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/

/*!
****************************************************************************************************
* sysmanager.c
****************************************************************************************************
*/
void 	SYSMGR_Init(void);
void 	SYSMGR_Init_Flag(void);
void 	SYSMGR_Handle(void);
uint8_t SYSMGR_InqRunMode(void);
void 	SYSMGR_SetRunMode(uint8_t runMode);
uint8_t SYSMGR_InqLastRunState(void);
uint8_t SYSMGR_InqRunState(void);
bool 	SYSMGR_SetRunState(uint8_t runState);
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);
void 	SYSMGR_Params_Init(uint32_t flash_addr);
void 	SYSMGR_Params_Restore(struct _RUN_PARA *params);
void 	SYSMGR_Read_Flash_Params(struct _RUN_PARA *params, uint32_t flash_addr);
int 	SYSMGR_Write_Flash_Params(struct _RUN_PARA *params, uint32_t flash_addr);



/*!
****************************************************************************************************
* sysmanager-inoperative.c
****************************************************************************************************
*/

bool 	SYSMGR_InoperativeInit(void);
bool 	SYSMGR_InoperativeIsDone(void);
void 	SYSMGR_InoperativeHandle(void);

/*!
****************************************************************************************************
* sysmanager-idle.c
****************************************************************************************************
*/

bool 	SYSMGR_IdleInit(void);
bool 	SYSMGR_IdleIsDone(void);
void 	SYSMGR_IdleHandle(void);

/*!
****************************************************************************************************
* sysmanager-running.c
****************************************************************************************************
*/
bool 	SYSMGR_RunningInit(void);
bool 	SYSMGR_RunningIsDone(void);
void 	SYSMGR_RunningHandle(void);


/*PID-DEBUG*/

/*!
****************************************************************************************************
* 功能描述：查询PID参数相关函数
****************************************************************************************************
*/
float 	SYSMGR_Para_PIDSP(void);
float 	SYSMGR_Para_PIDErr(void);
float 	SYSMGR_Para_PIDOutput(void);
float 	SYSMGR_Para_PIDSum(void);
float 	SYSMGR_Para_PIDKP(void);
float 	SYSMGR_Para_PIDKPMax(void);
float 	SYSMGR_Para_PIDKI(void);
float 	SYSMGR_Para_PIDKIMax(void);
float 	SYSMGR_Para_PIDKD(void);
float 	SYSMGR_Para_PIDKDMax(void);
float 	SYSMGR_Para_PIDSumMax(void);
float 	SYSMGR_Para_PIDErrMin(void);
float 	SYSMGR_Para_PIDErrMax(void);
float 	SYSMGR_Para_PIDOutMin(void);
float 	SYSMGR_Para_PIDOutMax(void);

/*!
****************************************************************************************************
* 功能描述：设置PID参数相关函数
****************************************************************************************************
*/
void 	SYSMGR_Para_SetPIDKP(float kp);
void 	SYSMGR_Para_SetPIDKPMax(float kpmax);
void 	SYSMGR_Para_SetPIDKI(float ki);
void 	SYSMGR_Para_SetPIDKIMax(float kimax);
void 	SYSMGR_Para_SetPIDKD(float kd);
void 	SYSMGR_Para_SetPIDKDMax(float kdmax);
void 	SYSMGR_Para_SetPIDSumMax(float summax);
void 	SYSMGR_Para_SetPIDErrMin(float errmin);
void 	SYSMGR_Para_SetPIDErrMax(float errmax);
void 	SYSMGR_Para_SetPIDOutMin(float outmin);
void 	SYSMGR_Para_SetPIDOutMax(float outmax);
float 	SYSMGR_Running_InqPIDSP(void);
float 	SYSMGR_Running_InqPIDErr(void);
float 	SYSMGR_Running_InqPIDOutput(void);
float 	SYSMGR_Running_InqPIDSum(void);




/*!
****************************************************************************************************
* 功能描述：查询流量计参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_Range_PT207(void);
float SYSMGR_Para_Limit_PT207(void);
float SYSMGR_Para_Ratio_PT207(void);
float SYSMGR_Para_Delta_PT207(void);
float SYSMGR_Para_OverTime_PT207(void);
float SYSMGR_Para_Range_PT206(void);
float SYSMGR_Para_Limit_PT206(void);
float SYSMGR_Para_Ratio_PT206(void);
float SYSMGR_Para_Delta_PT206(void);
float SYSMGR_Para_OverTime_PT206(void);
uint32_t SYSMGR_Para_Type_ONE(void);  // 修改返回类型为uint32_t
float SYSMGR_Para_Limit_ONE(void);
uint32_t SYSMGR_Para_Type_TWO(void);  // 修改返回类型为uint32_t
float SYSMGR_Para_Limit_TWO(void);
/*!
****************************************************************************************************
* 功能描述：设置PID参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetPIDKP(float kp);
void SYSMGR_Para_SetPIDKPMax(float kpmax);
void SYSMGR_Para_SetPIDKI(float ki);
void SYSMGR_Para_SetPIDKIMax(float kimax);
void SYSMGR_Para_SetPIDKD(float kd);
void SYSMGR_Para_SetPIDKDMax(float kdmax);
void SYSMGR_Para_SetPIDSumMax(float summax);
void SYSMGR_Para_SetPIDErrMin(float errmin);
void SYSMGR_Para_SetPIDErrMax(float errmax);
void SYSMGR_Para_SetPIDOutMin(float outmin);
void SYSMGR_Para_SetPIDOutMax(float outmax);
/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT207参数相关函数
****************************************************************************************************
*/

void SYSMGR_Para_SetRange_PT207(float range);
void SYSMGR_Para_SetLimit_PT207(float limit);
void SYSMGR_Para_SetRatio_PT207(float ratio);
void SYSMGR_Para_SetDelta_PT207(float delta);
void SYSMGR_Para_SetOverTime_PT207(float over_time);

/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT206参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetRange_PT206(float range);
void SYSMGR_Para_SetLimit_PT206(float limit);
void SYSMGR_Para_SetRatio_PT206(float ratio);
void SYSMGR_Para_SetDelta_PT206(float delta);
void SYSMGR_Para_SetOverTime_PT206(float over_time);
/*!
****************************************************************************************************
* 功能描述：设置流量计参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetType_ONE(uint32_t type);
void SYSMGR_Para_SetLimit_ONE(float limit);
void SYSMGR_Para_SetType_TWO(uint32_t type);
void SYSMGR_Para_SetLimit_TWO(float limit);

/*!
****************************************************************************************************
* sysmanager-stopped.c
****************************************************************************************************
*/
bool 	SYSMGR_StoppedInit(void);
bool 	SYSMGR_StoppedIsDone(void);
void 	SYSMGR_StoppedHandle(void);

/*!
****************************************************************************************************
* sysmanager-blast.c
****************************************************************************************************
*/
bool 	SYSMGR_BlastInit(void);
bool 	SYSMGR_BlastIsDone(void);
void 	SYSMGR_BlastHandle(void);



#endif /* SYSMANAGER_SYSMANAGER_H_ */
