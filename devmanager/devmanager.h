/*
 * devmanager.h
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */

#ifndef DEVMANAGER_DEVMANAGER_H_
#define DEVMANAGER_DEVMANAGER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif


/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define UP_PID_MAX (5)   //上传PID参数最大数量
/*!
****************************************************************************************************
* 配置定义
****************************************************************************************************
*/
/* 流量传感器 ----------------------------------------------------------------------------------- */
#define CONFIG_PUMP_NO_MIN          (1)     // 允许的枪号最小值（包括该值）
#define CONFIG_PUMP_NO_MAX          (200)   // 允许的枪号最大值（包括该值）
#define CONFIG_DEBUG_AUTOEXIT_TIME  (300)   // 维护模式下自动退出的时间，单位秒
#define CONFIG_MAX_WAIT_DUETIME     (9999)  // 等待启动加注的时间，单位秒

/*
 * PID
 * */
#define VREFIN	 (2435)     // 参考电压，单位：mV
#define SAMPLE_R (124.0)	// 取样电阻，单位：Ω
#define DAC_RESOLUTION          (65534)//DAC 16位分辨率 2^16 - 1 = 65534 未实测

// ----------------- 常量定义 -----------------
#define VALVE_CURRENT_MIN   4.0f    // 阀门电流最小值 (mA)
#define VALVE_CURRENT_MAX   20.0f   // 阀门电流最大值 (mA)
#define VFD_FREQ_MIN_HZ     0.0f    // 变频器最小频率 (Hz)
#define VFD_FREQ_MAX_HZ     100.0f  // 变频器最大频率 (Hz)
#define VFD_FREQ_BASE       50.0f   // 变频器基础频率 (Hz)
#define PID_MAX_SUM         1000.0f // PID积分限幅
#define PID_MIN_SUM        -1000.0f // PID积分限幅
#define PID_DEADBAND        0.1f    // PID死区
#define PRESSURE_ERR_THRESHOLD 0.1f // 压力偏差阈值(MPa)

// 压力控制范围定义
#define PRESSURE_LOW_THRESHOLD  0.6f  // 低压阈值(MPa)
#define PRESSURE_HIGH_THRESHOLD 1.0f  // 高压阈值(MPa)

// 输出限幅和速率限制
#define VALVE_RATE_LIMIT      0.5f   // 阀门电流变化率限制(mA/周期)
#define VFD_RATE_LIMIT        2.0f   // 变频器频率变化率限制(Hz/周期)
#define VFD_DEADBAND          1.0f   // 变频器死区(Hz)


/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
/* 流量传感器 ----------------------------------------------------------------------------------- */
// 流量计定时器枚举定义
enum {
    MSEC_TMR_ID_FLOW,                    // 流量计轮询定时器
    MSEC_TMR_ID_FLOW_OFLOW,              // 过流检测定时器
    MSEC_TMR_ID_FLOW_FUEL_LINK_TIMEOUT,  // 进气流量计链路超时定时器
    MSEC_TMR_ID_FLOW_RX_TIMEOUT,         // 接收字节超时定时器
};

// 流量计类型枚举定义
enum {
    DEVID_FLOW_MICRO = 1,  // 艾默生-罗斯蒙特
    DEVID_FLOW_EH,         // E+H
    DEVID_FLOW_ADS,        // 安迪生
};

/* PID控制器 ----------------------------------------------------------------------------------- */
enum {
    PID_SP = 1,         // 设定值
    PID_ERR,            // 误差值
    PID_OUTPUT,         // 输出值
    PID_SUM,            // 积分值
    PID_KP,             // 比例输出值
    PID_KI,             // 积分输出值
    PID_KD,             // 微分输出值
};

struct _PID_PARA {
    float KP;      // 比例系数
    float KPMax;   // 比例输出最大值
    float KI;      // 积分系数
    float KIMax;   // 积分输出最大值
    float KD;      // 微分系数
    float KDMax;   // 微分输出最大值
    float SumMax;  // 积分计算最大值
    float ErrMin;  // 误差最小值
    float ErrMax;  // 误差最大值
    float OutMin;  // 输出最小值
    float OutMax;  // 输出最大值
};

struct _PID {
    struct _PID_PARA Para;
    float           Sp;     // 设定值（SetPoint）
    float           Err[3]; // 误差：2-T2（前前时刻）误差，1-T1（前一时刻）误差，0-T0（当前时刻）误差
    float           Sum;    // 积分
    float           Out;    // 输出值
};

/*!
****************************************************************************************************
* 函数声明
****************************************************************************************************
*/
/* 流量计模块 ----------------------------------------------------------------------------------- */
// 初始化函数
void DEVMGR_FlowInit(void);
void DEVMGR_FlowHandle(void);

// 参数管理函数
int DEVMGR_FlowRestoreDefaultPara(void);
uint8_t DEVMGR_FlowInqType(void);
float DEVMGR_FlowInqLimit(void);
int DEVMGR_FlowUpdateType(uint8_t type);
int DEVMGR_FlowUpdateLimit(float limit);
int DEVMGR_FlowUpdatePara(uint8_t type, float limit);

// 过流检测控制函数
void DEVMGR_FlowOFlowChkEnable(void);
void DEVMGR_FlowOFlowChkDisable(void);

// 状态查询函数
bool DEVMGR_FlowIsFuelOnline(void);
bool DEVMGR_FlowIsClearDone(void);
bool DEVMGR_FlowIsClearSucceed(void);
bool DEVMGR_FlowIsFuelRefresh(void);

// 数据操作函数
void DEVMGR_FlowClearTotal(void);
void DEVMGR_FlowClrRefresh(void);

// 数据查询函数
float DEVMGR_FlowInqJinQiFlow(void);
float DEVMGR_FlowInqJinQiVolume(void);
float DEVMGR_FlowInqJinQiDensity(void);
float DEVMGR_FlowInqJinQiGain(void);
float DEVMGR_FlowInqJinQiTemperature(void);

/* PID控制器模块 --------------------------------------------------------------------------------- */
// 初始化函数
void PIDInit(struct _PID* d);

// 计算函数
float PIDCompute(struct _PID* d, float err);

// 获取函数
float PIDGetSP(struct _PID* d);
float PIDGetOutput(struct _PID* d);
float PIDGetKP(struct _PID* d);
float PIDGetKI(struct _PID* d);
float PIDGetKD(struct _PID* d);
float PIDGetKPMax(struct _PID* d);
float PIDGetKIMax(struct _PID* d);
float PIDGetKDMax(struct _PID* d);
float PIDGetSumMax(struct _PID* d);
float PIDGetErrMax(struct _PID* d);
float PIDGetErrMin(struct _PID* d);
float PIDGetOutMax(struct _PID* d);
float PIDGetOutMin(struct _PID* d);

// 设置函数
void PIDSetSP(struct _PID* d, float sp);
void PIDSetKP(struct _PID* d, float kp);
void PIDSetKI(struct _PID* d, float ki);
void PIDSetKD(struct _PID* d, float kd);
void PIDSetKPMax(struct _PID* d, float max);
void PIDSetKIMax(struct _PID *d, float max);
void PIDSetKDMax(struct _PID *d, float max);
void PIDSetSumMax(struct _PID *d, float max);
void PIDSetErrMax(struct _PID *d, float max);
void PIDSetErrMin(struct _PID *d, float min);
void PIDSetOutMax(struct _PID *d, float max);
void PIDSetOutMin(struct _PID *d, float min);


void PID_InitCascade(struct _PID* outer_pid,
                    struct _PID* inner_pid,
                    float outer_kp, float outer_ki, float outer_kd,
                    float inner_kp, float inner_ki, float inner_kd);
float PID_ComputeCascade(struct _PID* outer_pid,
                        struct _PID* inner_pid,
                        float pressure_sp,
                        float pressure_fb,
                        float* valve_output,
                        float* vfd_output);
void PID_ResetCascade(struct _PID* outer_pid, struct _PID* inner_pid);
uint8_t DEVMGR_ValveSetCurrent(uint8_t chN, float current);
void PIDInit(struct _PID* d);
float PIDCompute(struct _PID* d, float fbVal);




/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
/*!
****************************************************************************************************
* devmanager.c
****************************************************************************************************
*/
void 	DEVMGR_Init(void);
void 	DEVMGR_Handle(void);

/*!
****************************************************************************************************
* devmananger-ps.c
****************************************************************************************************
*/
void 	DEVMGR_PSInit(void);
void 	DEVMGR_PSHandle(void);
bool 	DEVMGR_PSIsLost(void);
float 	DEVMGR_PSInqValue(void);


uint8_t DEVMGR_ValveSetCurrent(uint8_t chN, float current);

#ifdef __cplusplus
}
#endif

#endif /* DEVMANAGER_DEVMANAGER_H_ */
