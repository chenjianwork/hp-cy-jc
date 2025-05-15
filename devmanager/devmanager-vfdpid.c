/*!
****************************************************************************************************
* 文件名称：devmanager-vfdpid.c
* 功能简介：PID算法模块实现，包含级联PID控制
* 文件作者：LUDONGDONG
* 创建日期：2025-03-16
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <math.h>

#include "commanager/commanager.h"
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"
#include "drvmanager/drvmanager-ads1120.h"
#include "drvmanager/drvmanager-dac8552.h"
/**
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
// 压力控制相关常量
#define PRESSURE_SP         0.8f    // 压力设定值(MPa)

// PID控制相关常量

/**
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/


/*!
* @brief 速率限制器结构体
*/
typedef struct {
    float last_valve_output;  // 上一次阀门输出(mA)
    float last_vfd_output;    // 上一次变频器输出(Hz)
    float valve_rate_limit;   // 阀门速率限制(mA/周期)
    float vfd_rate_limit;     // 变频器速率限制(Hz/周期)
} Rate_Limiter_t;

/**
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
//static struct _VFD_INFO gVFDInfo;
static Rate_Limiter_t gRateLimiter = {
    .last_valve_output = VALVE_CURRENT_MIN,
    .last_vfd_output = VFD_FREQ_BASE,
    .valve_rate_limit = VALVE_RATE_LIMIT,
    .vfd_rate_limit = VFD_RATE_LIMIT
};

/**
****************************************************************************************************
* 函数声明
****************************************************************************************************
*/

static float saturate(float value, float min, float max);
static float apply_rate_limit(float current, float last, float rate_limit);

/**
****************************************************************************************************
* 函数实现
****************************************************************************************************
*/

/*!

****************************************************************************************************
* @brief 值限幅函数
* @param value 输入值
* @param min   最小值
* @param max   最大值
* @return float 限幅后的值
****************************************************************************************************
*/
static float saturate(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/*!
****************************************************************************************************
* @brief 应用速率限制
* @param current    当前值
* @param last       上一次值
* @param rate_limit 速率限制
* @return float 限制后的值
****************************************************************************************************
*/
static float apply_rate_limit(float current, float last, float rate_limit)
{
    float delta = current - last;
    if (delta > rate_limit) {
        return last + rate_limit;
    } else if (delta < -rate_limit) {
        return last - rate_limit;
    }
    return current;
}

/*!
****************************************************************************************************
* @brief 级联PID控制计算，实现分裂范围控制
* @param outer_pid   外环PID控制器(压力环)
* @param inner_pid   内环PID控制器(频率环)
* @param pressure_sp 压力设定值(MPa)
* @param pressure_fb 压力反馈值(MPa)
* @param valve_output 阀门输出电流指针(4-20mA)
* @param vfd_output   变频器输出频率指针(Hz)
* @return float 阀门开度设定值(0-100%)
****************************************************************************************************
*/
float DEVMGR_PID_ComputeCascade(struct _PID* outer_pid,
                        struct _PID* inner_pid,
                        float pressure_sp,
                        float pressure_fb,
                        float* valve_output,
                        float* vfd_output)
{
    // 空指针保护
    if (!outer_pid || !inner_pid || !valve_output || !vfd_output) {
        return 0.0f;
    }

    float pressure_err = pressure_sp - pressure_fb;
    float valve_sp = 0.0f;
    float freq_sp = VFD_FREQ_BASE;
    float valve_current;
    float freq_out;

    // ---------- 分裂范围控制 ----------
    if (pressure_fb < PRESSURE_LOW_THRESHOLD) {
        // 低压区域：主要使用变频器，阀门保持最小开度
        valve_sp = 0.0f;
        freq_sp = VFD_FREQ_MAX_HZ;
    }
    else if (pressure_fb > PRESSURE_HIGH_THRESHOLD) {
        // 高压区域：主要使用阀门，变频器保持基础频率
        valve_sp = 100.0f;
        freq_sp = VFD_FREQ_BASE;
    }
    else {
        // 中压区域：协调控制
        // 外环PID计算阀门开度
        DEVMGR_PID_SetSP(outer_pid, pressure_sp);
        valve_sp = DEVMGR_PID_Compute(outer_pid, pressure_fb);  // PID内部会处理死区
        valve_sp = saturate(valve_sp, 0.0f, 100.0f);

        // 内环PID微调变频器频率
        float freq_err = (pressure_err > 0) ? 
                       (VFD_FREQ_MAX_HZ - VFD_FREQ_BASE) : 
                       (VFD_FREQ_BASE - VFD_FREQ_MIN_HZ);
        DEVMGR_PID_SetSP(inner_pid, freq_sp);
        float freq_adjust = DEVMGR_PID_Compute(inner_pid, freq_err);  // PID内部会处理死区
        freq_sp += freq_adjust;
    }

    // ---------- 阀门输出计算 ----------
    valve_current = VALVE_CURRENT_MIN + 
                   (VALVE_CURRENT_MAX - VALVE_CURRENT_MIN) * (valve_sp / 100.0f);
    valve_current = saturate(valve_current, VALVE_CURRENT_MIN, VALVE_CURRENT_MAX);
    
    // 应用阀门速率限制
    valve_current = apply_rate_limit(valve_current, 
                                   gRateLimiter.last_valve_output,
                                   gRateLimiter.valve_rate_limit);

    // ---------- 变频器输出计算 ----------
    freq_out = saturate(freq_sp, VFD_FREQ_MIN_HZ, VFD_FREQ_MAX_HZ);
    
    // 应用变频器速率限制
    freq_out = apply_rate_limit(freq_out,
                              gRateLimiter.last_vfd_output,
                              gRateLimiter.vfd_rate_limit);

    // ---------- 更新状态 & 写入设备 ----------
    gRateLimiter.last_valve_output = valve_current;
    gRateLimiter.last_vfd_output = freq_out;

    *valve_output = valve_current;
    *vfd_output = freq_out;

    // 写入设备
    COMMGR_VFD_WriteFrequency(freq_out);                    // 写入变频器
    DEVMGR_ValveSetCurrent(DAC_CMD_CH_A, valve_current); // 写入阀门

    return valve_sp;
}

/*!
****************************************************************************************************
* @brief 级联PID控制器初始化
* @param outer_pid 外环PID控制器(压力环)
* @param inner_pid 内环PID控制器(频率环)
* @param outer_kp  外环比例系数
* @param outer_ki  外环积分系数
* @param outer_kd  外环微分系数
* @param inner_kp  内环比例系数
* @param inner_ki  内环积分系数
* @param inner_kd  内环微分系数
****************************************************************************************************
*/
void DEVMGR_PID_InitCascade(struct _PID* outer_pid,
                    struct _PID* inner_pid,
                    float outer_kp, float outer_ki, float outer_kd,
                    float inner_kp, float inner_ki, float inner_kd)
{
    if (!outer_pid || !inner_pid) {
        return;
    }
    
    // 初始化外环PID控制器(压力环)
    DEVMGR_PID_Init(outer_pid);
    DEVMGR_PID_SetKP(outer_pid, outer_kp);
    DEVMGR_PID_SetKI(outer_pid, outer_ki);
    DEVMGR_PID_SetKD(outer_pid, outer_kd);
    DEVMGR_PID_SetSP(outer_pid, PRESSURE_SP);

    // 初始化内环PID控制器(频率环)
    DEVMGR_PID_Init(inner_pid);
    DEVMGR_PID_SetKP(inner_pid, inner_kp);
    DEVMGR_PID_SetKI(inner_pid, inner_ki);
    DEVMGR_PID_SetKD(inner_pid, inner_kd);
    DEVMGR_PID_SetSP(inner_pid, 0.0f);

    // 初始化速率限制器
    gRateLimiter.last_valve_output = VALVE_CURRENT_MIN;
    gRateLimiter.last_vfd_output = VFD_FREQ_BASE;
    gRateLimiter.valve_rate_limit = VALVE_RATE_LIMIT;
    gRateLimiter.vfd_rate_limit = VFD_RATE_LIMIT;
}

/*!
****************************************************************************************************
* @brief 级联PID控制器重置
* @param outer_pid 外环PID控制器(压力环)
* @param inner_pid 内环PID控制器(频率环)
****************************************************************************************************
*/
void DEVMGR_PID_ResetCascade(struct _PID* outer_pid, struct _PID* inner_pid)
{
    if (!outer_pid || !inner_pid) {
        return;
    }
    
    DEVMGR_PID_Init(outer_pid);
    DEVMGR_PID_Init(inner_pid);
    
    // 重置速率限制器
    gRateLimiter.last_valve_output = VALVE_CURRENT_MIN;
    gRateLimiter.last_vfd_output = VFD_FREQ_BASE;
}

/*!

****************************************************************************************************
* @brief 设置阀门输出电流
* @param chN    通道号
* @param current 输出电流值(4-20mA)
* @return uint8_t 错误码
****************************************************************************************************
*/
uint8_t DEVMGR_ValveSetCurrent(uint8_t chN, float current)
{
    // 确保电流在有效范围内
    if (current < VALVE_CURRENT_MIN) current = VALVE_CURRENT_MIN;
    if (current > VALVE_CURRENT_MAX) current = VALVE_CURRENT_MAX;

    // 初始化硬件
    ADS1120_Gpio_Init();
    DAC8552_GPIO_Init();

    // 写入DAC
    DAC8552_WriteChannel(chN, current);
    
    return ERROR_NONE;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于初始化PID控制器
* 注意事项：在调用该模块其他API前，必须先调用该模块
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_Init(struct _PID* d)
{
	d->Err[0]	   = 0;
	d->Err[1]	   = 0;
	d->Err[2]	   = 0;
	d->Sum		   = 0;
	d->Out		   = 0;
	d->Para.KP	   = 0;
	d->Para.KPMax  = 0;
	d->Para.KI	   = 0;
	d->Para.KIMax  = 0;
	d->Para.KD	   = 0;
	d->Para.KDMax  = 0;
	d->Para.SumMax = 0;
	d->Para.ErrMax = 0;
	d->Para.ErrMin = 0;
	d->Para.OutMax = 0;
	d->Para.OutMin = 0;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于计算PID控制器输出
* 注意事项：该方法必须周期调用
* 输入参数：fbVal -- 反馈值
* 输出参数：NA
* 返回参数：PID控制器新输出值
****************************************************************************************************
*/
float DEVMGR_PID_Compute(struct _PID* d, float fbVal)
{
	struct {
		float p;
		float i;
		float d;
	} o;
	float err;
	struct _PID_PARA* para = &d->Para;

	// 计算误差
	err = d->Sp - fbVal;

	// 死区控制
	if (fabs(err) < para->ErrMin) {
		err = 0;
	}

	// 限幅控制
	if (err > para->ErrMax) {
		err = para->ErrMax;
	} else if (err < (0 - para->ErrMax)) {
		err = 0 - para->ErrMax;
	}

	// 更新误差
	d->Err[2] = d->Err[1];	// 更新T2时刻误差
	d->Err[1] = d->Err[0];	// 更新T1时刻误差
	d->Err[0] = err;		// 更新T0时刻误差

	// 计算比例输出
	o.p = d->Err[0] * para->KP;
	// 比例输出限幅控制
	if (o.p > para->KPMax) {
		o.p = para->KPMax;
	} else if (o.p < (0 - para->KPMax)) {
		o.p = 0 - para->KPMax;
	}

	// 更新积分
	d->Sum += d->Err[0]; //积分可能为正,也可能为负
	// 积分限幅控制
	if (d->Sum > para->SumMax) {
		d->Sum = para->SumMax;
	} else if (d->Sum < (0 - para->SumMax)) {
		d->Sum = 0 - para->SumMax;
	}

	// 计算积分输出
	o.i = d->Sum * para->KI;
	// 积分输出限幅控制
	if (o.i > para->KIMax) {
		o.i = para->KIMax;
	} else if (o.i < (0 - para->KIMax)) {
		o.i = 0 - para->KIMax;
	}

	// 计算微分输出
	o.d = (d->Err[0] - d->Err[1]) * para->KD;
	// 微分限幅控制
	if (o.d > para->KDMax) {
		o.d = para->KDMax;
	} else if (o.d < (0 - para->KDMax)) {
		o.d = 0 - para->KDMax;
	}

	// 计算输出
	d->Out = o.p + o.i + o.d;
	// 输出死区控制
	if (fabs(d->Out) < para->ErrMin) {
		d->Out = 0;
	}
	// 输出限幅控制
	if (d->Out > para->ErrMax) {
		d->Out = para->ErrMax;
	} else if (d->Out < (0 - para->ErrMax)) {
		d->Out = 0 - para->ErrMax;
	}

	return d->Out;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的设定值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器设定值
****************************************************************************************************
*/
float DEVMGR_PID_GetSP(struct _PID* d)
{
	return d->Sp;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的输出值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器输出值
****************************************************************************************************
*/
float DEVMGR_PID_GetOutput(struct _PID *d)
{
	return d->Out;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的比例系数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器比例系数
****************************************************************************************************
*/
float DEVMGR_PID_GetKP(struct _PID *d)
{
	return d->Para.KP;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的积分系数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器积分系数
****************************************************************************************************
*/
float DEVMGR_PID_GetKI(struct _PID *d)
{
	return d->Para.KI;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的微分系数
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器微分系数
****************************************************************************************************
*/
float DEVMGR_PID_GetKD(struct _PID *d)
{
	return d->Para.KD;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的比例输出最大值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器比例输出最大值
****************************************************************************************************
*/
float DEVMGR_PID_GetKPMax(struct _PID *d)
{
	return d->Para.KPMax;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的积分输出最大值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器积分输出最大值
****************************************************************************************************
*/
float DEVMGR_PID_GetKIMax(struct _PID *d)
{
	return d->Para.KIMax;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的微分输出最大值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器微分输出最大值
****************************************************************************************************
*/
float DEVMGR_PID_GetKDMax(struct _PID *d)
{
	return d->Para.KDMax;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的积分求和最大值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器积分求和最大值
****************************************************************************************************
*/
float DEVMGR_PID_GetSumMax(struct _PID *d)
{
	return d->Para.SumMax;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的误差最大值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器误差最大值
****************************************************************************************************
*/
float DEVMGR_PID_GetErrMax(struct _PID *d)
{
	return d->Para.ErrMax;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的误差最小值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器比例误差最小值
****************************************************************************************************
*/
float DEVMGR_PID_GetErrMin(struct _PID *d)
{
	return d->Para.ErrMin;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的输出最大值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器输出最大值
****************************************************************************************************
*/
float DEVMGR_PID_GetOutMax(struct _PID *d)
{
	return d->Para.OutMax;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于获取PID控制器的输出最小值
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：PID控制器输出最小值
****************************************************************************************************
*/
float DEVMGR_PID_GetOutMin(struct _PID *d)
{
	return d->Para.OutMin;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的设定值
* 注意事项：NA
* 输入参数：sp -- 设定值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetSP(struct _PID* d, float sp)
{
	d->Sp = sp;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的比例系数
* 注意事项：NA
* 输入参数：kp -- 比例系数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetKP(struct _PID* d, float kp)
{
	d->Para.KP = kp;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的积分系数
* 注意事项：NA
* 输入参数：ki -- 积分系数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetKI(struct _PID* d, float ki)
{
	d->Para.KI = ki;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的微分系数
* 注意事项：NA
* 输入参数：kd -- 微分系数
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetKD(struct _PID* d, float kd)
{
	d->Para.KD = kd;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的比例输出最大值
* 注意事项：NA
* 输入参数：max -- 比例输出最大值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetKPMax(struct _PID* d, float max)
{
	d->Para.KPMax = max;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的积分输出最大值
* 注意事项：NA
* 输入参数：max -- 积分输出最大值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetKIMax(struct _PID* d, float max)
{
	d->Para.KIMax = max;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的微分输出最大值
* 注意事项：NA
* 输入参数：max -- 微分输出最大值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetKDMax(struct _PID* d, float max)
{
	d->Para.KDMax = max;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的积分求和最大值
* 注意事项：NA
* 输入参数：max -- 积分求和最大值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetSumMax(struct _PID* d, float max)
{
	d->Para.SumMax = max;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的误差最大值
* 注意事项：NA
* 输入参数：max -- 误差最大值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetErrMax(struct _PID* d, float max)
{
	d->Para.ErrMax = max;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的误差最小值
* 注意事项：NA
* 输入参数：min -- 误差最小值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetErrMin(struct _PID* d, float min)
{
	d->Para.ErrMin = min;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的输出最大值
* 注意事项：NA
* 输入参数：max -- 输出最大值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetOutMax(struct _PID *d, float max)
{
	d->Para.OutMax = max;
}

/*!
****************************************************************************************************
* 功能描述：该方法用于设置PID控制器的输出最小值
* 注意事项：NA
* 输入参数：min -- 输出最小值
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void DEVMGR_PID_SetOutMin(struct _PID *d, float min)
{
	d->Para.OutMin = min;
}









