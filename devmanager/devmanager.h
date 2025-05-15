/*!
****************************************************************************************************
* 文件名称：devmanager.h
* 功能简介：设备管理器模块的头文件，包含设备驱动的接口声明、常量和类型定义。
* 文件作者：47015
* 创建日期：2025年4月8日
* 版权声明：All Rights Reserved.
****************************************************************************************************
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
#define UP_PID_MAX (5)   // 上传PID参数最大数量

/*!
****************************************************************************************************
* 配置定义
****************************************************************************************************
*/
/* 流量传感器 ----------------------------------------------------------------------------------- */
/* 流量传感器相关配置 */
#define CONFIG_PUMP_NO_MIN          (1)     // 允许的枪号最小值（包括该值）
#define CONFIG_PUMP_NO_MAX          (200)   // 允许的枪号最大值（包括该值）
#define CONFIG_DEBUG_AUTOEXIT_TIME  (300)   // 维护模式下自动退出的时间，单位秒
#define CONFIG_MAX_WAIT_DUETIME     (9999)  // 等待启动加注的时间，单位秒

/* PID 控制器相关配置 */
#define VREFIN	                    (2435)  // 参考电压，单位：mV
#define SAMPLE_R                    (124.0f)// 取样电阻，单位：Ω
#define DAC_RESOLUTION              (65534) // DAC 16位分辨率 2^16 - 1 = 65534 (理论值，未实测)

/* 通用控制参数 */
#define VALVE_CURRENT_MIN           (4.0f)  // 阀门电流最小值 (mA)
#define VALVE_CURRENT_MAX           (20.0f) // 阀门电流最大值 (mA)
#define VFD_FREQ_MIN_HZ             (0.0f)  // 变频器最小频率 (Hz)
#define VFD_FREQ_MAX_HZ             (100.0f)// 变频器最大频率 (Hz)
#define VFD_FREQ_BASE               (50.0f) // 变频器基础频率 (Hz)


/* PID 参数 */
#define PID_MAX_SUM                 (1000.0f)// PID积分限幅 (正向)
#define PID_MIN_SUM                 (-1000.0f)// PID积分限幅 (反向)
#define PID_DEADBAND                (0.1f)  // PID死区
#define PRESSURE_ERR_THRESHOLD      (0.1f)  // 压力偏差阈值(MPa)

/* 压力控制范围 */
#define PRESSURE_LOW_THRESHOLD      (0.6f)  // 低压阈值(MPa)
#define PRESSURE_HIGH_THRESHOLD     (1.0f)  // 高压阈值(MPa)

/* 输出限幅和速率限制 */
#define VALVE_RATE_LIMIT            (0.5f)  // 阀门电流变化率限制(mA/周期)
#define VFD_RATE_LIMIT              (2.0f)  // 变频器频率变化率限制(Hz/周期)
#define VFD_DEADBAND                (1.0f)  // 变频器死区(Hz)

// 时间相关常量
#define RUNSTATE_CHECK_ONLINE_TIME  1000    // 运行状态检查时间(ms)
#define PID_PERIOD                  100     // PID计算周期(ms)

// 压力控制相关常量
#define PRESSURE_SP                 0.8f    // 压力设定值(MPa)
//#define PRESSURE_CHANGE_THRESHOLD   0.1f    // 压力变化阈值(MPa)



/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/


/* 流量传感器模块类型定义 --------------------------------------------------------------------- */

/**
 * @brief 流量计定时器ID枚举
 */
enum {
    MSEC_TMR_ID_FLOW,                    /**< 流量计轮询定时器 */
    MSEC_TMR_ID_FLOW_OFLOW,              /**< 过流检测定时器 */
    MSEC_TMR_ID_FLOW_FUEL_LINK_TIMEOUT,  /**< 进气流量计链路超时定时器 */
    MSEC_TMR_ID_FLOW_RX_TIMEOUT,         /**< 接收字节超时定时器 */
};

/**
 * @brief 流量计设备类型枚举
 */
enum {
    DEVID_FLOW_MICRO = 1,  /**< 艾默生-罗斯蒙特 */
    DEVID_FLOW_EH,         /**< E+H */
    DEVID_FLOW_ADS,        /**< 安迪生 */
};

/* PID控制器模块类型定义 ------------------------------------------------------------------------ */

/**
 * @brief PID内部参数索引枚举 (用于调试或特定场景访问)
 */
enum {
    PID_PARAM_SP = 1,     /**< 设定值 */
    PID_PARAM_ERR,        /**< 误差值 */
    PID_PARAM_OUTPUT,     /**< 输出值 */
    PID_PARAM_SUM,        /**< 积分值 */
    PID_PARAM_KP_OUT,     /**< 比例输出值 */
    PID_PARAM_KI_OUT,     /**< 积分输出值 */
    PID_PARAM_KD_OUT,     /**< 微分输出值 */
};

/**
 * @brief PID控制器参数结构体
 */
struct _PID_PARA {
    float KP;      /**< 比例系数 (Proportional Gain) */
    float KPMax;   /**< 比例项输出最大限制值 */
    float KI;      /**< 积分系数 (Integral Gain) */
    float KIMax;   /**< 积分项输出最大限制值 */
    float KD;      /**< 微分系数 (Derivative Gain) */
    float KDMax;   /**< 微分项输出最大限制值 */
    float SumMax;  /**< 积分累积最大限制值 */
    float ErrMin;  /**< 允许的最小误差值 (用于死区或迟滞比较) */
    float ErrMax;  /**< 允许的最大误差值 (用于死区或迟滞比较) */
    float OutMin;  /**< PID控制器总输出最小限制值 */
    float OutMax;  /**< PID控制器总输出最大限制值 */
};

/**
 * @brief PID控制器状态与数据结构体
 */
struct _PID {
    struct _PID_PARA Para; /**< PID参数配置 */
    float            Sp;   /**< 设定值 (SetPoint) */
    float            Err[3];/**< 误差数组: Err[0]当前误差, Err[1]上次误差, Err[2]上上次误差 */
    float            Sum;  /**< 积分累积值 */
    float            Out;  /**< PID控制器当前输出值 */
};


/*!
****************************************************************************************************
* 函数声明
****************************************************************************************************
*/
/* 流量计模块函数声明 ------------------------------------------------------------------------- */

/**
 * @brief 初始化流量计模块
 */
void DEVMGR_FlowInit(void);

/**
 * @brief 流量计模块周期处理函数
 */
void DEVMGR_FlowHandle(void);

/**
 * @brief 恢复流量计模块的默认参数
 * @return int 0 成功；其他 失败错误码
 */
int DEVMGR_FlowRestoreDefaultPara(void);

/**
 * @brief 查询当前流量计类型
 * @return uint8_t 流量计类型 (参考 DEVID_FLOW_xxx 枚举)
 */
uint8_t DEVMGR_FlowInqType(void);

/**
 * @brief 查询流量上限值
 * @return float 流量上限值
 */
float DEVMGR_FlowInqLimit(void);

/**
 * @brief 更新流量计类型
 * @param type 新的流量计类型 (参考 DEVID_FLOW_xxx 枚举)
 * @return int 0 成功；其他 失败错误码
 */
int DEVMGR_FlowUpdateType(uint8_t type);

/**
 * @brief 更新流量上限值
 * @param limit 新的流量上限值
 * @return int 0 成功；其他 失败错误码
 */
int DEVMGR_FlowUpdateLimit(float limit);

/**
 * @brief 更新流量计参数 (类型和上限)
 * @param type 新的流量计类型
 * @param limit 新的流量上限值
 * @return int 0 成功；其他 失败错误码
 */
int DEVMGR_FlowUpdatePara(uint8_t type, float limit);

/**
 * @brief 使能流量过流检测
 */
void DEVMGR_FlowOFlowChkEnable(void);

/**
 * @brief 禁止流量过流检测
 */
void DEVMGR_FlowOFlowChkDisable(void);

/**
 * @brief 查询进气流量计是否在线
 * @return bool true 在线；false 离线
 */
bool DEVMGR_FlowIsFuelOnline(void);

/**
 * @brief 查询总累计流量是否已清除完成
 * @return bool true 清除完成；false 未完成或正在清除
 */
bool DEVMGR_FlowIsClearDone(void);

/**
 * @brief 查询总累计流量是否清除成功
 * @return bool true 清除成功；false 清除失败
 */
bool DEVMGR_FlowIsClearSucceed(void);

/**
 * @brief 查询进气流量计数据是否已刷新
 * @return bool true 已刷新；false 未刷新
 */
bool DEVMGR_FlowIsFuelRefresh(void);

/**
 * @brief 清除流量计的总累计流量
 */
void DEVMGR_FlowClearTotal(void);

/**
 * @brief 清除流量计数据刷新标志
 */
void DEVMGR_FlowClrRefresh(void);

/**
 * @brief 查询进气瞬时流量
 * @return float 进气瞬时流量值
 */
float DEVMGR_FlowInqJinQiFlow(void);

/**
 * @brief 查询进气累计气量
 * @return float 进气累计气量值
 */
float DEVMGR_FlowInqJinQiVolume(void);

/**
 * @brief 查询进气密度
 * @return float 进气密度值
 */
float DEVMGR_FlowInqJinQiDensity(void);

/**
 * @brief 查询进气增益
 * @return float 进气增益值
 */
float DEVMGR_FlowInqJinQiGain(void);

/**
 * @brief 查询进气温度
 * @return float 进气温度值
 */
float DEVMGR_FlowInqJinQiTemperature(void);

/* PID控制器模块函数声明 -------------------------------------------------------------------------- */

/**
 * @brief 初始化PID控制器
 * @param pid 指向PID控制器的指针
 */
void DEVMGR_PID_Init(struct _PID* pid);

/**
 * @brief 计算PID输出
 * @param pid 指向PID控制器的指针
 * @param feedback_val 当前反馈值
 * @return float PID输出值
 */
float DEVMGR_PID_Compute(struct _PID* pid, float feedback_val);

/**
 * @brief 获取PID设定值
 * @param pid 指向PID控制器的指针
 * @return float 设定值
 */
float DEVMGR_PID_GetSP(struct _PID* pid);

/**
 * @brief 获取PID输出值
 * @param pid 指向PID控制器的指针
 * @return float 输出值
 */
float DEVMGR_PID_GetOutput(struct _PID* pid);

/**
 * @brief 获取PID比例系数KP
 * @param pid 指向PID控制器的指针
 * @return float KP值
 */
float DEVMGR_PID_GetKP(struct _PID* pid);

/**
 * @brief 获取PID积分系数KI
 * @param pid 指向PID控制器的指针
 * @return float KI值
 */
float DEVMGR_PID_GetKI(struct _PID* pid);

/**
 * @brief 获取PID微分系数KD
 * @param pid 指向PID控制器的指针
 * @return float KD值
 */
float DEVMGR_PID_GetKD(struct _PID* pid);

/**
 * @brief 获取PID比例项最大输出值
 * @param pid 指向PID控制器的指针
 * @return float 比例项最大输出值
 */
float DEVMGR_PID_GetKPMax(struct _PID* pid);

/**
 * @brief 获取PID积分项最大输出值
 * @param pid 指向PID控制器的指针
 * @return float 积分项最大输出值
 */
float DEVMGR_PID_GetKIMax(struct _PID* pid);

/**
 * @brief 获取PID微分项最大输出值
 * @param pid 指向PID控制器的指针
 * @return float 微分项最大输出值
 */
float DEVMGR_PID_GetKDMax(struct _PID* pid);

/**
 * @brief 获取PID积分和最大值
 * @param pid 指向PID控制器的指针
 * @return float 积分和最大值
 */
float DEVMGR_PID_GetSumMax(struct _PID* pid);

/**
 * @brief 获取PID误差最大值
 * @param pid 指向PID控制器的指针
 * @return float 误差最大值
 */
float DEVMGR_PID_GetErrMax(struct _PID* pid);

/**
 * @brief 获取PID误差最小值
 * @param pid 指向PID控制器的指针
 * @return float 误差最小值
 */
float DEVMGR_PID_GetErrMin(struct _PID* pid);

/**
 * @brief 获取PID输出最大值
 * @param pid 指向PID控制器的指针
 * @return float 输出最大值
 */
float DEVMGR_PID_GetOutMax(struct _PID* pid);

/**
 * @brief 获取PID输出最小值
 * @param pid 指向PID控制器的指针
 * @return float 输出最小值
 */
float DEVMGR_PID_GetOutMin(struct _PID* pid);

/**
 * @brief 设置PID设定值
 * @param pid 指向PID控制器的指针
 * @param sp 设定值
 */
void DEVMGR_PID_SetSP(struct _PID* pid, float sp);

/**
 * @brief 设置PID比例系数KP
 * @param pid 指向PID控制器的指针
 * @param kp KP值
 */
void DEVMGR_PID_SetKP(struct _PID* pid, float kp);

/**
 * @brief 设置PID积分系数KI
 * @param pid 指向PID控制器的指针
 * @param ki KI值
 */
void DEVMGR_PID_SetKI(struct _PID* pid, float ki);

/**
 * @brief 设置PID微分系数KD
 * @param pid 指向PID控制器的指针
 * @param kd KD值
 */
void DEVMGR_PID_SetKD(struct _PID* pid, float kd);

/**
 * @brief 设置PID比例项最大输出值
 * @param pid 指向PID控制器的指针
 * @param max 比例项最大输出值
 */
void DEVMGR_PID_SetKPMax(struct _PID* pid, float max);

/**
 * @brief 设置PID积分项最大输出值
 * @param pid 指向PID控制器的指针
 * @param max 积分项最大输出值
 */
void DEVMGR_PID_SetKIMax(struct _PID *pid, float max);

/**
 * @brief 设置PID微分项最大输出值
 * @param pid 指向PID控制器的指针
 * @param max 微分项最大输出值
 */
void DEVMGR_PID_SetKDMax(struct _PID *pid, float max);

/**
 * @brief 设置PID积分和最大值
 * @param pid 指向PID控制器的指针
 * @param max 积分和最大值
 */
void DEVMGR_PID_SetSumMax(struct _PID *pid, float max);

/**
 * @brief 设置PID误差最大值
 * @param pid 指向PID控制器的指针
 * @param max 误差最大值
 */
void DEVMGR_PID_SetErrMax(struct _PID *pid, float max);

/**
 * @brief 设置PID误差最小值
 * @param pid 指向PID控制器的指针
 * @param min 误差最小值
 */
void DEVMGR_PID_SetErrMin(struct _PID *pid, float min);

/**
 * @brief 设置PID输出最大值
 * @param pid 指向PID控制器的指针
 * @param max 输出最大值
 */
void DEVMGR_PID_SetOutMax(struct _PID *pid, float max);

/**
 * @brief 设置PID输出最小值
 * @param pid 指向PID控制器的指针
 * @param min 输出最小值
 */
void DEVMGR_PID_SetOutMin(struct _PID *pid, float min);

/**
 * @brief 初始化级联PID控制器
 * @param outer_pid 指向外环PID控制器的指针
 * @param inner_pid 指向内环PID控制器的指针
 * @param outer_kp 外环比例系数
 * @param outer_ki 外环积分系数
 * @param outer_kd 外环微分系数
 * @param inner_kp 内环比例系数
 * @param inner_ki 内环积分系数
 * @param inner_kd 内环微分系数
 */
void DEVMGR_PID_InitCascade(struct _PID* outer_pid,
                           struct _PID* inner_pid,
                           float outer_kp, float outer_ki, float outer_kd,
                           float inner_kp, float inner_ki, float inner_kd);
/**
 * @brief 计算级联PID输出
 * @param outer_pid 指向外环PID控制器的指针
 * @param inner_pid 指向内环PID控制器的指针
 * @param pressure_sp 压力设定值 (外环SP)
 * @param pressure_fb 压力反馈值 (外环PV)
 * @param valve_output 指向阀门输出值的指针 (内环输出, 通常是外环的输出作为内环的SP)
 * @param vfd_output 指向变频器输出频率的指针 (最终控制量)
 * @return float 通常返回内环的设定值 (即外环的输出), 具体根据实现可能不同
 */
float DEVMGR_PID_ComputeCascade(struct _PID* outer_pid,
                               struct _PID* inner_pid,
                               float pressure_sp,
                               float pressure_fb,
                               float* valve_output,
                               float* vfd_output);
/**
 * @brief 复位级联PID控制器状态
 * @param outer_pid 指向外环PID控制器的指针
 * @param inner_pid 指向内环PID控制器的指针
 */
void DEVMGR_PID_ResetCascade(struct _PID* outer_pid, struct _PID* inner_pid);

/* 阀门控制模块函数声明 ------------------------------------------------------------------------- */

/**
 * @brief 设置阀门电流
 * @param chN 通道号
 * @param current 电流值 (mA)
 * @return uint8_t 设置结果，通常0表示成功
 */
uint8_t DEVMGR_ValveSetCurrent(uint8_t chN, float current);

/*!
****************************************************************************************************
* devmanager.c (主设备管理逻辑)
****************************************************************************************************
*/

/**
 * @brief 初始化设备管理器
 */
void DEVMGR_Init(void);

/**
 * @brief 设备管理器周期处理函数
 */
void DEVMGR_Handle(void);

/*!
****************************************************************************************************
* devmanager-ps.c (压力传感器管理)
****************************************************************************************************
*/

/**
 * @brief 初始化压力传感器模块
 */
void DEVMGR_PSInit(void);

/**
 * @brief 压力传感器模块周期处理函数
 */
void DEVMGR_PSHandle(void);

/**
 * @brief 查询压力传感器是否丢失连接
 * @return bool true 丢失；false 连接正常
 */
bool DEVMGR_PSIsLost(void);

/**
 * @brief 查询当前压力值
 * @return float 当前压力值 (MPa)
 */
float DEVMGR_PSInqValue(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVMANAGER_DEVMANAGER_H_ */
