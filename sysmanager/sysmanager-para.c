/*
 * sysmanager-para.c
 * 本文件实现系统参数的读写操作，并允许将存储位置作为参数传入，
 * 在系统启动时加载参数进行初始化，确保使用正确的存储区域，并检测Flash中的数据有效性。
 */

#include <string.h>                 // 包含字符串操作函数，如memcmp、memset等
#include "stm32f4xx.h"              // 包含STM32F4系列MCU标准外设库
#include "stm32f4xx_flash.h"        // 包含Flash操作相关函数定义
#include "sysmanager/sysmanager.h"
#include "drvmanager/drvmanager.h"



// 计算结构体占用字节数
#define RUN_PARA_SIZE  (sizeof(struct _RUN_PARA))

//**********************************************************************
// 2. 全局变量：当前变频器参数
//**********************************************************************


//**********************************************************************
// 3. Flash写入函数（传入存储地址作为参数）
//**********************************************************************
/*
 * 函数功能：先对比Flash中已有的变频器参数数据与新参数是否一致，
 *           若不一致则先擦除目标Flash扇区，再写入新数据。
 * 参数：
 *   params     - 指向待写入变频器参数结构体的指针
 *   flash_addr - 存储区域的起始地址，作为参数传入
 * 返回值：
 *   写入成功或数据无需更新返回SUCCESS_t，否则返回ERROR_t
 */
int SYSMGR_Write_Flash_Params(struct _RUN_PARA *params, uint32_t flash_addr)
{
    struct _RUN_PARA flash_params;  // 用于存储Flash中已有的数据
    uint8_t *pData = (uint8_t*)params;       // 将结构体指针转换为字节指针
    uint8_t *pFlashData = (uint8_t*)&flash_params;  // 将Flash数据指针转换为字节指针
    uint32_t i;

    // 读取Flash中已有的数据到flash_params结构体
    for (i = 0; i < sizeof(struct _RUN_PARA); i++) {
        pFlashData[i] = *(__IO uint8_t*)(flash_addr + i);  // 逐字节读取数据
    }

    // 对比已有数据与新数据，如果完全一致则无需写入
    if (memcmp(&flash_params, params, sizeof(struct _RUN_PARA)) == 0)
    {
        return SUCCESS_t;  // 数据一致，直接返回成功
    }

    FLASH_Unlock();  // 解锁Flash允许写操作

    // 擦除目标Flash扇区
    // 注意：实际工程中需要根据flash_addr确定对应的扇区，本示例简化为直接使用FLASH_Sector_6
    if (FLASH_EraseSector(FLASH_Sector_6, VoltageRange_3) != FLASH_COMPLETE)
    {
        FLASH_Lock();    // 擦除失败后上锁Flash
        return ERROR_t;  // 返回错误状态
    }

    // 逐字节写入新参数数据到指定存储区域
    for (i = 0; i < sizeof(struct _RUN_PARA); i++)
    {
        if (FLASH_ProgramByte(flash_addr + i, pData[i]) != FLASH_COMPLETE)
        {
            FLASH_Lock();    // 写入失败后上锁Flash
            return ERROR_t;  // 返回错误状态
        }
    }

    FLASH_Lock();  // 写入完成后锁定Flash，防止误操作
    return SUCCESS_t;  // 写入成功
}

//**********************************************************************
// 4. Flash读取函数（传入存储地址作为参数）
//**********************************************************************
/*
 * 函数功能：从指定Flash存储区域读取变频器参数数据，
 *           将读取的数据直接存储到传入的结构体中。
 * 参数：
 *   params     - 指向用于保存读取数据的变频器参数结构体指针
 *   flash_addr - 存储区域的起始地址，作为参数传入
 * 返回值：
 *   无（读取结果存入结构体）
 */
void SYSMGR_Read_Flash_Params(struct _RUN_PARA *params, uint32_t flash_addr)
{
   uint8_t *pData = (uint8_t*)params;  // 将结构体指针转换为字节指针
	uint32_t i;

	// 逐字节从Flash中读取数据到结构体中
	for (i = 0; i < RUN_PARA_SIZE; i++)
	{
		pData[i] = *(__IO uint8_t*)(flash_addr + i);
	}
}

//**********************************************************************
// 5. 默认参数设置函数：若Flash数据无效则使用默认参数
//**********************************************************************
/*
 * 函数功能：为变频器参数赋予默认值，默认值可根据工程需求修改。
 * 参数：
 *   params - 指向变频器参数结构体的指针
 * 返回值：
 *   无
 */
void SYSMGR_Params_Restore(struct _RUN_PARA *params)
{
    // PID参数默认值
    params->pid_para.KP      = 1.5f;   // 默认比例系数
    params->pid_para.KPMax   = 10.0f;  // 默认比例输出最大值
    params->pid_para.KI      = 0.5f;   // 默认积分系数
    params->pid_para.KIMax   = 5.0f;   // 默认积分输出最大值
    params->pid_para.KD      = 0.1f;   // 默认微分系数
    params->pid_para.KDMax   = 2.0f;   // 默认微分输出最大值
    params->pid_para.SumMax  = 50.0f;  // 默认积分计算最大值
    params->pid_para.ErrMin  = -1.0f;  // 默认误差最小值
    params->pid_para.ErrMax  = 1.0f;   // 默认误差最大值
    params->pid_para.OutMin  = 0.0f;   // 默认输出最小值
    params->pid_para.OutMax  = 100.0f; // 默认输出最大值

    //压力变送器参数	PT207
    params->Range_PT207 = CONFIG_PS_DEFAULT_RANGE_PT207; // 默认量程
    params->Limit_PT207 = CONFIG_PS_DEFAULT_LIMIT_PT207; // 默认过压保护
    params->Ratio_PT207 = 1.0f; // 默认原始采样值缩放系数
    params->Delta_PT207 = 0.0f; // 默认原始采样值偏移系数
    params->OverTime_PT207 = CONFIG_PS_DEFAULT_OVER_TIME_PT207; // 默认过压时间
    
    //压力变送器参数	PT206
    params->Range_PT206 = CONFIG_PS_DEFAULT_RANGE_PT206; // 默认量程
    params->Limit_PT206 = CONFIG_PS_DEFAULT_LIMIT_PT206; // 默认过压保护
    params->Ratio_PT206 = 1.0f; // 默认原始采样值缩放系数
    params->Delta_PT206 = 0.0f; // 默认原始采样值偏移系数
    params->OverTime_PT206 = CONFIG_PS_DEFAULT_OVER_TIME_PT206; // 默认过压时间
    
    //流量计参数ONE
    params->Type_ONE = DEVID_FLOW_ADS; // 默认流量计类型
    params->Limit_ONE = 80.0f; // 默认流量上限

    //流量计参数TWO
    params->Type_TWO = DEVID_FLOW_ADS; // 默认流量计类型
    params->Limit_TWO = 80.0f; // 默认流量上限
}

//**********************************************************************
// 6. 参数初始化函数：系统启动时加载参数
//**********************************************************************
/*
 * 函数功能：系统启动时调用，先从传入存储区域中读取变频器参数，
 *           检查数据有效性，如果无效则赋予默认参数并写入Flash，
 *           最终将参数保存到全局变量供其他模块使用。
 * 参数：
 *   flash_addr - 存储区域的起始地址，作为参数传入
 * 返回值：
 *   无
 */
void SYSMGR_Params_Init(uint32_t flash_addr)
{
    // 从指定Flash区域中读取参数到全局变量
    SYSMGR_Read_Flash_Params(&gRUNPara, flash_addr);

    // 判断数据是否有效，这里以KP参数为例：如果KP不在合理范围(0~100)则认为数据无效
    if (gRUNPara.pid_para.KP < 0.0f || gRUNPara.pid_para.KP > 100.0f)
    {
        // 数据无效时，赋予默认参数
        SYSMGR_Params_Restore(&gRUNPara);
        // 同时将默认参数写入指定Flash区域，保证Flash数据正确
        SYSMGR_Write_Flash_Params(&gRUNPara, flash_addr);
    }

}


/*!
****************************************************************************************************
* 功能描述：查询PID参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_PIDKP(void)
{
    return gRUNPara.pid_para.KP;
}

float SYSMGR_Para_PIDKPMax(void)
{
    return gRUNPara.pid_para.KPMax;
}

float SYSMGR_Para_PIDKI(void)
{
    return gRUNPara.pid_para.KI;
}

float SYSMGR_Para_PIDKIMax(void)
{
    return gRUNPara.pid_para.KIMax;
}

float SYSMGR_Para_PIDKD(void)
{
    return gRUNPara.pid_para.KD;
}

float SYSMGR_Para_PIDKDMax(void)
{
    return gRUNPara.pid_para.KDMax;
}

float SYSMGR_Para_PIDSumMax(void)
{
    return gRUNPara.pid_para.SumMax;
}

float SYSMGR_Para_PIDErrMin(void)
{
    return gRUNPara.pid_para.ErrMin;
}

float SYSMGR_Para_PIDErrMax(void)
{
    return gRUNPara.pid_para.ErrMax;
}

float SYSMGR_Para_PIDOutMin(void)
{
    return gRUNPara.pid_para.OutMin;
}

float SYSMGR_Para_PIDOutMax(void)
{
    return gRUNPara.pid_para.OutMax;
}

/*!
****************************************************************************************************
* 功能描述：查询压力变送器PT207参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_Range_PT207(void)
{
    return gRUNPara.Range_PT207;
}

float SYSMGR_Para_Limit_PT207(void)
{
    return gRUNPara.Limit_PT207;
}

float SYSMGR_Para_Ratio_PT207(void)
{
    return gRUNPara.Ratio_PT207;
}

float SYSMGR_Para_Delta_PT207(void)
{
    return gRUNPara.Delta_PT207;
}

float SYSMGR_Para_OverTime_PT207(void)
{
    return gRUNPara.OverTime_PT207;
}

/*!
****************************************************************************************************
* 功能描述：查询压力变送器PT206参数相关函数
****************************************************************************************************
*/
float SYSMGR_Para_Range_PT206(void)
{
    return gRUNPara.Range_PT206;
}

float SYSMGR_Para_Limit_PT206(void)
{
    return gRUNPara.Limit_PT206;
}

float SYSMGR_Para_Ratio_PT206(void)
{
    return gRUNPara.Ratio_PT206;
}

float SYSMGR_Para_Delta_PT206(void)
{
    return gRUNPara.Delta_PT206;
}

float SYSMGR_Para_OverTime_PT206(void)
{
    return gRUNPara.OverTime_PT206;
}

/*!
****************************************************************************************************
* 功能描述：查询流量计参数相关函数
****************************************************************************************************
*/
uint32_t SYSMGR_Para_Type_ONE(void)  // 修改返回类型为uint32_t
{
    return gRUNPara.Type_ONE;
}

float SYSMGR_Para_Limit_ONE(void)
{
    return gRUNPara.Limit_ONE;
}

uint32_t SYSMGR_Para_Type_TWO(void)  // 修改返回类型为uint32_t
{
    return gRUNPara.Type_TWO;
}

float SYSMGR_Para_Limit_TWO(void)
{
    return gRUNPara.Limit_TWO;
}

/*!
****************************************************************************************************
* 功能描述：设置PID参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetPIDKP(float kp)
{
    gRUNPara.pid_para.KP = kp;
}

void SYSMGR_Para_SetPIDKPMax(float kpmax)
{
    gRUNPara.pid_para.KPMax = kpmax;
}

void SYSMGR_Para_SetPIDKI(float ki)
{
    gRUNPara.pid_para.KI = ki;
}

void SYSMGR_Para_SetPIDKIMax(float kimax)
{
    gRUNPara.pid_para.KIMax = kimax;
}

void SYSMGR_Para_SetPIDKD(float kd)
{
    gRUNPara.pid_para.KD = kd;
}

void SYSMGR_Para_SetPIDKDMax(float kdmax)
{
    gRUNPara.pid_para.KDMax = kdmax;
}

void SYSMGR_Para_SetPIDSumMax(float summax)
{
    gRUNPara.pid_para.SumMax = summax;
}

void SYSMGR_Para_SetPIDErrMin(float errmin)
{
    gRUNPara.pid_para.ErrMin = errmin;
}

void SYSMGR_Para_SetPIDErrMax(float errmax)
{
    gRUNPara.pid_para.ErrMax = errmax;
}

void SYSMGR_Para_SetPIDOutMin(float outmin)
{
    gRUNPara.pid_para.OutMin = outmin;
}

void SYSMGR_Para_SetPIDOutMax(float outmax)
{
    gRUNPara.pid_para.OutMax = outmax;
}

/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT207参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetRange_PT207(float range)
{
    gRUNPara.Range_PT207 = range;
}

void SYSMGR_Para_SetLimit_PT207(float limit)
{
    gRUNPara.Limit_PT207 = limit;
}

void SYSMGR_Para_SetRatio_PT207(float ratio)
{
    gRUNPara.Ratio_PT207 = ratio;
}

void SYSMGR_Para_SetDelta_PT207(float delta)
{
    gRUNPara.Delta_PT207 = delta;
}

void SYSMGR_Para_SetOverTime_PT207(float over_time)
{
    gRUNPara.OverTime_PT207 = over_time;
}

/*!
****************************************************************************************************
* 功能描述：设置压力变送器PT206参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetRange_PT206(float range)
{
    gRUNPara.Range_PT206 = range;
}

void SYSMGR_Para_SetLimit_PT206(float limit)
{
    gRUNPara.Limit_PT206 = limit;
}

void SYSMGR_Para_SetRatio_PT206(float ratio)
{
    gRUNPara.Ratio_PT206 = ratio;
}

void SYSMGR_Para_SetDelta_PT206(float delta)
{
    gRUNPara.Delta_PT206 = delta;
}

void SYSMGR_Para_SetOverTime_PT206(float over_time)
{
    gRUNPara.OverTime_PT206 = over_time;
}

/*!
****************************************************************************************************
* 功能描述：设置流量计参数相关函数
****************************************************************************************************
*/
void SYSMGR_Para_SetType_ONE(uint32_t type)
{
    gRUNPara.Type_ONE = type;
}

void SYSMGR_Para_SetLimit_ONE(float limit)
{
    gRUNPara.Limit_ONE = limit;
}

void SYSMGR_Para_SetType_TWO(uint32_t type)
{
    gRUNPara.Type_TWO = type;
}

void SYSMGR_Para_SetLimit_TWO(float limit)
{
    gRUNPara.Limit_TWO = limit;
}



