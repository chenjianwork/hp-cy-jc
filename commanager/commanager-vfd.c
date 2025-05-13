/*!
****************************************************************************************************
* 文件名称：commanager-vfd.c
* 功能简介：该文件是变频器通信模块的实现源文件，基于状态机的Modbus RTU通信
* 文件作者：LUDONGDONG
* 创建日期：2025-03-16
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <math.h>
#include "devmanager/devmanager.h"
#include "drvmanager/drvmanager.h"
#include "commanager/commanager.h"
#include "sysmanager/sysmanager.h"
#include <hqhp/crypto/crc.h>

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
// Modbus 地址定义
#define ATV_MODBUS_ID      1       // ATV320 默认地址为 1

// 扫描输入寄存器（COM.scanner input ICS-）
#define REG_FREQ          3202    // 输出频率 (Hz×10)
#define REG_CUR           3204    // 电机电流 (A×10)
#define REG_VOLT          3208    // 电机母线电压 (V×10)
#define REG_SPEED         8604    // 实际转速 (r/min)
#define REG_FAULT_CODE    7121    // 故障代码
#define REG_CMD           8501    // 控制命令寄存器

// 控制位掩码
#define CMD_START_FWD     0x0001  // 正转启动
#define CMD_START_REV     0x0002  // 反转启动
#define CMD_STOP          0x0000  // 停止（写 0 清除所有命令）
#define CMD_RESET_FAULT   0x0004  // 故障复位

// 变频器参数
#define VFD_FREQ_MAX_HZ   100.0f  // 变频器最大频率(Hz)
#define VFD_FREQ_MIN_HZ   0.0f    // 变频器最小频率(Hz)
#define VFD_FREQ_SCALE    10.0f   // 频率值放大倍数，用于保留小数位

// 通信参数定义
#define VFD_RX_BUF_SIZE   64      // 接收缓冲区大小
#define VFD_TX_BUF_SIZE   16      // 发送缓冲区大小
#define VFD_COMM_TIMEOUT  100     // 通信超时时间(ms)
#define VFD_MAX_RETRIES   3       // 最大重试次数

// 状态机状态定义
typedef enum {
    VFD_STATE_IDLE,           // 空闲状态
    VFD_STATE_SEND,           // 发送状态
    VFD_STATE_WAIT_RESP,      // 等待响应
    VFD_STATE_PROCESS_RESP,   // 处理响应
    VFD_STATE_CMD_SEND,       // 发送命令状态
    VFD_STATE_CMD_WAIT,       // 等待命令响应
    VFD_STATE_ERROR          // 错误状态
} VFD_STATE;

// 轮询项定义
typedef enum {
    VFD_POLL_FREQ = 0,    // 轮询频率
    VFD_POLL_CUR,         // 轮询电流
    VFD_POLL_VOLT,        // 轮询电压
    VFD_POLL_SPEED,       // 轮询转速
    VFD_POLL_FAULT,       // 轮询故障码
    VFD_POLL_NUM          // 轮询项数量
} VFD_POLL_ITEM;

// 命令类型定义
typedef enum {
    VFD_CMD_NONE = 0,        // 无命令
    VFD_CMD_START_FWD,       // 正转启动
    VFD_CMD_START_REV,       // 反转启动
    VFD_CMD_STOP,           // 停止
    VFD_CMD_RESET_FAULT     // 故障复位
} VFD_CMD_TYPE;

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
// 变频器信息结构体
struct _VFD_INFO {
    uint8_t  IsOnline;     // 在线状态
    uint16_t Status;       // 运行状态
    float    Frequency;    // 当前频率
    float    SetFrequency; // 设定频率
    float    Current;      // 电机电流
    float    Voltage;      // 电机电压
    float    Speed;        // 实际转速
    uint16_t FaultCode;    // 故障码
    uint8_t  State;        // 工作状态
};

// 状态机上下文结构体
struct _VFD_STATE {
    VFD_STATE state;                // 当前状态
    VFD_POLL_ITEM pollItem;         // 当前轮询项
    VFD_CMD_TYPE cmdType;           // 当前命令类型
    bool cmdPending;                // 命令等待标志
    uint8_t txBuf[VFD_TX_BUF_SIZE]; // 发送缓冲区
    uint8_t rxBuf[VFD_RX_BUF_SIZE]; // 接收缓冲区
    uint16_t txLen;                 // 发送长度
    uint16_t rxLen;                 // 接收长度
    uint16_t expectedRxLen;         // 期望接收长度
    uint16_t regValue;              // 寄存器值
    uint8_t retries;                // 重试计数
    bool respReceived;              // 响应接收标志
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
static struct _VFD_INFO gVFDInfo;   // 变频器信息

static struct _VFD_STATE gVFDState; // 状态机上下文

/*!
****************************************************************************************************
* 本地函数声明
****************************************************************************************************
*/
static void COMMGR_VFD_UART_RxCallback(int idx, uint8_t data);
static void COMMGR_VFD_StateMachine(void);
static void COMMGR_VFD_SendCommand(VFD_CMD_TYPE cmdType);

/*!
****************************************************************************************************
* 接口函数实现
****************************************************************************************************
*/
void COMMGR_VFDInit(void)
{
    // 初始化变频器信息
    memset(&gVFDInfo, 0, sizeof(gVFDInfo));
    gVFDInfo.State = WORK_INIT;
    
    // 初始化状态机
    memset(&gVFDState, 0, sizeof(gVFDState));
    gVFDState.state = VFD_STATE_IDLE;
    gVFDState.pollItem = VFD_POLL_FREQ;
    gVFDState.cmdType = VFD_CMD_NONE;
    gVFDState.cmdPending = false;

    // 初始化串口
    DRVMGR_UARTOpen(DRVID_UART_4, DRV_UART_BAUD19200, kUART_PARITY_EVEN);

    // 设置串口接收回调
    DRVMGR_UARTSetRxCallback(DRVID_UART_4, COMMGR_VFD_UART_RxCallback);
    
   // 启动超时定时器
    DRVMGR_MSTimerStart(MS_TMR_ID_VDF_MODBUS_TIMEOUT, VFD_COMM_TIMEOUT);
}

void COMMGR_VFDHandle(void)
{
    switch (gVFDInfo.State) {
        case WORK_INIT:
            gVFDInfo.State = WORK_DOING;
            break;
            
        case WORK_DOING:
            // 优先处理命令
            if (gVFDState.cmdPending && gVFDState.state == VFD_STATE_IDLE) {
                COMMGR_VFD_SendCommand(gVFDState.cmdType);
            }
            // 检查是否需要写入频率
            else if (gVFDInfo.SetFrequency != gVFDInfo.Frequency && 
                     gVFDState.state == VFD_STATE_IDLE) {
                COMMGR_VFD_WriteFrequency(gVFDInfo.SetFrequency);
            }
            // 推进状态机
            COMMGR_VFD_StateMachine();
            break;
            
        case WORK_EXIT:
            gVFDInfo.State = WORK_DONE;
            break;
            
        default:
            break;
    }
}

int COMMGR_VFDSetFrequency(float freq_hz)
{
    // 参数检查
    if(freq_hz < VFD_FREQ_MIN_HZ) freq_hz = VFD_FREQ_MIN_HZ;
    if(freq_hz > VFD_FREQ_MAX_HZ) freq_hz = VFD_FREQ_MAX_HZ;
    
    gVFDInfo.SetFrequency = freq_hz;
    gVFDInfo.IsOnline = true;
    return ERROR_NONE;
}

int COMMGR_VFDGetFrequency(float *freq_hz)
{
    *freq_hz = gVFDInfo.Frequency;
    return ERROR_NONE;
}

int COMMGR_VFDGetCurrent(float *current)
{
    *current = gVFDInfo.Current;
    return ERROR_NONE;
}

int COMMGR_VFDGetVoltage(float *voltage)
{
    *voltage = gVFDInfo.Voltage;
    return ERROR_NONE;
}

int COMMGR_VFDGetSpeed(float *speed)
{
    *speed = gVFDInfo.Speed;
    return ERROR_NONE;
}

int COMMGR_VFDGetFault(uint16_t *fault)
{
    *fault = gVFDInfo.FaultCode;
    return ERROR_NONE;
}

bool COMMGR_VFDIsOnline(void)
{
    return gVFDInfo.IsOnline;
}


int COMMGR_VFDStart(void)
{
    if (gVFDState.state == VFD_STATE_IDLE) {
        gVFDState.cmdPending = true;
        COMMGR_VFD_SendCommand(VFD_CMD_START_FWD);
        return ERROR_NONE;
    }
    return ERROR_BUSY;
}

int COMMGR_VFDStartReverse(void)
{
    if (gVFDState.state == VFD_STATE_IDLE) {
        gVFDState.cmdPending = true;
        COMMGR_VFD_SendCommand(VFD_CMD_START_REV);
        return ERROR_NONE;
    }
    return ERROR_BUSY;
}

int COMMGR_VFDStop(void)
{
    if (gVFDState.state == VFD_STATE_IDLE) {
        gVFDState.cmdPending = true;
        COMMGR_VFD_SendCommand(VFD_CMD_STOP);
        return ERROR_NONE;
    }
    return ERROR_BUSY;
}

int COMMGR_VFDResetFault(void)
{
    if (gVFDState.state == VFD_STATE_IDLE) {
        gVFDState.cmdPending = true;
        COMMGR_VFD_SendCommand(VFD_CMD_RESET_FAULT);
        return ERROR_NONE;
    }
    return ERROR_BUSY;
}

void COMMGR_VFD_WriteFrequency(float freq)
{
    uint16_t freqValue = (uint16_t)(freq * VFD_FREQ_SCALE);
    uint32_t crc;

    gVFDState.txBuf[0] = ATV_MODBUS_ID;
    gVFDState.txBuf[1] = 0x06;  // 写单个寄存器
    gVFDState.txBuf[2] = REG_FREQ >> 8;
    gVFDState.txBuf[3] = REG_FREQ & 0xFF;
    gVFDState.txBuf[4] = freqValue >> 8;
    gVFDState.txBuf[5] = freqValue & 0xFF;

    // 使用CRC_Compute计算CRC
    crc = CRC_Compute(CRC16_MODBUS, gVFDState.txBuf, 6);
    crc = CRC_ComputeComplete(CRC16_MODBUS, crc);

    gVFDState.txBuf[6] = crc & 0xFF;
    gVFDState.txBuf[7] = (crc >> 8) & 0xFF;

    gVFDState.txLen = 8;
    gVFDState.rxLen = 0;
    gVFDState.expectedRxLen = 8;
    gVFDState.respReceived = false;
    gVFDState.state = VFD_STATE_WAIT_RESP;

    DRVMGR_UARTSendBytes(DRVID_UART_4, gVFDState.txBuf, gVFDState.txLen);

    // 启动超时定时器
    DRVMGR_MSTimerStart(MS_TMR_ID_VDF_MODBUS_TIMEOUT, VFD_COMM_TIMEOUT);
}



/*!
****************************************************************************************************
* 本地函数实现
****************************************************************************************************
*/
static void COMMGR_VFD_UART_RxCallback(int idx, uint8_t data)
{
    if (idx != DRVID_UART_4) return;
    
    if (gVFDState.rxLen < VFD_RX_BUF_SIZE) {
        gVFDState.rxBuf[gVFDState.rxLen++] = data;
        if (gVFDState.rxLen >= gVFDState.expectedRxLen) {
            gVFDState.respReceived = true;
        }
    }
}

static void COMMGR_VFD_SendCommand(VFD_CMD_TYPE cmdType)
{
    uint16_t cmdValue;
    uint32_t crc;
    
    // 根据命令类型设置命令值
    switch (cmdType) {
        case VFD_CMD_START_FWD:
            cmdValue = CMD_START_FWD;
            break;
        case VFD_CMD_START_REV:
            cmdValue = CMD_START_REV;
            break;
        case VFD_CMD_STOP:
            cmdValue = CMD_STOP;
            break;
        case VFD_CMD_RESET_FAULT:
            cmdValue = CMD_RESET_FAULT;
            break;
        default:
            return;
    }
    
    // 构造写命令帧
    gVFDState.txBuf[0] = ATV_MODBUS_ID;
    gVFDState.txBuf[1] = 0x06;  // 写单个寄存器
    gVFDState.txBuf[2] = REG_CMD >> 8;
    gVFDState.txBuf[3] = REG_CMD & 0xFF;
    gVFDState.txBuf[4] = cmdValue >> 8;
    gVFDState.txBuf[5] = cmdValue & 0xFF;
    
    // 使用CRC_Compute计算CRC
    crc = CRC_Compute(CRC16_MODBUS, gVFDState.txBuf, 6);
    crc = CRC_ComputeComplete(CRC16_MODBUS, crc);
    
    gVFDState.txBuf[6] = crc & 0xFF;
    gVFDState.txBuf[7] = (crc >> 8) & 0xFF;
    
    // 设置发送参数
    gVFDState.txLen = 8;
    gVFDState.rxLen = 0;
    gVFDState.expectedRxLen = 8;  // 写命令响应长度
    gVFDState.respReceived = false;
    gVFDState.cmdType = cmdType;
    gVFDState.state = VFD_STATE_CMD_WAIT;
    
    // 发送命令
    DRVMGR_UARTSendBytes(DRVID_UART_4, gVFDState.txBuf, gVFDState.txLen);
    // 启动超时定时器
    DRVMGR_MSTimerStart(MS_TMR_ID_VDF_MODBUS_TIMEOUT, VFD_COMM_TIMEOUT);
}

static void COMMGR_VFD_StateMachine(void)
{
    uint32_t crc;
    
    switch (gVFDState.state) {
        case VFD_STATE_IDLE:
            // 构造读命令
            gVFDState.txBuf[0] = ATV_MODBUS_ID;
            gVFDState.txBuf[1] = 0x03;  // 读保持寄存器
            
            // 根据轮询项设置寄存器地址
            switch (gVFDState.pollItem) {
                case VFD_POLL_FREQ:
                    gVFDState.txBuf[2] = REG_FREQ >> 8;
                    gVFDState.txBuf[3] = REG_FREQ & 0xFF;
                    break;
                case VFD_POLL_CUR:
                    gVFDState.txBuf[2] = REG_CUR >> 8;
                    gVFDState.txBuf[3] = REG_CUR & 0xFF;
                    break;
                case VFD_POLL_VOLT:
                    gVFDState.txBuf[2] = REG_VOLT >> 8;
                    gVFDState.txBuf[3] = REG_VOLT & 0xFF;
                    break;
                case VFD_POLL_SPEED:
                    gVFDState.txBuf[2] = REG_SPEED >> 8;
                    gVFDState.txBuf[3] = REG_SPEED & 0xFF;
                    break;
                case VFD_POLL_FAULT:
                    gVFDState.txBuf[2] = REG_FAULT_CODE >> 8;
                    gVFDState.txBuf[3] = REG_FAULT_CODE & 0xFF;
                    break;
            }
            
            gVFDState.txBuf[4] = 0x00;  // 读1个寄存器
            gVFDState.txBuf[5] = 0x01;
            
            // 使用CRC_Compute计算CRC
            crc = CRC_Compute(CRC16_MODBUS, gVFDState.txBuf, 6);
            crc = CRC_ComputeComplete(CRC16_MODBUS, crc);
            
            gVFDState.txBuf[6] = crc & 0xFF;
            gVFDState.txBuf[7] = (crc >> 8) & 0xFF;
            
            gVFDState.txLen = 8;
            gVFDState.rxLen = 0;
            gVFDState.expectedRxLen = 7;
            gVFDState.respReceived = false;
            gVFDState.retries = 0;
            
            DRVMGR_UARTSendBytes(DRVID_UART_4, gVFDState.txBuf, gVFDState.txLen);
            gVFDState.state = VFD_STATE_WAIT_RESP;
        // 启动超时定时器
            DRVMGR_MSTimerStart(MS_TMR_ID_VDF_MODBUS_TIMEOUT, VFD_COMM_TIMEOUT);
            break;
            
        case VFD_STATE_WAIT_RESP:
            if (!DRVMGR_MSTimerIsExpiration(MS_TMR_ID_VDF_MODBUS_TIMEOUT)) {
                gVFDState.state = VFD_STATE_ERROR;
            }
            else if (gVFDState.respReceived) {
                gVFDState.state = VFD_STATE_PROCESS_RESP;
            }
            break;
            
        case VFD_STATE_PROCESS_RESP:
            if (gVFDState.rxLen == gVFDState.expectedRxLen) {
                // 使用CRC_Compute计算CRC
                crc = CRC_Compute(CRC16_MODBUS, gVFDState.rxBuf, gVFDState.rxLen - 2);
                crc = CRC_ComputeComplete(CRC16_MODBUS, crc);
                
                uint16_t respCrc = (gVFDState.rxBuf[gVFDState.rxLen - 1] << 8) | 
                                 gVFDState.rxBuf[gVFDState.rxLen - 2];
                
                if (crc == respCrc && gVFDState.rxBuf[0] == ATV_MODBUS_ID && 
                    gVFDState.rxBuf[1] == 0x03) {
                    gVFDState.regValue = (gVFDState.rxBuf[3] << 8) | gVFDState.rxBuf[4];
                    
                    switch (gVFDState.pollItem) {
                        case VFD_POLL_FREQ:
                            gVFDInfo.Frequency = gVFDState.regValue / VFD_FREQ_SCALE;
                            break;
                        case VFD_POLL_CUR:
                            gVFDInfo.Current = gVFDState.regValue / 10.0f;
                            break;
                        case VFD_POLL_VOLT:
                            gVFDInfo.Voltage = gVFDState.regValue / 10.0f;
                            break;
                        case VFD_POLL_SPEED:
                            gVFDInfo.Speed = (float)gVFDState.regValue;
                            break;
                        case VFD_POLL_FAULT:
                            gVFDInfo.FaultCode = gVFDState.regValue;
                            break;
                    }
                    
                    gVFDState.pollItem = (gVFDState.pollItem + 1) % VFD_POLL_NUM;
                    gVFDState.state = VFD_STATE_IDLE;
                    gVFDInfo.IsOnline = true;
                } else {
                    gVFDState.state = VFD_STATE_ERROR;
                }
            } else {
                gVFDState.state = VFD_STATE_ERROR;
            }
            break;
            
        case VFD_STATE_CMD_WAIT:
            // 检查命令响应超时
            if (!DRVMGR_MSTimerIsExpiration(MS_TMR_ID_VDF_MODBUS_TIMEOUT)) {
                gVFDState.state = VFD_STATE_ERROR;
            }
            // 检查是否收到命令响应
            else if (gVFDState.respReceived) {
                // 验证命令响应
                if (gVFDState.rxLen == gVFDState.expectedRxLen) {
                    // 使用CRC_Compute计算CRC
                    crc = CRC_Compute(CRC16_MODBUS, gVFDState.rxBuf, gVFDState.rxLen - 2);
                    crc = CRC_ComputeComplete(CRC16_MODBUS, crc);
                    
                    uint16_t respCrc = (gVFDState.rxBuf[gVFDState.rxLen - 1] << 8) | 
                                     gVFDState.rxBuf[gVFDState.rxLen - 2];
                    
                    if (crc == respCrc && gVFDState.rxBuf[0] == ATV_MODBUS_ID && 
                        gVFDState.rxBuf[1] == 0x06) {
                        // 命令执行成功
                        gVFDInfo.IsOnline = true;
                        gVFDState.cmdPending = false;
                        gVFDState.state = VFD_STATE_IDLE;
                    } else {
                        gVFDState.state = VFD_STATE_ERROR;
                    }
                } else {
                    gVFDState.state = VFD_STATE_ERROR;
                }
            }
            break;
            
        case VFD_STATE_ERROR:
            if (++gVFDState.retries < VFD_MAX_RETRIES) {
                // 如果是命令错误，重试命令
                if (gVFDState.cmdPending) {
                    gVFDState.state = VFD_STATE_CMD_WAIT;
                    COMMGR_VFD_SendCommand(gVFDState.cmdType);
                } else {
                    gVFDState.state = VFD_STATE_IDLE;
                }
            } else {
                gVFDState.cmdPending = false;
                gVFDState.pollItem = (gVFDState.pollItem + 1) % VFD_POLL_NUM;
                gVFDState.state = VFD_STATE_IDLE;
                gVFDInfo.IsOnline = false;
            }
            break;
    }
}
