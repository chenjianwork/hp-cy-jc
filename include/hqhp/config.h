/*!
****************************************************************************************************
* 文件名称：config.h
* 功能简介：该文件是配置头文件
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_CONFIG_H_
#define INCLUDE_HQHP_CONFIG_H_


// 秒定时器分配 ----------------------------------------------------------------------------------- //
enum {
	SMS_TMR_ID_CAN_OnLine_1,
	MSEC_TMR_ID_ADS_WAIT,
	TMR_ID_MAXIMUM
};

// 毫秒定时器分配 --------------------------------------------------------------------------------- //
enum {
	MS_TMR_ID_BOOT,			 // 引导程序使用
	MS_TMR_ID_AI,			 // AI扫描使用
	MS_TMR_ID_AO,			 // AO刷新使用
	MS_TMR_ID_DI,			 // DI扫描使用
	MS_TMR_ID_DO,			 // DO刷新使用
	MS_TMR_ID_RUN,			 // 运行指示灯使用
	MS_TMR_ID_WAIT_HW_READY, // 等待硬件就绪使用
	MS_TMR_ID_PLC_DB_UPDATE,
	MS_TMR_ID_PLC_RX_TIMEOUT,
	MS_TMR_ID_CPU_TX_HB,		// 安全CPU发送心跳使用
	MS_TMR_ID_CPU_RX_TIMEOUT,	// 安全CPU接收超时使用
	MS_TMR_ID_CPU_LINK_TIMEOUT, // 安全CPU链路超时使用
	MS_TMR_ID_CAN_OnLine,
	MS_TMR_ID_100MS,
	MS_TMR_ID_200MS,
	MS_TMR_ID_RUNSTATE_CHECK_ONLINE,	// 运行状态检查在线使用
	MS_TMR_ID_VDF_MODBUS_TIMEOUT,		// 变频器MODBUS通信超时使用
	MS_TMR_ID_VDF_PID_TIMEOUT,				// 变频器超时使用
	MSEC_TMR_ID_PS_OVPS,
	MS_TMR_ID_MAXIMUM
};

// 串口设备分配 --------------------------------------------------------------------------------- //
#define CONFIG_UART_RS485_1			 (DRVID_UART_1)
#define CONFIG_UART_RS485_1_BAUDRATE (9600)
#define CONFIG_UART_RS485_1_PARITY	 (kUART_PARITY_NONE)

#define CONFIG_UART_RS485_3		     (DRVID_UART_3)
#define CONFIG_UART_RS485_3_BAUDRATE (9600)
#define CONFIG_UART_RS485_3_PARITY   (kUART_PARITY_NONE)

#define CONFIG_UART_PLC_1		   (DRVID_UART_2)
#define CONFIG_UART_PLC_1_BAUDRATE (9600)
#define CONFIG_UART_PLC_1_PARITY   (kUART_PARITY_NONE)

#define CONFIG_UART_CanOpen_1				(DRVID_UART_4)
#define CONFIG_UART_CanOpen_1_BAUDRATE	(19200)
#define CONFIG_UART_CanOpen_1_PARITY		(kUART_PARITY_NONE)

#define CONFIG_UART_CanOpen_2				(DRVID_UART_5)
#define CONFIG_UART_CanOpen_2_BAUDRATE	(19200)
#define CONFIG_UART_CanOpen_2_PARITY		(kUART_PARITY_NONE)

/* 压力变送器  PT2077----------------------------------------------------------------------------------- */
#define CONFIG_PS_ZERO_VOL_PT207			(500)  // 压力变送器为0时，输出电压值，单位mV
#define CONFIG_PS_FULL_VOL_PT207			(2500) // 压力器变送器满量程时，输出电压值，单位mV
#define CONFIG_PS_DEFAULT_RANGE_PT207		(2.5)  // 压力变送器默认量程，单位MPa
#define CONFIG_PS_DEFAULT_LIMIT_PT207		(1.6)  // 压力变送器默认限压值，单位MPa
#define CONFIG_PS_DEFAULT_OVER_TIME_PT207	(1)    // 压力变送器默认过压时间，单位秒

/* 压力变送器  PT206----------------------------------------------------------------------------------- */
#define CONFIG_PS_ZERO_VOL_PT206			(500)  // 压力变送器为0时，输出电压值，单位mV
#define CONFIG_PS_FULL_VOL_PT206			(2500) // 压力器变送器满量程时，输出电压值，单位mV
#define CONFIG_PS_DEFAULT_RANGE_PT206		(2.5)  // 压力变送器默认量程，单位MPa
#define CONFIG_PS_DEFAULT_LIMIT_PT206		(1.6)  // 压力变送器默认限压值，单位MPa
#define CONFIG_PS_DEFAULT_OVER_TIME_PT206	(1)    // 压力变送器默认过压时间，单位秒


/* 温度变送器 ------------------------------------------------------------------------------------ */
#define CONFIG_TP_ZERO_VOL			(500)  // 压力变送器为0时，输出电压值，单位mV
#define CONFIG_TP_FULL_VOL			(2500) // 压力器变送器满量程时，输出电压值，单位mV
#define CONFIG_TP_DEFAULT_RANGE		(300)  // 压力变送器默认量程，单位MPa
#define CONFIG_TP_DEFAULT_LIMIT		(200)  // 压力变送器默认限压值，单位MPa
#define CONFIG_TP_DEFAULT_OVER_TIME	(1)    // 压力变送器默认过压时间，单位秒


// 流量计
#define CONFIG_UART_FLOW	     (DRVID_UART_3)
#define CONFIG_UART_FLOW_BAUD 	 (9600)
#define CONFIG_UART_FLOW_PARITY    (kUART_PARITY_NONE)


#endif /* INCLUDE_HQHP_CONFIG_H_ */
