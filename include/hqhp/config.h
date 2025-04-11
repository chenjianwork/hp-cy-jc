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
	MS_TMR_ID_CAN_OnLine_1,
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

#endif /* INCLUDE_HQHP_CONFIG_H_ */
