/*
 * commanager.h
 *
 *  Created on: 2025年4月8日
 *      Author: 47015
 */

#ifndef COMMANAGER_COMMANAGER_H_
#define COMMANAGER_COMMANAGER_H_


/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/config.h>
#include <hqhp/defs.h>
#include <hqhp/bitconverter.h>

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

/*接口函数*/

void 	COMMGR_Init(void);
void 	COMMGR_Handle(void);

/************************************CAN总线函数* ********************************************/
void 	CAN1_tx_data(void);
void 	CAN2_tx_data(void);
void 	CAN1_rx_data(void);
void 	CAN2_rx_data(void);
char 	Scan_Can_err(void);
void 	CAN_send_data(void);
void 	CAN_Frame_init(void);
void 	CAN_Send_OnLine(void);



/************************************DEBUG函数* ********************************************/
void 	COMMGR_DBGInit(void);
void 	COMMGR_DBGHandle(void);
/************************************vdf-Modbus函数* ********************************************/
void COMMGR_VFDInit(void);
void COMMGR_VFDHandle(void);
int COMMGR_VFDSetFrequency(float freq_hz);
int COMMGR_VFDGetFrequency(float *freq_hz);
int COMMGR_VFDGetCurrent(float *current);

int COMMGR_VFDGetVoltage(float *voltage);
int COMMGR_VFDGetSpeed(float *speed);
int COMMGR_VFDGetFault(uint16_t *fault);
bool COMMGR_VFDIsOnline(void);
int COMMGR_VFDStart(void);
int COMMGR_VFDStartReverse(void);
int COMMGR_VFDStop(void);
int COMMGR_VFDResetFault(void);
void COMMGR_VFD_WriteFrequency(float freq);




/*!
****************************************************************************************************
* 功能描述：获取CAN总线运行状态
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：0 - 空闲状态，1 - 备机状态
****************************************************************************************************
*/
uint8_t COMMGR_GetRunStateCAN(void);



#endif /* COMMANAGER_COMMANAGER_H_ */
