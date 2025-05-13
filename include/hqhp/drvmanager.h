/*!
****************************************************************************************************
* 文件名称：drvmanager.h
* 功能简介：该文件是驱动管理器模块的接口头文件
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef INCLUDE_HQHP_DRVMANAGER_H_
#define INCLUDE_HQHP_DRVMANAGER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/config.h>
#include <hqhp/defs.h>
#ifdef __cplusplus
extern "C" {
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define TMR_ID_NUMS	   64 // 秒软定时器个数上限
#define MS_TMR_ID_NUMS 64 // 毫秒软定时器个数上限

#define DRV_UART_BAUD2400	2400
#define DRV_UART_BAUD4800	4800
#define DRV_UART_BAUD9600	9600
#define DRV_UART_BAUD19200	19200
#define DRV_UART_BAUD38400	38400
#define DRV_UART_BAUD57600	57600
#define DRV_UART_BAUD115200 115200
#define DRV_UART_BAUD230400 230400

enum {
	DRVID_LED_RUN = 0, // 运行指示灯
	DRVID_LED_FAULT,   // 故障指示灯
};

enum {
	DRVID_SPI_1 = 0,
	DRVID_SPI_2,
	DRVID_SPI_3,
	DRVID_SPI_MAXIMUM
};

enum {
	DRVID_SPI_3_CS_SRAM = 0,
	DRVID_SPI_3_CS_W25Q,
	DRVID_SPI_3_CS_FRAM,
	DRVID_SPI_3_CS_MAXIMUM
};

enum {
	DRVID_UART_1 = 0,
	DRVID_UART_2,
	DRVID_UART_3,
	DRVID_UART_4,
	DRVID_UART_5,
	DRVID_UART_MAXIMUM
};

enum {
	kUART_PARITY_NONE, // 无校验
	kUART_PARITY_EVEN, // 偶校验
	kUART_PARITY_ODD   // 奇校验
};

enum _IO_ERR {
	IO_ERROR_NONE = 0,
	IO_ERROR_INVALID_VALUE,
	IO_ERROR_INVALID_POINTER,
	IO_ERROR_INVALID_DEVID,

	IO_ERROR_FLASH_NOINIT,
	IO_ERROR_FLASH_ERASE_FAILURE,
	IO_ERROR_FALSH_WRITE_FAILURE,
	IO_ERROR_FLASH_READ_FAILURE,

	IO_ERROR_BOOT_NOT_START = 0x0100,
	IO_ERROR_BOOT_INV_FILE_NAME,
	IO_ERROR_BOOT_INV_FILE_LENGTH,
	IO_ERROR_BOOT_INV_FILE_CHECKSUM,
	IO_ERROR_BOOT_INV_FILE_PKT_SIZE,
	IO_ERROR_BOOT_INV_FILE_PKT_INDEX,
	IO_ERROR_BOOT_EARSE_FLASH_FAILURE,
	IO_ERROR_BOOT_WRITE_FLASH_FAILURE,
	IO_ERROR_BOOT_SAVE_FLAG_FAILURE
};

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
typedef enum _IO_ERR IO_ERR;
typedef void (*DRV_UART_RX_CALLBACK)(int idx, uint8_t data);

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/

/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
void DRVMGR_GPIOInit(void);

void DRVMGR_Init(void);
void DRVMGR_Handle(void);

void DRVMGR_CPUReset(void);

void DRVMGR_ADCInqChnValue(float aiValues[8]);

void DRVMGR_LEDTurnOn(void);
void DRVMGR_LEDTurnOff(void);
void DRVMGR_LEDTwinkel(int idx);

bool DRVMGR_I2CCheckDevice(uint8_t address);
void DRVMGR_I2CRdBytes(uint8_t devAddr, uint8_t memAddr, uint8_t* data, size_t bytes);
void DRVMGR_I2CWrBytes(uint8_t devAddr, uint16_t memAddr, const uint8_t* data, size_t bytes);

void DRVMGR_SPISelectChip(int idx);
bool DRVMGR_SPIRWBytes(int idx, const uint16_t* txData, uint16_t* rxData, size_t bytes);
bool DRVMGR_SPIReadBytes(int idx, uint8_t* data, size_t bytes);
bool DRVMGR_SPISendBytes(int idx, const uint8_t* data, size_t bytes);

void DRVMGR_TimerDelayUs(uint16_t us);
void DRVMGR_TimerStart(int idx, uint16_t limit);
void DRVMGR_TimerCancel(int idx);
bool DRVMGR_TimerIsExpiration(int idx);
void DRVMGR_MSTimerStart(int idx, uint16_t limit);
void DRVMGR_MSTimerCancel(int idx);
bool DRVMGR_MSTimerIsExpiration(int idx);

void   DRVMGR_UARTOpen(int idx, uint32_t baudRate, uint8_t parity);
void   DRVMGR_UARTSetRxCallback(int idx, DRV_UART_RX_CALLBACK callback);
size_t DRVMGR_UARTSendBytes(int idx, const uint8_t* buf, size_t bytes);
void TASKMGR_PLCRxByteCallback(int idx, uint8_t data);
void TASKMGR_Engine_COMM_RxByteCallback(int idx, uint8_t data);

//void TASKMGR_PLCInit(void);

IO_ERR DRVMGR_BOOTStart(uint8_t* fileName, uint32_t fileLength, uint32_t fileChkSum);
IO_ERR DRVMGR_BOOTReceive(uint16_t pktIndex, uint8_t* buf, uint8_t len, uint16_t* expectedPktIdx);
IO_ERR DRVMGR_BOOTReceiveEnd(uint16_t pktIndex, uint8_t* buf, uint8_t len);
IO_ERR DRVMGR_BOOTActivatePrograme(void);
void   DRVMGR_BOOTJumpToAppPrograme(void);
bool   DRVMGR_BOOTIsCanJumpToApplication(void);
void   DRVMGR_BOOTIsCanJumpToUserApplication(bool isCanJump);
void   DRVMGR_BOOTSetDbgPrint(void (*DbgPrint)(const char* str));
void Init_CAN_ID(void);
void FEED(void);
void scan_RTI(void);
int Init_USER_Code(void);
void CAN_Send_OnLine(void);
extern unsigned char  ACK_can[8];
void CAN_Frame_init(void);
extern unsigned int Arbitration_CANID_TX;
extern uint8_t DownLoad_Step;

extern unsigned char *SpecialRamBlock;

/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_HQHP_DRVMANAGER_H_ */
