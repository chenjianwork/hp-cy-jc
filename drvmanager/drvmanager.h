/*!
****************************************************************************************************
* 文件名称：drvmanager.h
* 功能简介：该文件是驱动管理器模块的私有头文件
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#ifndef DRVMANAGER_DRVMANAGER_H_
#define DRVMANAGER_DRVMANAGER_H_

/*!
****************************************************************************************************
* 包含文件
****************************************************************************************************
*/
#include <hqhp/drvmanager.h>
#ifdef __cplusplus
extern "C" {
#endif

#define FVA401T13
//#define WEIFANG

#ifdef  FVA401T13
#define CMD_MUDBUS 0x04
#define Len_Engine_Data 38
#endif
#ifdef  WEIFANG
#define CMD_MUDBUS 0x03
#define Len_Engine_Data 94
#endif

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
#define SPECIAL_RAM_BLOCK_SIZE	(16 * 1024)

#define  ERROR_t    -1
#define  SUCCESS_t  0
#define Len_CanIDConfiginf 1024
#define Len_ModuleConfiginf 1024
#define Len_AnalogConfiginf 1024
#define Len_PLCProgCodef 12288

#define spi1_cs_H GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define spi1_cs_L GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define spi1_clk_H GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define spi1_clk_L GPIO_ResetBits(GPIOA, GPIO_Pin_5)

#define spi1_mosi_H GPIO_SetBits(GPIOA, GPIO_Pin_7)
#define spi1_mosi_L GPIO_ResetBits(GPIOA, GPIO_Pin_7)

#define spi1_miso_d GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)

#define DEV_BOOT_PKT_SIZE	  (128)
#define DEV_BOOT_PAGE_SIZE	  (1024)
#define DEV_BOOT_PKT_PER_PAGE (DEV_BOOT_PAGE_SIZE / DEV_BOOT_PKT_SIZE)
#define Vref_adc 3.290
extern float V_4mA;

enum {
	BOOT_STATE_READY,
	BOOT_STATE_STARTED,
	BOOT_STATE_CHECKSUM
};

typedef struct  {
	int16_t			 Engine_Speed;    //转速 s
	int16_t			 Oil_pressure;//机油压力（滤后）
	int16_t			 Freshwater_Temperature;//淡水温度 A s
	int16_t			 Engine_oil_Temperature;//机油温度 s
	int16_t			 Inlet_Pressure_1;//进气压力 1
	int16_t			 Inlet_Pemperature_1;//进气温度 1
	int16_t			 Fuel_Pressure;//燃油压力

}_Engine_DATA;

#define RUN_LED_ON_TIME (200) // 运行指示灯每秒钟点亮时间，单位毫秒
#define RUN_LED_Fast_ON_TIME (100) // 运行指示灯每秒钟点亮时间，单位毫秒
#define RUN_CAN_OnLine_TIME (2) // 秒钟点亮时间，单位秒



// ---------------------------------------------------------------------------------------------- //
// 引导区





// 引导区
#define FLASH_BOOT_BASE (0x08000000u)
#define FLASH_BOOT_SIZE (48 * 1024u)

// 应用区
#define FLASH_APP_BASE (0x08040000)
#define FLASH_APP_SIZE (256 * 1024u) // 最大不能超过768K，这里选择512K可以缩短擦除时间

// 标志区
#define FLASH_FLAG_BASE (0x0800C000)
#define FLASH_FLAG_SIZE (16 * 1024u)

// 缓存区
#define FLASH_CACHE_BASE (0x08120000)
#define FLASH_CACHE_SIZE (512 * 1024u) // 最大不能超过896K，这里选择512K可以缩短擦除时间
/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/

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
void DRVMGR_CPUInit(void);
void DRVMGR_CPUHandle(void);

void DRVMGR_LEDInit(void);
void DRVMGR_LEDHandle(void);
void DRVMGR_LEDTwinkle(void);

void DRVMGR_GPIOInit(void);
void Scan_Digital_Input(void);
void  Digital_PutOut(void);

void DRVMGR_I2CInit(void);
void DRVMGR_I2CHandle(void);

void DRVMGR_SPIInit(void);
void DRVMGR_SPIHandle(void);

void DRVMGR_BOOTInit(void);
void DRVMGR_BOOTHandle(void);

void DRVMGR_TimerInit(void);
void DRVMGR_TimerHandle(void);

void DRVMGR_UARTInit(void);
void DRVMGR_UARTHandle(void);


void DRVMGR_CANInit(void);
void CAN1_rx_data(void);
void CAN2_rx_data(void);
void CAN_send_data(void);
char Scan_Can_err(void);

void GPIOInit(void);
void Read_From_FLASH(void);
int Stored_TO_FLASH(void);
void ADC_Data_Deal(void);
int CMD_analyze(void);
void Download_Mode(void);
void LCD_Data_Read(void);
void LCD_Data_Store(void);
void DownLoad_Data_Deal(void);
void enable_PutOut(void);
void Read_JZ_LCD_data(void);
uint16_t ADC_CJ_MUX(unsigned char Chn);
void adc_jz_Deal(unsigned char Chn);
void Store_JZ_data(void);
int EraseSector_test(void);
void store_data(unsigned int a,unsigned int b,int value);
void Read_JZ_LCD_data(void);
void Init_Data_Arrays(void);
void rst_bit62_Data_Arrays(void);

unsigned char CheckXor(const char *strData,unsigned int len);
extern unsigned char Cycle_Flag;

extern uint8_t DownLoad_Step;
extern uint8_t Data_DownLoad_Complete_Flag;
extern uint8_t analog_zj_Mode;
extern uint8_t analog_zj_Complete_Flag;
extern char res_CmdAnalyze,res_CAN_S;

extern unsigned int CAN_ID_Num;//CAN ID 数量
extern unsigned int cnt_send_numb;//本地发送CAN ID数量
extern unsigned char Arbitration_CAN;//0:CAN1 1:CAN2
extern unsigned int Store_CANID[256][2];
extern unsigned char Data_Arrays[256][64];

extern unsigned int CAN_ID_TX_DigitIn;
extern unsigned int CAN_ID_TX_DigitOut;
extern unsigned int CAN_ID_TX_Ain;
extern unsigned int CAN_ID_TX_DigitVar;
extern unsigned char iAcnt;

extern unsigned char Analog_Config_Inf[Len_AnalogConfiginf];

extern unsigned char Stored_Data_Flag;
extern unsigned char Stored_Data_Address[50];
extern unsigned char jz_data[128];
extern unsigned char jz_data_A[128];
extern float jz_ADC_current[8];
extern uint16_t analog_zj_Chn_Flag;
#define Arbitration_CANID 0x00A10550  //data:0X55,0x00,0x00,0x00,0xcc,0x3c,0xc3,0x33
extern void Engine_Data_Request(int idx,uint8_t Slave_address,int16_t flag_com,_Engine_DATA Engine_Parameter_IN);
extern void TASKMGR_Engine_COMM1_RxByteCallback(int idx, uint8_t data);
extern void TASKMGR_Engine_COMM3_RxByteCallback(int idx, uint8_t data);
extern uint16_t DATA_REC_COMPLETE1,DATA_REC_COMPLETE2;
extern uint16_t State_get_data;
extern void init_edge(void);
/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/

#ifdef __cplusplus
}
#endif
#endif /* DRVMANAGER_DRVMANAGER_H_ */
