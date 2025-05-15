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

// 定义SPI相关GPIO引脚
#define SPI_SCK_PIN      GPIO_Pin_10   // SCK时钟线，PB10
#define SPI_MOSI_PIN     GPIO_Pin_3    // MOSI主发数据，PC3
#define SPI_MISO_PIN     GPIO_Pin_2    // MISO主收数据，PC2
#define SPI_CS_ADC_PIN   GPIO_Pin_1    // 片选ADS1120，PC1
#define SPI_CS_DAC_PIN   GPIO_Pin_4    // 片选DAC8552，PC4
#define SPI_DRDY_PIN     GPIO_Pin_5    // ADS1120数据就绪，PC5

// 定义GPIO端口
#define SPI_SCK_PORT     GPIOB
#define SPI_MOSI_PORT    GPIOC
#define SPI_MISO_PORT    GPIOC
#define SPI_CS_PORT      GPIOC
#define SPI_DRDY_PORT    GPIOC

#define DAC_CMD_CH_A   0x10	// 写更新通道A
#define DAC_CMD_CH_B   0x24	// 写更新通道B


// 定义设备类型
#define DEVICE_ADC       0    // ADS1120
#define DEVICE_DAC       1    // DAC8552

// DAC8552控制字节定义
#define DAC8552_CMD_WRITE_A    0x10    // 写入并更新DAC-A
#define DAC8552_CMD_WRITE_B    0x24    // 写入并更新DAC-B
#define DAC8552_CMD_WRITE_ALL  0x30    // 同时更新两个DAC

// ADS1120命令定义
#define ADS1120_CMD_RESET      0x06    // 复位
#define ADS1120_CMD_START      0x08    // 启动/同步转换
#define ADS1120_CMD_POWERDOWN  0x02    // 掉电
#define ADS1120_CMD_RDATA      0x10    // 读取数据
#define ADS1120_CMD_RREG       0x20    // 读寄存器
#define ADS1120_CMD_WREG       0x40    // 写寄存器

//designation registers
#define ADC_REG0                    0x00 // 0000 0000
#define ADC_REG1                    0x01 // 0000 0001
#define ADC_REG2                    0x02 // 0000 0010
#define ADC_REG3                    0x03 // 0000 0011


//setings for REG0
#define ADC_REG0_MUX_AIN0_AIN1      0x00 // 0000 0000
#define ADC_REG0_MUX_AIN0_AIN2      0x10 // 0001 0000
#define ADC_REG0_MUX_AIN0_AIN3      0x20 // 0010 0000
#define ADC_REG0_MUX_AIN1_AIN2      0x30 // 0011 0000
#define ADC_REG0_MUX_AIN1_AIN3      0x40 // 0100 0000
#define ADC_REG0_MUX_AIN2_AIN3      0x50 // 0101 0000
#define ADC_REG0_MUX_AIN1_AIN0      0x60 // 0110 0000
#define ADC_REG0_MUX_AIN3_AIN2      0x70 // 0111 0000
#define ADC_REG0_MUX_AIN0_AVSS      0x80 // 1000 0000
#define ADC_REG0_MUX_AIN1_AVSS      0x90 // 1001 0000
#define ADC_REG0_MUX_AIN2_AVSS      0xA0 // 1010 0000
#define ADC_REG0_MUX_AIN3_AVSS      0xB0 // 1011 0000
#define ADC_REG0_MUX_MODE_VREF      0xC0 // 1100 0000
#define ADC_REG0_MUX_MODE_AVDD_AVSS 0xD0 // 1101 0000
#define ADC_REG0_MUX_MODE_14        0xE0 // 1110 0000
#define ADC_REG0_MUX_RESERVED       0xF0 // 1111 0000

#define ADC_REG0_GAIN1              0x00 // 0000 0000
#define ADC_REG0_GAIN2              0x02 // 0000 0010
#define ADC_REG0_GAIN4              0x04 // 0000 0100
#define ADC_REG0_GAIN8              0x06 // 0000 0110
#define ADC_REG0_GAIN16             0x08 // 0000 1000
#define ADC_REG0_GAIN32             0x0A // 0000 1010
#define ADC_REG0_GAIN64             0x0C // 0000 1100
#define ADC_REG0_GAIN128            0x0E // 0000 1110

#define ADC_REG0_PGA_BYPASS_ENABLE  0x00 // 0000 0000
#define ADC_REG0_PGA_BYPASS_DISABLE 0x01 // 0000 0001

//setings for REG1
#define ADC_REG1_DR_NORM_MODE_20SPS	0x00 // 0000 0000
#define ADC_REG1_DR_NORM_MODE_45SPS	0x20 // 0010 0000
#define ADC_REG1_DR_NORM_MODE_90SPS	0x40 // 0100 0000
#define ADC_REG1_DR_NORM_MODE_175SPS	0x60 // 0110 0000
#define ADC_REG1_DR_NORM_MODE_330SPS	0x80 // 1000 0000
#define ADC_REG1_DR_NORM_MODE_600SPS	0xA0 // 1010 0000
#define ADC_REG1_DR_NORM_MODE_1000SPS	0xC0 // 1100 0000

#define ADC_REG1_MODE_NORMAL		0x00 // 0000 0000
#define ADC_REG1_MODE_DUTY_CYCLE	0x08 // 0000 1000
#define ADC_REG1_MODE_TURBO			0x10 // 0001 0000
#define ADC_REG1_MODE_RESERVED		0x18 // 0001 1000

#define ADC_REG1_CM_SINGLE			0x00 // 0000 0000  //单次转换
#define ADC_REG1_CM_CONTINUOUS		0x04 // 0000 0100 //连续转换

#define ADC_REG1_TS_DISABLE			0x00 // 0000 0000
#define ADC_REG1_TS_ENABLE			0x02 // 0000 0010

#define ADC_REG1_BCS_OFF			0x00 // 0000 0000
#define ADC_REG1_BCS_ON				0x01 // 0000 0001

//setings for REG2
#define ADC_REG2_VREF_INTERNAL               0x00 // 0000 0000
#define ADC_REG2_VREF_EXTERNAL_REFP0_REFN0   0x40 // 0100 0000
#define ADC_REG2_VREF_EXTERNAL_REFP1_REFN1   0x80 // 1000 0000
#define ADC_REG2_VREF_ANALOG                 0xC0 // 1100 0000

#define ADC_REG2_FIR_NO             0x00 // 0000 0000
#define ADC_REG2_FIR_SIMULTANEOUS   0x10 // 0001 0000
#define ADC_REG2_FIR_50             0x20 // 0010 0000
#define ADC_REG2_FIR_60             0x30 // 0011 0000

#define ADC_REG2_PSW_OPEN           0x00 // 0000 0000
#define ADC_REG2_PSW_CLOSE          0x08 // 0000 1000

#define ADC_REG2_IDAC_OFF           0x00 // 0000 0000
#define ADC_REG2_IDAC_RESERVED      0x01 // 0000 0001
#define ADC_REG2_IDAC_50u           0x02 // 0000 0010
#define ADC_REG2_IDAC_100u          0x03 // 0000 0011
#define ADC_REG2_IDAC_250u          0x04 // 0000 0100
#define ADC_REG2_IDAC_500u          0x05 // 0000 0101
#define ADC_REG2_IDAC_1000u         0x06 // 0000 0110
#define ADC_REG2_IDAC_1500u         0x07 // 0000 0111

//setings for REG3
#define ADC_REG3_I1MUX_DISABLED     0x00 // 0000 0000
#define ADC_REG3_I1MUX_AIN0_REFP1   0x20 // 0010 0000
#define ADC_REG3_I1MUX_AIN1         0x40 // 0100 0000
#define ADC_REG3_I1MUX_AIN2         0x60 // 0110 0000
#define ADC_REG3_I1MUX_AIN3_REFN1   0x80 // 1000 0000
#define ADC_REG3_I1MUX_REFP0        0xA0 // 1010 0000
#define ADC_REG3_I1MUX_REFN0        0xC0 // 1100 0000
#define ADC_REG3_I1MUX_RESERVED     0xE0 // 1110 0000

#define ADC_REG3_I2MUX_DISABLED     0x00 // 0000 0000
#define ADC_REG3_I2MUX_AIN0_REFP1   0x04 // 0000 0100
#define ADC_REG3_I2MUX_AIN1         0x08 // 0000 1000
#define ADC_REG3_I2MUX_AIN2         0x0C // 0000 1100
#define ADC_REG3_I2MUX_AIN3_REFN1   0x10 // 0001 0000
#define ADC_REG3_I2MUX_REFP0        0x14 // 0001 0100
#define ADC_REG3_I2MUX_REFN0        0x18 // 0001 1000
#define ADC_REG3_I2MUX_RESERVED     0x1C // 0001 1100

#define ADC_REG3_DRDYM_ON           0x00 // 0000 0000
#define ADC_REG3_DRDYM_Off          0x02 // 0000 0010

#define ADC_REG3_RESERVED           0x00 // 0000 0000

//constant for calculate impedance
#define ADC_U_REF     2.5 //s

#define ADC_R_REF     994.4010318
#define ADC_U_REF_INTER     2.048
#define ADC_I_REF     1.0E-3
#define ADC_PRECISION 32768
#define ADC_GAIN      1.0
#define ADC_LIMIT_MIN      9000
#define ADC_LIMIT_MAX      27600
#define ADC_EXTREF_MIN     23600
#define ADC_EXTREF_MAX     30150
#define ADC_AIN0_MIN     17000
#define ADC_AIN0_MAX     22000
#define ADC_AIN1_MIN     19000
#define ADC_AIN1_MAX     25000
#define ADC_UNAP_MIN     12000
#define ADC_UNAP_MAX     14000
#define ADC_UREF_MIN     3200
#define ADC_UREF_MAX     4800
#define TEMP_STEP     0.03125
#define TEMP_BORDER     8192
#define TEMP_LOW        16385
#define TEMP_CORRECT    49152
#define TEMP_MINUS_SIXTY    1920

//constant for calculate temperature
#define ADC_PT100_CONST_C0 -245.19
#define ADC_PT100_CONST_C1 2.5293
#define ADC_PT100_CONST_C2 -0.066046
#define ADC_PT100_CONST_C3 4.0422E-3
#define ADC_PT100_CONST_C4 -2.0697E-6
#define ADC_PT100_CONST_C5 -0.025422
#define ADC_PT100_CONST_C6 1.6883E-3
#define ADC_PT100_CONST_C7 -1.3601E-6

#define ADS1120_REF_R_IA (119.5) //采样电阻,单位:欧

#define AD_MIN_mA (4.0)  // 4mA
#define AD_MAX_mA (20.0)  // 20mA
#define ADC_BUFFER_SIZE (2)



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

struct _TIMER1 {
	bool	 Enable;   // 使能
	int32_t	 Interval; // 定时间隔，单位毫秒
	uint32_t DueTime;  // 到期时间，单位毫秒
};




#define RUN_LED_ON_TIME (200) // 运行指示灯每秒钟点亮时间，单位毫秒
#define RUN_LED_Fast_ON_TIME (100) // 运行指示灯每秒钟点亮时间，单位毫秒
#define RUN_CAN_OnLine_TIME (2) // 秒钟点亮时间，单位秒



// ---------------------------------------------------------------------------------------------- //
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

#define RUNSTATE_RUNNING_CANID   0x00002F30		//备机状态CANID
#define Arbitration_CANID 0x00A10550  //data:0X55,0x00,0x00,0x00,0xcc,0x3c,0xc3,0x33
/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
unsigned char CheckXor(const char *strData,unsigned int len);
extern unsigned char Cycle_Flag;

 char res_CmdAnalyze,res_CAN_S;
// 下载相关变量
 uint8_t DownLoad_Step;                  // 下载步骤
 uint8_t analog_zj_Mode;                 // 校准模式
 uint16_t analog_zj_Chn_Flag;            // 模拟量校准通道标志
 uint8_t analog_zj_Complete_Flag;        // 校准完成标志
 uint8_t Data_DownLoad_Complete_Flag;    // 数据下载完成标志
 uint32_t Len_DownLoad_Data;             // 下载数据长度
 unsigned char Flag_Download;            // 下载标志
 unsigned int RxBytes;



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
extern uint16_t DATA_REC_COMPLETE1,DATA_REC_COMPLETE2;
extern uint16_t State_get_data;

uint16_t ADC1120Data[4];



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
void 	DRVMGR_CPUInit(void);
void 	DRVMGR_CPUHandle(void);
void 	DRVMGR_LEDInit(void);
void 	DRVMGR_LEDHandle(void);
void 	DRVMGR_LEDTwinkle(void);
void 	DRVMGR_GPIOInit(void);
void 	Scan_Digital_Input(void);
void  	Digital_PutOut(void);
void 	DRVMGR_I2CInit(void);
void 	DRVMGR_I2CHandle(void);
void 	DRVMGR_SPIInit(void);
void 	DRVMGR_SPIHandle(void);
void 	DRVMGR_BOOTInit(void);
void 	DRVMGR_BOOTHandle(void);
void 	DRVMGR_TimerInit(void);
void 	DRVMGR_TimerHandle(void);
void 	DRVMGR_UARTInit(void);
void 	DRVMGR_UARTHandle(void);
void 	DRVMGR_CANInit(void);
void 	GPIOInit(void);
void 	Read_From_FLASH(void);
int 	Stored_TO_FLASH(void);
void 	ADC_Data_Deal(void);
int 	CMD_analyze(void);
void 	Download_Mode(void);
void 	LCD_Data_Read(void);
void 	LCD_Data_Store(void);
void 	DownLoad_Data_Deal(void);
void 	enable_PutOut(void);
void 	Read_JZ_LCD_data(void);
uint16_t ADC_CJ_MUX(unsigned char Chn);
void 	adc_jz_Deal(unsigned char Chn);
void 	Store_JZ_data(void);
int 	EraseSector_test(void);
void 	store_data(unsigned int a,unsigned int b,int value);
void 	Read_JZ_LCD_data(void);
void 	Init_Data_Arrays(void);
void 	rst_bit62_Data_Arrays(void);
extern void Engine_Data_Request(int idx,uint8_t Slave_address,int16_t flag_com,_Engine_DATA Engine_Parameter_IN);
extern void TASKMGR_Engine_COMM1_RxByteCallback(int idx, uint8_t data);
extern void TASKMGR_Engine_COMM3_RxByteCallback(int idx, uint8_t data);
void delay_ms(uint32_t s);
extern void init_edge(void);

/*!
****************************************************************************************************
* drvmanager-ads1120.c
****************************************************************************************************
*/
uint8_t Ads1120_SPI_RW(uint8_t byte);
void adc_set_reg(uint8_t reg, uint8_t value);
void adc_set_regs(uint8_t start_reg, uint8_t number_reg, uint8_t values[]);
void adc_reset(void);
void adc_start(void);
void adc_buffer_clear(uint8_t buffer[]);
void Ads1120_Restart(void);
// 函数声明
void DRVMGR_ADS1120_Init(void);
void ADS1120_ConfigChannel(uint8_t channel);
void ADS1120_ReadAllChannels(void);
void DRVMGR_ADS1120_Handle(void);
/*!
****************************************************************************************************
* drvmanager-dac8552.c
****************************************************************************************************
*/

void DAC8552_SPI_SendByte(uint8_t byte);
void DAC8552_WriteChannel(uint8_t channel, uint16_t data);
void DRVMGR_DACSetOutputCurrent(uint8_t chN, float mA);
void DEVMGR_DAC8552_Init(void);
void DEVMGR_DAC8552_Hand(void);

void SPI_SetCS(uint8_t device, uint8_t level);
void SPI_SetMOSI(uint8_t level);
void SPI_SetSCK(uint8_t level);
/*!
****************************************************************************************************
* 内联函数
****************************************************************************************************
*/

#ifdef __cplusplus
}
#endif
#endif /* DRVMANAGER_DRVMANAGER_H_ */
