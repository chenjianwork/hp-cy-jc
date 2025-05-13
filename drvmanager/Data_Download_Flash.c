/*!
****************************************************************************************************
* 文件名称：Data_Download_Flash.c
* 功能简介：数据下载和Flash存储管理模块
* 文件作者：liyan
* 创建日期：2023年2月15日
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"
#include "drvmanager/drvmanager.h"
#include <hqhp/config.h>

/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/
// 通信相关定义
#define TX_BUF_SIZE           (32)    // 发送数据缓存大小，单位字节
#define RX_BUF_SIZE           (32)    // 接收数据缓存大小，单位字节
#define RX_FBUF_SIZE          (32)    // 接收数据缓存大小，单位字节
#define RX_FRAME_MIN_SIZE     (6)     // 单次接收最小帧长度，单位字节
#define RX_FRAME_MAX_SIZE     (32)    // 单次接收最大帧长度，单位字节
#define RX_BYTE_TIMEOUT       (100)   // 字节接收超时，单位毫秒

// Flash存储相关定义
#define DATA_FLASH_SAVE_NUM   (2)     // 存储数据个数
#define FLASH_SAVE_ADDR       ADDR_FLASH_SECTOR_4  // 扇区有64kb的大小，一般存几个数据已经足够

// Flash扇区地址定义
#define ADDR_FLASH_SECTOR_0   ((u32)0x08000000)   // 扇区0起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_1   ((u32)0x08004000)   // 扇区1起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_2   ((u32)0x08008000)   // 扇区2起始地址, 16 Kbytes
#define ADDR_FLASH_SECTOR_3   ((u32)0x0800C000)   // 扇区3起始地址, 16 Kbytes   flag
#define ADDR_FLASH_SECTOR_4   ((u32)0x08010000)   // 扇区4起始地址, 64 Kbytes   adc data
#define ADDR_FLASH_SECTOR_5   ((u32)0x08020000)   // 扇区5起始地址, 128 Kbytes  PLC code
#define ADDR_FLASH_SECTOR_6   ((u32)0x08040000)   // 扇区6起始地址, 128 Kbytes  APP
#define ADDR_FLASH_SECTOR_7   ((u32)0x08060000)   // 扇区7起始地址, 128 Kbytes  APP
#define ADDR_FLASH_SECTOR_8   ((u32)0x08080000)   // 扇区8起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_9   ((u32)0x080A0000)   // 扇区9起始地址, 128 Kbytes
#define ADDR_FLASH_SECTOR_10  ((u32)0x080C0000)   // 扇区10起始地址,128 Kbytes
#define ADDR_FLASH_SECTOR_11  ((u32)0x080E0000)   // 扇区11起始地址,128 Kbytes

/*!
****************************************************************************************************
* 类型定义
****************************************************************************************************
*/
// PLC管理结构体
struct _PLC_MGR {
    bool    IsOnline;          // 是否在线
    bool    IsHasWrOperation;  // 是否有写操作
    bool    HasFrame;          // 是否有帧
    int     UARTIdx;           // UART索引
    size_t  RxBytes;           // 接收字节数
};

/*!
****************************************************************************************************
* 全局变量
****************************************************************************************************
*/
// 下载相关变量
 uint8_t DownLoad_Step;                  // 下载步骤
 uint8_t analog_zj_Mode;                 // 校准模式
 uint16_t analog_zj_Chn_Flag;            // 模拟量校准通道标志
 uint8_t analog_zj_Complete_Flag;        // 校准完成标志
 uint8_t Data_DownLoad_Complete_Flag;    // 数据下载完成标志
 uint32_t Len_DownLoad_Data;             // 下载数据长度
 unsigned char Flag_Download;            // 下载标志
 unsigned int RxBytes;
// 通信缓冲区
static uint8_t txBuf_ACT[5] = {0Xaa, 0xcc, 0x3c, 0xc3, 0x33};
static uint8_t txBuf_ACT_jz[6] = {0Xaa, 0xcc, 0x00, 0x3c, 0xc3, 0x33};

// 配置数据缓冲区
extern unsigned char CAN_ID_Config_Inf[1024];
extern unsigned char Module_Config_Inf[1024*12];
extern unsigned char Analog_Config_Inf[1024];  // 0:类型，通道1：(1～4，最小参考值，5～8：最大参考值)......
extern unsigned char PLC_Prog_Code[1024];

// 校准数据缓冲区
 unsigned char jz_data[128];
 unsigned char jz_data_A[128];
 float jz_ADC_current[8];

// 触摸屏数据存储
static uint8_t Data_Arrays_Store[20][66];      // 触摸屏下载的数据存储数组

// PLC管理实例
static struct _PLC_MGR G_PLCMGR;

/*!
****************************************************************************************************
* 函数声明
****************************************************************************************************
*/
// Flash操作函数
static uint16_t STMFLASH_GetFlashSector(u32 addr);
static int write_flash(uint8_t *FlashWriteBuf, uint32_t num, uint32_t StartAddr);
static void read_flash(uint8_t *FlashReadBuf, uint32_t num, uint32_t StartAddr);
 int Stored_TO_FLASH(void);
 void Read_From_FLASH(void);

// 数据校验函数
 unsigned char CheckXor(const char *strData, unsigned int len);

// 数据存储函数
 void Store_Data(void);
 void Store_JZ_data(void);
 void LCD_Data_Store(void);
 void Read_JZ_LCD_data(void);

// PLC通信函数
 void TASKMGR_PLCInit(void);
 void TASKMGR_PLCRxByteCallback(int idx, uint8_t data);
 void DownLoad_Data_Deal(void);

/*!
****************************************************************************************************
* 函数实现
****************************************************************************************************
*/
//通过地址获取扇区位置
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10;
	return FLASH_Sector_11;
}

/*写入数据到FLASH*/
void Store_Data(void)
{
	unsigned int i,j;
	memset(SpecialRamBlock, 0, SPECIAL_RAM_BLOCK_SIZE);
	FEED();
	j=0;
	for(i=0;i<1024;i++)
	{
		SpecialRamBlock[i]=CAN_ID_Config_Inf[j];
		j++;
	}
	j=0;
	for(i=1024;i<2048;i++)
	{
		SpecialRamBlock[i]=Module_Config_Inf[j];
		j++;
	}
	j=0;
	for(i=2048;i<1024*3;i++)
	{
		SpecialRamBlock[i]=Analog_Config_Inf[j];
		j++;
	}
	j=0;
	for(i=1024*3;i<1024*12;i++)
	{
		SpecialRamBlock[i]=PLC_Prog_Code[j];
		j++;
	}
	FEED();
	Flag_Download=Stored_TO_FLASH();
	FEED();
	memset(SpecialRamBlock, 0, SPECIAL_RAM_BLOCK_SIZE);
	Init_USER_Code();
}


/*从FLASH读出数据,并存放入相应的4个buf*/
int Init_USER_Code(void)
{
	unsigned int i,j;

	j=0;
	Read_From_FLASH();

	for(i=0;i<1024;i++)
	{
		CAN_ID_Config_Inf[j]=SpecialRamBlock[i];
		j++;
	}
	j=0;
	for(i=1024;i<2048;i++)
	{
		Module_Config_Inf[j]=SpecialRamBlock[i];
		j++;
	}
	j=0;
	for(i=2048;i<1024*3;i++)
	{
		Analog_Config_Inf[j]=SpecialRamBlock[i];
		j++;
	}
	j=0;
	for(i=1024*3;i<1024*12;i++)
	{
		PLC_Prog_Code[j]=SpecialRamBlock[i];
		j++;
	}
	return SUCCESS_t;

}
//将数据写入内存 8位数据
/*
 * FlashWriteBuf ：数据buf
 * num:数据长度
 * StartAddr：flash地址
 *
 * */
int write_flash(uint8_t *FlashWriteBuf,uint32_t num,uint32_t StartAddr)
{

	//FEED();
	FLASH_Unlock();	//解锁
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	if (FLASH_COMPLETE != FLASH_EraseSector(STMFLASH_GetFlashSector(StartAddr),VoltageRange_3)) //擦除扇区内容
    {
		return ERROR_t;
	}
	//FEED();
	for (int i = 0; i < num; i++)
	{
		if (FLASH_COMPLETE != FLASH_ProgramByte(StartAddr, FlashWriteBuf[i]))	//写入8位数据
		{
			return ERROR_t;
		}
		StartAddr += 1;	//8位数据偏移两个位置
		//FEED();
	}

	FLASH_Lock();	//上锁

	return SUCCESS_t;
}

//从内存读数据 8位数据
void read_flash(uint8_t *FlashReadBuf,uint32_t num,uint32_t StartAddr)
{
	for (int i = 0; i < num; i++)
	{
		FlashReadBuf[i] = *(__IO uint8_t*)StartAddr;
		StartAddr += 1;
	}
}

int Stored_TO_FLASH(void)
{
	int flag;
	//FEED();
	flag=write_flash(SpecialRamBlock,SPECIAL_RAM_BLOCK_SIZE,ADDR_FLASH_SECTOR_5);
	//FEED();
	return flag;
}

void Read_From_FLASH(void)
{
	read_flash(SpecialRamBlock,SPECIAL_RAM_BLOCK_SIZE,ADDR_FLASH_SECTOR_5);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于初始化PLC通信任务模块
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void TASKMGR_PLCInit(void)
{
	G_PLCMGR.IsOnline		  = false;
	G_PLCMGR.IsHasWrOperation = false;
	G_PLCMGR.HasFrame		  = false;
	G_PLCMGR.RxBytes		  = 0;
	// 打开串口通道#1，并设置接收回调函数
	DRVMGR_UARTOpen(CONFIG_UART_RS485_1, CONFIG_UART_RS485_1_BAUDRATE, CONFIG_UART_RS485_1_PARITY);
	DRVMGR_UARTSetRxCallback(CONFIG_UART_RS485_1, TASKMGR_Engine_COMM1_RxByteCallback);
	// 打开串口通道#3，并设置接收回调函数
	DRVMGR_UARTOpen(CONFIG_UART_RS485_3, CONFIG_UART_RS485_3_BAUDRATE, CONFIG_UART_RS485_3_PARITY);
	DRVMGR_UARTSetRxCallback(CONFIG_UART_RS485_3, TASKMGR_Engine_COMM3_RxByteCallback);
	// 打开串口通道#2，并设置接收回调函数
	DRVMGR_UARTOpen(CONFIG_UART_PLC_1, CONFIG_UART_PLC_1_BAUDRATE, CONFIG_UART_PLC_1_PARITY);
	DRVMGR_UARTSetRxCallback(CONFIG_UART_PLC_1, TASKMGR_PLCRxByteCallback);

	// 启动接收超时定时器
	DRVMGR_MSTimerStart(MS_TMR_ID_PLC_RX_TIMEOUT, RX_BYTE_TIMEOUT);
}

/*!
****************************************************************************************************
* 功能描述：该方法用于处理从串口中接收到的字节数据，并完成定帧处理
* 注意事项：NA
* 输入参数：idx  -- 串口驱动标识
*          data -- 串口接收到的字节数据
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
void TASKMGR_PLCRxByteCallback(int idx, uint8_t data)
{
   unsigned int i;
   G_PLCMGR.UARTIdx=idx;
   DRVMGR_MSTimerStart(MS_TMR_ID_PLC_RX_TIMEOUT, RX_BYTE_TIMEOUT);

   // 标志为1代表上次的数据未处理完成，直接返回
   if (Data_DownLoad_Complete_Flag == 1) {
	   return;
   }
   FEED();
   //校准数据下发
   if(analog_zj_Mode==1)
   {
	   SpecialRamBlock[RxBytes] = data;
	   RxBytes++;
	   if((SpecialRamBlock[0]==0xaa)&&(SpecialRamBlock[1]==0x55)&&(SpecialRamBlock[7]==0xcc))
	   {
		   adc_jz_Deal(SpecialRamBlock[2]);
		   analog_zj_Chn_Flag = analog_zj_Chn_Flag|(1<<SpecialRamBlock[2]);//相应通道校准标志 置1;
		   RxBytes=0;
		   txBuf_ACT_jz[2]=SpecialRamBlock[2];

		   DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT_jz, 6);
		   DRVMGR_TimerDelayUs(10000);
		   for(i=0;i<8;i++)
		   {
			   SpecialRamBlock[i]=0;
		   }

	   }
	   if((SpecialRamBlock[0]==0xC3)&&(SpecialRamBlock[1]==0x3C)&&(SpecialRamBlock[2]==0xFF)&&(SpecialRamBlock[7]==0xcc))
	   {
		   RxBytes=0;
		   analog_zj_Complete_Flag=1;
		   txBuf_ACT_jz[2]=0xFF;

		   DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT_jz, 6);
		   DRVMGR_TimerDelayUs(10000);
		   for(i=0;i<8;i++)
		   {
			   SpecialRamBlock[i]=0;
		   }
	   }
   }
   if(analog_zj_Mode!=1)
   {

	   switch (DownLoad_Step)
	   {
	   case 0:
		   SpecialRamBlock[RxBytes] = data;
		   RxBytes++;
			if((SpecialRamBlock[0]==0xaf)&&(SpecialRamBlock[1]==0xaa)&&(SpecialRamBlock[6]==0xfa))
			{
				DownLoad_Step=1;
				analog_zj_Mode=0;
				RxBytes=0;
				for(i=0;i<8;i++)
					SpecialRamBlock[i] = 0x00;
				DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT, 5);
				DRVMGR_TimerDelayUs(10000);
				   for(i=0;i<8;i++)
				   {
					   SpecialRamBlock[i]=0;
				   }
			}
			if((SpecialRamBlock[0]==0xAA)&&(SpecialRamBlock[1]==0xCC)&&(SpecialRamBlock[2]==0x33)&&(SpecialRamBlock[7]==0xCC))
			{
				analog_zj_Mode=1;
				RxBytes=0;
				txBuf_ACT_jz[2]=0x55;
				DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT_jz, 6);
				DRVMGR_TimerDelayUs(10000);
				   for(i=0;i<8;i++)
				   {
					   SpecialRamBlock[i]=0;
				   }

			}
			break;
	   case 1:
		   SpecialRamBlock[RxBytes] = data;
		   if((RxBytes==0)&&(SpecialRamBlock[RxBytes]!=0xAA))
			   RxBytes=0;
		   else
			   RxBytes++;
			if ((SpecialRamBlock[0]==0xaa)&&(SpecialRamBlock[1]==0x01)&&(SpecialRamBlock[4]==0xcc)&&(RxBytes>=4))
			{
				Len_DownLoad_Data=(SpecialRamBlock[2]<<8)|SpecialRamBlock[3];
				DownLoad_Step=2;
			}
			break;
	   case 2:
		   SpecialRamBlock[RxBytes] = data;
		   RxBytes++;
			if(RxBytes>=Len_DownLoad_Data+8)
			{
				for(i=0;i<Len_DownLoad_Data;i++)
				{
					CAN_ID_Config_Inf[i]=SpecialRamBlock[i+8];
				}
				RxBytes=0;
				for(i=0;i<Len_DownLoad_Data;i++)
					SpecialRamBlock[i]=0x00;
				DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT, 5);
				DRVMGR_TimerDelayUs(10000);
				Len_DownLoad_Data=0;
				DownLoad_Step=3;
			}
			break;
	   case 3:
		   SpecialRamBlock[RxBytes] = data;
		   if((RxBytes==0)&&(SpecialRamBlock[RxBytes]!=0xAA))
			   RxBytes=0;
		   else
			   RxBytes++;
			if ((SpecialRamBlock[0]==0xaa)&&(SpecialRamBlock[1]==0x02)&&(SpecialRamBlock[4]==0xcc)&&(RxBytes>=4))
			{
				Len_DownLoad_Data=(SpecialRamBlock[2]<<8)|SpecialRamBlock[3];
				DownLoad_Step=4;
			}
			break;
	   case 4:
		   SpecialRamBlock[RxBytes] = data;
		   RxBytes++;
			if(RxBytes>=Len_DownLoad_Data+8)
			{
				for(i=0;i<Len_DownLoad_Data;i++)
				{
					Module_Config_Inf[i]=SpecialRamBlock[i+8];
				}
				RxBytes=0;
				for(i=0;i<Len_DownLoad_Data;i++)
					SpecialRamBlock[i]=0x00;
				DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT, 5);
				DRVMGR_TimerDelayUs(10000);
				Len_DownLoad_Data=0;
				DownLoad_Step=5;
			}
			break;
	   case 5:
		   SpecialRamBlock[RxBytes] = data;
		   if((RxBytes==0)&&(SpecialRamBlock[RxBytes]!=0xAA))
			   RxBytes=0;
		   else
			   RxBytes++;
			if ((SpecialRamBlock[0]==0xaa)&&(SpecialRamBlock[1]==0x03)&&(SpecialRamBlock[4]==0xcc)&&(RxBytes>=4))
			{
				Len_DownLoad_Data=(SpecialRamBlock[2]<<8)|SpecialRamBlock[3];
				DownLoad_Step=6;
			}
			break;
	   case 6:
		   SpecialRamBlock[RxBytes] = data;
		   RxBytes++;
			if(RxBytes>=Len_DownLoad_Data+8)
			{
				for(i=0;i<Len_DownLoad_Data;i++)
				{
					PLC_Prog_Code[i]=SpecialRamBlock[i+8];
				}
				RxBytes=0;
				for(i=0;i<Len_DownLoad_Data;i++)
					SpecialRamBlock[i]=0x00;
				DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT, 5);
				DRVMGR_TimerDelayUs(10000);
				Len_DownLoad_Data=0;
				DownLoad_Step=7;
			}
			break;
	   case 7:
		   SpecialRamBlock[RxBytes] = data;
		   if((RxBytes==0)&&(SpecialRamBlock[RxBytes]!=0xAA))
			   RxBytes=0;
		   else
			   RxBytes++;
			if ((SpecialRamBlock[0]==0xaa)&&(SpecialRamBlock[1]==0x04)&&(SpecialRamBlock[4]==0xcc)&&(RxBytes>=4))
			{
				Len_DownLoad_Data=(SpecialRamBlock[2]<<8)|SpecialRamBlock[3];
				DownLoad_Step=8;
			}
			break;
	   case 8:
		   SpecialRamBlock[RxBytes] = data;
		   RxBytes++;
			if(RxBytes>=Len_DownLoad_Data+8)
			{
				for(i=0;i<Len_DownLoad_Data;i++)
				{
					Analog_Config_Inf[i]=SpecialRamBlock[i+8];
				}
				RxBytes=0;
				for(i=0;i<8;i++)
					SpecialRamBlock[i]=0x00;
				DRVMGR_UARTSendBytes(DRVID_UART_2, txBuf_ACT, 5);
				DRVMGR_TimerDelayUs(10000);
				Len_DownLoad_Data=0;
				DownLoad_Step=0;
				Data_DownLoad_Complete_Flag=1;
			}
			break;
	   default :
		   break;
	   }
   }
}
void DownLoad_Data_Deal(void)
{
	if(Data_DownLoad_Complete_Flag==1)//用户程序存储
	{
		Store_Data();
		Init_CAN_ID();
		DRVMGR_CANInit();
		Data_DownLoad_Complete_Flag=0;
		DownLoad_Step=0;
	}
	if(analog_zj_Complete_Flag==1)//模拟量校准模式
	{
		Store_JZ_data();
		analog_zj_Mode=0;
		analog_zj_Complete_Flag=0;
	}
}

/*
 * *把触摸屏的数据存储到FLASH
 */
void LCD_Data_Store(void)
{
	unsigned int i,j;
	unsigned char n=0;
	uint8_t *P=&Data_Arrays_Store[0][0];
	uint8_t lcd_data_read[2048];
	memset(lcd_data_read, 0, sizeof(lcd_data_read));
	read_flash((uint8_t *)lcd_data_read,2048,ADDR_FLASH_SECTOR_4);
	if(Stored_Data_Flag==1)
	{
		for(i=0;i<10;i++)
			for(j=0;j<66;j++)
			{
				Data_Arrays_Store[i][j]=0;
			}

		for(i=0;i<50;i++)
		{
			 if(Stored_Data_Address[i]==0xAA)
			 {
				 Data_Arrays_Store[n][0]=0xAA;
				 Data_Arrays_Store[n][1]=i;//在二维存储数组(Data_Arrays)对应位置
				 for(j=0;j<64;j++)
					 {
						 Data_Arrays_Store[n][j+2]=Data_Arrays[i][j];
					 }
				 n++;
			 }
		}
	  for(i=0;i<1024;i++)
	  {
		  lcd_data_read[i]=*P++;
	  }
	 write_flash((uint8_t *)lcd_data_read,2048,ADDR_FLASH_SECTOR_4);

	}
}

/*写入数据到FLASH*/
void Store_JZ_data(void)
{
	uint8_t i;
	uint8_t jz_data_read[2048];
	memset(jz_data_read, 0, sizeof(jz_data_read));
	read_flash((uint8_t *)jz_data_read,2048,ADDR_FLASH_SECTOR_4);
	for(i=0;i<8;i++)
	{
		if((analog_zj_Chn_Flag>>i)&0x1)
		{
			jz_data_read[1024+i*4+0]=jz_data[i*4+0];
			jz_data_read[1024+i*4+1]=jz_data[i*4+1];
			jz_data_read[1024+i*4+2]=jz_data[i*4+2];
			jz_data_read[1024+i*4+3]=jz_data[i*4+3];
		}
	}
	write_flash((uint8_t *)jz_data_read,2048,ADDR_FLASH_SECTOR_4);
	Read_JZ_LCD_data();
}

void Read_JZ_LCD_data(void)
{
	unsigned int i,j;
	uint8_t n=0;
	uint8_t  JZ_LCD_DATA[2048];
	uint8_t *P=&Data_Arrays_Store[0][0];

	int jz_data_A_1[8];
	read_flash((uint8_t *)JZ_LCD_DATA,2048,ADDR_FLASH_SECTOR_4);
	for(i=0;i<8;i++)
	{
		jz_data_A_1[i]=(int32_t)((JZ_LCD_DATA[1024+i*4+0]<<0)|((JZ_LCD_DATA[1024+i*4+1]<<8)&0x0000FF00)|((JZ_LCD_DATA[1024+i*4+2]<<16)&0x00FF0000)|((JZ_LCD_DATA[1024+i*4+3]<<24)&0xFF000000));
		jz_ADC_current[i]=(float)(jz_data_A_1[i]/1000.00);
		if((jz_ADC_current[i]>2)||(jz_ADC_current[i]<-2))
	   {
		   jz_ADC_current[i]=0;
	   }
	}
	for(i=0;i<1024;i++)
	{
		*P++=JZ_LCD_DATA[i];
	}
	for(i=0;i<10;i++)
	{
		 if(Data_Arrays_Store[i][0]==0xAA)
		 {
			 n=Data_Arrays_Store[i][1];//在二维存储数组(Data_Arrays)对应位置
			 for(j=0;j<64;j++)
				 {
				   Data_Arrays[n][j]=Data_Arrays_Store[i][j+2];
				 }
		 }
	}
}
//
int EraseSector_test(void)
{
	FLASH_Unlock();	//解锁
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	if (FLASH_COMPLETE != FLASH_EraseSector(STMFLASH_GetFlashSector(ADDR_FLASH_SECTOR_4),VoltageRange_2)) //擦除扇区内容
    {
		FLASH_Lock();	//上锁
		return ERROR_t;
	}
	else
	{

		return SUCCESS_t;
	}

}







