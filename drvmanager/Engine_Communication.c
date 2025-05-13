/*
 * Engine_Communication.c
 *
 *  Created on: 2024年10月26日
 *      Author: 李彦坤
 */

#include <string.h>
#include "stm32f4xx.h"
#include "drvmanager/drvmanager.h"
#include <hqhp/config.h>
#include <stdio.h>



uint16_t State_get_data;



//typedef struct  {
//	uint16_t			 Engine_Speed;    //转速
//	uint16_t			 Security_Rotation_Speed;//安保转速
//	uint16_t			 Oil_pressure_filtered;//机油压力（滤后）
//	uint16_t			 Security_oil_Pressure;//安保机油压力
//	uint16_t			 Oil_pressure_Pre_filter;//机油压力（滤前）
//	uint16_t			 Freshwater_Temperature_A;//淡水温度 A
//	uint16_t			 Freshwater_Temperature_B;//淡水温度 B
//	uint16_t			 Freshwater_Pressure;//淡水压力
//	uint16_t			 Engine_oil_Temperature;//机油温度
//	uint16_t			 Turbine_Speed_1;//涡轮转速 1
//	uint16_t			 Turbine_oil_Pressure_1;//涡轮机油压力 1
//	uint16_t			 Turbine_oil_Temperature_1;//涡轮机油温度 1
//	uint16_t			 Turbine_Speed_2;//涡轮转速 2
//	uint16_t			 Turbine_oil_Pressure_2;//涡轮机油压力 2
//	uint16_t			 Turbine_oil_Temperature_2;//涡轮机油温度 2
//	uint16_t			 Starting_air_Pressure;//起动空气压力
//	uint16_t			 Control_air_Pressure;//控制空气压力
//	uint16_t			 Boost_Pressure_1;//增压压力 1
//	uint16_t			 Boost_Temperature_1;//增压温度 1
//	uint16_t			 Boost_Pressure_2;//增压压力 2
//	uint16_t			 Boost_Temperature_2;//增压温度 2
//	uint16_t			 Inlet_Pressure_1;//进气压力 1
//	uint16_t			 Inlet_Pemperature_1;//进气温度 1
//	uint16_t			 Inlet_Pressure_2;//进气压力 2
//	uint16_t			 Inlet_Pemperature_2;//进气温度 2
//	uint16_t			 Fuel_Pressure;//燃油压力
//	uint16_t			 Fuel_Temperature;//燃油温度
//	uint16_t			 Seawater_Pressure;//海水压力
//	uint16_t			 Water_Temperature;//海水温度
//	uint16_t			 Fuel_leakage_Pressure;//燃油泄漏压力
//	uint16_t			 Poor_lubricating_oil_filter;//滑油滤器差
//	uint16_t			 Main_power_supply_voltage;//主电源电压
//
//}_Engine_DATA;


uint8_t Engine_Data_Host[256];
uint8_t Rx_1_Bytes;

uint8_t Engine_Data_Auxiliary[256];
uint8_t Rx_3_Bytes;

/************************************************************************************
  多项式为G(x)=x^16+x^15+x^2+1    CRC寄存器预置值为0x0000   多项式写为0xa001
************************************************************************************/

uint16_t const crc16_table[256] =
{
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/*****************************************************************************
** 函数名:
** 描述:
*****************************************************************************/
uint16_t crc16_byte(uint16_t crc, uint8_t data)
{
   return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

/*****************************************************************************
** 函数名:
** 描述:  crc:crc寄存器的初值    	buffer：指向数组的指针	  len：数组长度
*****************************************************************************/
uint16_t ComputeCRC(uint8_t *buffer,uint16_t len1)	//gxm20110406
{
	uint16_t crc;		//初值
	crc=0;
	while (len1--)
	{
		crc = crc16_byte(crc, *buffer++);
	}
	return crc;
}
/*****************************************************************************
** 函数名:(多项式与基础的不同)
** 描述:  crc:crc寄存器的初值    	buffer：指向数组的指针	  len：数组长度
*****************************************************************************/
uint16_t ComputeCRC1(uint8_t *buffer,uint16_t len1)	//gxm20110406
{
	uint16_t crc;		//初值
	crc=0xffff;
	while (len1--)
	{
		crc = crc16_byte(crc, *buffer++);
	}
	return crc;
}

// ******************************************
//Write_Engine_Data
//入口：Slave_address:从机地址；Start_Addr：读取从机数据的起始地址；Len：读取从机数据长度
//出口：
//说明：写发动机数据
// ******************************************
void Read_Engine_Data_CMD(int idx,uint8_t Slave_address,uint8_t cmd,uint16_t Start_Addr,uint16_t Len)
{
	unsigned int intTemp;

	uint8_t TX1_Buffer[32];
	TX1_Buffer[0] = Slave_address;  //从机地址
	TX1_Buffer[1] = cmd;  //功能码
	TX1_Buffer[2] = Start_Addr>>8;  //起始地址2字节
	TX1_Buffer[3] = Start_Addr;  //起始地址2字节
	TX1_Buffer[4] = Len>>8;
	TX1_Buffer[5] = Len;
	intTemp = ComputeCRC1(TX1_Buffer,6);
	TX1_Buffer[6] = (unsigned char)(intTemp);
	TX1_Buffer[7] = (unsigned char)(intTemp>>8);
	if(idx==1)
	{
	  DRVMGR_UARTSendBytes(DRVID_UART_1, TX1_Buffer, 8);
	}
	else if(idx==3)
	{
		DRVMGR_UARTSendBytes(DRVID_UART_3, TX1_Buffer, 8);
	}

}
// ******************************************
//Write_Engine_Data
//入口：Slave_address:从机地址；
//出口：
//说明：写发动机数据
// ******************************************
int16_t Deal_data(void)
{
	int16_t DAT;
	DAT=((Engine_Data_Host[3]<<8)|Engine_Data_Host[4]);
	return DAT;
}
void Engine_Data_Request(int idx,uint8_t Slave_address,int16_t flag_com,_Engine_DATA Engine_Parameter_IN)
{
	Read_Engine_Data_CMD(idx,Slave_address,CMD_MUDBUS,0,Len_Engine_Data);//发动机转速
//	switch (State_get_data)
//	{
//	case 0:
//		Read_Engine_Data_CMD(idx,Slave_address,CMD_MUDBUS,0,Len_Engine_Data);//发动机转速
//		if(flag_com==1)
//		{
//			flag_com=0;
//			State_get_data=1;
//			Engine_Parameter_IN.Engine_Speed=Deal_data();
//		}
//		break;
//	case 1:
//		Read_Engine_Data_CMD(idx,Slave_address,CMD_MUDBUS,1,Len_Engine_Data);//机油压力
//		if(flag_com==1)
//		{
//			flag_com=0;
//			State_get_data=1;
//			Engine_Parameter_IN.Oil_pressure=Deal_data();
//		}
//		break;
//	case 2:
//		Read_Engine_Data_CMD(idx,Slave_address,CMD_MUDBUS,2,Len_Engine_Data);//水温
//		if(flag_com==1)
//		{
//			flag_com=0;
//			State_get_data=1;
//			Engine_Parameter_IN.Freshwater_Temperature=Deal_data();
//		}
//		break;
//	case 3:
//		Read_Engine_Data_CMD(idx,Slave_address,CMD_MUDBUS,3,Len_Engine_Data);//机油温度
//		if(flag_com==1)
//		{
//			flag_com=0;
//			State_get_data=1;
//			Engine_Parameter_IN.Engine_oil_Temperature=Deal_data();
//		}
//		break;
//	case 4:
//		Read_Engine_Data_CMD(idx,Slave_address,CMD_MUDBUS,7,Len_Engine_Data);//燃气压力
//		if(flag_com==1)
//		{
//			flag_com=0;
//			State_get_data=1;
//			Engine_Parameter_IN.Fuel_Pressure=Deal_data();
//		}
//		break;
//	default:
//		break;
//	}

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

void TASKMGR_Engine_COMM1_RxByteCallback(int idx, uint8_t data)
{
	unsigned int intTemp;
	Engine_Data_Host[Rx_1_Bytes++] = data;
	if((Engine_Data_Host[0]==1)&&(Engine_Data_Host[0]==CMD_MUDBUS)&&(Rx_1_Bytes>=(Len_Engine_Data+5)))
	{
		intTemp = ComputeCRC1(Engine_Data_Host,Rx_1_Bytes-2);
		if((((Engine_Data_Host[Rx_1_Bytes-2]<<8)&0xFF00)|(Engine_Data_Host[Rx_1_Bytes-1]&0X00FF))==intTemp)
		{
			//memcpy(&Engine_Parameter_Host.Engine_Speed, &Engine_Data_Host[3], 64);
			DATA_REC_COMPLETE1=1;
			Deal_data();
		}

	}
}

void TASKMGR_Engine_COMM3_RxByteCallback(int idx, uint8_t data)
{
	unsigned int intTemp;
	Engine_Data_Auxiliary[Rx_1_Bytes++] = data;
	if((Engine_Data_Auxiliary[0]==1)&&(Engine_Data_Auxiliary[0]==0x03)&&(Rx_1_Bytes>=(Len_Engine_Data+5)))
	{
		intTemp = ComputeCRC1(Engine_Data_Auxiliary,Rx_1_Bytes-2);
		if((((Engine_Data_Auxiliary[Rx_1_Bytes-2]<<8)&0xFF00)|(Engine_Data_Auxiliary[Rx_1_Bytes-1]&0X00FF))==intTemp)
		{
			//memcpy(&Engine_Parameter_Auxiliary_1.Engine_Speed, &Engine_Data_Auxiliary[3], 64);
			DATA_REC_COMPLETE2=1;
			Deal_data();
		}

	}
}
