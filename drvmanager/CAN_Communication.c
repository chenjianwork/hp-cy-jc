/*
 * CAN_Communication.c
 *
 *  Created on: 2023年2月16日
 *      Author: liyan
 */
#include "stm32f4xx.h"
#include "drvmanager.h"
#include "stm32f4xx_can.h"

CanRxMsg CanRxMsg_1;
CanRxMsg CanRxMsg_2;

CanTxMsg CanTxMsg_DigitIn; //本地数字量输入消息盒
CanTxMsg CanTxMsg_DigitOut;//本地数字量输出消息盒
CanTxMsg CanTxMsg_DigitAin;//中间模拟量输入消息盒
CanTxMsg CanTxMsg_DigitVar;//本地中间变量消息盒
CanTxMsg Can_Online_Frame_can1;//CAN总线在线帧
CanTxMsg Can_Online_Frame_can2;//CAN总线在线帧

//本地中间变量消息盒
unsigned char iAcnt;
unsigned char Stored_Data_Address[50];//需要存储到Flash的数据的位置
unsigned char Stored_Data_Flag;
#define TIME_CANTX_Delay 5000

void CAN1_tx_data(void)
{
	uint8_t fifo;
	unsigned int i,j;
	if((CAN_ID_TX_DigitIn&0x80000000)==0x80000000)//发送开关量输入
		{
			CanTxMsg_DigitIn.ExtId=CAN_ID_TX_DigitIn & 0x1FFFFFFF;
			for(i=0;i<8;i++)
			{
				CanTxMsg_DigitIn.Data[i]=Data_Arrays[0][i];
			}
			fifo=CAN_Transmit(CAN1,&CanTxMsg_DigitIn);
			i=0;
			 while((CAN_TransmitStatus(CAN1,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
			    {
			     i++;
			    }
		}

	//DRVMGR_TimerDelayUs(5000);
	if((CAN_ID_TX_DigitOut&0x80000000)==0x80000000)//发送开关量输出
		{
		CanTxMsg_DigitOut.ExtId=CAN_ID_TX_DigitOut & 0x1FFFFFFF;
			for(i=0;i<8;i++)
			{
				CanTxMsg_DigitOut.Data[i]=Data_Arrays[1][i];
			}
			fifo=CAN_Transmit(CAN1,&CanTxMsg_DigitOut);
			i=0;
			 while((CAN_TransmitStatus(CAN1,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
			    {
			     i++;
			    }
		}

	//DRVMGR_TimerDelayUs(5000);

	if((CAN_ID_TX_Ain&0x80000000)==0x80000000)//发送模拟量输入
		{
		CanTxMsg_DigitAin.ExtId=CAN_ID_TX_Ain & 0x1FFFFFFF;
          while(((Analog_Config_Inf[iAcnt]&0x80)!=0x80)&&(iAcnt<8))
          {
        	  iAcnt=iAcnt+1;
          }
		  if((Analog_Config_Inf[iAcnt]&0x80)==0x80)
			{
				CanTxMsg_DigitAin.Data[0]= iAcnt;
				CanTxMsg_DigitAin.Data[1]= Data_Arrays[2][iAcnt*4+0];
				CanTxMsg_DigitAin.Data[2]= Data_Arrays[2][iAcnt*4+1];
				CanTxMsg_DigitAin.Data[3]= Data_Arrays[2][iAcnt*4+2];
				CanTxMsg_DigitAin.Data[4]= Data_Arrays[2][iAcnt*4+3];
				fifo=CAN_Transmit(CAN1,&CanTxMsg_DigitAin);
				i=0;
				while((CAN_TransmitStatus(CAN1,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
					    {
					     i++;
					    }
				for(j=0;j<1500;j++);

			}
			if(iAcnt>7)
			{
				iAcnt=0;
			}
			else
			{
			 iAcnt=iAcnt+1;
			}
		}

	//DRVMGR_TimerDelayUs(5000);
	if((CAN_ID_TX_DigitVar&0x80000000)==0x80000000)//发送开关量输入
		{
		    CanTxMsg_DigitVar.ExtId=CAN_ID_TX_DigitVar & 0x1FFFFFFF;
			for(i=0;i<8;i++)
			{
				CanTxMsg_DigitVar.Data[i]=Data_Arrays[3][i];
			}
			fifo=CAN_Transmit(CAN1,&CanTxMsg_DigitVar);
			i=0;
			 while((CAN_TransmitStatus(CAN1,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
			    {
			     i++;
			    }
		}

}

void CAN2_tx_data(void)
{
	uint8_t fifo;
	unsigned int i,j;
	if((CAN_ID_TX_DigitIn&0x80000000)==0x80000000)//发送开关量输入
		{
			CanTxMsg_DigitIn.ExtId=CAN_ID_TX_DigitIn & 0x1FFFFFFF;
			for(i=0;i<8;i++)
			{
				CanTxMsg_DigitIn.Data[i]=Data_Arrays[0][i];
			}
			fifo=CAN_Transmit(CAN2,&CanTxMsg_DigitIn);
			i=0;
			 while((CAN_TransmitStatus(CAN2,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
			    {
			     i++;
			    }
		}

	//DRVMGR_TimerDelayUs(5000);
	if((CAN_ID_TX_DigitOut&0x80000000)==0x80000000)//发送开关量输出
		{
		CanTxMsg_DigitOut.ExtId=CAN_ID_TX_DigitOut & 0x1FFFFFFF;
			for(i=0;i<8;i++)
			{
				CanTxMsg_DigitOut.Data[i]=Data_Arrays[1][i];
			}
			fifo=CAN_Transmit(CAN2,&CanTxMsg_DigitOut);
			i=0;
			 while((CAN_TransmitStatus(CAN2,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
			    {
			     i++;
			    }
		}
	//DRVMGR_TimerDelayUs(5000);
	if((CAN_ID_TX_Ain&0x80000000)==0x80000000)//发送模拟量输入
		{
		CanTxMsg_DigitAin.ExtId=CAN_ID_TX_Ain & 0x1FFFFFFF;
          while(((Analog_Config_Inf[iAcnt]&0x80)!=0x80)&&(iAcnt<8))
          {
        	  iAcnt=iAcnt+1;
          }
		  if((Analog_Config_Inf[iAcnt]&0x80)==0x80)
			{
				CanTxMsg_DigitAin.Data[0]= iAcnt;
				CanTxMsg_DigitAin.Data[1]= Data_Arrays[2][iAcnt*4+0];
				CanTxMsg_DigitAin.Data[2]= Data_Arrays[2][iAcnt*4+1];
				CanTxMsg_DigitAin.Data[3]= Data_Arrays[2][iAcnt*4+2];
				CanTxMsg_DigitAin.Data[4]= Data_Arrays[2][iAcnt*4+3];
				fifo=CAN_Transmit(CAN2,&CanTxMsg_DigitAin);
				i=0;
				while((CAN_TransmitStatus(CAN2,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
					    {
					     i++;
					    }
				for(j=0;j<1500;j++);

			}
			if(iAcnt>7)
			{
				iAcnt=0;
			}
			else
			{
			 iAcnt=iAcnt+1;
			}
		}
	//DRVMGR_TimerDelayUs(5000);
	if((CAN_ID_TX_DigitVar&0x80000000)==0x80000000)//发送开关量输入
		{
		    CanTxMsg_DigitVar.ExtId=CAN_ID_TX_DigitVar & 0x1FFFFFFF;
			for(i=0;i<8;i++)
			{
				CanTxMsg_DigitVar.Data[i]=Data_Arrays[3][i];
			}
			fifo=CAN_Transmit(CAN2,&CanTxMsg_DigitVar);
			i=0;
			 while((CAN_TransmitStatus(CAN2,fifo)!=CANTXOK)&&(i<TIME_CANTX_Delay))
			    {
			     i++;
			    }
		}
}

/*
 *CAN1数据接收处理
 */
void CAN1_rx_data(void)
{
	unsigned int i,j;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) == SET)//判断中断，
	{
	  CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);//清除中断标志
	  CAN_Receive(CAN1,CAN_FIFO0,&CanRxMsg_1);//从FIFO0接收数据
	}
	if(CanRxMsg_1.ExtId==Arbitration_CANID)//接收来自显示屏的CAN总线切换命令
	{
		if(CanRxMsg_1.Data[0]==0x55)
		  Arbitration_CAN=CanRxMsg_1.Data[1];//0x00:can1,0x01:can2
	}
	else
	{
		for(j=0;j<(CAN_ID_Num-cnt_send_numb);j++)
		{
			   if(Store_CANID[j][1]==CanRxMsg_1.ExtId)
			    {
					  if(Store_CANID[j][0]>=53)//接收外部开关量
						{
						   for(i=0;i<8;i++)
							  {
							   Data_Arrays[Store_CANID[j][0]][i]=CanRxMsg_1.Data[i];
							  }
						}
					  else if((Store_CANID[j][0]>=11)&&(Store_CANID[j][0]<=50))//接收外部模拟量
						{
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_1.Data[0]*4+0]=CanRxMsg_1.Data[1];
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_1.Data[0]*4+1]=CanRxMsg_1.Data[2];
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_1.Data[0]*4+2]=CanRxMsg_1.Data[3];
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_1.Data[0]*4+3]=CanRxMsg_1.Data[4];

						  //标识关键数据（屏的配置参数），其作用是存储这些数据到FLASH
						  if((CanRxMsg_1.Data[5]==0xAA)&&((CanRxMsg_1.Data[6]==0xCC)||(CanRxMsg_1.Data[6]==0x55)))
							  Stored_Data_Address[Store_CANID[j][0]]=0xAA;
						  //Stored_Data_Flag==1时，需存储的数据下发完成
						  if((CanRxMsg_1.Data[5]==0xAA)&&(CanRxMsg_1.Data[6]==0x55))
						  {
							  Stored_Data_Flag=1;
							  LCD_Data_Store();
						  }
						}

			    }
		}

	}
}
/*
 *CAN2数据接收处理函数
 */
void CAN2_rx_data(void)
{
	unsigned int i,j;
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0) == SET)
	{
	  CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
	  CAN_Receive(CAN2,CAN_FIFO0,&CanRxMsg_2);
	}
	if(CanRxMsg_2.ExtId==Arbitration_CANID)
	{
		if(CanRxMsg_2.Data[0]==0x55)
		  Arbitration_CAN=CanRxMsg_2.Data[1];
	}
	else
	{
		for(j=0;j<(CAN_ID_Num-cnt_send_numb);j++)
		{
			   if(Store_CANID[j][1]==CanRxMsg_2.ExtId)
			    {
					  if(Store_CANID[j][0]>=53)//接收外部开关量
						{
						   for(i=0;i<8;i++)
							  {
							   Data_Arrays[Store_CANID[j][0]][i]=CanRxMsg_2.Data[i];
							  }
						}
					  else if((Store_CANID[j][0]>=11)&&(Store_CANID[j][0]<=50))//接收外部模拟量
						{
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_2.Data[0]*4+0]=CanRxMsg_2.Data[1];
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_2.Data[0]*4+1]=CanRxMsg_2.Data[2];
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_2.Data[0]*4+2]=CanRxMsg_2.Data[3];
						  Data_Arrays[Store_CANID[j][0]][CanRxMsg_2.Data[0]*4+3]=CanRxMsg_2.Data[4];

						  //标识关键数据（屏的配置参数），其作用是存储这些数据到FLASH
						  if((CanRxMsg_2.Data[5]==0xAA)&&((CanRxMsg_2.Data[6]==0xCC)||(CanRxMsg_2.Data[6]==0x55)))
							  Stored_Data_Address[Store_CANID[j][0]]=0xAA;
						  //Stored_Data_Flag==1时，需存储的数据下发完成
						  if((CanRxMsg_2.Data[5]==0xAA)&&(CanRxMsg_2.Data[6]==0x55))
						  {
							  Stored_Data_Flag=1;
							  LCD_Data_Store();
						  }
						}
			    }
		}
	}
}
/*CAN总线数据错误检测
 * 同时检测2路CAN总线错误计数,当检测某一路错误超过256个时，切换至另外一条CAN总线。
 * */
char Scan_Can_err(void)
{
	unsigned int counter_tx_err_1,counter_rx_err_1,counter_tx_err_2,counter_rx_err_2;
	char state=0;

		counter_tx_err_1=CAN_GetLSBTransmitErrorCounter(CAN1);//得到CAN1发送错误
		counter_rx_err_1=CAN_GetReceiveErrorCounter(CAN1);//得到CAN1接收错误
		counter_tx_err_2=CAN_GetLSBTransmitErrorCounter(CAN2);//得到CAN2发送错误
		counter_rx_err_2=CAN_GetReceiveErrorCounter(CAN2);//得到CAN2接收错误

		if((counter_tx_err_1>255)||(counter_rx_err_1>255))//任意错误超过255时，置切换标志
		{
			//Arbitration_CAN=1;
			state=1;
			DRVMGR_CANInit();
		}
		else if((counter_tx_err_2>255)||(counter_rx_err_2>255))//任意错误超过255时，置切换标志
		{
			//Arbitration_CAN=0;
			state=2;
			DRVMGR_CANInit();
		}
		else
		{
			state=0;
		}
	return state;//返回状态
}

/*
 * CAN总线切换
 */
void CAN_send_data(void)
{
	if(Arbitration_CAN==1)
	{
		CAN2_tx_data();
	}
	else
	{
		CAN1_tx_data();
	}
}
/*
 * 本地发送数据消息盒初始化
 */
void CAN_Frame_init(void)
{

    //初始化开关量输入消息盒
	CanTxMsg_DigitIn.IDE=CAN_ID_EXT;
	CanTxMsg_DigitIn.RTR=CAN_RTR_DATA;
	CanTxMsg_DigitIn.DLC=8;
	//初始化开关量输出消息盒
	CanTxMsg_DigitOut.IDE=CAN_ID_EXT;
	CanTxMsg_DigitOut.RTR=CAN_RTR_DATA;
	CanTxMsg_DigitOut.DLC=8;
	//初始化模拟量输入消息盒
	CanTxMsg_DigitAin.IDE=CAN_ID_EXT;
	CanTxMsg_DigitAin.RTR=CAN_RTR_DATA;
	CanTxMsg_DigitAin.DLC=8;
	//初始化中间变量输入消息盒
	CanTxMsg_DigitVar.IDE=CAN_ID_EXT;
	CanTxMsg_DigitVar.RTR=CAN_RTR_DATA;
	CanTxMsg_DigitVar.DLC=8;

}
/*
 * CAN总线在线帧
 */
void CAN_Send_OnLine(void)
{
	uint8_t fifo1;
	uint32_t i;
	//初始化CAN总线在线帧消息盒
	res_CAN_S=Scan_Can_err();
        for(i=0;i<8;i++)
        	Can_Online_Frame_can1.Data[i]=ACK_can[i];
        Can_Online_Frame_can1.Data[1]=Arbitration_CAN;
        Can_Online_Frame_can1.Data[2]=res_CAN_S;
	Can_Online_Frame_can1.IDE=CAN_ID_EXT;
	Can_Online_Frame_can1.ExtId=Arbitration_CANID_TX;
	Can_Online_Frame_can1.DLC=8;
	Can_Online_Frame_can1.RTR=CAN_RTR_DATA;
	i=0;
	fifo1=0;
	fifo1=CAN_Transmit(CAN1,&Can_Online_Frame_can1);
	 while((CAN_TransmitStatus(CAN1,fifo1)!=CANTXOK)&&(i<TIME_CANTX_Delay))
	    {
	     i++;
	    }
	for(i=0;i<8;i++)
	  Can_Online_Frame_can2.Data[i]=ACK_can[i];
	Can_Online_Frame_can2.Data[1]=Arbitration_CAN;
	Can_Online_Frame_can2.Data[2]=res_CAN_S;
	Can_Online_Frame_can2.IDE=CAN_ID_EXT;
	Can_Online_Frame_can2.ExtId=Arbitration_CANID_TX;
	Can_Online_Frame_can2.DLC=8;
	Can_Online_Frame_can2.RTR=CAN_RTR_DATA;
	 fifo1=0;
	 i=0;
	 fifo1=CAN_Transmit(CAN2,&Can_Online_Frame_can2);
	 while((CAN_TransmitStatus(CAN2,fifo1)!=CANTXOK)&&(i<TIME_CANTX_Delay))
	    {
	     i++;
	    }

}


