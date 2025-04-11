/*!
****************************************************************************************************
* 文件名称：main.c
* 功能简介：该文件是应用程序入口程序的实现源文件
* 文件作者：HQHP
* 创建日期：2023-01-11
* 版权声明：All Rights Reserved.
****************************************************************************************************
*/
#include <hqhp/drvmanager.h>
#include <hqhp/devmanager.h>
#include <hqhp/dbgmanager.h>
#include <hqhp/taskmanager.h>
#include "drvmanager/drvmanager.h"
#include "stm32f4xx.h"
#include <string.h>


/*!
****************************************************************************************************
* 常量定义
****************************************************************************************************
*/

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
uint16_t DATA_REC_COMPLETE1,DATA_REC_COMPLETE2;
unsigned int Digital_Input_Data;
char res_CmdAnalyze,res_CAN_S;
extern unsigned int RxBytes;
extern void DRVMGR_TimerDelayUs(uint16_t us);
_Engine_DATA Engine_Parameter_Host;
_Engine_DATA Engine_Parameter_Auxiliary_1;
/*!
****************************************************************************************************
* 本地声明
****************************************************************************************************
*/
/*!
****************************************************************************************************
* 接口函数
****************************************************************************************************
*/
void Init_Flag(void)
{
	DownLoad_Step=0;
	Arbitration_CAN=0;//默认CAN1 传输数据;
	Data_DownLoad_Complete_Flag=0;
	analog_zj_Complete_Flag=0;
	analog_zj_Mode=0;
	iAcnt=0;
	Stored_Data_Flag=0;//存储关键数据标识（触摸屏配置文件）
	RxBytes=0;
	analog_zj_Chn_Flag=0;
    V_4mA =Vref_adc/5;
    State_get_data=0;
    DATA_REC_COMPLETE1=0;
    DATA_REC_COMPLETE2=0;
}
/*!
****************************************************************************************************
* 功能描述：该方法是主函数接口
* 注意事项：NA
* 输入参数：NA
* 输出参数：NA
* 返回参数：NA
****************************************************************************************************
*/
int main(int argc, const char* argv[])
{
	unsigned char rst_bit62;
	unsigned int j;
	rst_bit62=0;
	GPIOInit();
	Init_Flag();
	memset(SpecialRamBlock, 0, SPECIAL_RAM_BLOCK_SIZE);
	Init_USER_Code();//初始化用户数据;
	init_edge();
	//EraseSector_test();
	//LCD_Data_Store();
	Read_JZ_LCD_data();
	DRVMGR_Init();
	ADC_Data_Deal();//模拟量采集
//	for(j=0;j<300;j++)
//	 DRVMGR_TimerDelayUs(60000);
	Init_Data_Arrays();

	DRVMGR_TimerStart(MS_TMR_ID_CAN_OnLine_1, RUN_CAN_OnLine_TIME);
	DRVMGR_MSTimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME); // 启动运行指示灯定时器
	DRVMGR_MSTimerStart(MS_TMR_ID_100MS, 100);
	DRVMGR_MSTimerStart(MS_TMR_ID_PLC_RX_TIMEOUT, 100);
	FEED();
	Digital_PutOut();
	enable_PutOut();
    //test_flash();
	while (1)
	{
		if (DRVMGR_MSTimerIsExpiration(MS_TMR_ID_100MS))
		{
			if(rst_bit62==1){rst_bit62_Data_Arrays();}
			else if(rst_bit62<5){rst_bit62=rst_bit62+1;	}
			// 重启定时器
			DRVMGR_MSTimerStart(MS_TMR_ID_100MS, 100);
			Scan_Digital_Input();//采集数字量输入

			ADC_Data_Deal();//模拟量采集
			res_CmdAnalyze=CMD_analyze();
			Digital_PutOut();//数字量输出
			scan_RTI();//定时器
			CAN_send_data();//本地数据发送
			//DRVMGR_LEDTwinkle();//指示灯闪烁
		}
		if(DRVMGR_MSTimerIsExpiration(MS_TMR_ID_RUN))
		{
			if((res_CmdAnalyze==0)||(res_CAN_S>0))
				DRVMGR_MSTimerStart(MS_TMR_ID_RUN, RUN_LED_Fast_ON_TIME);
			else
			    DRVMGR_MSTimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME);
			FEED();
			DRVMGR_LEDTwinkle();//指示灯闪烁
		}
		if(DRVMGR_MSTimerIsExpiration(MS_TMR_ID_PLC_RX_TIMEOUT))
		{
			DRVMGR_MSTimerStart(MS_TMR_ID_PLC_RX_TIMEOUT, 100);
			RxBytes=0;
		}
		//秒：CAN在线帧，发动机数据
		if(DRVMGR_TimerIsExpiration(MS_TMR_ID_CAN_OnLine_1))
		{
			DRVMGR_TimerStart(MS_TMR_ID_CAN_OnLine_1, RUN_CAN_OnLine_TIME);
			Engine_Data_Request(1,1,DATA_REC_COMPLETE1,Engine_Parameter_Host);//主发动机；
			Engine_Data_Request(3,1,DATA_REC_COMPLETE2,Engine_Parameter_Auxiliary_1);//辅发动机；
			CAN_Send_OnLine();
		}
		DownLoad_Step=0;
		analog_zj_Mode=0;
		RxBytes=0;
		analog_zj_Chn_Flag=0;
		Download_Mode();


		//res_CAN_S=Scan_Can_err();//检测CAN总线错误
		//LCD_Data_Store();
	}
}
