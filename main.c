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
#include "sysmanager/sysmanager.h"
#include "commanager/commanager.h"
#include <string.h>

int main(void)
{
	bool isOn;

	// 加上断言可以确保避免一些错误
	STATIC_ASSERT(MAJOR_ERR_TYPE_LAST <= ERR_TYPE_MAX_NUMS);
	SYSMGR_Init();

	isOn = false;
	DRVMGR_MSTimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME);

	while(1){
		FEED();
		DRVMGR_Handle();
		DEVMGR_Handle(); //未编写代码
		COMMGR_Handle(); //未编写代码
		SYSMGR_Handle();

		if (isOn) {
			if (DRVMGR_MSTimerIsExpiration(MS_TMR_ID_RUN)) {
				isOn = false;
				DRVMGR_MSTimerStart(MS_TMR_ID_RUN, 1000 - RUN_LED_ON_TIME);
				DRVMGR_LEDTurnOff();
			}
		} else {
			if (DRVMGR_MSTimerIsExpiration(MS_TMR_ID_RUN)) {
				isOn = true;
				DRVMGR_MSTimerStart(MS_TMR_ID_RUN, RUN_LED_ON_TIME);
				DRVMGR_LEDTurnOn();
			}
		}
	}
	return 0;
}


