/*
 * ADC_Acquisition.c
 *
 *  Created on: 2023年2月16日
 *      Author: liyan
 */
#include "stm32f4xx.h"
#include "drvmanager.h"

float V_4mA;
unsigned short DATA_adc[9]={};
int conver_s1_x,AA,BB;
int AIN_S[8];
float min,max;
float ADC_input_Voltage[8],ADC_input_Current[8],Sensor_real_data[8];
extern unsigned char Analog_Config_Inf[Len_AnalogConfiginf];
unsigned char ADC128S_Channel_ADRR[8]={0, 1, 2, 3, 4, 5, 6, 7};
void Delay_us(unsigned int us)
{
	unsigned i;
	while(us--){
	for(i=0;i<1000;i++)		;
	}
}



/*模拟量采集*/
uint16_t get_adc_data(uint8_t Chn)
{
	uint16_t rxData;
	uint16_t txData = (ADC128S_Channel_ADRR[Chn] << 3) << 8; // 通道地址需要左移3位

	DRVMGR_TimerDelayUs(1);
	spi1_cs_L;
	DRVMGR_TimerDelayUs(1);
	DRVMGR_SPIRWBytes(DRVID_SPI_1,&txData, &rxData, 1);
	spi1_cs_H;
	DRVMGR_TimerDelayUs(1);

	return rxData;
}
uint16_t ADC_CJ_MUX(unsigned char Chn)
{
	uint16_t i, j,n, temp, isSorted;
	uint16_t data_adc_cnv[14];
	uint16_t sum=0;
	for(i=0;i<10;i++)
	{
		data_adc_cnv[i]=get_adc_data(Chn);
	}
	//优化算法：最多进行 n-1 轮比较
	for(i=0; i<10-1; i++)
	{
		isSorted = 1;  //假设剩下的元素已经排序好了
		for(j=0; j<10-1-i; j++)
		{
			if(data_adc_cnv[j] > data_adc_cnv[j+1])
			{
				temp = data_adc_cnv[j];
				data_adc_cnv[j] = data_adc_cnv[j+1];
				data_adc_cnv[j+1] = temp;
				isSorted = 0;  //一旦需要交换数组元素，就说明剩下的元素没有排序好
			}
		}
		if(isSorted) break;
	}
	//去掉2两个最大值和两个最小值,求10个中间数的平均值
	for(n=2; n<8; n++)
	{
		sum = sum + data_adc_cnv[n];
	}
	return sum/6;
}
void ADC_Data_Deal(void)
{
    char i=0;
	char j=16;
//	uint16_t DATA_adc_c;

	for(i=0;i<8;i++)
	{
		DATA_adc[i]=ADC_CJ_MUX(i);
	}
//	DATA_adc_c=DATA_adc[0];
//	for(i=0;i<7;i++)
//	{
//		DATA_adc[i]=DATA_adc[i+1];
//	}
//	DATA_adc[7]=DATA_adc_c;
	for(i=0;i<8;i++)
	 {
		conver_s1_x=(Analog_Config_Inf[i*8+j+0]<<0)|(Analog_Config_Inf[i*8+j+1]<<8)|(Analog_Config_Inf[i*8+j+2]<<16)|(Analog_Config_Inf[i*8+j+3]<<24);
		min=conver_s1_x/1000.0;//传感器量程的下限
		conver_s1_x=(Analog_Config_Inf[i*8+j+4]<<0)|(Analog_Config_Inf[i*8+j+5]<<8)|(Analog_Config_Inf[i*8+j+6]<<16)|(Analog_Config_Inf[i*8+j+7]<<24);
		max=conver_s1_x/1000.0;//传感器量程的上限
		ADC_input_Voltage[i]=(((float)(DATA_adc[i]))/0xFFF)*Vref_adc;//计算模拟量输入电压，此处ADC参考电压为3.3V
		ADC_input_Current[i]=(ADC_input_Voltage[i]/160.00f)*1000.00;
		ADC_input_Current[i]=ADC_input_Current[i]+jz_ADC_current[i];
		if(ADC_input_Current[i]<=4)
			ADC_input_Current[i]=4;

		//Sensor_real_data[i]=(float)((float)((max-min)*(ADC_input_Voltage[i]-V_4mA)/2.56f)+min);//计算传感器实际数值
		Sensor_real_data[i]=(float)((float)((max-min)*(ADC_input_Current[i]-4)/16)+min)*1.001;//计算传感器实际数值

		AIN_S[i] =(int)(Sensor_real_data[i]*1000.00f);
		store_data(2,i,AIN_S[i]);
	 }
 }
void adc_jz_Deal(unsigned char Chn)
{
	uint16_t DATA_adc_of_12mA;
	float Adc_Voltage_of_12mA,Adc_Current_of_12mA;
	int32_t ADC_Current_Offset;
	DATA_adc_of_12mA=ADC_CJ_MUX(Chn);
	Adc_Voltage_of_12mA=(((float)DATA_adc_of_12mA)/0xFFF)*Vref_adc;
	Adc_Current_of_12mA=(Adc_Voltage_of_12mA/160.00f)*1000.00;//mA
	ADC_Current_Offset=(12.000-Adc_Current_of_12mA)*1000;
	jz_data[Chn*4+0]=(ADC_Current_Offset>>0)&0xFF;
	jz_data[Chn*4+1]=(ADC_Current_Offset>>8)&0xFF;
	jz_data[Chn*4+2]=(ADC_Current_Offset>>16)&0xFF;
	jz_data[Chn*4+3]=(ADC_Current_Offset>>24)&0xFF;
}

