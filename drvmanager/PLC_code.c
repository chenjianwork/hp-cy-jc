/*
 * PLC_code.c
 *
 *  Created on: 2023年2月14日
 *      Author: liyan
 */
#include "drvmanager.h"
#include "stm32f4xx.h"


#define D_COUNT  8

#define LDP	0XD8	//取上升沿
#define LDF	0Xd9	//取下降沿
#define ANDP	0Xda	//与上升沿
#define ANDF	0XdB	//与下降沿
#define ORP	0Xdc	//或上升沿
#define ORF	0xdd	//或下降沿
#define LDA	0xC0 	    //取模拟量
#define LsTA	0xC1	//小于
#define LsTD	0xC2	//小于：直接数
#define GrTA	0xC3	//大于
#define GrTD	0xC4	//大于：直接数

#define EQUA	0xC5	//等于
#define EQUD	0xC6	//等于：直接数
#define GrEQ	0xC7	//大于等于
#define GrEQD	0xC8	//大于等于 直接数
#define LsEQ	0xC9	//小于等于
#define LsEQD	0xCA	//小于等于 直接数
#define ADD	    0xB0	//+
#define ADDB	0xB1	//加 (直接数的情况)
#define DEC	    0xB2	//-
#define DECB	0xB3	//减 (直接数的情况)
#define MUL	0xB5	//乘法
#define MULB	0xB6	//乘法(直接数的情况)
#define ELI	0xB7	//乘法
#define ELIB	0xB8	//乘法(直接数的情况)


#define EQM	    0xB4	//=中间变量
#define LD	    0xD0	//取
#define LDI	0xD1	//取反
#define AND	0xD2	//与
#define ANI	0xD3	//与反
#define OR	0xD4	//或
#define ORI	0xD5	//或反
#define ANDB	0xD6
#define ORB	0xD7
#define RTI1 0xE0	//timer

#define OUT	0xA0	//输出：
#define SET	0xA1	//设置1
#define RST	0xA2	//清除0
#define MPS	0xA3	//压堆栈
#define MRD	0xA4	//读堆栈
#define MPP	0xA5	//弹堆栈


/*********************************Din_sALL二维数组*****************************************/
/****************Data_Arrays[0][]                 :本地数字量输入存储地址********************/
/****************Data_Arrays[1][]                 :本地数字量输出存储地址********************/
/****************Data_Arrays[2][]                 :本地模拟量输入存储地址********************/
/****************Data_Arrays[3][0]-Data_Arrays[3][8] :本地中间变量存储地址***62：初始化(100ms初始化脉冲)，63：常闭触点***/
/****************Data_Arrays[4][]~Data_Arrays[10][]  :保留******************/
/****************Data_Arrays[11][]~Data_Arrays[50][] :外部模拟量数据接收存储地址****************/
/****************Data_Arrays[51][]                 :本地运算中间结果存储地址********/
/****************Data_Arrays[52][]                 :定时器标志*****************/
/****************Data_Arrays[53][]~Data_Arrays[255][] :外部开关量存储地址：包括协议转换模块、触摸屏、中间变量、开关量***/
/****************Data_Arrays[256][]~Data_Arrays[512][] **/
unsigned char Data_Arrays[256][64];


unsigned char *SpecialRamBlock = (unsigned char *)0x10000000;
unsigned char Falling_Edge[256][64];//数字量IO下降沿
unsigned char Rising_Edge[256][64];//数字量IO上升沿
unsigned char Storage_Rising_Edge[256][64];
unsigned char Storage_Falling_Edge[256][64];



unsigned int  RX_Data_CONFIG3_LENGTH;



/* CAN ID 配置文件
 * CAN_ID_Config_Inf[1024]:
 * CAN_ID_Config_Inf[0]:后面CAN ID 的数量<256;
 * CAN_ID_Config_Inf[1]~CAN_ID_Config_Inf[253];每个CAN ID占用5各字节,每5个字节的第一个字节为位置信息;
 *
 * Module_Config_Inf[1024]:模块配置信息,Module_Config_Inf[0],模块编号
 *
 *
 * Analog_Config_Inf[1024]:模拟量配置信息
 * Analog_Config_Inf[0]~Analog_Config_Inf[15]:16个通道信息:(0xXX0:4~20mA,电流型,第8位表示：0x8X、使能该通道CAN发送,0x0X、禁止该通道CAN发送
 * Analog_Config_Inf[16]~Analog_Config_Inf[19]:表示1通道传感器测量范围最小值，
 * Analog_Config_Inf[20]~Analog_Config_Inf[23]:表示1通道传感器测量范围最大值......
 *
 * PLC_Prog_Code[1024*12]:用户代码存储区
 */
unsigned char CAN_ID_Config_Inf[1024]= { 0 };
unsigned char Module_Config_Inf[1024] = { 0 };
unsigned char Analog_Config_Inf[1024] = { 0 };	//0:类型， 通道1：(1～4，最小参考值，5～8：最大参考值)......
unsigned char PLC_Prog_Code[12288]= { 0 };

unsigned int CAN_ID_Num;//CAN ID 数量
unsigned int cnt_send_numb;//本地发送CAN ID数量
unsigned char Arbitration_CAN;//0:CAN1 1:CAN2
unsigned int MASK_d,MASK_d1;
unsigned int Arbitration_CANID_TX;//模块编号，同时也是在线帧CAN ID；


/*Store_CANID:the Max number of CAN ID is 256,the first byte (Store_CANID[x][0]) is address,the scend byte is CAN ID  */
unsigned int Store_CANID[256][2];
unsigned int INid[256];

unsigned int CAN_ID_TX_DigitIn;
unsigned int CAN_ID_TX_DigitOut;
unsigned int CAN_ID_TX_Ain;
unsigned int CAN_ID_TX_DigitVar;
unsigned char  ACK_can[8]={0Xaa,0x00,0x00,0x00,0xcc,0x3c,0xc3,0x33};



unsigned int Ms[16];
int dataF[8];
float fz;
float fz1;
float fz2;
int  fzu32;
unsigned char tx_data[8];
unsigned char *CAN_Tx;
unsigned char Din_sALL_L;
unsigned int AIN_overflow_flag = 0;
void Init_Data_Arrays(void)
{
   Data_Arrays[3][7]=Data_Arrays[3][7]|0xC0;
}
void rst_bit62_Data_Arrays(void)
{
   Data_Arrays[3][7]=(Data_Arrays[3][7]&(~0x40));//复位第bit62;
}
void Init_CAN_ID(void)
 {
	unsigned int j,i,a,z;
	unsigned int temp_mask;
	MASK_d=0x1FFFFFFF;
	unsigned int Arbitration_CANID_C;
	Arbitration_CANID_C=Arbitration_CANID;
	cnt_send_numb=0;
    a=CAN_ID_Config_Inf[0];
    CAN_ID_Num= CAN_ID_Config_Inf[0];//CAN ID 数量
	Arbitration_CANID_TX=Module_Config_Inf[0]<<13;//模块编号，同时也是在线帧CAN ID；
	/*
	 * 初始化数组
	 */
	for(i=0;i<256;i++)
	  {
		Store_CANID[i][0]=0xFF;
		Store_CANID[i][1]=0x1FFFFFFF;
	  }
	for(i=0;i<256;i++)
	  {
	   INid[i]=0x1FFFFFFF;
	  }
	i=0;
	INid[51]=Module_Config_Inf[0]<<13;
	//Arbitration_CANID=Module_Config_Inf[0]<<13;//此处需要考虑
	//MASK_d=Arbitration_CANID_TX;
	ACK_can[1]=Module_Config_Inf[0];
	z=0;
    for(j=1;j<1024;)
  	  {
    	//把所有的CAN ID 对号入座进行存储
		INid[CAN_ID_Config_Inf[j]]=((CAN_ID_Config_Inf[j+1]<<0)|(CAN_ID_Config_Inf[j+2]<<8)|(CAN_ID_Config_Inf[j+3]<<16)|(CAN_ID_Config_Inf[j+4]<<24));

		if(CAN_ID_Config_Inf[j]<=10)//本地发送CANID
		  {
			j=j+5;
			i++;
			cnt_send_numb=cnt_send_numb+1;
		  }
		else//外部接收CANID
		  {
			//MASK_d ^=(INid[CAN_ID_Config_Inf[j]]&0x1FFFFFFF);
			Store_CANID[z][0]=CAN_ID_Config_Inf[j];
			Store_CANID[z][1]=INid[CAN_ID_Config_Inf[j]]&0x1FFFFFFF;
			z++;
			j=j+5;
			i++;
		  }

		if(i>=a)//结束
		  break;
	  }
      Store_CANID[z][0]=255;
      Store_CANID[z][1]=Arbitration_CANID_C;
      for(i=0;i<z+1;i++)
      {
    	  temp_mask=Store_CANID[i][1]^(~Store_CANID[0][1]);
    	  MASK_d &=temp_mask;
      }

      MASK_d<<=3;
	  //MASK_d=(~MASK_d)&0x1FFFFFFF;//(~MASK_d)&0x1FFFFFFF;
	  //MASK_d1=0x1FFFFF00;
	  CAN_ID_TX_DigitIn=INid[0];
	  CAN_ID_TX_DigitOut=INid[1];
      CAN_ID_TX_Ain=INid[2];
      CAN_ID_TX_DigitVar=INid[3];
 }
void Wirte_Digital_Bit(unsigned int a,unsigned int b,unsigned char value);
/*定时器结构体*/
struct RTIS
 {
	unsigned char EN;//定时器使能标志位
	unsigned int RTI_data_max;//定时器最大时间
	unsigned int RTI_data;//定时器实时时间
 }RTI_S[64];
 /**/
 void RTI_set(unsigned int i)
   {
 		 if(RTI_S[i].EN==1)
 		   {
 		    if(RTI_S[i].RTI_data>=(RTI_S[i].RTI_data_max-1)*100)
 		     {
 		   	   //Data_Arrays[52][i]=1;
 			   Wirte_Digital_Bit(52,i,1);
 		     }
 			else
 			 {
 			   RTI_S[i].RTI_data=RTI_S[i].RTI_data+100;
 			 }
 		   }
 		 else
 		   {
 		   	RTI_S[i].RTI_data=0;
 			//Data_Arrays[52][i]=0;
 			Wirte_Digital_Bit(52,i,0);
 		   }
   }
 /*main函数调用，main函数实时扫描定时器使能标志位*/
 void scan_RTI(void)
 {
 	unsigned char i;
 	for(i=0;i<32;i++)
 	{
 		RTI_set(i);
 	}
 }



/*命令解析函数*/

unsigned int Read_Digital_Bit(unsigned int a,unsigned int b)
  {
	unsigned int data1,data2,data3,data4;
		 data4=a;
	     data2=b%8;
		 data3=b/8;
		 data1=(Data_Arrays[data4][data3]>>data2)&0x01;
	 return  data1;
  }

void Wirte_Digital_Bit(unsigned int a,unsigned int b,unsigned char value)
  {
	unsigned int data2,data3,data4;
		 data4=a;
	     data2=b%8;
		 data3=b/8;
	   if(value==0x01)
		  Data_Arrays[data4][data3]=(1<<data2)|Data_Arrays[data4][data3];
       else
       	  Data_Arrays[data4][data3]=(~(1<<data2))&Data_Arrays[data4][data3];
  }
int GET_analy(unsigned int a,unsigned int b)
	{
	   int data4;
	  data4=(Data_Arrays[a][b*4+0]<<0)|(Data_Arrays[a][b*4+1]<<8)|(Data_Arrays[a][b*4+2]<<16)|(Data_Arrays[a][b*4+3]<<24);
	  return  data4;

	}
void store_data(unsigned int a,unsigned int b,int value)
	{
	   Data_Arrays[a][b*4+0]=value>>0;
	   Data_Arrays[a][b*4+1]=value>>8;
	   Data_Arrays[a][b*4+2]=value>>16;
	   Data_Arrays[a][b*4+3]=value>>24;
	}
void init_edge(void)
{
	int m,n;
	 for(m=0;m<256;m++)
	 {
		 for(n=0;n<64;n++)
		 {
			 Storage_Rising_Edge[m][n]=0;
		 }
	 }

}
int CMD_analyze(void)
 {
	unsigned int a,b,c,d;
	int * dataP=dataF;

	unsigned int i=0,m,n;
	unsigned int cmd_num;		 //一个循环里面有多少指令
		 //data2:多目运算
	unsigned int data2;
	unsigned char s=0;
	cmd_num=(PLC_Prog_Code[0]|(PLC_Prog_Code[1]<<8));
	   		   i=2;
			   dataP++;
			   while((cmd_num>0)&&(i<8192))
			     {
				   if(PLC_Prog_Code[i]==LD)
				     {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   dataP++;
					   *dataP=  Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				   else if(PLC_Prog_Code[i]==LDP)//取上升沿
				     {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   dataP++;
					   *dataP=  Read_Digital_Bit(a,b);
					   i=i+5;
					   if((*dataP==1)&&(Rising_Edge[a][b]==0))
					     *dataP=1;
					   else
					     *dataP=0;
						 Storage_Rising_Edge[a][b]=Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				   else if(PLC_Prog_Code[i]==LDF)//取下降沿
				     {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   dataP++;
					   *dataP=  Read_Digital_Bit(a,b);
					   i=i+5;
					   if((*dataP==0)&&(Falling_Edge[a][b]==1))
					     *dataP=1;
					   else
					     *dataP=0;

						Storage_Falling_Edge[a][b]=Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				   else if(PLC_Prog_Code[i]==LDA)
				     {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   i=i+5;
					   dataP++;
					   *dataP=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   cmd_num--;
					 }
				   else if(PLC_Prog_Code[i]==LDI)
				     {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   dataP++;
					   *dataP=((~Read_Digital_Bit(a,b))&0x01);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==AND)
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   *dataP=(*dataP) & Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ANDP)//与上升沿
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   if(*dataP==1)
						   {
						   if((Read_Digital_Bit(a,b)==1)&&(Rising_Edge[a][b]==0))
						     {
							     *dataP=1;
							 }
							else
							 {
								 *dataP=0;
							 }
							}
						else
						    {
							 *dataP=0;
							}
						Storage_Rising_Edge[a][b]=Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ANDF)//与下降沿
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   if(*dataP==1)
						   {
						   if((Read_Digital_Bit(a,b)==0)&&(Falling_Edge[a][b]==1))
						     {
							     *dataP=1;
							 }
							else
							 {
								  *dataP=0;
							 }
							}
						else
						    {
							 *dataP=0;
							}
						Storage_Falling_Edge[a][b]=Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ANI)
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   *dataP=(*dataP) & ((~Read_Digital_Bit(a,b))&0x01);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ANDB)
					 {

					   data2=(*dataP) & (*(dataP-1));
					   dataP--;
					   *dataP=data2;
					   i=i+5;
					   cmd_num--;
					 }

				    else if(PLC_Prog_Code[i]==OR)
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   *dataP=(*dataP) | Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ORP)//或上升沿
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   if(*dataP==1)
					     {
						  *dataP=1;
						 }
					   else
					     {
						   if((Read_Digital_Bit(a,b)==1)&&(Rising_Edge[a][b]==0))
						     {
							     *dataP=1;
							 }
							else
							 {
								 *dataP=0;
							 }
						 }
							Storage_Rising_Edge[a][b]=Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ORF)//或下降沿
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   if(*dataP==1)
					     {
						  *dataP=1;
						 }
					   else
					     {
						   if((Read_Digital_Bit(a,b)==0)&&(Rising_Edge[a][b]==1))
						     {
							     *dataP=1;
							 }
							else
							 {
								 *dataP=0;
							 }
						 }
							Storage_Falling_Edge[a][b]=Read_Digital_Bit(a,b);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ORI)
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   *dataP=(*dataP) | ((~(Read_Digital_Bit(a,b)))&0x01);
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ORB)
					 {
					   data2=(*dataP) | (*(dataP-1));
					   dataP--;
					   *dataP=data2;
					   i=i+5;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==RTI1) //定时器
					 {
					 	a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
						b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
						i=i+5;
					  if((*dataP))
						  {
						   RTI_S[a].EN=1;
						   RTI_S[a].RTI_data_max=b;
						   //RTI_S[a].RTI_data=0;
						  }
					   else
					      {
						   RTI_S[a].EN=0;
						   RTI_S[a].RTI_data_max=b;
						   RTI_S[a].RTI_data=0;
						   Wirte_Digital_Bit(52,a,0);
						   //Data_Arrays[52][a]=0;
						  }
						dataP=dataF;
						cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==MPS)  //存堆栈
					 {
					   s++;
					   Ms[s]=*dataP ;
					   i=i+5;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==MRD)  //读堆栈
					 {
					   *dataP=Ms[s];
					   i=i+5;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==MPP)  //弹堆栈 ，地址-1
					 {
					   *dataP=Ms[s] ;
					   i=i+5;
					   s--;
					   cmd_num--;
					 }
 					else if(PLC_Prog_Code[i]==EQUD)
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   //conv((PLC_Prog_Code[ d ]<<0)|(PLC_Prog_Code[ c ]<<8)|(PLC_Prog_Code[ b ]<<16)|(PLC_Prog_Code[ a ]<<24));
					   if(fz==fz1)
					      *dataP=1;
					   else
					      *dataP=0;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==EQUA)
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   i=i+5;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   fz1=fzu32/1000.00;
					   if(fz==fz1)
					      *dataP=1;
					   else
					      *dataP=0;

					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==LsTD)
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   if(fz<fz1)
						  *dataP=1;
					   else
					      *dataP=0;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==LsTA)
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;
					   fz1=fzu32/1000.00;
					   //conv(fz_a);
					   if(fz<fz1)
					      *dataP=1;
					   else
					      *dataP=0;

					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==GrTD)
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   if(fz> fz1)
					      *dataP=1;
					   else
					      *dataP=0;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==GrTA)
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;
					   fz1=fzu32/1000.00;
					   if(fz>fz1)
					      *dataP=1;
					   else
					      *dataP=0;

					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==LsEQD) //小于等于	直接数
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   if(fz<=fz1)
					      *dataP=1;
					   else
					      *dataP=0;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==LsEQ) //小于等于	地址
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;
					   fz1=fzu32/ .00;
					   if(fz<=fz1)
					      *dataP=1;
					   else
					      *dataP=0;

					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==GrEQD) //大于等于 直接数
					 {
					   fz=(*dataP)/1000.00;
					   fz2=	(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   if(fz>=fz1)
					      *dataP=1;
					   else
					      *dataP=0;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==GrEQ)	 //大于等于
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;
					   fz1=fzu32/1000.00;
					   if(fz>=fz1)
					      *dataP=1;
					   else
					      *dataP=0;

					   cmd_num--;
					 }
					else if(PLC_Prog_Code[i]==ADD)	 //+地址
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;
					   fz1=fzu32/(float)1000.00;
					   fz=fz1+fz;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ADDB) //+直接数
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   fz=fz1+  fz;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
					   cmd_num--;
					 }
					else if(PLC_Prog_Code[i]==DEC)	 //-地址
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;
					   fz1=fzu32/1000.00;
					   fz=fz-fz1;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==DECB) //-直接数
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   fz=fz-fz1;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==MUL) //× 乘法
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;
					   fz1=fzu32/1000.00;
					   fz=fz*fz1;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==MULB) //× 乘法 直接数
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
					   fz=fz*fz1;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ELI) //除法
					 {
					   fz=(*dataP)/1000.00;
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   fzu32=GET_analy(a,b);//Data_Arrays[ a][ b ];
					   i=i+5;

					   fz1=fzu32/1000.00;
						 if(fz1==0)
							 {
								 *dataP=0xFFFFFFFF;
								}
						 else
						 {
					   fz=fz/fz1;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
						 }
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==ELIB) //除法直接数
					 {
					   fz=(*dataP)/1000.00;
					   a=i+1;
					   b=i+2;
					   c=i+3;
					   d=i+4;
					   i=i+5;
					   fzu32=((PLC_Prog_Code[ a ]<<0)|(PLC_Prog_Code[ b ]<<8)|(PLC_Prog_Code[ c ]<<16)|(PLC_Prog_Code[ d ]<<24));
					   fz1=fzu32/1000.00;
						 if(fz1==0)
							 {
								 *dataP=0xFFFFFFFF;
								}
						 else
						 {
					   fz=fz/fz1;
					   fz=fz*(float)1000.00;
					   fzu32=fz;
					   *dataP=fzu32;
             }
					   cmd_num--;
					 }
					else if(PLC_Prog_Code[i]==EQM)	//=于存储
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=((PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8))-1;
					   i=i+5;
					   //Data_Arrays[a][b ]=(*dataP);
					   dataP--;
					   if(*dataP)
					     {
						  dataP++;
					      store_data(a,b,(*dataP));
						 }
					   else
					     {
						  dataP++;
						 }

					   dataP=dataF;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==SET)	//
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;

					   if((*dataP))
					     {
						   Wirte_Digital_Bit(a,b,1);
						 }
					   dataP=dataF;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==RST)	//
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   if((*dataP))
					     {
						   Wirte_Digital_Bit(a,b,0);
						 }
					   dataP=dataF;
					   cmd_num--;
					 }
				    else if(PLC_Prog_Code[i]==OUT)	//
					 {
					   a=(PLC_Prog_Code[i+1]<<0)|(PLC_Prog_Code[i+2]<<8);
					   b=(PLC_Prog_Code[i+3]<<0)|(PLC_Prog_Code[i+4]<<8);
					   i=i+5;
					   if((*dataP))
					     {
						     Wirte_Digital_Bit(a,b,1);
						 }
					   else
					     {
						     Wirte_Digital_Bit(a,b,0);
						  }
					   dataP=dataF;
					   cmd_num--;
					   //i++;
					  // break;
					 }
					else if((PLC_Prog_Code[i+0]==0xCC)&& (PLC_Prog_Code[i+1]==0xC3)&&(PLC_Prog_Code[i+2]==0x3C)&&(PLC_Prog_Code[i+3]==0x33))	//结束标志	0xCC,0xC3,0x3C,0x33
					 {

						 for(m=0;m<256;m++)
						  for(n=0;n<64;n++)
						   {
						   Rising_Edge[m][n]=Storage_Rising_Edge[m][n];
						   }
						 for(m=0;m<256;m++)
						  for(n=0;n<64;n++)
						   {
						   Falling_Edge[m][n]=Storage_Falling_Edge[m][n];
						   }
					   return 1;

					 }
					else
					 {

						 for(m=0;m<256;m++)
						  for(n=0;n<64;n++)
						   {
						   Rising_Edge[m][n]=Storage_Rising_Edge[m][n];
						   }
						 for(m=0;m<256;m++)
						  for(n=0;n<64;n++)
						   {
						   Falling_Edge[m][n]=Storage_Falling_Edge[m][n];
						   }
							 return 0;
					 }

				 }

						 for(m=0;m<256;m++)
						  for(n=0;n<64;n++)
						   {
						   Rising_Edge[m][n]=Storage_Rising_Edge[m][n];
						   }
						 for(m=0;m<256;m++)
						  for(n=0;n<64;n++)
						   {
						   Falling_Edge[m][n]=Storage_Falling_Edge[m][n];
						   }
				 return 0;
}



  void delay_us(uint32_t s)
	{
		 uint32_t i,j;
      for(i=0;i<s;i++)
        {
					 for(j=0;j<30;j++);
        }
  }
void delay_ms(uint32_t s)
	{
		 uint32_t i,j;
      for(i=0;i<s;i++)
        {
					 for(j=0;j<25000;j++);
        }
  }
