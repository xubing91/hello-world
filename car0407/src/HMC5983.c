#include "MS5803-01BA.h"
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File


//int16   HMC_TESTRg_r2[3]={0,0,0};
//int16	HMC_REGIS_r2[3]={0,0,0};//用于测试HMC前三个寄存器



void HMC_Init()
{

//
	HMC_CS=0;
	HMC_CS=1;

	DELAY_US(200);	

	HMC_CS=0;
	HMC_WR(0X00);//寄存器A,地址指针自动加1
	HMC_WR(0XF8);//8-avera，75Hz,noraml measurement
	HMC_CS=1;
	DELAY_US(200);	
	HMC_CS=0;
	HMC_WR(0X01);//寄存器A,地址指针自动加1
	HMC_WR(0XA0);//8-avera，75Hz,noraml measurement
	HMC_CS=1;
	DELAY_US(200);	
	HMC_CS=0;
	HMC_WR(0X02);//寄存器A,地址指针自动加1
	HMC_WR(0X00);//8-avera，75Hz,noraml measurement
	HMC_CS=1;


/*
	HMC_CS=0;
	HMC_WR(0X40);//寄存器A,地址指针自动加1
	HMC_WR(0XF8);//8-avera，75Hz,noraml measurement
	HMC_WR(0XA0);//Gain=6
	HMC_WR(0X00);//连续测量
	HMC_CS=1;
*/
	/*

	HMC_WR(0XF8);//8-avera，75Hz,noraml measurement
//	HMC_WR(0XD8);
	//留有问题，75Hz，8次测量平均值？？？
	HMC_WR(0X20);//Gain=1
	HMC_WR(0X00);//连续测量
	*/



//	HMC_WR(0X00);//8-avera，75Hz,noraml measurement
//	HMC_WR(0XD8);
	//留有问题，75Hz，8次测量平均值？？？
//	HMC_WR(0X00);//Gain=1
//	HMC_WR(0X00);//连续测量
//




	DELAY_US(500);
	//HMC_TESTABC(HMC_REGIS_r2);
	//HMC_TEST(HMC_TESTRg_r2);
	
//	HMC_CS=1;

}

int16 MagX=0;
int16 MagY=0;
int16 MagZ=0;
void	HMC_ReadMag()
{
	
	unsigned char MagXH=0;
	unsigned char  MagXL=0;

	unsigned char  MagYH=0;
	unsigned char  MagYL=0;

	unsigned char  MagZH=0;
	unsigned char  MagZL=0;

	HMC_CS=0;

	
		HMC_WR(0XC3);//寄存器A的地址
		MagXH=HMC_WR(0)&0XFF;
		MagXL=HMC_WR(0)&0XFF;
		MagZH=HMC_WR(0)&0XFF;
		MagZL=HMC_WR(0)&0XFF;
		MagYH=HMC_WR(0)&0XFF;
		MagYL=HMC_WR(0)&0XFF;
	


/*
		HMC_CS=0;
		HMC_WR(0X83);//寄存器A的地址
		MagXH=HMC_WR(0)&0XFF;
		HMC_CS=1;
  		
		HMC_CS=0;
	    HMC_WR(0X84);//寄存器A的地址
		MagXL=HMC_WR(0)&0XFF;
		HMC_CS=1;


		HMC_CS=0;
		HMC_WR(0X85);//寄存器A的地址
		MagZH=HMC_WR(0)&0XFF;
		HMC_CS=1;


		HMC_CS=0;
		HMC_WR(0X86);//寄存器A的地址
		MagZL=HMC_WR(0)&0XFF;
		HMC_CS=1;
	
		HMC_CS=0;
		HMC_WR(0X87);//寄存器A的地址
		MagYH=HMC_WR(0)&0XFF;
		HMC_CS=1;

		HMC_CS=0;
		HMC_WR(0X88);//寄存器A的地址
		MagYL=HMC_WR(0)&0XFF;
		HMC_CS=1;

*/


		MagX=(int16)MagXH;
		MagX<<=8;
		MagX=MagX|MagXL;

		MagZ=(int16)MagZH;
		MagZ<<=8;
		MagZ=MagZ|MagZL;

		MagY=(int16)MagYH;
		MagY<<=8;
		MagY=MagY|MagYL;

	//标记位检测，用于测试
		
	
//		 FLAG_HMC=HMC_WR(0)&0XFF;
//		 LOCK_HMC=FLAG_HMC>>1;
//		 DRY_HMC=FLAG_HMC&0X01;

	HMC_CS=1;
}
/*
char HMC_FLAG()
{
	HMC_CS=0;
		HMC_WR(0X89);
		 FLAG_HMC=HMC_WR(0)&0XFF;
//		 LOCK_HMC=FLAG_HMC>>1;
//		 DRY_HMC=FLAG_HMC&0X01;
	HMC_CS=1;
	return FLAG_HMC;
}
*/
/*
int16  HMC_TEST(int16* HMC_TESTRg)
{
	HMC_CS=0;
	HMC_WR(0XCA);//读三个检测寄存器
	HMC_TESTRg[0]=	HMC_WR(0)&0XFF;
	HMC_TESTRg[1]=	HMC_WR(0)&0XFF;
	HMC_TESTRg[2]=	HMC_WR(0)&0XFF;
	HMC_CS=1;
	return 0;
}
*/
/*
int16  HMC_TESTABC(int16* HMC_REGIS)
{
	HMC_CS=0;
	HMC_WR(0XC0);//读三个检测寄存器
	HMC_REGIS[0]=	HMC_WR(0)&0XFF;
	HMC_REGIS[1]=	HMC_WR(0)&0XFF;
	HMC_REGIS[2]=	HMC_WR(0)&0XFF;
	HMC_CS=1;
	return 0;
	
	
}

*/

/*
int16 TEMP_HMC=0;
void	HMC_ReaTemp()
{
	unsigned char tempH_HMC=0;
	unsigned char tempL_HMC=0;
	
	HMC_CS=0;
	HMC_WR(0XB1);
	tempH_HMC=HMC_WR(0)&0XFF;
	tempL_HMC=HMC_WR(0)&0XFF;
	TEMP_HMC=(int16)tempH_HMC;
	TEMP_HMC<<=8;
	TEMP_HMC=TEMP_HMC|tempL_HMC;
	HMC_CS=1;
}

int16 HMC_CalcTemp()
{
	HMC_ReaTemp();
	TEMP_HMC>>=7;//手册21页
	TEMP_HMC+=25;//摄氏度表示
	return 	TEMP_HMC;
}
*/
//int16 HMC_raw[3]={0,0,0};
//double Ratio_HMC=0.73;//Gain=1
double Ratio_HMC=2.56;//Gain=5
//double Ratio_HMC=0.92;//Gain=1
const int16 HMCCOUNTbegin = 400;	// 开始点
const int16 HMCCOUNTend = 410;	// 结束点
const int16 HMC_threshold=100;//阈值

int cnt_HMC=0;
int16	cmps_raw_[3] = {0, 0, 0};	// raw data of compass pre cycle
int16	cmps_sum[3]={0,0,0};
void HMC_CalcMag(int16* HMC_raw)
{

	



	
		
	
	if(cnt_HMC<HMCCOUNTbegin) 
	{
	cnt_HMC++;
		HMC_ReadMag();
		HMC_raw[0]=MagX*Ratio_HMC;//单位为mGa；
		HMC_raw[1]=-MagY*Ratio_HMC;
		HMC_raw[2]=-MagZ*Ratio_HMC;
	}

	else if(cnt_HMC<HMCCOUNTend)
	{
	cnt_HMC++;	
		HMC_ReadMag();
		HMC_raw[0]=MagX*Ratio_HMC;//单位为mGa；
		HMC_raw[1]=-MagY*Ratio_HMC;
		HMC_raw[2]=-MagZ*Ratio_HMC;
		cmps_sum[0]=cmps_sum[0]+HMC_raw[0];
		cmps_sum[1]=cmps_sum[1]+HMC_raw[1];
		cmps_sum[2]=cmps_sum[2]+HMC_raw[2];
	}
	else 
	{
	if(cnt_HMC==HMCCOUNTend)
	{
		cnt_HMC++;
	cmps_raw_[0]=	cmps_sum[0]/(HMCCOUNTend-HMCCOUNTbegin);
	cmps_raw_[1]=	cmps_sum[1]/(HMCCOUNTend-HMCCOUNTbegin);
	cmps_raw_[2]=	cmps_sum[2]/(HMCCOUNTend-HMCCOUNTbegin);
	}
	else
	{
		cmps_raw_[0]=HMC_raw[0];
		cmps_raw_[1]=HMC_raw[1];
		cmps_raw_[2]=HMC_raw[2];
	}
	
		HMC_ReadMag();
		HMC_raw[0]=MagX*Ratio_HMC;//单位为mGa；
		HMC_raw[1]=-MagY*Ratio_HMC;
		HMC_raw[2]=-MagZ*Ratio_HMC;

		
		
		HMC_raw[0]=abs((HMC_raw[0]-cmps_raw_[0]))>HMC_threshold? cmps_raw_[0]:HMC_raw[0];
		HMC_raw[1]=abs((HMC_raw[1]-cmps_raw_[1]))>HMC_threshold? cmps_raw_[1]:HMC_raw[1];
		HMC_raw[2]=abs((HMC_raw[2]-cmps_raw_[2]))>HMC_threshold? cmps_raw_[2]:HMC_raw[2];	
	}
	
		
/*		HMC_raw[0]=	MagX*Ratio_HMC;//单位为mGa；
		HMC_raw[1]=	MagY*Ratio_HMC;
		HMC_raw[2]=	MagZ*Ratio_HMC;
*/
		//利用外接模块而且是按照特定的方位黏上去
		//输出数据和PCB的坐标轴对应，
		//PCB前，HMC后
//X=X,Y=y,Z=z;

//如今版本，是X=-x,Y=y,Z=-z
	
}





unsigned char iHMC;
unsigned char retHMC;

//焊接在电路板上
/*
unsigned char HMC_WR(unsigned char command)
{
	retHMC = 0;
	
	for(iHMC = 0; iHMC<8; iHMC++)
	{// 20MHz
		HMC_CLK = 0;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		HMC_MOSI = command >> 7;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		command <<= 1;
		HMC_CLK = 1;
		retHMC = (retHMC<<1) | HMC_MISO;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		// the codes takes about 0.25 us
	}
	
	//修改时钟的方式
		for(iHMC = 0; iHMC<8; iHMC++)
	{// 20MHz
	//clk=0
		//HMC_CLK = 0;
			GpioDataRegs.GPACLEAR.bit.GPIO15=1;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		if((command >> 7)&0x01)
			GpioDataRegs.GPASET.bit.GPIO16=1;
			else
			GpioDataRegs.GPACLEAR.bit.GPIO16=1;
		//HMC_MOSI = command >> 7;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		command <<= 1;
		//HMC_CLK = 1;

			GpioDataRegs.GPASET.bit.GPIO15=1;
		retHMC = (retHMC<<1) | HMC_MISO;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		// the codes takes about 0.25 us
	}
	return retHMC;
}
*/
//外接模块上的

unsigned char HMC_WR(unsigned char command)
{
	retHMC = 0;
	/*
	for(iHMC = 0; iHMC<8; iHMC++)
	{// 20MHz
		HMC_CLK = 0;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		HMC_MOSI = command >> 7;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		command <<= 1;
		HMC_CLK = 1;
		retHMC = (retHMC<<1) | HMC_MISO;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		// the codes takes about 0.25 us
	}
	*/
	//左上角错误的
	////////////////////////////////////////////
	/*
	//修改时钟的方式
		for(iHMC = 0; iHMC<8; iHMC++)
	{// 20MHz
	//clk=0
		//HMC_CLK = 0;
			GpioDataRegs.GPCCLEAR.bit.GPIO76=1;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		if((command >> 7)&0x01)
			GpioDataRegs.GPCSET.bit.GPIO77=1;
			else
			GpioDataRegs.GPCCLEAR.bit.GPIO77=1;
		//HMC_MOSI = command >> 7;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		command <<= 1;
		//HMC_CLK = 1;

			GpioDataRegs.GPCSET.bit.GPIO76=1;
		retHMC = (retHMC<<1) | HMC_MISO;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		// the codes takes about 0.25 us
	}
	return retHMC;
	*/
	//////////////////////////////////////////////

	
	//修改时钟的方式
		for(iHMC = 0; iHMC<8; iHMC++)
	{// 20MHz
	//clk=0
		//HMC_CLK = 0;
			GpioDataRegs.GPACLEAR.bit.GPIO15=1;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		if((command >> 7)&0x01)
			GpioDataRegs.GPASET.bit.GPIO16=1;
			else
			GpioDataRegs.GPACLEAR.bit.GPIO16=1;
		//HMC_MOSI = command >> 7;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		command <<= 1;
		//HMC_CLK = 1;

			GpioDataRegs.GPASET.bit.GPIO15=1;
		retHMC = (retHMC<<1) | HMC_MISO;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		// the codes takes about 0.25 us
	}
	return retHMC;
	
	//使用气压计的4根线
	/*
		//修改时钟的方式
		for(iHMC = 0; iHMC<8; iHMC++)
	{// 20MHz
	//clk=0
		//HMC_CLK = 0;
			GpioDataRegs.GPACLEAR.bit.GPIO21=1;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		if((command >> 7)&0x01)
			GpioDataRegs.GPASET.bit.GPIO23=1;
			else
			GpioDataRegs.GPACLEAR.bit.GPIO23=1;
		//HMC_MOSI = command >> 7;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		command <<= 1;
		//HMC_CLK = 1;

			GpioDataRegs.GPASET.bit.GPIO21=1;
		retHMC = (retHMC<<1) | HMC_MISO;
		DELAY_US(0.25);
			DELAY_US(0.25);
				DELAY_US(0.25);
					DELAY_US(0.25);
		// the codes takes about 0.25 us
	}
	return retHMC;
	*/
	
}


unsigned char HMC_WR0(unsigned char command)
{
	unsigned char iRet = 0;
	HMC_CS = 0;
	iRet = HMC_WR(command);
	HMC_CS = 1;
	DELAY_US(0.25);
	return iRet;

}
/*
Uint16 HMC_WR16(unsigned char command)
{
	Uint16 iRet = 0;
	HMC_CS = 0;
	HMC_WR(command);
	iRet = HMC_WR(0) & 0xFF;
	iRet <<= 8;
	iRet = iRet | (HMC_WR(0) & 0xFF);    //8位＋8位=16位
	HMC_CS = 1;
	DELAY_US(0.25);
	return iRet;
}

Uint32 HMC_WR24(unsigned char command)
{
	Uint32 iRet = 0;
	HMC_CS = 0;
	HMC_WR(command);
	iRet = HMC_WR(0) & 0xFF;
	iRet = (iRet<<8) | (HMC_WR(0) & 0xFF);    //8位＋8位=16位
	iRet = (iRet<<8) | (HMC_WR(0) & 0xFF);    //16位＋8位=24位
	HMC_CS = 1;
	DELAY_US(0.25);
	return iRet;
}

*/


