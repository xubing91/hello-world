/*
	MS5803_01BA, barometer
	Description: Run MS5803_Reset() and MS5803_GetPROM() after power-on,
	then run MS5803_GetTemp() when needed.
*/
//HMC
#ifndef MS5803_01BA_H__
#define MS5803_01BA_H__

#include "DSP2833x_Device.h"
//为了调试罗盘，修改端口
#define	MS5803_CS		GpioDataRegs.GPADAT.bit.GPIO23
#define	MS5803_MOSI		GpioDataRegs.GPADAT.bit.GPIO20
#define	MS5803_CLK		GpioDataRegs.GPADAT.bit.GPIO22
#define	MS5803_MISO		GpioDataRegs.GPADAT.bit.GPIO21
/*
#define	MS5803_CS		GpioDataRegs.GPBDAT.bit.GPIO59
#define	MS5803_MOSI		GpioDataRegs.GPADAT.bit.GPIO23
#define	MS5803_CLK		GpioDataRegs.GPADAT.bit.GPIO21
#define	MS5803_MISO		GpioDataRegs.GPADAT.bit.GPIO20
*/
/*
#define	MS5803_CS		GpioDataRegs.GPCDAT.bit.GPIO79
#define	MS5803_MOSI		GpioDataRegs.GPCDAT.bit.GPIO77
#define	MS5803_CLK		GpioDataRegs.GPCDAT.bit.GPIO76
#define	MS5803_MISO		GpioDataRegs.GPCDAT.bit.GPIO78
*/
/*
	Reset the barometer
*/
void	MS5803_Reset();
void	MS5803_Init();

/*
	Read PROM (128 bit of calibration words)
*/
void	MS5803_GetPROM();

/*
	Convert D1 -> for pressure, 32 bit
	OSR:4096, max converting time: 9.04 ms, Resolution RMS: 0.012 mbar
*/
void	MS5803_ConvertPrs();

/* 
	Convert D2 -> for temperature, 32 bit
*/
void	MS5803_ConvertTemp();

/*
	Read D1
	! Must be used after MS5803_ConvertPrs()
*/
void	MS5803_ReadPrs();

/*
	Read D2
	! Must be used after MS5803_ConvertTemp()
*/
void	MS5803_ReadTemp();

/*
	Calculate pressure using raw data, with 2-order temperature compensation
*/
int32	MS5803_CalcPres();
int32	MS5803_GetTemp();

unsigned char MS5803_WR(unsigned char command);
unsigned char MS5803_WR0(unsigned char command);
Uint16 MS5803_WR16(unsigned char command);
Uint32 MS5803_WR24(unsigned char command);
//外接模块的引脚对应
//这个对应的引脚是在大板子上的最左上角的四个
//  正确性有待讨论
/*
#define	HMC_CLK		GpioDataRegs.GPCDAT.bit.GPIO76
#define HMC_MOSI	GpioDataRegs.GPCDAT.bit.GPIO77
#define	HMC_MISO	GpioDataRegs.GPCDAT.bit.GPIO78
#define HMC_CS		GpioDataRegs.GPCDAT.bit.GPIO79
*/
//下面是焊接在电路板上的引脚对应

#define	HMC_CLK		GpioDataRegs.GPADAT.bit.GPIO15
#define HMC_MOSI	GpioDataRegs.GPADAT.bit.GPIO16
#define	HMC_MISO	GpioDataRegs.GPADAT.bit.GPIO13
#define HMC_CS		GpioDataRegs.GPADAT.bit.GPIO14


//重新飞的4根线
//失败的
/*
#define	HMC_CLK		GpioDataRegs.GPBDAT.bit.GPIO33
#define HMC_MOSI	GpioDataRegs.GPBDAT.bit.GPIO32
#define	HMC_MISO	GpioDataRegs.GPBDAT.bit.GPIO60
#define HMC_CS		GpioDataRegs.GPBDAT.bit.GPIO61

*/

//7月30日待修改

//使用气压计的4线
/*
#define	HMC_CS			GpioDataRegs.GPBDAT.bit.GPIO59
#define	HMC_MOSI		GpioDataRegs.GPADAT.bit.GPIO23
#define	HMC_CLK			GpioDataRegs.GPADAT.bit.GPIO21
#define	HMC_MISO		GpioDataRegs.GPADAT.bit.GPIO20
*/
//以下用测试版本
/*
#define	HMC_CLK		GpioDataRegs.GPCDAT.bit.GPIO79
#define HMC_MOSI	GpioDataRegs.GPCDAT.bit.GPIO77
#define	HMC_MISO	GpioDataRegs.GPCDAT.bit.GPIO75
#define HMC_CS		GpioDataRegs.GPCDAT.bit.GPIO73
*/
/*
	Reset the barometer
*/
void	HMC_Reset();
void	HMC_Init();
int16  HMC_TEST(int16* HMC_TESTRg);
int16  HMC_TESTABC(int16* HMC_REGIS);
char HMC_FLAG();
/*
	Read PROM (128 bit of calibration words)
*/
//void	HMC_GetPROM();

/*
	Convert D1 -> for pressure, 32 bit
	OSR:4096, max converting time: 9.04 ms, Resolution RMS: 0.012 mbar
*/
void	HMC_ConvertPrs();

/* 
	Convert D2 -> for temperature, 32 bit
*/
//void	MS5803_ConvertTemp();

/*
	Read D1
	! Must be used after MS5803_ConvertPrs()
*/
void	HMC_ReadMag();
void	HMC_ReaTemp();

/*
	Read D2
	! Must be used after MS5803_ConvertTemp()
*/
//void	MS5803_ReadTemp();

/*
	Calculate pressure using raw data, with 2-order temperature compensation
*/
void HMC_CalcMag(int16* HMC_rae);
int16 HMC_CalcTemp();
char MC_FLAG();
//int32	MS5803_GetTemp();

unsigned char HMC_WR(unsigned char command);
unsigned char HMC_WR0(unsigned char command);
Uint16 HMC_WR16(unsigned char command);
Uint32 HMC_WR24(unsigned char command);



#endif


//添加

