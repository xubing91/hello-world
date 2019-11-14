#ifndef HAL_H__
#define HAL_H__

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#define	  AIN1_Head	GpioDataRegs.GPCDAT.bit.GPIO66
#define	  AIN2_Head	GpioDataRegs.GPCDAT.bit.GPIO67
//#define	  BIN1_Head	GpioDataRegs.GPCDAT.bit.GPIO68
//#define	  BIN2_Head	GpioDataRegs.GPCDAT.bit.GPIO69
#define	  BIN1_Head	GpioDataRegs.GPBDAT.bit.GPIO47//black
#define	  BIN2_Head	GpioDataRegs.GPCDAT.bit.GPIO80

#define	  AIN1_Tail	GpioDataRegs.GPCDAT.bit.GPIO70
#define	  AIN2_Tail	GpioDataRegs.GPCDAT.bit.GPIO71
#define	  BIN1_Tail	GpioDataRegs.GPCDAT.bit.GPIO72
#define	  BIN2_Tail	GpioDataRegs.GPCDAT.bit.GPIO73

#define	  STBY1	    GpioDataRegs.GPADAT.bit.GPIO20
//#define	  STBY2	    GpioDataRegs.GPADAT.bit.GPIO69
#define	  STBY2     GpioDataRegs.GPADAT.bit.GPIO21

#define	TLC2543_CS 	GpioDataRegs.GPBDAT.bit.GPIO39
#define TLC2543_OUT	GpioDataRegs.GPADAT.bit.GPIO30
#define TLC2543_CLK	GpioDataRegs.GPADAT.bit.GPIO0
#define TLC2543_IN	GpioDataRegs.GPADAT.bit.GPIO31//这里的定义沿用原来错误的叫法

#define HQ7001_CS	GpioDataRegs.GPADAT.bit.GPIO4
#define HQ7001_CS_SET()		GpioDataRegs.GPASET.bit.GPIO4=1
#define HQ7001_CS_CLEAR()	GpioDataRegs.GPACLEAR.bit.GPIO4=1
#define HQ7001_CLK	GpioDataRegs.GPADAT.bit.GPIO6
#define	HQ7001_CLK_CLEAR()	GpioDataRegs.GPACLEAR.bit.GPIO6=1
#define	HQ7001_CLK_SET()	GpioDataRegs.GPASET.bit.GPIO6=1
#define HQ7001_MISO	GpioDataRegs.GPADAT.bit.GPIO17
#define HQ7001_MOSI	GpioDataRegs.GPADAT.bit.GPIO12
#define	HQ7001_MOSI_SET()	GpioDataRegs.GPASET.bit.GPIO12=1
#define	HQ7001_MOSI_CLEAR()	GpioDataRegs.GPACLEAR.bit.GPIO12=1

#define URM37_TRIG	GpioDataRegs.GPBDAT.bit.GPIO53

#define	SD_CS		GpioDataRegs.GPADAT.bit.GPIO11
#define	SD_MOSI		GpioDataRegs.GPADAT.bit.GPIO10
#define	SD_CLK		GpioDataRegs.GPADAT.bit.GPIO9
#define	SD_MISO		GpioDataRegs.GPADAT.bit.GPIO8

//#define MPU6000_CS		GpioDataRegs.GPCDAT.bit.GPIO71
//#define MPU6000_MOSI	GpioDataRegs.GPCDAT.bit.GPIO72
//#define	MPU6000_CLK 	GpioDataRegs.GPCDAT.bit.GPIO70
//#define	MPU6000_MISO	GpioDataRegs.GPCDAT.bit.GPIO73
//#define RE1_UR		GpioDataRegs.GPCDAT.bit.GPIO73
//#define RE1_DR		GpioDataRegs.GPCDAT.bit.GPIO71
//#define MPU6000_MOSI	GpioDataRegs.GPCDAT.bit.GPIO72
//#define	MPU6000_CLK 	GpioDataRegs.GPCDAT.bit.GPIO70
////#define	MPU6000_MISO	GpioDataRegs.GPCDAT.bit.GPIO73

// Init all hardware
void	Hardware_Init();
void TB6612_Portconfig();
// Port Configuration
void	Re1_portConfig();
void	TLC2543_PortConfig();
void	Scia_PortConfig();
void	Scib_PortConfig();
void	Scic_PortConfig();
void	PWM_PortConfig();
void	ECap_PortConfig();
void	URM37_PortConfig();
void	HQ7001_PortConfig();
void	SD_PortConfig();
void	MS5803_PortConfig();
void	HMC5983_PortConfig();

// Module Initialzation (DSP Modules & Onboard Modules)
void	HQ7001_Init();
void	Scia_Init();
void	Scib_Init();
void	Scic_Init();
void	ECap_Init();
void	PWM_Init();
void	URM37_Init();
void	SD_Init();
void	GPS_Init();

// Data Exchange
void TLC2543_Read(unsigned char addr, int16* data);	// send out address, get data
unsigned char HQ7001_Ex(unsigned char data);		// send out data, get return value
void HQ7001_Get(unsigned char* data); // XHXLYHYLZHZL
void Scia_Xmit(unsigned char c);
void Scib_Xmit(unsigned char c);
void Scic_Xmit(unsigned char c);
void Scia_msg(unsigned char * msg, unsigned char n);
void Scic_msg(unsigned char * msg, unsigned char n);
unsigned char SD_Ex(unsigned char data);
#define SetPWM1(x)	(EPwm1Regs.CMPB = x * 15)	// us 
#define SetPWM2(x)	(EPwm2Regs.CMPB = x * 15)
#define SetPWM3(x)	(EPwm3Regs.CMPB = x * 15)
#define SetPWM4(x)	(EPwm4Regs.CMPB = x * 15)
#endif
