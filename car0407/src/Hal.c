#include "Hal.h"
#include "MS5803-01BA.h"
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "DSP2833x_ECap_defines.h"
#include <string.h>
/*	Init all hardware	*/
void Hardware_Init()
{
	//Re1_portConfig();
	ECap_PortConfig();
	TLC2543_PortConfig();
	HQ7001_PortConfig();
	DELAY_US(5000);
	HQ7001_Init();

	//添加新罗盘
	DELAY_US(5000);
	HMC5983_PortConfig();
	DELAY_US(5000);
	HMC_Init();
	DELAY_US(5000);
	//
	SD_PortConfig();
	SD_Init();
	Scia_PortConfig();
	Scia_Init();
	//GPS_Init();
	Scib_PortConfig();
	Scib_Init();
	Scic_PortConfig();
	Scic_Init();
	ECap_Init();
	//init_zone6();
	PWM_PortConfig();
	PWM_Init();
	TB6612_Portconfig();

}

/*	=============================
	Section 1: Port Configuration
	=============================
*/
void TB6612_Portconfig()
{
	EALLOW;

	GpioCtrlRegs.GPCDIR.bit.GPIO66=1;    //设置输出方向  1为output
	GpioCtrlRegs.GPCDIR.bit.GPIO67=1;
	//GpioCtrlRegs.GPCDIR.bit.GPIO68=1;
	GpioCtrlRegs.GPCDIR.bit.GPIO69=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO47=1;
	GpioCtrlRegs.GPCDIR.bit.GPIO80=1;
	GpioCtrlRegs.GPCDIR.bit.GPIO70=1;
	GpioCtrlRegs.GPCDIR.bit.GPIO71=1;
	GpioCtrlRegs.GPCDIR.bit.GPIO72=1;
	GpioCtrlRegs.GPCDIR.bit.GPIO73=1;
	GpioCtrlRegs.GPADIR.bit.GPIO20=1;
	GpioCtrlRegs.GPADIR.bit.GPIO21=1;


	GpioCtrlRegs.GPCMUX1.bit.GPIO66=0;   // 通用GPIO口
	GpioCtrlRegs.GPCMUX1.bit.GPIO67=0;
	//GpioCtrlRegs.GPCMUX1.bit.GPIO68=0;
	//GpioCtrlRegs.GPCMUX1.bit.GPIO69=0;
	GpioCtrlRegs.GPBMUX1.bit.GPIO47=0;
	GpioCtrlRegs.GPCMUX2.bit.GPIO80=0;
	GpioCtrlRegs.GPCMUX1.bit.GPIO70=0;
	GpioCtrlRegs.GPCMUX1.bit.GPIO71=0;
	GpioCtrlRegs.GPCMUX1.bit.GPIO72=0;
	GpioCtrlRegs.GPCMUX1.bit.GPIO73=0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO20=0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO21=0;
	EDIS;
}

void TLC2543_PortConfig()
{
/*
	EALLOW;
	// All Set as GPIOs
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;
	// I/O
	GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;	// CS~
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;	// Device Input
	GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;	// CLK
	GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0;	// Device Output
	EDIS;
	// Initial Status
	TLC2543_CS = 1;
	TLC2543_OUT = 0;
	TLC2543_CLK = 0;
	DELAY_US(10);
	TLC2543_CS = 0;
	DELAY_US(10); //?
	*/
	EALLOW;
	// All Set as GPIOs
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
	// I/O
	GpioCtrlRegs.GPBDIR.bit.GPIO39= 1;	// CS~
	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;	// Device Input
	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;	// CLK
	GpioCtrlRegs.GPADIR.bit.GPIO31 = 0;	// Device Output
	EDIS;
	// Initial Status
	TLC2543_CS = 1;
	TLC2543_OUT = 0;
	TLC2543_CLK = 0;
	DELAY_US(10);
	TLC2543_CS = 0;
	DELAY_US(10); //?
}

void Scia_PortConfig()
{
	EALLOW;
	// Version I:	GPIO36 - SCIRXDA; GPIO35 - SCITXDA(unavailable)
	// Version II:	GPIO28 - SCIRXDA; GPIO29 - SCITXDA
/* Enable internal pull-up for the selected pins */
	//GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;	   // Enable pull-up for GPIO36 (SCIRXDA)
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	   // Enable pull-up for GPIO29 (SCITXDA)
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation	
	EDIS;
}

void Scib_PortConfig()
{
	EALLOW;
/* Enable internal pull-up for the selected pins */
	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	   // Enable pull-up for GPIO19 (SCIRXDB)
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	   // Enable pull-up for GPIO18 (SCITXDB)
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO28 (SCIRXDB)
/* Configure SCI-B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDB operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDB operation	

	EDIS;
}

void Scic_PortConfig()
{
	EALLOW;
/* Enable internal pull-up for the selected pins */
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;	   // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO28 (SCIRXDB)
/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO62 for SCITXDC operation	

/* For Reset~ Pin */
	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1; // output
	EDIS;
}

void PWM_PortConfig()
{
	EALLOW;
	/* Configure as EPWM */
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;

	EDIS;
}

void URM37_PortConfig()
{
/*
	EALLOW;

	// GPIO0 - output to TRIG
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;	
	// ECAP5 - input from URM-PWM
	// 在ECAP_Config中已有

	EDIS;
	*/
		EALLOW;

	// GPIO0 - output to TRIG
	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;	
	// ECAP5 - input from URM-PWM
	// 在ECAP_Config中已有

	EDIS;
}

void ECap_PortConfig()
{
	EALLOW;

	// Enable Pull-Up Resistors
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;

	// Select as ECapX
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 1;
	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 1;

	// Synch Input
	GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0;  
	GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;  
	GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;  
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0;  
	GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 0;  
	GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 0;  

	EDIS;
}

void HQ7001_PortConfig()
{
/*
	EALLOW;
	// All Set as GPIOs
	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;
	// I/O
	GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1; // SIMO
	GpioCtrlRegs.GPBDIR.bit.GPIO55 = 0; // SOMI
	GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1; // CLK
	GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1; // CS~
	EDIS;
	HQ7001_CS = 1;
	HQ7001_CLK = 1;
	HQ7001_MOSI = 0;
	*/
		EALLOW;
	// All Set as GPIOs
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
	// I/O
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1; // SIMO
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0; // SOMI
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1; // CLK
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1; // CS~
	EDIS;
	HQ7001_CS = 1;
	HQ7001_CLK = 1;
	HQ7001_MOSI = 0;
}

void SD_PortConfig()
{
/*
	EALLOW;
	// All Set as GPIOs
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
	GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;
	GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;
	// I/O
	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;	// CS~
	GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;	// Device Input
	GpioCtrlRegs.GPCDIR.bit.GPIO84 = 1;	// CLK
	GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0;	// Device Output
	EDIS;
	SD_CS = 1;
	SD_MOSI = 0;
	SD_CLK = 0;
	*/

	EALLOW;
	// All Set as GPIOs
	GpioCtrlRegs.GPAMUX1.bit.GPIO11= 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
	// I/O
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;	// CS~
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;	// Device Input
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;	// CLK
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;	// Device Output
	EDIS;
	SD_CS = 1;
	SD_MOSI = 0;
	SD_CLK = 0;
}

void	MS5803_PortConfig()
{
/*
	EALLOW;
	// All set as GPIOs
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;
	// I/O
	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;	// CS~
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;	// Device Input
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;	// CLK
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;	// Device Output
	EDIS;
	MS5803_CS = 1;
	MS5803_MOSI = 0;
	MS5803_CLK = 1;
	MS5803_CS = 0;
	*/
		EALLOW;
	// All set as GPIOs
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
	// I/O
	GpioCtrlRegs.GPADIR.bit.GPIO23= 1;	// CS~
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;	// Device Input
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;	// CLK
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;	// Device Output
	EDIS;
	MS5803_CS = 1;
	MS5803_MOSI = 0;
	MS5803_CLK = 1;
	MS5803_CS = 0;
}
void	HMC5983_PortConfig()
{
	EALLOW;
	// All set as GPIOs
//以下是焊接在电路上的引脚配置

	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
	GpioCtrlRegs.GPAMUX1.bit.GPIO13= 0;
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
	// I/O
	GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;	// CS~
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;	// Device Input
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;	// CLK
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;	// Device Output



	//以下是测试版本
/*

	GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0;
	GpioCtrlRegs.GPCMUX1.bit.GPIO75= 0;
	GpioCtrlRegs.GPCMUX1.bit.GPIO77= 0;
	GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0;
	// I/O
	GpioCtrlRegs.GPCDIR.bit.GPIO73 = 1;	// CS~
	GpioCtrlRegs.GPCDIR.bit.GPIO77= 1;	// Device Input
	GpioCtrlRegs.GPCDIR.bit.GPIO79 = 1;	// CLK
	GpioCtrlRegs.GPCDIR.bit.GPIO75 = 0;	// Device Output
*/

	EDIS;
	HMC_CS = 1;
	HMC_MOSI = 0;
	HMC_CLK = 1;
	HMC_CS = 0;

}
/*	================================
	Section 2: Module Initialization
	================================
*/
void	HQ7001_Init()
{
	HQ7001_CS = 0;
	HQ7001_Ex(0x20);
	HQ7001_Ex(0xD7);	// 11010111
	HQ7001_CS = 1;
	HQ7001_CS = 0;
	HQ7001_Ex(0x21);
	HQ7001_Ex(0xE5);	// 11100101
	HQ7001_CS = 1;
//	Accl_Ex(0x22);
//	Accl_Ex(0x02);	// 00000010
}

void Scia_Init()
{
/* SciA FIFO Init */
    SciaRegs.SCIFFTX.all=0xE040;	// EF40: + 15Bytes for int
    SciaRegs.SCIFFRX.all=0x2041;	// 该配置有问题？、、、、、、、、、、、、、、
    SciaRegs.SCIFFCT.all=0x0;	
	SciaRegs.SCIFFRX.bit.RXFFIENA = 1; // Rx Int Enable

/* SciA Init */
// Note: Clocks were turned on to the SCIA peripheral
// in the InitSysCtrl() function
 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
//	      SciaRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
//	      SciaRegs.SCILBAUD    =0x0028;
		SciaRegs.SCIHBAUD = 0x0000;
		SciaRegs.SCILBAUD = 0x0024;	// 115200 / LSPCLK = 135MHz/4
	#endif
	#if (CPU_FRQ_100MHZ)
      SciaRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 20MHz.
      SciaRegs.SCILBAUD    =0x0015;
	#endif
	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

}

void Scib_Init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //ScibRegs.SCIFFTX.all=0x8000;
 	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA = 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
	      ScibRegs.SCIHBAUD    =0x0000;  // 9600 baud @LSPCLK = 37.5MHz.
	      ScibRegs.SCILBAUD    =0x0048;
//	      ScibRegs.SCILBAUD    =0x0050;
	#endif
	#if (CPU_FRQ_100MHZ)
      ScicRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      ScicRegs.SCILBAUD    =0x0044;
	#endif
	ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}
/*
void Scib_Init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	//ScibRegs.SCICTL2.all =0x0003;
	//ScibRegs.SCICTL2.bit.TXINTENA = 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
	      ScibRegs.SCIHBAUD    =0x0000;  // 9600 baud @LSPCLK = 37.5MHz.
	      ScibRegs.SCILBAUD    =0x0008;
	#endif
	#if (CPU_FRQ_100MHZ)
      SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      SciaRegs.SCILBAUD    =0x0044;
	#endif
      ScibRegs.SCIFFTX.all=0xC028;
      ScibRegs.SCIFFRX.all=0x002F;
      ScibRegs.SCIFFCT.all=0x00;
	ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
	   ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;
	   ScibRegs.SCIFFRX.bit.RXFIFORESET=1;


}
*/
void Scic_Init()
{
/* SciC FIFO Init */
    ScicRegs.SCIFFTX.all=0xE040;
    ScicRegs.SCIFFRX.all=0x2041;	// ori: 2041
    ScicRegs.SCIFFCT.all=0x0;
	ScicRegs.SCIFFRX.bit.RXFFIENA = 1; // Rx Int Enable

/* SciC Init */
// Note: Clocks were turned on to the SCIC peripheral
// in the InitSysCtrl() function
 	ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA = 1;
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
//	      ScicRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
//	      ScicRegs.SCILBAUD    =0x0028;
		ScicRegs.SCIHBAUD = 0x0000;
//		ScicRegs.SCILBAUD = 0x0028;	// 115200 / LSPCLK = 135MHz/4
		ScicRegs.SCILBAUD = 0x0024;	// 115200 / LSPCLK = 135MHz/4
	#endif
	#if (CPU_FRQ_100MHZ)
      ScicRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 20MHz.
      ScicRegs.SCILBAUD    =0x0015;
	#endif
	ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

/* Reset Cancelled*/
	GpioDataRegs.GPBDAT.bit.GPIO58 = 1;
}

void ECap_Init()
{
	// Code snippet for CAP mode Delta Time, Rising and Falling
	// edge triggers
	// Initialization Time
	//=======================
	// ECAP module 1 config
	ECap1Regs.ECEINT.all = 0x0000;	// Disable all capture interrupts
	ECap1Regs.ECCLR.all = 0xFFFF;	// Clear all CAP interrupt flags
	ECap1Regs.ECCTL1.bit.CAPLDEN = EC_DISABLE;
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_FREEZE;
	
//	ECap1Regs.ECCTL1.bit.FREE_SOFT = 0x11;
	ECap1Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap1Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
	ECap1Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
	ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
	ECap1Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
	ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
	ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run

	ECap1Regs.ECEINT.bit.CEVT2 = EC_ENABLE; //enable capture event 2 interrupt.
	ECap1Regs.ECEINT.bit.CEVT4 = EC_ENABLE; //enable capture event 4 interrupt. 
	ECap1Regs.ECCLR.all = 0xFF;	
	// Code snippet for CAP mode Delta Time, Rising and Falling
	// edge triggers
	// Initialization Time
	//=======================
	// ECAP module 2 config
	ECap2Regs.ECEINT.all = 0x0000;	// Disable all capture interrupts
	ECap2Regs.ECCLR.all = 0xFFFF;	// Clear all CAP interrupt flags
	ECap2Regs.ECCTL1.bit.CAPLDEN = EC_DISABLE;
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = EC_FREEZE;
	
//	ECap2Regs.ECCTL1.bit.FREE_SOFT = 0x11;
	ECap2Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap2Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap2Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap2Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
	ECap2Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
	ECap2Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
	ECap2Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
	ECap2Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
	ECap2Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap2Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap2Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap2Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap2Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run}

	ECap2Regs.ECEINT.bit.CEVT2 = EC_ENABLE; //enable capture event 2 interrupt.
	ECap2Regs.ECEINT.bit.CEVT4 = EC_ENABLE; //enable capture event 4 interrupt. 
	ECap2Regs.ECCLR.all = 0xFF;	

	// Code snippet for CAP mode Delta Time, Rising and Falling
	// edge triggers
	// Initialization Time
	//=======================
	// ECAP module 3 config
	ECap3Regs.ECEINT.all = 0x0000;	// Disable all capture interrupts
	ECap3Regs.ECCLR.all = 0xFFFF;	// Clear all CAP interrupt flags
	ECap3Regs.ECCTL1.bit.CAPLDEN = EC_DISABLE;
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = EC_FREEZE;
	
//	ECap3Regs.ECCTL1.bit.FREE_SOFT = 0x11;
	ECap3Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap3Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap3Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap3Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
	ECap3Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
	ECap3Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
	ECap3Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
	ECap3Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
	ECap3Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap3Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap3Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap3Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
	ECap3Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap3Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run

	ECap3Regs.ECEINT.bit.CEVT2 = EC_ENABLE; //enable capture event 2 interrupt.
	ECap3Regs.ECEINT.bit.CEVT4 = EC_ENABLE; //enable capture event 4 interrupt. 
	ECap3Regs.ECCLR.all = 0xFF;	

	// Code snippet for CAP mode Delta Time, Rising and Falling
	// edge triggers
	// Initialization Time
	//=======================
	// ECAP module 4 config
	ECap4Regs.ECEINT.all = 0x0000;	// Disable all capture interrupts
	ECap4Regs.ECCLR.all = 0xFFFF;	// Clear all CAP interrupt flags
	ECap4Regs.ECCTL1.bit.CAPLDEN = EC_DISABLE;
	ECap4Regs.ECCTL2.bit.TSCTRSTOP = EC_FREEZE;
	
//	ECap4Regs.ECCTL1.bit.FREE_SOFT = 0x11;
	ECap4Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap4Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap4Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap4Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
	ECap4Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
	ECap4Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
	ECap4Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
	ECap4Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
	ECap4Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap4Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap4Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap4Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
	ECap4Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap4Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	ECap4Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run

	ECap4Regs.ECEINT.bit.CEVT2 = EC_ENABLE; //enable capture event 2 interrupt.
	ECap4Regs.ECEINT.bit.CEVT4 = EC_ENABLE; //enable capture event 4 interrupt. 
	ECap4Regs.ECCLR.all = 0xFF;	

	/* ECap5 for Ultrasonic sensor 
	// Code snippet for CAP mode Delta Time, Rising and Falling
	// edge triggers
	// Initialization Time
	//=======================
	// ECAP module 1 config
	ECap5Regs.ECEINT.all = 0x0000;	// Disable all capture interrupts
	ECap5Regs.ECCLR.all = 0xFFFF;	// Clear all CAP interrupt flags
	ECap5Regs.ECCTL1.bit.CAPLDEN = EC_DISABLE;
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = EC_FREEZE;
	
	//ECap5Regs.ECCTL1.bit.FREE_SOFT = 0x11;
	ECap5Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap5Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap5Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap5Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
	ECap5Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
	ECap5Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
	ECap5Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
	ECap5Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
	ECap5Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap5Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap5Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap5Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
	ECap5Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap5Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run

	ECap5Regs.ECEINT.bit.CEVT2 = EC_ENABLE; //enable capture event 2 interrupt.
	ECap5Regs.ECEINT.bit.CEVT4 = EC_ENABLE; //enable capture event 4 interrupt. 
	ECap5Regs.ECCLR.all = 0xFF;	
	*/

	// Code snippet for CAP mode Delta Time, Rising and Falling
	// edge triggers
	// Initialization Time
	//=======================
	// ECAP module 1 config
	ECap6Regs.ECEINT.all = 0x0000;	// Disable all capture interrupts
	ECap6Regs.ECCLR.all = 0xFFFF;	// Clear all CAP interrupt flags
	ECap6Regs.ECCTL1.bit.CAPLDEN = EC_DISABLE;
	ECap6Regs.ECCTL2.bit.TSCTRSTOP = EC_FREEZE;
	
	//ECap6Regs.ECCTL1.bit.FREE_SOFT = 0x11;
	ECap6Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap6Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap6Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap6Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
	ECap6Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
	ECap6Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;
	ECap6Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
	ECap6Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;
	ECap6Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap6Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap6Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap6Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
	ECap6Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap6Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	ECap6Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run

	ECap6Regs.ECEINT.bit.CEVT2 = EC_ENABLE; //enable capture event 2 interrupt.
	ECap6Regs.ECEINT.bit.CEVT4 = EC_ENABLE; //enable capture event 4 interrupt. 
	ECap6Regs.ECCLR.all = 0xFF;	
}

void PWM_Init()
{
	EPwm1Regs.TBPRD = 0;			// Phase = 0
	EPwm1Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm1Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm1Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm1Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm1Regs.TBPRD = 10000;
	EPwm1Regs.CMPB = 0;		// ! 15 / 13.5 e1MHz

	EPwm2Regs.TBPRD = 0;			// Phase = 0
	EPwm2Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm2Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm2Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm2Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm2Regs.TBPRD = 10000;
	EPwm2Regs.CMPB = 0;

	EPwm3Regs.TBPRD = 0;			// Phase = 0
	EPwm3Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm3Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm3Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm3Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm3Regs.TBPRD = 10000;
	EPwm3Regs.CMPB = 0;

	EPwm4Regs.TBPRD = 0;			// Phase = 0
	EPwm4Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm4Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm4Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm4Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm4Regs.TBPRD = 10000;
	EPwm4Regs.CMPB = 0;

}

void URM37_Init()
{
	URM37_TRIG = 1;	// 平时高电平

	// Code snippet for CAP mode Delta Time, Rising and Falling
	// edge triggers
	// Initialization Time
	//=======================
	// ECAP module 1 config
	EALLOW;
	ECap5Regs.ECEINT.all = 0x0000;	// Disable all capture interrupts
	ECap5Regs.ECCLR.all = 0xFFFF;	// Clear all CAP interrupt flags
	ECap5Regs.ECCTL1.bit.CAPLDEN = EC_DISABLE;
	ECap5Regs.ECCTL2.bit.TSCTRSTOP = EC_FREEZE;
	
	//ECap5Regs.ECCTL1.bit.FREE_SOFT = 0x11;
	ECap5Regs.ECCTL1.bit.CAP1POL = EC_RISING;
	ECap5Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
	ECap5Regs.ECCTL1.bit.CAP3POL = EC_RISING;
	ECap5Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
	ECap5Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;
	ECap5Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
	ECap5Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;
	ECap5Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
	ECap5Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
	ECap5Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
	ECap5Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
	ECap5Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
	ECap5Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
	ECap5Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
	
	ECap5Regs.ECEINT.bit.CEVT1 = 1;
	ECap5Regs.ECEINT.bit.CEVT3 = 1;

	ECap5Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run
	EDIS;
}

unsigned char cnt_SD = 0;
unsigned char cRet;	// char returned
unsigned char bSDExist = 0;	// if the Micro SD Card EXISTS?

void SD_Init()
{
	for (cnt_SD = 0; cnt_SD<10; cnt_SD++) // 74个脉冲
		SD_Ex(0xff);
	SD_CS = 0;

	// RESET指令
	SD_Ex(0x40);
	SD_Ex(0);
	SD_Ex(0);
	SD_Ex(0);
	SD_Ex(0);
	SD_Ex(0x95);

	for(cnt_SD=0; cnt_SD<200 ;cnt_SD++)
	{
		cRet = SD_Ex(0xff);
		if( cRet != 0xff )
			break;
	} // 应返回0x01
	if (cRet == 0xff)	// Reset Failed
		return;

	SD_CS = 1;
	SD_Ex(0xff); // 空时钟
	
	do{
		SD_CS = 0;
		SD_Ex(0x41);
		SD_Ex(0);
		SD_Ex(0);
		SD_Ex(0);
		SD_Ex(0);
		SD_Ex(0xff);
		for(cnt_SD=0; cnt_SD<200 ;cnt_SD++)
		{
			cRet = SD_Ex(0xff);
			if( cRet != 0xff )
				break;
		} // 应返回0x00，否则重试
		if(cRet == 0xff)	// Initialzation Failed
			return;

		SD_CS = 1;
		SD_Ex(0xff); // 空时钟
	}while(cRet != 0x00);
	bSDExist = 1;
}

/*	================================
	Section 3: Data Exchange
	================================
*/
/* TLC2543: send out address, get data */
unsigned char cnt_TLC;
Uint16 output_TLC;
void TLC2543_Read(unsigned char addr, int16* data)	
{// 1 MHz clock
	TLC2543_CS = 0;
	*data = 0;
	output_TLC = addr<<12 | 0x0D00;// 16-bit mode, MSB first, bipolar: 1101
	for(cnt_TLC = 0; cnt_TLC < 16; cnt_TLC++)
	{
		TLC2543_OUT = output_TLC>>15;	// output
		output_TLC = output_TLC<<1;	
		DELAY_US(0.5);

		TLC2543_CLK = 1;				// clock rises
		*data = (*data)<<1 | TLC2543_IN;	// read a bit
		DELAY_US(0.5);
		TLC2543_CLK = 0;				// clock falls
	}
	TLC2543_CS = 1;
}

/* HQ7001: 
	HQ7001_Ex: send out a byte, return a byte
	HQ7001_Get:send out a command, return acceleration data
 */
unsigned char cnt_HQ7001;
unsigned char ret_HQ7001;
unsigned char HQ7001_Ex(unsigned char data) 
{// 1MHz
	ret_HQ7001 = 0;
	DELAY_US(0.5);// added
	for(cnt_HQ7001=0; cnt_HQ7001<8; cnt_HQ7001++)
	{
		HQ7001_CLK_CLEAR();
		if((data&0xff)>>7)
			HQ7001_MOSI_SET();
		else
			HQ7001_MOSI_CLEAR();
		data = data << 1;
		DELAY_US(0.5);
		HQ7001_CLK_SET();
		ret_HQ7001 = (ret_HQ7001<<1) | HQ7001_MISO;
		DELAY_US(0.5);
	}
	return ret_HQ7001;
}
void HQ7001_Get(unsigned char* data)	// XHXLYHYLZHZL
{
	HQ7001_CS_CLEAR();
	HQ7001_Ex(0xE8);
	data[0] = HQ7001_Ex(0);
	data[1] = HQ7001_Ex(0);
	data[2] = HQ7001_Ex(0);
	data[3] = HQ7001_Ex(0);
	data[4] = HQ7001_Ex(0);
	data[5] = HQ7001_Ex(0);
	HQ7001_CS_SET();
}

/* Sci a/b/c:
	transmit 1 char
*/
void Scia_Xmit(unsigned char c)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST > 14) {}// 岂不失去了FIFO的作用？
    SciaRegs.SCITXBUF=c;
}	

void Scib_Xmit(unsigned char c)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}// 岂不失去了FIFO的作用？
    ScibRegs.SCITXBUF=c;
}	

void Scic_Xmit(unsigned char c)
{
    while (ScicRegs.SCIFFTX.bit.TXFFST > 14) {}// 岂不失去了FIFO的作用？
    ScicRegs.SCITXBUF=c;
}

/* Sci a/c:
	transmit 1 message
*/
void Scia_msg(unsigned char * msg, unsigned char n)
{
    int i;
    i = 0;
    for(i=0; i<n; i++)
    {
        Scia_Xmit(msg[i]);
    }
}
void Scic_msg(unsigned char * msg, unsigned char n)
{
    int i;
    i = 0;
    for(i=0; i<n; i++)
    {
        Scic_Xmit(msg[i]);
    }
}

unsigned char iSD;
unsigned char retSDEx;
/*
unsigned char SD_Ex(unsigned char data)
{// 386- clocks
	retSDEx = 0;
	for(iSD=0; iSD<8; iSD++)
	{	// 44 clocks ~ 3.4MHz
		SD_CLK = 0;
		SD_MOSI = data>>7; 
		data = data << 1;
		// DELAY_US(0.5);
		SD_CLK = 1;
		retSDEx = (retSDEx<<1) | SD_MISO;
		// DELAY_US(0.5);
	}
	return retSDEx;
}
*/
unsigned char SD_Ex(unsigned char data)
{// 386- clocks
	retSDEx = 0;
	for(iSD=0; iSD<8; iSD++)
	{	// 44 clocks ~ 3.4MHz
	//	SD_CLK = 0;

		GpioDataRegs.GPACLEAR.bit.GPIO9=1;//CLK
		if ((data>>7)&0X01)
		GpioDataRegs.GPASET.bit.GPIO10 =1; //MOSI
		else
		GpioDataRegs.GPACLEAR.bit.GPIO10 =1; 
		DELAY_US(0.25);
		data = data << 1;
		 
	//	SD_CLK = 1;

		GpioDataRegs.GPASET.bit.GPIO9=1;//CLK
		retSDEx = (retSDEx<<1) | SD_MISO;
		// DELAY_US(0.5);
	}
	return retSDEx;
}
//char* strGPSInit = "$GCCMD,OUTPUT,com1,gphpp,0.1*FF\x0D\x0A$GCCMD,OUTPUT,com1,gprmc,0*FF\x0D\x0A$GCCMD,SAVE CONFIG*FF\0x0D\0x0A";
//void GPS_Init()
//{
//	Scia_msg((unsigned char*)strGPSInit, strlen(strGPSInit));
//}

