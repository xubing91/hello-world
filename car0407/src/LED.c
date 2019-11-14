#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "DSP2833x_ECap_defines.h"
#include <stdio.h>
#include "Filter.h"
#include "Controller.h"
#include "remote.h"
#define	  LED1	GpioDataRegs.GPADAT.bit.GPIO1
// #define	  LED2	GpioDataRegs.GPBDAT.bit.GPIO61

interrupt void ISRTimer0(void);
void configtestled(void);
Uint16*  globalAddr = (Uint16*)(0x100000);
Uint32 offset;

Uint16 	data16[3];
unsigned char	data8[40];	 // U8 Êý¾ÝÖÐ×ª

#define	AD_CS 	GpioDataRegs.GPADAT.bit.GPIO2
#define AD_OUT	GpioDataRegs.GPADAT.bit.GPIO4
#define AD_CLK	GpioDataRegs.GPBDAT.bit.GPIO50
#define AD_IN	GpioDataRegs.GPBDAT.bit.GPIO53
#define ACCL_CS	GpioDataRegs.GPBDAT.bit.GPIO57
#define ACCL_CLK	GpioDataRegs.GPBDAT.bit.GPIO56
#define ACCL_MISO	GpioDataRegs.GPBDAT.bit.GPIO55
#define ACCL_MOSI	GpioDataRegs.GPBDAT.bit.GPIO54
#define URM_TRIG	GpioDataRegs.GPADAT.bit.GPIO0

void ECap_Config();
void ECap_Init();
void AD_Config();
void AD_Read(unsigned char addr, Uint16* data);	// send out address, get data
void Accl_Config();
unsigned char Accl_Ex(unsigned char data);		// send out data, get return value
void Accl_Init();
void Accl_Get(unsigned char* data);	// XHXLYHYLZHZL
void Scia_Config();
void Scia_Init();
void Scia_Xmit(unsigned char c);
void Scia_msg(unsigned char * msg, unsigned char n);
void Scib_Config();
void Scib_Init();
void Scib_Xmit(unsigned char c);
void Scic_Config();
void Scic_Init();
void Scic_Xmit(unsigned char c);
void Scic_msg(unsigned char * msg, unsigned char n);
void PWM_Config();
void PWM_Init();
void URM_Config();
void URM_Init();
interrupt void URM_ISR();
void init_zone6();
void 	InitI2C(void);
void	SendData(Uint16 data);
Uint16 	ReadData(unsigned char  *RamAddr, unsigned char	RomAddress, Uint16 number);
Uint16 	WriteData(unsigned char	*Wdata, unsigned char	RomAddress, Uint16	number);
unsigned char srf08range = 200;
unsigned char srf08gain = 0x10;
Uint16	I2C_xrdy();
Uint16	I2C_rrdy();
void main(void)
{
//	unsigned char addr[3] = {0, 1, 2};
	unsigned char cnt, i;
	int* itemp;
	Uint32 timer[12];
	unsigned char timerc[6];
	Uint16 data[6];

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example
   InitXintf16Gpio();	//zq

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2833x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &ISRTimer0;
   PieVectTable.ECAP5_INT = &URM_ISR;
   //PieVectTable.XINT13 = &cpu_timer1_isr;
   //PieVectTable.TINT2 = &cpu_timer2_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in DSP2833x_CpuTimers.c
   InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 150MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 150, 10000);
   //ConfigCpuTimer(&CpuTimer1, 150, 1000000);
   //ConfigCpuTimer(&CpuTimer2, 150, 1000000);
	StartCpuTimer0();

// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
// which is connected to CPU-Timer 1, and CPU int 14, which is connected
// to CPU-Timer 2:
    IER |= M_INT1;
   //IER |= M_INT13;
   //IER |= M_INT14;

// Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	
	// Enable ECap5 Int
	IER |= M_INT4;
	PieCtrlRegs.PIEIER4.bit.INTx5 = 1;

// Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    //configtestled();
	ECap_Config();
	AD_Config();
	Accl_Config();
	DELAY_US(5000);
	Accl_Init();
	Scia_Config();
	Scia_Init();
	Scib_Config();
	Scib_Init();
	Scic_Config();
	Scic_Init();
	ECap_Init();
	//init_zone6();
	PWM_Config();
	PWM_Init();
	URM_Config();
	URM_Init();

//	LED1=0;
//	LED2=0;
	i = 0;
//	for(offset = 0; offset < 0x700; offset++)
//		Scia_msg((unsigned char*)(globalAddr + offset), 1);
/*
	for(offset = 0; offset < 0x7ffff; offset++)
	{
		if (*(globalAddr + offset) != (offset & 0xffff))
			printf("%ld ", offset);
	}
*/
    for(; ;)
    {   

/*
		timer[0] = 1.0L * ECap1Regs.CAP2 / ECap1Regs.CAP3 * 1000;
		timer[1] = 1.0L * ECap2Regs.CAP2 / ECap2Regs.CAP3 * 1000;
		timer[2] = 1.0L * ECap3Regs.CAP2 / ECap3Regs.CAP3 * 1000;
		timer[3] = 1.0L * ECap4Regs.CAP2 / ECap4Regs.CAP3 * 1000;
		timer[4] = 1.0L * ECap5Regs.CAP2 / ECap5Regs.CAP3 * 1000;
		timer[5] = 1.0L * ECap6Regs.CAP2 / ECap6Regs.CAP3 * 1000;
		for( i=0; i<6; i++)
			timerc[i] = (unsigned char)(timer[i]);

		Scia_msg(timerc, 6);
		DELAY_US(10000);
*/		
/*
		for(cnt=0; cnt<6; cnt++)
		{
			AD_Read(cnt, &(data[cnt]));
			DELAY_US(10);
		}

		printf("%d %d %d %d %d %d\n", (Uint16)(data[0]), (Uint16)(data[1]), (Uint16)(data[2]), (Uint16)(data[3]), (Uint16)(data[4]), (Uint16)(data[5]));
*/	
/*
	Accl_Get(dAccl);
	Scia_Xmit((unsigned char)(dAccl[0]));
	Scia_Xmit(dAccl[1]);
	Scia_Xmit(dAccl[2]);
	printf("%d %d %d\n", dAccl[0], dAccl[1], dAccl[2] );
*/

/*
	Scib_Xmit(0x21);
	DELAY_US(10000);

	cnt = ScibRegs.SCIFFRX.bit.RXFFST;
	for (i=0; i<cnt; i++)
		data[i] = ScibRegs.SCIRXBUF.all;
	for (i=0; i<cnt; i++)
		printf("%d ", data[i]);

	printf("%d ", ScibRegs.SCIFFRX.bit.RXFFST);
	printf("\n");
*/
/*
		Scic_Xmit('a');
*/
/*
	cnt = ScicRegs.SCIFFRX.bit.RXFFST;
	if(cnt > 0)
	{
		for (i=0; i<cnt; i++)
			data[i] = ScicRegs.SCIRXBUF.all;
		for (i=0; i<cnt; i++)
			printf("%d ", data[i]);

		printf("%d \n", ScicRegs.SCIFFRX.bit.RXFFST);
	}
*/
	}
}

union sensors{
	double datas[12];
	struct{
		double accx;
		double accy;
		double accz;
		double gyrx;
		double gyry;
		double gyrz;
		double magx;
		double magy;
		double magz;
		double accxa;
		double accya;
		double accza;
	}data;
}sensor_raw, sensor;
double att[3];


#define SetRotor1(x)	(EPwm1Regs.CMPB = x * 15)
#define SetRotor2(x)	(EPwm2Regs.CMPB = x * 15)
#define SetRotor3(x)	(EPwm3Regs.CMPB = x * 15)
#define SetRotor4(x)	(EPwm4Regs.CMPB = x * 15)
// ½«¼ÓËÙ¶È¼Æ¡¢ÍÓÂÝÒÇÔ­Ê¼Êý¾Ý×ªÎªm/s^2£¬deg/s
double K_sensor[12] = {0.000581258338693, -0.000585113302634, -0.000596543473195, 0.013098847364511, 0.012750149172609, 0.013250664689549, -0.001207233143704, -0.001287264188984, -0.001277112601871, 1, 1, 1};
double B_sensor[12] = {-0.055451528151945, -0.062736558188126, 0.52333181535163, 0,0,0 , -13.307103654456375, -14.220142042617127, -14.784392821534997, 0, 0, 0};
unsigned char cX;
double 	fGyro_B[3] = {0, 0, 0};
int16	iGyro_B[3] = {0, 0, 0};
double	att_set[3] = {0, 0, 0};	// Éè¶¨×ËÌ¬½Ç
int		t_rotor[4];		// ÐýÒí¿ØÖÆ²¨ÐÎÂö¿í£¨us£©
unsigned char	xbCmd[4];	// Zigbee Command
double t_start = 1, t_end = 1;
unsigned char t_used;
Uint16 urm_val = 0;	// mm
unsigned char URM_cnt = 0;
double height = 0, height_mm = 0;	// m
interrupt void ISRTimer0(void)
{
	unsigned char i, cnt = 0;
    CpuTimer0.InterruptCount++;

   // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    CpuTimer0Regs.TCR.bit.TIF=1;
    CpuTimer0Regs.TCR.bit.TRB=1;

	// ¼ÆÊ±
	t_used = (unsigned char)(100*t_end/t_start);	// ÉÏÒ»ÖÜÆÚÊ¹ÓÃµÄÊ±¼ä
	t_start = (double)(CpuTimer0.RegsAddr->TIM.all);


	// Æô¶¯´ÅÇ¿¼ÆÊý¾Ý¶ÁÈ¡
	Scib_Xmit(0x21);
	// Æô¶¯URM37Âö³åºÍ¸ß¶È¿ØÖÆÁ¿¼ÆËã
	URM_cnt = (URM_cnt + 1) % 5;
	if (URM_cnt == 0)
	{
		URM_TRIG = 0;
		// ¸ß¶È¿ØÖÆÁ¿¼ÆËã
		if( urm_val > 6000 ) // ÎÞÐ§£¬Ìø¹ý´¦Àí
			;
		else
		{// ÂË´óÌø¶¯
			if ( urm_val > height_mm + 250 ) // 25cm / 0.05s = 5m/s
				height_mm += 250;
			else if ( urm_val < height_mm - 250)
				height_mm -= 250;
			else
				height_mm = urm_val;

			height = height_mm / 1000;
		}
		Controller_Height(&height, &(data8[22]));
	}
		
	// ¶ÁÈ¡Êý×Ö¼ÓËÙ¶È¼ÆÊý¾Ý
	Accl_Get(data8);
	sensor_raw.data.accx = (int16)(data8[0]<<8 | data8[1]);
	sensor_raw.data.accy = (int16)(data8[2]<<8 | data8[3]);
	sensor_raw.data.accz = (int16)(data8[4]<<8 | data8[5]);
	// ¶ÁÈ¡A/D£¨Ä£Äâ¼ÓÙ¶È¼ÆºÍÍÓÂÝÒÇ£©Êý¾Ý
	AD_Read(0, data16);
	for(cnt=0; cnt<6; cnt++)
	{
		DELAY_US(10);
		AD_Read(cnt+1, &(data16[0]));
		data8[6 + (cnt<<1)] = data16[0] >> 8;
		data8[7 + (cnt<<1)] = data16[0] & 0xFF;
	}
	sensor_raw.data.gyrz = (int16)(data8[6]<<8 | data8[7]);
	sensor_raw.data.gyry = (int16)(data8[8]<<8 | data8[9]);
	sensor_raw.data.gyrx = (int16)(data8[10]<<8 | data8[11]);
	cX = data8[6];	data8[6] = data8[10];	data8[10] = cX;	// ½»»»£¬ÒòÎªÍÓÂÝÒÇÊý¾Ý´ÎÐòÎªZYX
	cX = data8[7];	data8[7] = data8[11]; 	data8[11] = cX;
	sensor_raw.data.accxa = (int16)(data8[12]<<8 | data8[13]);
	sensor_raw.data.accya = (int16)(data8[14]<<8 | data8[15]);
	sensor_raw.data.accza = (int16)(data8[16]<<8 | data8[17]);
	// ÍÓÂÝÒÇÔ¤´¦Àí
	if (CpuTimer0.InterruptCount <= 1000)
	{ // Ç°Ò»Ç§µã£¬Êä³ö½ÇËÙ¶ÈÎª0£¬Í¬Ê±¼ÆËãÆ½¾ùÖµ
		fGyro_B[0] += sensor_raw.data.gyrx;
		fGyro_B[1] += sensor_raw.data.gyry;
		fGyro_B[2] += sensor_raw.data.gyrz;
		sensor_raw.data.gyrx = 0;
		sensor_raw.data.gyry = 0;
		sensor_raw.data.gyrz = 0;
	}
	else
	{ // ´Ó1001µãÆð£¬ÓÃÐÂµÄÆ«²îB
		if (CpuTimer0.InterruptCount == 1001)
		{
			B_sensor[3] = - fGyro_B[0] / 1000 * K_sensor[3];
			B_sensor[4] = - fGyro_B[1] / 1000 * K_sensor[4];
			B_sensor[5] = - fGyro_B[2] / 1000 * K_sensor[5];
		}
	}
	// ¶ÁÈ¡´ÅÇ¿¼ÆÊý¾Ý
	cnt = ScibRegs.SCIFFRX.bit.RXFFST;
	for (i=0; i<cnt; i++)
		cX = ScibRegs.SCIRXBUF.all;
	// ÏÂÃæÎªÓÐÐ§´úÂë¡£ÉÏÃæÖ»¶Á²»ÓÃ
	//	data8[i+18] = ScibRegs.SCIRXBUF.all;
	// sensor_raw.data.magx = (int16)(data8[18]<<8 | data8[19]);
	// sensor_raw.data.magy = (int16)(data8[20]<<8 | data8[21]);
	// sensor_raw.data.magz = (int16)(data8[22]<<8 | data8[23]);
	
	// Ô­Ê¼Êý¾Ý±ä»»
	for (i=0; i<12; i++)
		sensor.datas[i] = sensor_raw.datas[i] * K_sensor[i] + B_sensor[i];
	// ·½Ïò¶ÔÆë£¬ÒÑÍ³Ò»µ½ÉÏÃæµÄK B²ÎÊý
	// sensor.data.accxa = -sensor.data.accxa;
	// sensor.data.accya = -sensor.data.accya;
	// sensor.data.accza = -sensor.data.accza;
	// sensor.data.accy = -sensor.data.accy;
	// sensor.data.accz = -sensor.data.accz;
	// ÂË²¨
	Filter_CF(sensor.datas, att);
	data8[24] = (int16)(att[0]*18000/PI) >> 8;
	data8[25] = (int16)(att[0]*18000/PI) & 0xff;
	data8[26] = (int16)(att[1]*18000/PI) >> 8;
	data8[27] = (int16)(att[1]*18000/PI) & 0xff;
	data8[28] = (int16)(att[2]*18000/PI) >> 8;
	data8[29] = (int16)(att[2]*18000/PI) & 0xff;

	// »ñÈ¡²¢Ê¹ÓÃÒ£¿ØÆ÷ÐÅÏ¢
	SetChannel(3, 1.0L * ECap3Regs.CAP2 / ECap3Regs.CAP3);
	SetChannel(4, 1.0L * ECap4Regs.CAP2 / ECap4Regs.CAP3);
	GetChannel(3, data16+0);
	data16[0] = (data16[0] + 50)/100*100; // ÀëÉ¢»¯£¬±£Ö¤ÊýÖµÎÈ¶¨¡£²½³¤100£¨0.5N£©
	SetThro(*data16);
	data8[20] = (data16[0]) >> 8;
	data8[21] = data16[0] & 0xff;
	GetChannel(4, data16+0);
	SetRud(att_set+2, *data16);
//	data8[22] = (data16[0]) >> 8;
//	data8[23] = data16[0] & 0xff;

	// ¿ØÖÆÆ÷£¬¼°Æäµ÷ÊÔÊä³ö£¨Ê¹ÓÃdata[12] - [17]£¬ÃèÊöÈýÎ¬¿ØÖÆ²î·ÖÁ¿£©
	// 2sÖ®ÄÚÎªÍÓÂÝÒÇ±ê¶¨Ê±¼ä£¬µç»ú²»Êä³ö
	SetU1();
	GetYaw(att_set + 2);
	Controller(att, att_set, t_rotor, &(data8[12]));
	SetRotor1(CpuTimer0.InterruptCount <= 200 ? 150 : t_rotor[0]);
	SetRotor2(CpuTimer0.InterruptCount <= 200 ? 150 : t_rotor[1]);
	SetRotor3(CpuTimer0.InterruptCount <= 200 ? 150 : t_rotor[2]);
	SetRotor4(CpuTimer0.InterruptCount <= 200 ? 150 : t_rotor[3]);

	// ÏÈÊÕ£¬ÔÙ·¢£¬ÊÕÖ¸Áî£¨Ä¿Ç°¾ÍÊÇÉèÖÃ²ÎÊý£©
	// ËÄ×Ö½Ú£ºÖ¸Áî£¬²ÎÊý£¬²ÎÊý£¬Ð£Ñé
	cnt = ScicRegs.SCIFFRX.bit.RXFFST;
	data8[30] = 0x80;
	if (cnt == 6)
	{
		for(i=0; i<cnt; i++)
			xbCmd[i] = ScicRegs.SCIRXBUF.all;
		if (SetCtrlPara(xbCmd))
			data8[30] = 0x00;
	}
	else if(cnt > 6)
	{// Çå¿Õ£¬×÷·Ï
		for(i=0; i<cnt; i++)
			ScicRegs.SCIRXBUF.all;
	}
	// Ê±¼äÎ»£¨30 µÍÆßÎ»£©
	data8[30] |= t_used;

	// ·¢ËÍURM37Êý¾Ý
	data8[18] = urm_val >> 8;
	data8[19] = urm_val & 0xff;
	
	for(i=0; i<32; i++)
	{
		if(data8[i] == 0x80)
			data8[i] = 0x81;
	}
	data8[31] = 0x80;
	Scia_msg(data8, 32);
	if (CpuTimer0.InterruptCount > 5)	// ¸ø50msÊ±¼äÆô¶¯
		Scic_msg(data8, 32);

	// ½áÊøURM37 TRIG
	URM_TRIG = 1;

	// ¼ÆÊ±
	t_end = (double)(CpuTimer0.RegsAddr->TIM.all);

/*
	globalAddr[0] = data8[24];
	globalAddr[1] = data8[25];
	globalAddr[2] = data8[26];
	globalAddr[3] = data8[27];
	globalAddr[4] = data8[28];
	globalAddr[5] = data8[29];
	globalAddr += 6;
*/
}

void configtestled(void)
{
   EALLOW;
//   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0; // GPIO60 = GPIO60
//   GpioCtrlRegs.GPADIR.bit.GPIO1 = 1; 
//   GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0; // GPIO61 = GPIO61
//   GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
   EDIS;
}

//===========================================================================
// No more.
//===========================================================================

void AD_Config()
{
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
	AD_CS = 1;
	AD_OUT = 0;
	AD_CLK = 0;
	DELAY_US(10);
	AD_CS = 0;
	DELAY_US(10); //?
}

void Accl_Config()
{
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
	ACCL_CS = 1;
	ACCL_CLK = 1;
	ACCL_MOSI = 0;
}

void AD_Read(unsigned char addr, Uint16* data)	// send out address, get data
{
	unsigned char i;
	Uint16 output;
	AD_CS = 0;
	*data = 0;
	output = addr<<12 | 0x0D00;// 16-bit mode, MSB first, bipolar: 1101
	for(i = 0; i < 16; i++)
	{
		AD_OUT = output>>15;	// output
		output = output<<1;	
		DELAY_US(0.5);

		AD_CLK = 1;				// clock rises
		*data = (*data)<<1 | AD_IN;	// read a bit
		DELAY_US(0.5);
		AD_CLK = 0;				// clock falls
	}
	AD_CS = 1;

}

unsigned char Accl_Ex(unsigned char data)
{
	unsigned char i = 0;
	unsigned char ret = 0;
	for(i=0; i<8; i++)
	{
		ACCL_CLK = 0;
		ACCL_MOSI = data>>7;
		data = data << 1;
		DELAY_US(0.5);
		ACCL_CLK = 1;
		ret = (ret<<1) | ACCL_MISO;
		DELAY_US(0.5);
	}
	return ret;
}

void Accl_Init()
{
	ACCL_CS = 0;
	Accl_Ex(0x20);
	Accl_Ex(0xD7);
	ACCL_CS = 1;
	ACCL_CS = 0;
	Accl_Ex(0x21);
	Accl_Ex(0x65);	// 01100101
	ACCL_CS = 1;
//	Accl_Ex(0x22);
//	Accl_Ex(0x02);	// 00000010
}
	
void Accl_Get(unsigned char* data)	// XHXLYHYLZHZL
{
	ACCL_CS = 0;
	Accl_Ex(0xE8);
	data[0] = Accl_Ex(0);
	data[1] = Accl_Ex(0);
	data[2] = Accl_Ex(0);
	data[3] = Accl_Ex(0);
	data[4] = Accl_Ex(0);
	data[5] = Accl_Ex(0);
	ACCL_CS = 1;
}

void Scia_Config()
{
	EALLOW;
/* Enable internal pull-up for the selected pins */
	//GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;	   // Enable pull-up for GPIO36 (SCIRXDA)
	GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;	   // Enable pull-up for GPIO35 (SCITXDA)
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
	GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 3;  // Asynch input GPIO36 (SCIRXDA)
/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;   // Configure GPIO36 for GPIO operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 1;   // Configure GPIO35 for SCITXDA operation	
	GpioCtrlRegs.GPBDIR.bit.GPIO36 = 0;		// Input(RX)
	EDIS;
}

void Scib_Config()
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

void Scic_Config()
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

void Scia_Init()
{
/* SciA FIFO Init */
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x204f;
    SciaRegs.SCIFFCT.all=0x0;	

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
	      SciaRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
	      SciaRegs.SCILBAUD    =0x0028;
	#endif
	#if (CPU_FRQ_100MHZ)
      SciaRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 20MHz.
      SciaRegs.SCILBAUD    =0x0015;
	#endif
	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

}

void Scib_Init()
{
/* SciB FIFO Init */
    ScibRegs.SCIFFTX.all=0xE040;
    ScibRegs.SCIFFRX.all=0x204f;
    ScibRegs.SCIFFCT.all=0x0;	

/* SciB Init */
// Note: Clocks were turned on to the SCIB peripheral
// in the InitSysCtrl() function
 	ScibRegs.SCICCR.all =0x0087;   // 2 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA = 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
	      ScibRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
	      ScibRegs.SCILBAUD    =0x00E7;
	#endif
	#if (CPU_FRQ_100MHZ)
      ScibRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      ScibRegs.SCILBAUD    =0x0044;
	#endif
	ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

}

void Scic_Init()
{
/* SciC FIFO Init */
    ScicRegs.SCIFFTX.all=0xE040;
    ScicRegs.SCIFFRX.all=0x204f;
    ScicRegs.SCIFFCT.all=0x0;

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
	      ScicRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
	      ScicRegs.SCILBAUD    =0x0028;
	#endif
	#if (CPU_FRQ_100MHZ)
      ScicRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 20MHz.
      ScicRegs.SCILBAUD    =0x0015;
	#endif
	ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

/* Reset Cancelled*/
	GpioDataRegs.GPBDAT.bit.GPIO58 = 1;
}

void Scia_Xmit(unsigned char c)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST > 14) {}// Æñ²»Ê§È¥ÁËFIFOµÄ×÷ÓÃ£¿
    SciaRegs.SCITXBUF=c;
}	

void Scib_Xmit(unsigned char c)
{
    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}// Æñ²»Ê§È¥ÁËFIFOµÄ×÷ÓÃ£¿
    ScibRegs.SCITXBUF=c;
}	

void Scic_Xmit(unsigned char c)
{
    while (ScicRegs.SCIFFTX.bit.TXFFST > 14) {}// Æñ²»Ê§È¥ÁËFIFOµÄ×÷ÓÃ£¿
    ScicRegs.SCITXBUF=c;
}	
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
void PWM_Config()
{
	EALLOW;
	/* Configure as EPWM */
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;

	EDIS;
}

void PWM_Init()
{
//	EPwm1Regs.TBPRD = 1200;			// Period = 1201 TBCLK counts


	EPwm1Regs.TBPRD = 0;			// Phase = 0
	EPwm1Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm1Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm1Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm1Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm1Regs.TBPRD = 30000;
	EPwm1Regs.CMPB = 150*15;

	EPwm2Regs.TBPRD = 0;			// Phase = 0
	EPwm2Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm2Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm2Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm2Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm2Regs.TBPRD = 30000;
	EPwm2Regs.CMPB = 150*15;

	EPwm3Regs.TBPRD = 0;			// Phase = 0
	EPwm3Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm3Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm3Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm3Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm3Regs.TBPRD = 30000;
	EPwm3Regs.CMPB = 150*15;

	EPwm4Regs.TBPRD = 0;			// Phase = 0
	EPwm4Regs.TBCTL.all = 0xE030 | 0<<10 | 5<<7 ;	// Count up; Phase loading disabled; Shadow ...
	// 2^x = CLKDIV; 2*y = ; TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
	EPwm4Regs.CMPCTL.all = 0x0000;	// B-Shadow; A-Shadow; LoadBMode-Zero; LoadAMode-Zero
	EPwm4Regs.AQCTLB.bit.CBU = 1;	// reset when go up to CMPB
	EPwm4Regs.AQCTLB.bit.ZRO = 2;	// set when zero
	EPwm4Regs.TBPRD = 30000;
	EPwm4Regs.CMPB = 150*15;
}

void init_zone6(void)
{

    // Make sure the XINTF clock is enabled
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;
	// Configure the GPIO for XINTF with a 16-bit data bus
	// This function is in DSP2833x_Xintf.c
	InitXintf16Gpio();
    EALLOW;
    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = SYSCLKOUT
    XintfRegs.XINTCNF2.bit.XTIMCLK = 0;
    // Buffer up to 3 writes
    XintfRegs.XINTCNF2.bit.WRBUFF = 3;
    // XCLKOUT is disabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 1;
    // XCLKOUT = XTIMCLK
    XintfRegs.XINTCNF2.bit.CLKMODE = 0;

    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING6.bit.XWRLEAD = 1;
    XintfRegs.XTIMING6.bit.XWRACTIVE = 2;
    XintfRegs.XTIMING6.bit.XWRTRAIL = 1;
    // Zone read timing
    XintfRegs.XTIMING6.bit.XRDLEAD = 1;
    XintfRegs.XTIMING6.bit.XRDACTIVE = 3;
    XintfRegs.XTIMING6.bit.XRDTRAIL = 0;

    // don't double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING6.bit.X2TIMING = 0;

    // Zone will not sample XREADY signal
    XintfRegs.XTIMING6.bit.USEREADY = 0;
    XintfRegs.XTIMING6.bit.READYMODE = 0;

    // 1,1 = x16 data bus
    // 0,1 = x32 data bus
    // other values are reserved
    XintfRegs.XTIMING6.bit.XSIZE = 3;
    EDIS;

   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.
   asm(" RPT #7 || NOP");
}

void ECap_Config()
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

}


Uint16 WriteData(unsigned char	*Wdata, unsigned char	DataAddress, Uint16	number)
{
   Uint16 i;
   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
      return I2C_BUS_BUSY_ERROR;
   }
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = 0x70;
   I2caRegs.I2CCNT = number + 1;
   I2caRegs.I2CDXR = DataAddress; //Send high byte of RomAddress
   I2caRegs.I2CMDR.all = 0x6E20;
      while(!I2C_xrdy());
   for (i=0; i<number; i++)
   {
      while(!I2C_xrdy());
      I2caRegs.I2CDXR = *Wdata;
      Wdata++;
	  if (I2caRegs.I2CSTR.bit.NACK == 1)
   		  return	I2C_BUS_BUSY_ERROR;
   }   	
   return I2C_SUCCESS;   
}

Uint16 ReadData(unsigned char *RamAddr, unsigned char	DataAddress, Uint16 number)
{
   Uint16  i,Temp;

   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
       return I2C_BUS_BUSY_ERROR;
   }
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = 0x70;
   I2caRegs.I2CCNT = 1;
   I2caRegs.I2CDXR = DataAddress; //Send high byte of RomAddress
   I2caRegs.I2CMDR.all = 0x6620; 
      while(!I2C_xrdy());
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   DELAY_US(50);		
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = 0x70;
   I2caRegs.I2CCNT = number;	 
   I2caRegs.I2CMDR.all = 0x6C20; 
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   for(i=0;i<number;i++)
   {
      while(!I2C_rrdy());
   	  Temp = I2caRegs.I2CDRR;
	  if (I2caRegs.I2CSTR.bit.NACK == 1)
   		  return	I2C_BUS_BUSY_ERROR;
   	  *RamAddr = Temp;
   	  RamAddr++;
   }
   return I2C_SUCCESS;
}


Uint16	I2C_xrdy()
{
	Uint16	t;
	t = I2caRegs.I2CSTR.bit.XRDY;
	return t;
}

Uint16	I2C_rrdy()
{
	Uint16	t;
	t = I2caRegs.I2CSTR.bit.RRDY;
	return t;
}

void URM_Config()
{
	EALLOW;

	// GPIO0 - output to TRIG
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;	
	// ECAP5 - input from URM-PWM
	// ÔÚECAP_ConfigÖÐÒÑÓÐ

	EDIS;
}
void URM_Init()
{
	URM_TRIG = 1;	// Æ½Ê±¸ßµçÆ½

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

interrupt void URM_ISR()
{
	if(ECap5Regs.ECFLG.bit.CEVT1)
	{
		urm_val = (unsigned int)(0.5+ECap5Regs.CAP1/750);
		ECap5Regs.ECCLR.bit.CEVT1 = 1;
	}
	if(ECap5Regs.ECFLG.bit.CEVT3)
	{
		urm_val = (unsigned int)(0.5+ECap5Regs.CAP3/750);
		ECap5Regs.ECCLR.bit.CEVT3 = 1;
	}
	ECap5Regs.ECCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
