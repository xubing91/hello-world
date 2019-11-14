#include "MS5803-01BA.h"
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

typedef unsigned long long Uint64;
typedef long long int64;
/* calibration data from PROM */
Uint16 SENS_T1 = 40127;	// Pressure sensitivity
Uint16 OFF_T1 = 36924;	// Pressure offset
Uint16 TCS = 23317;		// Temperature coefficient of pressure sensitivity
Uint16 TCO = 23282;		// Temperature coefficient of pressure offset
Uint16 T_REF = 33464;	// Reference temperature
Uint16 TEMPSENS = 28312;// Temperature coefficient of the temperature
int32 dT = 0;			// Difference between actural and reference temperature
int32 TEMP = 0;			// Actural temperature(-40...85degC with 0.01degC resolution)
int64 OFF = 0;			// Offset at actual temperature
int64 SENS = 0;			// Sensitivity at actual temperature

int32 D1 = 0;			// Digital pressure value
int32	D2 = 0;			// Digital temperature value
/*
	Reset the barometer
*/
unsigned char bMS5803_On = 0;
void	MS5803_Reset()
{
	/*8b in*/
	MS5803_CS = 0;
	MS5803_WR(0x1E);	// reset command
	DELAY_US(3000);		// 2.8ms
	bMS5803_On = MS5803_MISO;	// 0 while initializing, 1 when done
	MS5803_CS = 1;

}
void	MS5803_Init()
{
	MS5803_Reset();		// reset to initialize
	MS5803_GetPROM();	// get constants
}

/*
	Read PROM (128 bit of calibration words)
*/
unsigned char cHigh, cLow;
void	MS5803_GetPROM()
{
	/*8b in, 16b out*/
	SENS_T1 = MS5803_WR16(0xA2);	// PROM Read, Ad = 1 -> Read C1
	OFF_T1 = MS5803_WR16(0xA4);	// PROM Read, Ad = 2 -> Read C2
	TCS = MS5803_WR16(0xA6);	// PROM Read, Ad = 3 -> Read C3
	TCO = MS5803_WR16(0xA8);	// PROM Read, Ad = 4 -> Read C4
	T_REF = MS5803_WR16(0xAA);	// PROM Read, Ad = 5 -> Read C5
	TEMPSENS = MS5803_WR16(0xAC);	// PROM Read, Ad = 6 -> Read C6
}

/*
	Convert D1 -> for pressure, 32 bit
	OSR:4096, max converting time: 9.04 ms, Resolution RMS: 0.012 mbar
*/
unsigned char bPrs = 0;	// pressure available
unsigned char bTemp = 0;// temperature available
void	MS5803_ConvertPrs()
{
	MS5803_WR0(0x48);
	bPrs = 1;
}

/* 
	Convert D2 -> for temperature, 32 bit
*/
void	MS5803_ConvertTemp()
{
	MS5803_WR0(0x58);
	bTemp = 1;
}

/*
	Read D1
	! Must be used after MS5803_ConvertPrs()
*/
void	MS5803_ReadPrs()
{
	if(bPrs == 1)
		D1 = (int32)MS5803_WR24(0x00);
	bPrs = 0;
}

/*
	Read D2
	! Must be used after MS5803_ConvertTemp()
*/
void	MS5803_ReadTemp()
{
	if(bTemp == 1)
		D2 = (int32)MS5803_WR24(0x00);
	bTemp = 0;
}	

/*
	Calculate pressure using raw data, with 2-order temperature compensation
*/
int32	MS5803_CalcPres()
{// based on MS5803-01BA.pdf, pp. 7-8
	int32 P = 0;
	int32 T2 = 0;
	int64 SENS2 = 0;
	int64 OFF2 = 0;

	/* Calculate temperature */
	dT = D2 - (((int32)T_REF)<<8);
	TEMP = (int32)(2000 + (((int64)dT * (int64)TEMPSENS)>>23));
	/* Calculate temperature compensated OFF and SENS -> for pressure */
	OFF = (((int64)OFF_T1) << 16) + (((int64)TCO * (int64)dT) >> 7);
	SENS = (((int64)SENS_T1) << 15) + (((int64)TCS * (int64)dT) >> 8);
	/* Second order temperature compensation (for TEMP, OFF, SENS)*/
	if(TEMP < 2000)
	{
		T2 = (int32)(((int64)dT * (int64)dT) >> 31);
		OFF2 = 3 * (int64)(TEMP - 2000) * (int64)(TEMP - 2000);
		SENS2 = (7 * (int64)(TEMP - 2000) * (int64)(TEMP - 2000)) >> 3;
		if(TEMP < -1500)
		{
			SENS2 = SENS2 + 2 * (TEMP + 1500) * (TEMP + 1500);
		}
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
		if(TEMP >= 4500)
		{
			SENS2 = SENS2 - ((((int64)TEMP - 4500)*((int64)TEMP - 4500)) >> 3);
		}
	}
	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
	/* Calculate temperature compensated pressure */
	P = (int32)((((D1 * SENS) >> 21) - OFF) >> 15);
	return P;// Pa
}
int32 MS5803_GetTemp()
{
	return TEMP;
}

unsigned char iMS5803;
unsigned char retMS5803;
unsigned char MS5803_WR(unsigned char command)
{
	retMS5803 = 0;
	for(iMS5803 = 0; iMS5803<8; iMS5803++)
	{// 20MHz
		MS5803_CLK = 0;
		DELAY_US(0.25);
		MS5803_MOSI = command >> 7;
		DELAY_US(0.25);
		command <<= 1;
		MS5803_CLK = 1;
		retMS5803 = (retMS5803<<1) | MS5803_MISO;
		DELAY_US(0.25);
		// the codes takes about 0.25 us
	}
	return retMS5803;
}

unsigned char MS5803_WR0(unsigned char command)
{
	unsigned char iRet = 0;
	MS5803_CS = 0;
	iRet = MS5803_WR(command);
	MS5803_CS = 1;
	DELAY_US(0.25);
	return iRet;

}

Uint16 MS5803_WR16(unsigned char command)
{
	Uint16 iRet = 0;
	MS5803_CS = 0;
	MS5803_WR(command);
	iRet = MS5803_WR(0) & 0xFF;
	iRet <<= 8;
	iRet = iRet | (MS5803_WR(0) & 0xFF);
	MS5803_CS = 1;
	DELAY_US(0.25);
	return iRet;
}

Uint32 MS5803_WR24(unsigned char command)
{
	Uint32 iRet = 0;
	MS5803_CS = 0;
	MS5803_WR(command);
	iRet = MS5803_WR(0) & 0xFF;
	iRet = (iRet<<8) | (MS5803_WR(0) & 0xFF);
	iRet = (iRet<<8) | (MS5803_WR(0) & 0xFF);
	MS5803_CS = 1;
	DELAY_US(0.25);
	return iRet;
}
