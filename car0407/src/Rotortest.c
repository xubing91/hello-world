#include "para.h"
#include "Fml.h"
#include "filter.h"
#include "trajectory.h"
#include "formation.h"
#include "decentralized.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "MS5803-01BA.h"
#include "chirp.h"
interrupt void ISRTimer0(void);
#define NULL 0
#define FLY_WITH_GPS

const Uint16	addr_server = ADDRESS_SERVER;
const Uint16	addr_leader = ADDRESS_LEADER;
const Uint16	addr_follower = ADDRESS_FOLLOWER;
Msg	msg;
Cmd cmdR;
Cmd cmdS;
Msg* msgR;
unsigned char bInitializing = 1;
Uint32 sectorSD = 0;	// start writing from sector
void	PieVectTableRegister();	// remap the interrupts used to the ISR functions
void	InterruptsEnable();		// Enable all interrupts used in this program
void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example
   // InitXintf16Gpio();	//zq

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
   PieVectTableRegister();	
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in DSP2833x_CpuTimers.c
   InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 150MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 135, CYCLE); // 500Hz
   //ConfigCpuTimer(&CpuTimer1, 150, 1000000);
   //ConfigCpuTimer(&CpuTimer2, 150, 1000000);
	StartCpuTimer0();

// Enable all interrupts used in this program
	InterruptsEnable();
// Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

	Hardware_Init();
	bInitializing = 0;
    for(; ;)
    {   

	}
}




// 将加速度计、陀螺仪原始数据转为m/s^2，deg/s
int16 	acc_raw[3] = {0, 0, 0};		// raw data of accelorater
int16 	gyro_raw[3] = {0, 0, 0};	// raw data of gyroscope
int16	cmps_raw[3] = {0, 0, 0};	// raw data of compass
double	mag[3] = {10, 0, 0};		// magnitude ( -bias) from compass
double	yaw = 0;					// data of compass (rad)
double	acc[3] = {0, 0, -GRAVITY};	// data of accelorater (m/s^2)
double	acc_f[3] = {0, 0, -GRAVITY};	// data of accelorater (m/s^2) - lowpass filtered
double	gyro[3] = {0, 0, 0};		// data of gyroscope (rad/s)
double	att_a[3] = {0, 0, 0};// attitude from acc&cmps
double	att[3] = {0, 0, 0}; 		// attitude
double	att_set[3] =  {0, 0, 0};	// set-value of  attitude
double	posGPS[3] = {0, 0, 0};			// position NEU (m)
double	vGPS[3] = {0, 0, 0};		// GPS speed NEU (m/s)
double	pos[3] = {0, 0, 0};			// position NEU (m/s) [may be filtered]
double	vel[3] = {0, 0, 0};			// velocity NEU (m/s) [may be filtered]
double	height_US = 0.05;			// height by Ultrasonic Sensor (m)
double	height = 0.05;				// height (m)
double	pos_set[3] = {0, 0, 0};		// set-value of position -- NED(m)
double	pos_zero[3] = {0, 0, 0};	// zero of position in GPS's view -- NED(m)
double	spd_zero[3] = {0, 0, 0};	// reference of speed -- NED(m)
double	spd_set[3] = {0, 0, 0};		// set-value of speed -- NED(m/s)
double	acc_set[3] = {0, 0, 0};		// set-value of acceleration -- NED(m/s^2)
double	rcctrl[6] = {1.5, 1.5, 1, 1.5, 1.5, 1.5};	// radio controller pulse-width (ms)
double	output[4] = {0, 0, 0, 0};	// 输出（净推力和三个差分输出） (N, Nm, Nm, Nm)
int16	nsvGPS = 0;					// NSV of GPS
double	trackGPS = 0;					// GPS 航线
double	gspeedGPS = 0;					// GPS 地速
long double	timeGPS = 0;			// Time output of GPS
double	pwm[4] = {ROTOR_INIT, ROTOR_INIT, ROTOR_INIT, ROTOR_INIT};	// PWM for rotors (us)
int16	pwm_i[4] = {ROTOR_INIT, ROTOR_INIT, ROTOR_INIT, ROTOR_INIT};	// PWM for rotors (us)
unsigned char bAutoMode = 0;		// is Auto Mode?
unsigned char b2AutoMode = 0;		// just switch to Auto Mode?
unsigned char bGPSnormal = 0;		// is GPS working normally?
//Agent	aNeighbors = AGENT_INIT(ADDRESS_NEIGHBOR);	// Neighbor Agent; will be extended to more agents
Agent	aSelf = AGENT_INIT(ZIGBEE_ADDRESS);			// Self Agent
// Design about bGPS normal:
// -WRITE:
// MANUAL->AUTO && nsv>0 --> GPS is normal
// nsv = 0 				 --> GPS is abnormal 
// -READ:
// GPS is normal: auto positioning in auto mode; send out position information
// //
char*	wrlStr = 0;
int		ack = 0;					// acknowledgement for wireless
extern double	RC_width[6];
#ifdef	SENSOR_GPS_GPS1000
extern struct GPHPP GPS;
#endif
#ifdef	SENSOR_GPS_UBLOX
extern struct PUBX GPS;
#endif
int16	n_poscycle = 1;	// 0~49，收到GPS定位信息在50个周期（0.1s）中的哪一个；该周期发编队指令，25周期后发飞机信息
int16	swarmflag = 0;	// [14bits][sent][received]

interrupt void ISRTimer0(void)
{
   // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
    CpuTimer0Regs.TCR.bit.TIF=1;
    CpuTimer0Regs.TCR.bit.TRB=1;
		
	if(bInitializing == 1)	// must finish initialization
		return;

    CpuTimer0.InterruptCount++;

	if(URM37_TRIG)	// reset trigger signal
		URM37_TRIG = 0;
	if(msgR = IsMsgReady())
	{
		GetCmd(msgR, &cmdR);
		switch(GetCmdType(&cmdR))
		{
		case CMD_SETVAR_TYPE:
			SetVar(&cmdR, pwm);
			URM37_TRIG = 1;
			ack |= 0x0001;
			break;
		case CMD_CHIRP_TYPE:
			SetChirp(&cmdR);
			URM37_TRIG = 1;
			ack |= 0x0001;
			break;
		default:
			break;
		}
		MsgReset(msgR);
	}
	ChirpOut(&pwm[3]);
	SetRotor(pwm);
	pwm_i[0] = (int16)(pwm[0] + 0.5);
	pwm_i[1] = (int16)(pwm[1] + 0.5);
	pwm_i[2] = (int16)(pwm[2] + 0.5);
	pwm_i[3] = (int16)(pwm[3] + 0.5);

	// data transmitting
	if(CpuTimer0.InterruptCount % 100 ==25 && IsSendOK())
	{
		MsgReset(&msg);
		MsgSetDest(&msg, &addr_server);
		MsgSetSrc(&msg);
		MsgAppend_rad(&msg, att, 3);
		MsgAppend_float(&msg, pos_set, 3);
		// MsgAppend_int(cmps_raw, 3);
		MsgAppend_rad(&msg, att_set, 3);
		MsgAppend_int(&msg, pwm_i, 4);
		MsgAppend_float(&msg, posGPS, 2);	// horizontal position
		MsgAppend_float(&msg, &height, 1);
		MsgAppend_float(&msg, vGPS, 2);	// horizontal speed
		MsgAppend_int(&msg, &nsvGPS, 1);
		MsgAppend_float(&msg, pos, 3);
		MsgAppend_float(&msg, vel, 3);
		// MsgAppend_float(rcctrl, 6);	// added temporarily
		MsgAppend_int(&msg, &ack, 1);	ack &= 0xFFFE;
// output compass

		MsgSend(&msg);
	}
}

void	PieVectTableRegister()
{
   PieVectTable.TINT0 = &ISRTimer0;
   PieVectTable.SCITXINTC = &ISRSciTxCFifo;
   PieVectTable.SCIRXINTA = &ISRSciRxAFifo;
   PieVectTable.ECAP1_INT = &RC1_ISR;
   PieVectTable.ECAP2_INT = &RC2_ISR;
   PieVectTable.ECAP3_INT = &RC3_ISR;
   PieVectTable.ECAP4_INT = &RC4_ISR;
   PieVectTable.ECAP6_INT = &RC6_ISR;
   PieVectTable.ECAP5_INT = &USonic_ISR;
   PieVectTable.SCIRXINTC = &ISRSciRxCFifo;
	//PieVectTable.XINT13 = &cpu_timer1_isr;
   //PieVectTable.TINT2 = &cpu_timer2_isr;
}

void	InterruptsEnable()
{
// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
// which is connected to CPU-Timer 1, and CPU int 14, which is connected
// to CPU-Timer 2:
    IER |= M_INT1;
   //IER |= M_INT13;
   //IER |= M_INT14;

// Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	
	// Enable ECap5 Int
	// IER |= M_INT4;
	// PieCtrlRegs.PIEIER4.bit.INTx5 = 1;

// Enable SCITXINTC in the PIE: Group 8 interrupt 6
	IER |= M_INT8;
	PieCtrlRegs.PIEIER8.bit.INTx6 = 1;

// Enable SCIRXINTC in the PIE: Group 9 interrupt 1
	IER |= M_INT9;
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;

// Enable SCIRXINTC in the PIE: Group 8.5
	IER |= M_INT8;
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;

// Enalbe ECAPx in the PIE: Group 4 interrupt x
	IER |= M_INT4;
	PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
	PieCtrlRegs.PIEIER4.bit.INTx2 = 1;
	PieCtrlRegs.PIEIER4.bit.INTx3 = 1;
	PieCtrlRegs.PIEIER4.bit.INTx4 = 1;
	PieCtrlRegs.PIEIER4.bit.INTx5 = 1;
	PieCtrlRegs.PIEIER4.bit.INTx6 = 1;
}
//===========================================================================
// No more.
//===========================================================================



	


