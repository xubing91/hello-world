#include "para.h"
#include "Fml.h"
#include "filter.h"
#include "trajectory.h"
#include "formation.h"
#include "decentralized.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "flightFSM.h"
#include "mathex.h"
#include "math.h"
interrupt void ISRTimer0(void);
#define NULL 0
#define r 0.03// the radius of the wheel
#define kw 0.025//
const double w_sat = 54;//
double w_wheel=0;

#define FLY_WITH_GPS

// about data transmitting
#define FLIGHTCONTROL
//#define	BOARDTEST
//#define FORMATIONFLGIHT

// send formation data - for debugging use
#define FORMATION_SEND

int16	com_period_formation = 20;	//100;	// 5 Hz	// < 50 to be invalidate
int16	com_period_report = 400;	// 1 Hz // < 50 to be invalidate

#if defined(XBEE)
const Uint32    addr_server[2] = ADDRESS_SERVER;
#else
const Uint16	addr_server = ADDRESS_SERVER;
#endif

Msg	msg;
Cmd cmdR;
Cmd cmdS;
Msg* msgR;
ScdFlight scdFlight;
unsigned char bInitializing = 1;
Uint32 sectorSD = 0;	// start writing from sector
void	PieVectTableRegister();	// remap the interrupts used to the ISR functions
void	InterruptsEnable();		// Enable all interrupts used in this program
void main(void)
{
   InitSysCtrl();
   DINT;
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();

   EALLOW;  // This is needed to write to EALL
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();

   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTableRegister();	
   EDIS;    // This is needed to disable write to EALLOW protected registers

   InitCpuTimers();   // For this example, only initialize the Cpu Timers


   ConfigCpuTimer(&CpuTimer0, 135, CYCLE); // 500Hz 400hz

	StartCpuTimer0();

// Enable all interrupts used in this program
	InterruptsEnable();
// Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

	Hardware_Init();
	ComTimerReset();
	SD_SetStartSector(&sectorSD);
	bInitializing = 0;

	FlightScdDesign(&scdFlight);

    for(; ;)
    {   
		WriteNextBlock(sectorSD);
		sectorSD ++;

	}
}

// 将加速度计、陀螺仪原始数据转为m/s^2，deg/s
double w1=0;
double w2=0;
double w3=0;
double w4=0;
double t0=0;        //发送指令后时间起始值
int16 	acc_raw[3] = {0, 0, 0};		// raw data of accelorater
int16 	gyro_raw[3] = {0, 0, 0};	// raw data of gyroscope
int16	cmps_raw[3] = {0, 0, 0};	// raw data of compass
int32	pres = 101000;				// pressure (Pa)
int16	pres_biased = 1000;		// pressure with [-100000 Pa] bias
double	tempa = 2000;				// temperature (0.01degC from barometer)
double	mag[3] = {10, 0, 0};		// magnitude ( -bias) from compass
double	yaw = 0;					// data of compass (rad)
double	acc[3] = {0, 0, -GRAVITY};	// data of accelorater (m/s^2)
double	acc_f[3] = {0, 0, -GRAVITY};	// data of accelorater (m/s^2) - lowpass filtered
double	acc_bodyframe[3] = {0, 0, 0};		//data of acc in body fame
double	gyro[3] = {0, 0, 0};		// data of gyroscope (rad/s)
double	att_a[3] = {0, 0, 0};// attitude from acc&cmps
double	att[3] = {0, 0, 0}; 		// attitude
double	att2[3] = {0, 0, 0}; 		// attitude-角度
double  att_zero =0;                //目前的想法是作为起始偏航角[UWB]
double  att_error=0;                //目前是作为偏航角与起始偏航角之差
double	att_set[3] =  {0, 0, 0};	// set-value of  attitude
double	posGPS[3] = {0, 0, 0};			// position NEU (m)
unsigned char bGPSUpdate = 0;		// whether GPS updated in this cycle
extern unsigned char bUWBUpdate = 0;
double	vGPS_ori[3] = {0, 0, 0};		// GPS speed NEU (m/s)
double	vGPS_dif[3] = {0, 0, 0};
double	vGPS[3] = {0, 0, 0};		// GPS speed NEU (m/s)
double	pos[3] = {0, 0, 0};			    // position NEU (m) [may be filtered]
double	pos2[3] = {0, 0, 0};			// position NEU (m) [may be filtered]
double	pos3[3] = {0, 0, 0};			// position NEU (m) [may be filtered]
double	pos4[3] = {0, 0, 0};			// position NEU (m) [may be filtered]

double	uwbpos[3] = {0, 0, 0};
double  UWB_angle = 0;
double	pos_r[3] = {0, 0, 0};		// position relative to O (m): pos_r = pos - p_c
double	p_c[3] = {0, 0, 0};			// center position (m)
double	vel[3] = {0, 0, 0};			// velocity NEU (m/s) [may be filtered]



double	height = 0;				// height (m)


double	vHeight = 0;				// velocity of height (m)
double	pos_set[3] = {0, 0, 0};		// set-value of position -- NED(m)
double	pos_zero[3] = {0, 0, 0};	// zero of position in GPS's view -- NED(m)

double	spd_set[3] = {0, 0, 0};		// set-value of speed -- NED(m/s)
double	uwbvel[3] = {0, 0, 0};

double	spd_sett_[3] = {0, 0, 0};		// set-value of speed of t_-- NED(m/s)
double  v_ctrl[3]={0, 0, 0};        //电机控制速度量
double	acc_set[3] = {0, 0, 0};		// set-value of acceleration -- NED(m/s^2)
double	rcctrl[6] = {1.5, 1.5, 1, 1.5, 1.5, 1.5};	// radio controller pulse-width (ms)
double	output[4] = {0, 0, 0, 0};	// 输出（净推力和三个差分输出） (N, Nm, Nm, Nm)
int16	nsvGPS = 0;					// NSV of GPS

long double	timeGPS = 0;			// Time output of GPS
double	pwm[4] = {0, 0, 0, 0};	// PWM for rotors (us)
int16	pwm_i[4] = {ROTOR_INIT, ROTOR_INIT, ROTOR_INIT, ROTOR_INIT};	// PWM for rotors (us)
unsigned char bAutoMode = 0;		// is Auto Mode?
unsigned char bGPSnormal = 0;		// is GPS working normally?
struct Fsm	fsm = FSM_DEFAULT;			// finite state machine for flight mode switching
struct State{
	unsigned int bSD:1;	// SD Card available
	unsigned int bSwTx:1;	// Swarm - Transmitting
	unsigned int bSwRx:1;	// Swarm - Receiving
};
union Status{
	int				all;
	struct State 	bit;
}bStatus = {0};	// working status
extern unsigned char bSDExist;
const double v_zero3[3] = {0, 0, 0};

Agent	aRef = AGENT_INIT(ZIGBEE_ADDRESS);

Agent	aNeighbors[N_SWARM] = {AGENT_INIT(ADDRESS_INVALID)};	// Neighbor Agent !: a need for extension, shouldn't list all addresses here
Agent	aSelf = AGENT_INIT(ZIGBEE_ADDRESS);			// Self Agent
double	w[N_SWARM] = {0};	// weights of neighbors; initialized as 0
unsigned char i_w = 0;
double 	w_all = 0;		// sum of w
unsigned char iNeighbors = 0;		// indicate which neighbor of ADDRESS_NEIGHBORS I send error of formation to


int		ack = 0;					// acknowledgement for wireless
extern double	RC_width[6];

#ifdef	SENSOR_GPS_UBLOX
extern struct PUBX GPS;
#endif

int16	swarmflag = 0;	// [14bits][sent][received]
extern unsigned char	bShutdown;	// extern; no good
extern unsigned char	bThroOff;	// extern; no good

extern Traj traj_ab;
extern Traj traj_ref;
//extern Traj traj_ref;
int timer_count = 0;	// timer count, 500Hz - 0~499, set to 0 when GPSTime % 1 == 0


int16   HMC_raw[3]={0,0,0};



double tGPS_1=0;//上一个时刻
double tGPS_0=0;//当前时刻

int FLAG_HeightFliter=1;//表示高度滤波的不同方式

extern double intg_height;
extern double basicOut;
//以下测试在走航迹的过程中飞机的航向角需要转动后对准航线
int	YawSpinRun=0;//1表示没有对准，飞机不执行正常原来的航迹
				//0表示飞机开始执行直线航迹
int FlagYawRun = 1;//如果是表示飞机执行航迹的时候需要转动
					//否则是按照原来的不用转动

double RCpos_set[2] = {0,0};//X,Y pos Get from RC
double RCspd_set[2] = {0,0};
double RCacc_set[2] = {0,0};
extern double homepoint[3] ;//将homepoint传输下来看数据对否

double ure[3]={0,0,0};//用于记录ux与uy,滤波之前
extern double offsetvx;
double time_zero = 0;

double n[3] = {0,0,0};
int16 sendType = 0;
extern double Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Ki_x, Ki_y, Ki_z;
interrupt void ISRTimer0(void)
{
   // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
    CpuTimer0Regs.TCR.bit.TIF=1;
    CpuTimer0Regs.TCR.bit.TRB=1;
		
	if(bInitializing == 1)	// must finish initialization
		return;

    CpuTimer0.InterruptCount++;
    EINT;

	if(GPSPacketReady())	// 10 Hz
	{
		//GetGPSSpeed(vGPS_ori);
		//tGPS_0=CpuTimer0.InterruptCount;
		bGPSUpdate = 1;
		GPSPacketRead();
		GetGPSPosition(posGPS);
		GetGPSSpeed(vGPS_ori);

		if((tGPS_0-tGPS_1)>0)
		{
		GetGPSSpeed_V4(vGPS_dif,posGPS,(tGPS_0-tGPS_1)*DTIME);
		}
		tGPS_1=tGPS_0;

		GetGPSNSV(&nsvGPS);
		GetGPSTime(&timeGPS);


	}


   	sendType = 100;
   	FlightScdRun(&scdFlight);
	msgR = IsMsgReady();
	if((msgR) || scdFlight.isCmd)
	{
		if(msgR)
			GetCmd(msgR, &cmdR);
		else	// no msgR && isScheduleCmd
			FlightScdCmd(&scdFlight, &cmdR);
		
		switch(GetCmdType(&cmdR))
		{
		case CMD_SETSCHEDULE_TYPE:
			ack |= 0x0001;
			scdFlight.isOn = 1;
			scdFlight.iSchedule = -1;
			scdFlight.t = 0;
			//FlightScdOn(&scdFlight, &cmdR);
			break;
		case CMD_DESCENDDEBUG_TYPE:
			
			
			if(cmdR.code==CMD_GPSDEBUG_ON)
			{
			FlagYawRun = 0;//飞机不执行转动偏航值
			}
			if(cmdR.code==CMD_GPSDEBUG_OFF)
			{
			FlagYawRun = 1;//飞机执行转动偏航值

			}
	

			swarmflag |= 0100;//
			ack |= 0x0001;
			break;
		case CMD_SETPARA_TYPE:
			SetCtrlPara(&cmdR);
			ack |= 0x0001;
			break;
		case CMD_SETDEST_TYPE:
			SetDestPos(&cmdR);
			ack |= 0x0001;
			break;

		case CMD_SETREF_TYPE:
			//SetRefDestPos(&cmdR);
			ack |= 0x0001;
			break;

		case CMD_SETSTATE_TYPE:
			switch(cmdR.code)
			{
				case CMD_SETSTATE_TAKEOFF:
					fsm.cmd = CMD_SETSTATE_TAKEOFF;
					break;
				case CMD_SETSTATE_LAND:
					fsm.cmd = CMD_SETSTATE_LAND;
					break;
				case CMD_SETSTATE_HALT:
					SetDestPos(&cmdR);	
					break;
				default:
					break;
			}
			ack |= 0x0001;
			break;
		case CMD_FORMATION_TYPE:
			//SetFormation(&cmdR);
			swarmflag |= 0x0001;
			break;
		case CMD_FORMD_TYPE:
#if defined(DECENT_SCHEME_1)
			Cmd2AgentStates(&cmdR, &aSelf, aNeighbors, w);
#elif defined(DECENT_SCHEME_2)
			Cmd2AgentStatesRef(&cmdR, &aSelf, aNeighborsr, w);
#endif
#if defined(XBEE)
                 swarmflag |= (0xFFFFFFFF & (cmdR.srcL));
#else
			swarmflag |= (0xFF & (cmdR.src));
#endif
			break;
		case CMD_SETDFPARA_TYPE:
	//		SetDFPara(&cmdR);
			ack |= 0x0001;
			break;
		case CMD_SETVAR_TYPE:	// expand the code here, which is not suggested, and for debug use only
			if(cmdR.code == CMD_SETVAR_COMRATE)
			{
				if(cmdR.para[4] != 0x7FFF)	// validate parameter
				{
					if(cmdR.para[4] > 40)
					{
						com_period_formation =(int16)( cmdR.para[4] *0.4);	// ms -> 500 Hz
						bStatus.bit.bSwTx = 1;
					}
					else if(cmdR.para[4] < 0)
					{
						com_period_formation = 1;	// never transmit
						bStatus.bit.bSwTx = 0;
					}
				}
				if(cmdR.para[5] != 0x7FFF)	// validate parameter
				{
					if(cmdR.para[5] > 40)
						com_period_report = (int16)(cmdR.para[5] *0.4);		// ms -> 500 Hz
					else if(cmdR.para[5] < 0)
						com_period_report = 1;		// never transmit
				}
			}
			else if(cmdR.code == CMD_SETCOMTOPO_SCHEME)
			{
			//	SetComTopo(&cmdR);
			}

			break;
		default:
			break;
		}
		if(msgR)
			MsgReset(msgR);
	}
	ComTimerRun(w);	// communication timer for decentralized formation flight

	// Get data
	GetAccRaw(acc_raw);
	GetGyroRaw(gyro_raw);
	//GetCmpsRaw(cmps_raw);
	HMC_CalcMag(cmps_raw);//修改的实验


	ConvAcc(acc_raw, acc);
	AccFilter(acc, acc_f);
	MagFilter(cmps_raw, mag);
	ConvGyro(gyro_raw, gyro);
	CalcAtt_a(acc_f, mag, att_a);

	bStatus.bit.bSD = bSDExist;
	// sum of w 
	w_all = 0;
	for(i_w=0; i_w<N_SWARM; i_w++)
		w_all += w[i_w];
	if(w_all > 1e-5)
	{
		bStatus.bit.bSwRx = 1;
		swarmflag |= 0x0200;
	}
	else	// 0
	{
		bStatus.bit.bSwRx = 0;
		// swarmflag |= 0x0000
	}
	// SwTx 写在设置 com_period_formation 的地方


	// Filter
	AttFilter(att_a, gyro, att);
	pos[0] = uwbpos[0];
	pos[1] = uwbpos[1];
	//PosFilter(uwbpos,uwbvel,acc,pos,vel,att_a,bUWBUpdate);


	att2[0] = rad2deg(att[0]);
	att2[1] = rad2deg(att[1]);
#ifdef UWB_GUIDANCE
	att2[2] = UWB_angle;
	att[2] = att2[2]/180*PI;
#else
	att2[2] = rad2deg(att[2]);
#endif
	//得到机体坐标系的三个加速度
//	CalcAcc_bodyframe(acc_f, att, acc_bodyframe);

	if(CpuTimer0.InterruptCount>=8000)
	{
		height = 0.05;	// pos in NED frame
	}

	if (CpuTimer0.InterruptCount<8000)
	{
		att_zero=att2[2];
	}

	if(CpuTimer0.InterruptCount>=8000)
	{
	  if (att_zero>=0)
	  {
		if((att2[2]>=att_zero) && (att2[2]<=180))
		{att_error = - (att2[2]-att_zero);}
		else if((att2[2]>=-180)&&(att2[2]<=(att_zero-180)))
		{att_error = -(180-att_zero+180-fabs(att2[2]));}
		else if((att2[2]<att_zero)&&(att2[2]>=0))
		{att_error = att_zero - att2[2];}
		else
		{att_error=att_zero + fabs(att2[2]);}
	   }
	   else if(att_zero<0)
	   {
		if((att2[2]>=att_zero)&&(att2[2]<=0))
		{att_error=att_zero-att2[2];}
		else if((att2[2]>=0) && (att2[2]<=180-fabs(att_zero)))
		{att_error = -(fabs(att_zero)+att2[2]);}
		else if( (att2[2]>=-180) && (att2[2]<=att_zero))
		{att_error = fabs(att2[2]) - fabs(att_zero);}
		else
		{att_error = 180 - fabs(att_zero) + 180 - att2[2];}
	    }
	}
	
	SetAgent(&aSelf, pos, vel, pos_set, spd_set);
	if(t0 != 0)
	{
#if defined(DECENT_SCHEME_1)
	   GetSetValue(&traj_ab, pos_set,spd_set,acc_set,(CpuTimer0.InterruptCount*DTIME-t0));
	   DecentPosCtrl(pos,pos_set,&aSelf, acc_set, v_ctrl, vel, ure,aNeighbors,w);
#endif
       w1 = Saturation((-1/r*(-v_ctrl[0] - v_ctrl[1] - Kp_z * att_error)),w_sat);//-0.163w- kw * att_error
       DELAY_US(100);
	   w2 = Saturation((-1/r*( v_ctrl[0] - v_ctrl[1] + Kp_z * att_error)),w_sat);//+0.163w+ kw * att_error
	   DELAY_US(100);
	   w3 = Saturation((-1/r*( v_ctrl[0] - v_ctrl[1] - Kp_z * att_error)),w_sat);//-0.163w- kw * att_error
	   DELAY_US(100);
	   w4 = Saturation((-1/r*(-v_ctrl[0] - v_ctrl[1] + Kp_z * att_error)),w_sat);//+0.163w+ kw * att_error

	}
	pwm[0] = w1;
	pwm[1] = w2;
	pwm[2] = w3;
	pwm[3] = w4;
	PWMset(w1,w2,w3,w4);


#if defined(CONTROL_DECENT1)	// send part
#ifdef FORMATION_SEND
	if((CpuTimer0.InterruptCount % (com_period_formation) == (FORMATION_INDEX)) && IsSendOK())	// 5Hz // !  com_period_formation>>1
	{
		iNeighbors = (iNeighbors+1) % 3;
		if(iNeighbors<N_NEIGHBORS)	// Only when GPS is normal and I'm close-looped can I send out my error
		{
			AgentStates2Cmd(&aSelf, &cmdS, iNeighbors);
#if defined(XBEE)
                if((cmdS.destH != ADDRESS_INVALID_H) && (cmdS.destL != ADDRESS_INVALID_L))  // ADDRESS_INVALID is used to indicate that the packet won't be sent out
//              if(cmdS.dest != ADDRESS_INVALID)    // ADDRESS_INVALID is used to indicate that the packet won't be sent out
                {
                    Cmd2Msg(&cmdS, &msg);
//                    MsgSend(&msg);
                    swarmflag |= 0x0100;
                }
#else
			if(cmdS.dest != ADDRESS_INVALID)	// ADDRESS_INVALID is used to indicate that the packet won't be sent out
			{
				Cmd2Msg(&cmdS, &msg);
				MsgSend(&msg);
				swarmflag |= 0x0100;
			}
#endif
		}
	}
#endif
#elif defined(CAR_RED)
#ifdef FORMATION_SEND
	if((CpuTimer0.InterruptCount % (com_period_formation>>1) == (FORMATION_INDEX*10)) && IsSendOK())	// 5Hz // !  com_period_formation>>1
	{
		iNeighbors = (iNeighbors+1) % 1;
		if(iNeighbors<N_NEIGHBORS)	// Only when GPS is normal and I'm close-looped can I send out my error
		{
			AgentStates2Cmd(&aSelf, &cmdS, iNeighbors);
			if(cmdS.dest != ADDRESS_INVALID)	// ADDRESS_INVALID is used to indicate that the packet won't be sent out
			{
				Cmd2Msg(&cmdS, &msg);
				MsgSend(&msg);
				swarmflag |= 0x0100;
			}
		}
	}
#endif
#elif defined(CAR_YELLOW)
#ifdef FORMATION_SEND
	if((CpuTimer0.InterruptCount % (com_period_formation>>1) == (FORMATION_INDEX*10)) && IsSendOK())	// 5Hz // !  com_period_formation>>1
	{
		iNeighbors = (iNeighbors+1) % 1;
		if(iNeighbors<N_NEIGHBORS)	// Only when GPS is normal and I'm close-looped can I send out my error
		{
			AgentStates2Cmd(&aSelf, &cmdS, iNeighbors);
			if(cmdS.dest != ADDRESS_INVALID)	// ADDRESS_INVALID is used to indicate that the packet won't be sent out
			{
				Cmd2Msg(&cmdS, &msg);
				MsgSend(&msg);
				swarmflag |= 0x0100;
			}
		}
	}
#endif
#elif defined(CAR_BLACK)
#ifdef FORMATION_SEND
	if((CpuTimer0.InterruptCount % (com_period_formation>>1) == (FORMATION_INDEX*10)) && IsSendOK())	// 5Hz // !  com_period_formation>>1
	{
		iNeighbors = (iNeighbors+2) % 1;
		if(iNeighbors<N_NEIGHBORS)	// Only when GPS is normal and I'm close-looped can I send out my error
		{
			AgentStates2Cmd(&aSelf, &cmdS, iNeighbors);
			if(cmdS.dest != ADDRESS_INVALID)	// ADDRESS_INVALID is used to indicate that the packet won't be sent out
			{
				Cmd2Msg(&cmdS, &msg);
				MsgSend(&msg);
				swarmflag |= 0x0100;
			}
		}
	}
#endif
#else
	#error ERROR: Node type of the swarm not specified!
#endif


	// data transmitting
	if(CpuTimer0.InterruptCount % com_period_report == (19 + FORMATION_INDEX*40) && IsSendOK())
	{
		MsgReset(&msg);
#if defined(XBEE)
        MsgAppendFrame(&msg,addr_server);//填充XBEE需要的数据传输格式
#else
		MsgSetDest(&msg, &addr_server);
#endif
		MsgSetSrc(&msg);
#if defined(FORMATIONFLGIHT)
		// <- For formation flight, less data
		MsgAppend_byte(&msg, CMD_MESSAGE_FORMATIONFLIGHT);
		MsgAppend_rad(&msg, att, 3);
		MsgAppend_float(&msg, pos_r, 2);	// horizontal position
		MsgAppend_float(&msg, &height, 1);
		MsgAppend_int(&msg, &nsvGPS, 1);
		MsgAppend_int(&msg, &bStatus.all, 1);
		MsgAppend_int(&msg, &ack, 1);	ack &= 0xFFFE;
		MsgAppend_byte(&msg, 0);	// checksum, but OK to use 0 
		// 9 16-bit number ->
#elif defined(FLIGHTCONTROL)
		MsgAppend_rad(&msg, att, 3);
		MsgAppend_float(&msg, pos_set, 3);
		// MsgAppend_int(cmps_raw, 3);
		MsgAppend_rad(&msg, att_set, 3);
		MsgAppend_int(&msg, pwm_i, 4);
		MsgAppend_float(&msg, pos, 2);	// horizontal position
		MsgAppend_float(&msg, &height, 1);
		MsgAppend_float(&msg, vGPS, 2);	// horizontal speed
		MsgAppend_int(&msg, &nsvGPS, 1);

		MsgAppend_float(&msg, pos_set, 3);

		MsgAppend_int(&msg, &pres_biased, 1);
		MsgAppend_float(&msg, &intg_height, 1);
		MsgAppend_float(&msg, &intg_height, 1);

		MsgAppend_int(&msg, &ack, 1);	ack &= 0xFFFE;
// output compass
#elif defined(BOARDTEST)
		MsgAppend_rad(&msg, att, 3);
		MsgAppend_int(&msg, &pres_biased, 1);
		MsgAppend_float(&msg, &tempa, 1);
		MsgAppend_float(&msg, &height_US, 1);
		MsgAppend_int(&msg, pwm_i, 4);
		MsgAppend_float(&msg, acc, 3);
		MsgAppend_float(&msg, gyro, 3);
		MsgAppend_int(&msg, cmps_raw, 3);
		MsgAppend_float(&msg, RC_width, 6);
		MsgAppend_int(&msg, &ack, 1);	ack &= 0xFFFE;
		// added
#else
	#error  WARNING: no sensor specified for height control!
#endif
#if defined(XBEE)
            MsgAppendAddFrame(&msg);
#endif
		MsgSend(&msg);
	}

		if(CpuTimer0.InterruptCount%5 == 1)	// 会有一些报头和长周期数据
	{
		SDBufAppend_zeros(1);
		SDBufAppend_buflen();	// 16-bit
		SDBufAppend_int((int*)(&CpuTimer0.InterruptCount), 2);
		SDBufAppend_float(posGPS, 3);//posgps
	//	SDBufAppend_float(reftrack, 3);
		SDBufAppend_float(&vHeight, 1);
		SDBufAppend_int(&sendType, 1);
		SDBufAppend_float(&height, 1);
		SDBufAppend_float2(&timeGPS, 1);
		SDBufAppend_float(vGPS, 3);//vgps
	//	SDBufAppend_float(reftrackv, 3);
		SDBufAppend_float(pos_set, 3);//pos_set

		SDBufAppend_float(&basicOut, 1);

		SDBufAppend_float(spd_set, 3);//spd_set
		SDBufAppend_float(vel, 3);//pos_zero
		SDBufAppend_float(pos, 3);
		SDBufAppend_float(uwbvel, 3);//vel
		// 31 * 2 / 31*2
		SDBufAppend_zeros(0);
		// 62 Bytes per cycle
	}
	// 普通数据
	if(1)	// 普通数据，500Hz
	{
		
	//	SDBufAppend_int(acc_raw, 3);
		SDBufAppend_float(vGPS_ori, 3);
		SDBufAppend_float(v_ctrl, 3);
	//	SDBufAppend_float(acc, 3);
		SDBufAppend_float(vGPS_dif, 3);//vGPS_dif

		SDBufAppend_float(acc_f, 3);
		SDBufAppend_rad(att, 3);

		SDBufAppend_rad(gyro, 3);
		SDBufAppend_rad(att_a, 3);
		SDBufAppend_rad(att_set, 3);
		SDBufAppend_float(RC_width, 6);
		SDBufAppend_float(pwm, 4);
		SDBufAppend_int(cmps_raw, 3);
		
		SDBufAppend_float(&att_zero, 1);
		SDBufAppend_float(&att_error, 1);
		SDBufAppend_float(&t0, 1);
		SDBufAppend_float(&tempa, 1);
		SDBufAppend_int(&swarmflag, 1);
		SDBufAppend_zeros(3);
		// 42 * 2 / 45*2
		// 90 Bytes per cycle
	}
	if(CpuTimer0.InterruptCount%5 == 0)	// end of a packet
		SDBufReset();

	// data logging	// 500Hz-inner 100Hz-outer



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
   PieVectTable.SCIRXINTB = &SCIRXINTb_ISR;//获取UWB定位数据
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
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
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

