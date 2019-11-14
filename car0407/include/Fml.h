/* Functional Module Layer */
#ifndef FML_H__
#define FML_H__

#include "Hal.h"
#include "MS5803-01BA.h"
#include "filter.h"
#include "message.h"
#define DTIME	0.0025
#define DTIMEuwb 0.1
#define CYCLE	2500	// cycle of control: 2000us / 500Hz
#define DT_ATT	0.0025

#define ROTOR_INIT	990
void UWBDataAppend(unsigned char data);
void PWMset(double w1,double w2,double w3,double w4);
interrupt void SCIRXINTb_ISR(void);
unsigned char UWBPacketReady();
float Ascll2Float(unsigned char* buf);
void UWBPacketRead();

struct UWB_Data
{
	char  Status;//UWB定位帧帧头-@
	float x;//接收到主站解算的X坐标
	float y;//接收到主站解算的Y坐标
	float z;//接收到主站解算的Z坐标
	float vx;//接收到主站解算的X坐标
	float vy;//接收到主站解算的Y坐标
	float vz;//接收到主站解算的Z坐标
	float angle;
	char Lable;//标签编号
	int number;//定位帧序号
};

void	GetAccRaw(int16* acc_raw);		// X Y Z
void	GetGyroRaw(int16* gyro_raw);	// X Y Z
void	GetCmpsRaw(int16* cmps_raw);
void	GetYaw(double* yaw);			// Z，与控制周期相关
void	ConvAcc(const int16* acc_raw, double* acc);	// XYZ, convert raw to [m/s^2]
void	ConvGyro(const int16* gyro_raw, double* gyro);	// XYZ, convert raw to [rad/s]
void	GyroFilter(const double* gyro, double* gyro_f); // XYZ, 1-order low pass
void	CalcAtt_a(const double* acc, const double* mag, double* att_a);	// XYZ, [m/s^2] [rad] [rad]
void	CalcAcc_bodyframe(const double* acc_f, const double* att,  double* acc_bodyframe);
void	AttFilter(const double* att_a, const double* gyro, double* att); // XYZ, attitude from acc&cmps[rad]; gyro[rad/s]; attitude[rad]
struct filter_pair{
	double	x;	// 当前值
	double	x_;	// 上一周期值
};
void    Filter_1LPV(struct filter_pair* input, struct filter_pair* output, double fc);
void	Filter_1LP(struct filter_pair* input, struct filter_pair* output, double fc);
void	AccFilter(const double* acc, double* acc_f);	// XYZ, low pass
void	MagFilter(const int16* cmps_raw, double* mag);	// XYZ, low pass

void	MsgSend(Msg* msg);
void	ZigbeeSend();
void	ZigbeeResetAddress();
unsigned char IsSendOK();
void	MsgAppend_byte(Msg* msg, unsigned char byte);
void	MsgAppend_float(Msg* msg, const double* data, const unsigned char num);	// 发浮点数（小数点后2位,±327.68）
void	MsgAppend_rad(Msg* msg, const double* data, const unsigned char num);	//	把弧度转成角度发出（小数点后2位）
void	MsgAppend_int(Msg* msg, const int* data, const unsigned char num);	// 发整形（±32768）
unsigned char	MsgIsEmpty();	// check if the Msg queue is empty
interrupt void ISRSciTxCFifo(void);
interrupt void ISRSciRxCFifo(void);
void WirelessDataAppend(unsigned char data);
char*	IsWRLDataReady();
void	GetRC(double* pulsewidth);	// refresh pulse-width(ms) of RC // not finished yet
void	GetHeightURM37(double* height);	// Get Height(from Ultrosonic sensor)
void	URM_Trigger();		// generate Trigger signal for URM37
void	RCAttSet(double* rcctrl, double* att_set, const double* att);	// Set Attitude Reference by Radio Controller

//6.16 add
void	RCPosSet(double* rcctrl, double* pos, double*RCpos_set, double* RCspd_set, double*RCacc_set);
void	RCYawSet(double* rcctrl,double* yaw_set);

unsigned char IsAutoMode(double ChannelSwitch);		// if in auto mode?
interrupt void RC3_ISR(void);
interrupt void RC1_ISR(void);
interrupt void RC2_ISR(void);
interrupt void RC4_ISR(void);
interrupt void RC6_ISR(void);
interrupt void USonic_ISR(void);
interrupt void ISRSciRxAFifo(void);
inline void GPSDataAppend(unsigned char data);
unsigned char GPSPacketReady();
unsigned char	GPSPacketRead();
void 	SetGPSPositonZero();
void	GetGPSPosition(double* p);	// position
void	GetGPSSpeed(double* v);		// speed
void	GetGPSSpeed_V4(double* vGPS, double*posGPS ,double time); //Get speed using new meehod
void	GetGPSNSV(int16* n);			// number of satellites
void 	GetGPSTime(long double* t);		// time of arrival
void	GetGPSTrack(double* trackGPS);	// track of GPS
void	GetGPSGSpeed(double* gspeedGPS);// Ground speed of GPS
#define SetRotor1(x)	SetPWM1(x)	// us
#define SetRotor2(x)	SetPWM2(x)
#define SetRotor3(x)	SetPWM3(x)
#define SetRotor4(x)	SetPWM4(x)
void 	Output2PWM(const double* output, double* pwm, int16* pwm_i);	// pwm(us)
void 	SetRotor(const double *pwm);
void 	GetThrottle(double * throttle, double * output);	// throtle(ms); output(N)
void	AttCtrl(const double * att_set, const double * att, double* output, const double height);	// (rad) (rad) (N N Nm)
void	AttCtrl_V4(const double * att_set, const double * att,  const double * acc_bodyframe,  const double * gyro, double* output, const double height);
double	DiffSat(double x, double x_, double sat);			// speed saturation
void	HPosCtrl(const double* posGPS, const double* pos_zero, const double* spd_zero, const double* speed, const double* pos_set,  const double* spd_set, double* att_set, const double* att, const double* acc_set, const double height);	// NED(m), rad
void 	HeightCtrl(double height_set, double height, double height_zero, double spd_set, double vheight, double acc_set, const double* att, double* output, const double heightU);
void 	HeightCtrlGetSetPoint(double* zSet, double vz, double* att, double* zOut);
void 	HeightCtrlGPS(double height_set, double height, double height_zero, double spd_set, double vheight, double acc_set, const double* att, double* output);
void 	HeightCtrlMix(double height_set, double height, double height_zero, double spd_set, double vheight, double acc_set, const double* att, double* output);
void 	HeightCtrlReset(double heightc, double vHeight, double height_set);
void	SetVector(const double* vsrc, double* vdest, const unsigned char n);
void	HPosCtrlReset(const double* attset);
void	SetVar(const Cmd* cmd, double* var);

void	WriteNextBlock(Uint32 SectorAddress);
void	SD_ReadBlock(Uint32 SectorAddress);
void	SD_WriteBlock(Uint16 *data, Uint32 SectorAddress);
void	SD_SetStartSector(Uint32* StartSector);
inline	void SD_BufferAppend();
void	SDBufReset();
void	SDBufAppend_int(int* data, unsigned char num);
void	SDBufAppend_float(double* data, unsigned char num);
void	SDBufAppend_float2(long double* data, unsigned char num);
void	SDBufAppend_rad(double* data, unsigned char num);
void	SDBufAppend_zeros(unsigned char num);
void	SDBufAppend_buflen();

void Baro2Height(int32 p, double* height);
void Baro2Height_cal(double hUS, unsigned char bResetFilter);
void Baro2Height_cal2(double hBAAcc, double hUSAcc);
void	HeightFilterBA(double heightUS, const double* acc_f, const double* att, double* height, double* vHeight);
void	HeightFilterBA_SetBiasA(double ba);
void	HeightFilterBA_SetValue(double height, double vHeight);
void	HeightFilterGPS(const double* HeightGet, const double* att, const double* ab, double* Vheight);

struct GPHPP{
	int16	GPSWeek;
	long double	GPSTime;	// 一周中第几秒
	long double	Latitude;	
	long double Longitude;
	double	Altitude;
	double	Track;
	double	GSpeed;
	double	Ve;
	double	Vn;
	double	Vu;
	double	Ae;
	double	An;
	double	Au;
	int16	NSV;
};

struct GPRMC{
	// Date
	long double 	GPSTime;	// 当天时间
	unsigned char 	PosStatus;// position status: A(valid)/V
	long double 	Latitude;
	long double 	Longitude;
	double 	GSpeed;
	double 	Track;
	int16 	NSV;	// not true
	//-- calculated --
	double	Ve;
	double	Vn;
	//-- unavailable --
	double Vu;
	double Altitude;
};

struct PUBX{
	long double GPSTime;	// 当天时间
	long double Latitude;
	long double Longitude;
	double	Altitude;
	char	NavStat[2];
	double	Hacc;
	double	Vacc;
	double	GSpeed;	// Speed over Ground - SOG
	double	Track;	// Course over Ground - COG
	double	Vu;		
	double	HDOP;
	double	VDOP;
	double	TDOP;
	int16	NSV;
	////
	double	Ve;
	double	Vn;
};

void	SetCtrlPara(Cmd* cmd);
void	SetDestPos(Cmd* cmd);
void	SetRefDestPos(Cmd* cmd);
Msg* 	IsMsgReady();

#define N_SCHEDULE	10	// number of commands in the schedule
typedef struct{
	unsigned char isOn;	// is the schedule running
	unsigned char isCmd;// is command being generated
	double	t;			// timer of the schedule
	char iSchedule;	// number of schedule running
	int16	rawSchedule[N_SCHEDULE][11];

}ScdFlight;
void	FlightScdRun(ScdFlight* scdFlight);
void	FlightScdOn(ScdFlight* scdFlight, Cmd* cmd);
void	FlightScdDesign(ScdFlight* scdFlight);
void	FlightScdCmd(ScdFlight* scdFlight, Cmd* cmd);





#endif

