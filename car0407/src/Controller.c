#include "Controller.h"

/* 电机输入-输出特性说明

电机输入为500Hz（2ms）矩形波，脉宽有效范围0.2ms-1.2ms（饱和），0.15ms启动
输出升力（N）-输入脉宽（ms）关系为：
K=0.09586 B=0.17933
输出转矩（Nm）-输入脉宽（ms）关系为：
K=5.7335 B=0.16166
输出升力上限在8~9N，程序中限为8N；输出转矩上限在0.15Nm多，程序中限为0.15Nm

*/
#define	PI	3.1415926
#define PsaiMdf(x) (x>PI ? (x-2*PI) : (x<=-PI ? x+2*PI : x))	// psai ~ (-pi, pi]

// 最外层的控制器，输入state姿态/位置/etc，输入input设定姿态/位置/etc，输出四个脉宽
double err_roll = 0;
double err_roll_p = 0;	// previous
double err_pitch = 0;
double err_pitch_p = 0;
double err_yaw = 0;
double err_yaw_p = 0;
double Kp_roll = 0;
double Kd_roll = 0.;
double Kp_pitch = 0;
double Kd_pitch = 0;
double Kp_yaw = 0;  //0.1468;
double Kd_yaw = 0;  //0.2;
double f_roll = 1000;
double f_pitch = 1000;
double f_yaw = 1000;
double Ku_roll = 0, Ku_roll_ = 0, Ky_roll_ = 0;
double Ku_pitch = 0, Ku_pitch_ = 0, Ky_pitch_ = 0;
double Ku_yaw = 0, Ku_yaw_ = 0, Ky_yaw_ = 0;
double U1 = -1, U2 = 0, U3 = 0 , U4 = 0;	// 中间四位控制量，前三个升力和/差，最后一个扭矩差
double U2_= 0, U3_ = 0, U4_ = 0;	// 上一周期的控制量
double U2_o = 0, U3_o = 0, U4_o = 0;	// 实际控制量，已经做了限幅
double T1, T2, T3, T4;	// 中间四位输出量，对应U1-U4的脉宽组合
double t1, t2, t3, t4;	// 算出的输出量，未限幅
const double DT_c = 0.01;	// 控制步长
// 升力 - 脉宽转换系数(ms)
#define K_f2t	0.09586
#define B_f2t 	0.17933
// 扭矩 - 脉宽转换系数(ms)
#define K_t2t   5.7335
#define B_t2t   0.16166
// const double b = 4.109e-5;		// 升力系数
// const double d = 6.0327e-7; 	// 转矩系数
const double PULSE_MIN = 0.15;	// 0.15-0.2之间部分为启动，可能不稳定
const double PULSE_MAX = 1.2 - 0.0005;
// 差分输出限幅值（相当于每个因素对电机脉宽的调整量只有100us以内）
const double U2_sat = 0.2/K_f2t;
const double U3_sat = 0.2/K_f2t;
const double U4_sat = 0.4/K_t2t;

unsigned char bRollOn = 0, bPitchOn = 0;	// 是否开启控制横滚/俯仰的旋翼组
#define PULSE_RANGE(x) (x<PULSE_MIN ? PULSE_MIN : (x>PULSE_MAX ? PULSE_MAX : x))
void Controller(double* state, double* input, int* output, unsigned char* debug)
{// 状态输入目前为3（姿态rad）；设定输入目前为4（姿态rad+油门%）；输出四个数（脉宽us）;调试输出6个int8（3个int6）
	// 更新状态
	err_roll_p = err_roll;
	err_pitch_p = err_pitch;
	err_yaw_p = err_yaw;
	err_roll = input[0] - state[0];
	err_pitch = input[1] - state[1];
	err_yaw = PsaiMdf(input[2] - state[2]);
	// 计算U1-4
	// U1 = 15;			// 升力和
	U2_ = U2;
	U3_ = U3;
	U4_ = U4;
	U2 = Kp_roll*err_roll + Kd_roll/DT_c*(err_roll-err_roll_p);
	U3 = Kp_pitch*err_pitch + Kd_pitch/DT_c*(err_pitch-err_pitch_p);
	U4 = Kp_yaw*err_yaw + Kd_yaw/DT_c*(err_yaw-err_yaw_p);
	U2_o = (U2>U2_sat) ? U2_sat : ((U2<-U2_sat) ? -U2_sat : U2);// 限幅
	U3_o = (U3>U3_sat) ? U3_sat : ((U3<-U3_sat) ? -U3_sat : U3);
	U4_o = (U4>U4_sat) ? U4_sat : ((U4<-U4_sat) ? -U4_sat : U4);
	debug[0] = (int)(U2_o*1000) >> 8;
	debug[1] = (int)(U2_o*1000) & 0xff;
	debug[2] = (int)(U3_o*1000) >> 8;
	debug[3] = (int)(U3_o*1000) & 0xff;
	debug[4] = (int)(U4_o*1000) >> 8;
	debug[5] = (int)(U4_o*1000) & 0xff;

	// 计算T1-4
	T1 = U1 * K_f2t + 4 * B_f2t;// t1+t2+t3+t4
	T2 = U2_o * K_f2t;			// t2-t4
	T3 = U3_o * K_f2t;			// t1-t3
	T4 = U4_o * K_t2t;			// t1+t3-t2-t4

	// 转换为输出脉宽t1-t4
	t1 = 0.25*T1 + 0.5*T3 + 0.25*T4;
	t2 = 0.25*T1 + 0.5*T2 - 0.25*T4;
	t3 = 0.25*T1 - 0.5*T3 + 0.25*T4;
	t4 = 0.25*T1 - 0.5*T2 - 0.25*T4;
	// 输出限幅
	t1 = PULSE_RANGE(t1);
	t2 = PULSE_RANGE(t2);
	t3 = PULSE_RANGE(t3);
	t4 = PULSE_RANGE(t4);
	// 关推进组
	if (!bRollOn)
	{
		t2 = 0.15;
		t4 = 0.15;
	}
	if (!bPitchOn)
	{
		t1 = 0.15;
		t3 = 0.15;
	}
	// 关油门
	if (U1 < 0)
	{
		t1 = 0.15;
		t2 = 0.15;
		t3 = 0.15; 
		t4 = 0.15;
	}
	// 输出
	output[0] = (int)(t1*1000 + 0.5);
	output[1] = (int)(t2*1000 + 0.5);
	output[2] = (int)(t3*1000 + 0.5);
	output[3] = (int)(t4*1000 + 0.5);
	// 还没考虑偏航翻转啊！！
}

// 输入当前高度，指令高度。输出高度控制信号至Uh，通过SetU1()作用于U1。输出一个字调试信息
double Uh = 0, Uh_p = 0;	// 当前和之前的Uh
double err_height = 0, err_height_p = 0; // 当前和之前的误差
double Uh_sat = 4;			// 允许4N的附加作用
double Kp_height = 0, Kd_height = 0;
double height_set = 0.35;
const double DT_h = 0.05;	// 高度控制步长
void Controller_Height(double* height, unsigned char* debug)
{
	err_height_p = err_height;
	err_height = height_set - (*height);
	Uh_p = Uh;
	Uh = Kp_height * err_height + Kd_height / DT_h * (err_height - err_height_p);

	// 限幅
	Uh = (Uh > Uh_sat) ? Uh_sat : ((Uh < -Uh_sat) ? -Uh_sat : Uh);

	// 调试输出
	debug[0] = (int)(Uh*1000) >> 8;
	debug[1] = (int)(Uh*1000) & 0xff;
}

#define SET_KFAI	4
#define SET_KSITA	5
#define	SET_KPSAI	6
#define SET_F		7
#define SET_FC_LPF	8
#define	SET_KHEIGHT	9
#define	SET_HEIGHT	10
extern	double fc_lp;
unsigned char cmd, parity;
unsigned int para1, para2;
double K;
unsigned char SetCtrlPara(unsigned char* input)
{
	// input
	cmd = input[0];
	para1 = (input[1]<<8) | (input[2] & 0xff);
	para2 = (input[3]<<8) | (input[4] & 0xff);
	parity = input[5];

	// parity
	if(((cmd + input[1] + input[2] + input[3] + input[4] - parity) & 0xff) != 0)
		return 0;	// 无效

	// Set parameters
	switch(cmd)
	{
	case SET_KFAI:
		if ((para1==0xffff) && (para2==0xffff))
			bRollOn = 0;
		else
		{
			bRollOn = 1;
			Kp_roll = (double)(para1) / 100.0;
			Kd_roll = (double)(para2) / 100.0;
			K = 1 + 2.0/DT_c/f_roll;
			Ky_roll_ = (1-2.0/DT_c/f_roll)/K;
			Ku_roll = (2*Kd_roll/DT_c + Kp_roll*(1+2/DT_c/f_roll))/K;
			Ku_roll_ = (Kp_roll*(1-2/DT_c/f_roll) - 2*Kd_roll/DT_c)/K;
		}
		break;
	case SET_KSITA:
		if ((para1==0xffff) && (para2==0xffff))
			bPitchOn = 0;
		else
		{
			bPitchOn = 1;
			Kp_pitch = (double)(para1) / 100.0;
			Kd_pitch = (double)(para2) / 100.0;
			K = 1 + 2.0/DT_c/f_pitch;
			Ky_pitch_ = (1-2.0/DT_c/f_pitch)/K;
			Ku_pitch = (2*Kd_pitch/DT_c + Kp_pitch*(1+2/DT_c/f_pitch))/K;
			Ku_pitch_ = (Kp_pitch*(1-2/DT_c/f_pitch) - 2*Kd_pitch/DT_c)/K;
		}
		break;
	case SET_KPSAI:
		Kp_yaw = (double)(para1) / 100.0;
		Kd_yaw = (double)(para2) / 100.0;
		K = 1 + 2.0/DT_c/f_yaw;
		Ky_yaw_ = (1-2.0/DT_c/f_yaw)/K;
		Ku_yaw = (2*Kd_yaw/DT_c + Kp_yaw*(1+2/DT_c/f_yaw))/K;
		Ku_yaw_ = (Kp_yaw*(1-2/DT_c/f_yaw) - 2*Kd_yaw/DT_c)/K;
		break;
	case SET_F:
		f_roll = (double)(para1) / 100.0;
		f_pitch = (double)(para1) / 100.0;
		f_yaw = (double)(para1) / 100.0;
		break;
	case SET_FC_LPF:
		fc_lp = (double)(para1) / 100.0;
		break;
	case SET_KHEIGHT:
		Kp_height = (double)(para1) / 100.0;
		Kd_height = (double)(para2) / 100.0;
		break;
	case SET_HEIGHT:
		height_set = (double)(para1) / 100.0;
		break;
	default:
		break;
	}
	return 1;
}

/* Set Throttle */
double	thro = -1;
void SetThro(unsigned int val)
{ // val: 占空比*100000:5000-10000
	if(val < 5500)
		thro = -1;
	else if(val > 9500)
		thro = 20;
	else
		thro = (double)(val-5500) * 0.005;
}

double rud_set = 0;
void SetRud(double* yaw_set, unsigned int val)
{
	if(val<7600 && val>7400)
		;// nothing
	else
	{// 还有！标准化
		rud_set -= DT_c * PI * ((int)(val)-7500) / 1000;
		rud_set = PsaiMdf(rud_set);
	}
	*yaw_set = rud_set;
}

void GetYaw(double* yaw)
{
	*yaw = rud_set;
}


void SetU1()
{
	if (thro < 0)
		U1 = -1;
	else
		U1 = thro + Uh;
}
