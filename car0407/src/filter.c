#include "filter.h"
#include "para.h"
#include <math.h>
#include <mathex.h>
#define POSFILTER_ON
/* 
	位置滤波器
	输入：GPS位置，速度；姿态
	输出：位置，速度
	运行频率：内环频率
*/
double p_hat[3] = {0, 0, 0};	// 位置估计值
double v_hat[3] = {0, 0, 0};	// 速度估计值
double p_hat_[3] = {0, 0, 0};	// 上一周期位置估计值
double v_hat_[3] = {0, 0, 0};	// 上一周期速度估计值
double pi_[3] = {0, 0, 0};	// 上一周期输入位置
double vi_[3] = {0, 0, 0};	// 上一周期输入速度
double ag_[3] = {0, 0, 0};	// 上一周期加速度
double ev[3] = {0, 0, 0};	// 速度误差（估计值和测量值之间）
double ep[3] = {0, 0, 0};	// 位置误差（估计值和测量值之间）
static const double kv[3] = {1, 1,1};	// 速度环反馈系数（截止频率）
static const double kp[3] = {0.5, 0.5, 0.5};	// 位置环反馈系数（截止频率）
const double	biasGravity = -0.57;//这个是由于加速度在竖直方向的误差造成的一个滤波偏差，看可不可以暂时
double diffvmax = 1.2;
//使用这样的方式消除
double t_=0;
double t =0;
void	SpdFilterUWB(const double* pos_x,
					const double* pos_y,
					double* vx_f, double* vy_f)
{
	static double x = 0.1, x_ = 0.1, y = 0.1, y_ = 0.1;
	static double diffVx=0, diffVy=0;//差分得到的速度
	//static double ax_h = 0.0, ax_h_ = 0.0, ay_h = 0.0, ay_h_ = 0.0;	// a_hat for this and last cycle

	static struct filter_pair vx_in = {0,0},vy_in = {0,0};
	static struct filter_pair vx_out = {0,0},vy_out = {0,0};
	//需要差分

	x_ = x;
	y_ = y;
	x = (*pos_x);
	y = (*pos_y);

	diffVx = Saturation((x - x_)/DTIMEuwb,diffvmax);
	diffVy = Saturation((y - y_)/DTIMEuwb,diffvmax);

	//
	//ax_h_ = ax_h;
	//ax_h = cos(angle)*acc[0] - sin(angle) * acc[1];
	//ay_h_ = ay_h;
	//ay_h = sin(angle)*acc[0] + cos(angle) * acc[1];
	//

	vx_in.x_ = vx_in.x;
	vx_in.x =  diffVx;
	vy_in.x_ = vy_in.x;
	vy_in.x =  diffVy;

	Filter_1LPV(&vx_in,&vx_out,2);
	Filter_1LPV(&vy_in,&vy_out,2);
		//	+ (ax_h + ax_h_)* 0.5 * DTIME_INNERLOOP;

	if(vx_f)
	{
		*vx_f = vx_out.x;
		*vy_f = vy_out.x;
		//*vy_f = vy;
	}

}
void	SpdFilterUWB2(const double* pos_x,
					const double* pos_y,
					double* vx_f, double* vy_f)
{
	//滑动窗口滤波
	static double x2 = 0.1, x2_ = 0.1, y2 = 0.1, y2_ = 0.1;
	static double diffVx2=0, diffVy2=0;//差分得到的速度
	//static double ax_h = 0.0, ax_h_ = 0.0, ay_h = 0.0, ay_h_ = 0.0;	// a_hat for this and last cycle

//	static struct filter_pair vx_in = {0,0},vy_in = {0,0};
//	static struct filter_pair vx_out = {0,0},vy_out = {0,0};
	static double vx[4] = {0,0,0,0};
	static double vy[4] = {0,0,0,0};
	static double vx_mean;
	static double vy_mean;
	//需要差分

	x2_ = x2;
	y2_ = y2;
	x2 = (*pos_x);
	y2 = (*pos_y);

	diffVx2 = Saturation((x2 - x2_)/DTIMEuwb,diffvmax);
	diffVy2 = Saturation((y2 - y2_)/DTIMEuwb,diffvmax);
	vx[0] = vx[1];
	vx[1] = vx[2];
	vx[2] = vx[3];
	vx[3] = diffVx2;

	vy[0] = vy[1];
	vy[1] = vy[2];
	vy[2] = vy[3];
	vy[3] = diffVy2;


	vx_mean = (0.5*vx[0]+0.7*vx[1]+0.9*vx[2]+1.9*vx[3])/4;
	vy_mean = (0.5*vy[0]+0.7*vy[1]+0.9*vy[2]+1.9*vy[3])/4;

	if(vx_f)
	{
		*vx_f = vx_mean;
		*vy_f = vy_mean;
		//*vy_f = vy;
	}

}
void	PosFilter(const double* pi,	// input position, ground frame, NEU
				  const double* vi, // input velocity, ground frame, NEU
				  const double* ab, // input accelaration, body frame, FRD
				  double*	po,		// output position, ground frame, NEU
				  double*	vo, 	// output position, ground frame, NEU
				  const double* att,// attitude
				  unsigned char bUWB)// new GPS data
{
	double	ag[3];	// acceleration, ground frame

	/* 加速度变换: ground frame -> body frame */
	ag[0] = cos(att[1]) * cos(att[2]) * ab[0]
		    + (-cos(att[0])*sin(att[2]) + sin(att[0])*sin(att[1])*cos(att[2])) * ab[1]
			+ (sin(att[0])*sin(att[2]) + cos(att[0])*sin(att[1])*cos(att[2])) * ab[2];
	ag[1] = cos(att[1]) * sin(att[2]) * ab[0]
			+ (cos(att[0])*cos(att[2]) + sin(att[0])*sin(att[1])*sin(att[2])) * ab[1]
			+ (-sin(att[0])*cos(att[2]) + cos(att[0])*sin(att[1])*sin(att[2])) * ab[2];
	ag[2] = -sin(att[1]) * ab[0]
			+ sin(att[0]) * cos(att[1]) * ab[1]
			+ cos(att[0])*cos(att[1]) * ab[2];
	ag[2] = -(ag[2] + (GRAVITY + biasGravity));

	if(bUWB)
	{
		ev[0] = vi[0] - v_hat[0];
		ev[1] = vi[1] - v_hat[1];
		ev[2] = vi[2] - v_hat[2];
		ep[0] = pi[0] - p_hat[0];
		ep[1] = pi[1] - p_hat[1];
		ep[2] = pi[2] - p_hat[2];
	}

	/* 速度环梯形积分 */
	vo[0] = v_hat[0] + 0.5*(ag[0] + ag_[0] + kv[0]*ev[0]*2) * DTIME_INNERLOOP;
	v_hat_[0] = v_hat[0];
	v_hat[0] = vo[0];
	vo[1] = v_hat[1] + 0.5*(ag[1] + ag_[1] + kv[1]*ev[1]*2) * DTIME_INNERLOOP;
	v_hat_[1] = v_hat[1];
	v_hat[1] = vo[1];
	vo[2] = v_hat[2] + 0.5*(ag[2] + ag_[2] + kv[2]*ev[2]*2) * DTIME_INNERLOOP;
	v_hat_[2] = v_hat[2];
	v_hat[2] = vo[2];

	/* 位置环梯形积分 */
	po[0] = p_hat[0] + 0.5*(v_hat[0] + v_hat_[0] + kp[0]*ep[0]*2) * DTIME_INNERLOOP;
	p_hat_[0] = p_hat[0];
	p_hat[0] = po[0];
	po[1] = p_hat[1] + 0.5*(v_hat[1] + v_hat_[1] + kp[1]*ep[1]*2) * DTIME_INNERLOOP;
	p_hat_[1] = p_hat[1];
	p_hat[1] = po[1];
	po[2] = p_hat[2] + 0.5*(v_hat[2] + v_hat_[2] + kp[2]*ep[2]*2) * DTIME_INNERLOOP;
	p_hat_[2] = p_hat[2];
	p_hat[2] = po[2];

	/* 历史变量更新 */
	ag_[0] = ag[0];
	ag_[1] = ag[1];
	ag_[2] = ag[2];
	pi_[0] = pi[0];
	pi_[1] = pi[1];
	pi_[2] = pi[2];
	vi_[0] = vi[0];
	vi_[1] = vi[1];
	vi_[2] = vi[2];

	/* if POSFILTER_ON not defined, then filter doesn't really output */
#if defined(POSFILTER_ON)
	// nothing happens
#else
	po[0] = pi[0];
	po[1] = pi[1];
	po[2] = pi[2];
	vo[0] = vi[0];
	vo[1] = vi[1];
	vo[2] = vi[2];
#endif


}
// maybe shouldn't be here
double FilterRun_2o(struct filter_2o* fil, double input)
{
	fil->x_[0] = fil->x[0];
	fil->x_[1] = fil->x[1];
	fil->x_[2] = fil->x[2];
	fil->x_[1] = fil->x[1];
	fil->x[1] = (input + fil->x_[0] - fil->x_[1]*(1-2/(DTIME_INNERLOOP*fil->freq))) / (1+2/(DTIME_INNERLOOP*fil->freq));
	fil->x[2] = (fil->x[1] + fil->x_[1] - fil->x_[2]*(1-2/(DTIME_INNERLOOP*fil->freq))) / (1+2/(DTIME_INNERLOOP*fil->freq));
	fil->x[0] = input;
	return fil->x[2];
}

void FilterReset_2o(struct filter_2o* fil)
{
	fil->x[0] = 0;
	fil->x[1] = 0;
	fil->x[2] = 0;
	fil->x_[0] = 0;
	fil->x_[1] = 0;
	fil->x_[2] = 0;
}

/*
	Description: Filter for height. Currently using with ULTRASONIC sensor and ACCELERATOR
	@ param-i, HeightUS: float, US height, 0~5(m), 10m(around) for INVALID DATA
	@ param-i, acc_f[]: float, 3-axis acceleration, m/s^2, XYZ
	@ param-i, att[]: float, 3-axis attitude, rad, XYZ
	@ param-o, height: float, filtered height
	@ param-o, vHeight: float, filtered velocity of height
*/
#define ULTRASONIC_SAT 6.00
const double Khv = 3, Kha = 2;	// feedback gain to v and a
const double Khba = 0.1;		// integral gain for bias_a
int cntUSoff = 0;			// US off time(cycle)
int cntUSjump = 0;			// US jumping away time(cycle)

static double bias_a = 0, bias_a_ = 0;	// bias of a for this and last cycle
void	HeightFilter(const double heightUS,
		 			 const double* acc_f,
		 			 const double* att,
		 			 double* height,
		 			 double* vHeight)
{
	static double h_h = 0.1, h_h_ = 0.1;	// height_hat for this and last cycle
	static double v_h = 0.0, v_h_ = 0.0;	// v_hat for this and last cycle
	static double a_h = 0.0, a_h_ = 0.0;	// a_hat for this and last cycle
	static double err_height_h = 0.0, err_height_h_ = 0.0;	// err_height_hat for this and last cycle
	
	// get err_height_hat
	if(cntUSoff > 500)
	{// US not available for a long time
		if(heightUS < ULTRASONIC_SAT)
		{// back to normal: reset everything
			h_h = heightUS;		h_h_ = heightUS;
			v_h = 0;			v_h_ = 0;
			a_h = 0;			a_h_ = 0;
			err_height_h = 0;	err_height_h_ = 0;
			cntUSoff = 0;
			cntUSjump = 0;
		}
		else	// still abnormal
			return;	// do not integral any longer
	}
	else if(heightUS >= ULTRASONIC_SAT)	// US not available
	{
		cntUSoff ++;
		err_height_h = 0;	err_height_h_ = 0;	// pure integral
	}
	else	// US available
	{
		cntUSoff = 0;
		if(fabs(heightUS - h_h) < 0.25)	// 5m/s?
		{// normal
			err_height_h_ = err_height_h;
			err_height_h = heightUS - h_h;
			cntUSjump = 0;
		}
		else	// jump a big STEP!
		{
			cntUSjump ++;
			err_height_h = 0;	err_height_h_ = 0;	// pure integral
			if(cntUSjump > 75)	// 3 US cycle
			{// move on to the new point
				h_h = heightUS;		h_h_ = heightUS;
				cntUSjump = 0;
			}
		}
	}

	// integral process
	bias_a_ = bias_a;
	bias_a = bias_a_ + Khv * Khba * (err_height_h + err_height_h_) * 0.5 * DTIME_INNERLOOP;
	if(bias_a > 2)	// bias_a in [-2, 2]m/s^2
		bias_a = 2;
	else if (bias_a < -2)
		bias_a = -2;
	// else bias_a remains;
	a_h_ = a_h;
	a_h = - (-sin(att[1]) * acc_f[0] + sin(att[0]) * cos(att[1]) * acc_f[1] + cos(att[0]) * cos(att[1]) * acc_f[2])
		  - (GRAVITY + biasGravity)
		  + Khv * Kha * err_height_h
		  + bias_a;
	v_h_ = v_h;
	v_h = v_h_ + (a_h + a_h_) * 0.5 * DTIME_INNERLOOP;
	h_h_ = h_h;
	h_h = h_h_ + (v_h_ + v_h + Khv * (err_height_h + err_height_h_)) * 0.5 * DTIME_INNERLOOP;
	
	// output
	if(height)
		*height = h_h;
	if(vHeight)
		*vHeight = v_h;
}

double	HeightFilter_GetBiasA()
{
	return bias_a;
}




void	HeightFilterGPS(const double* HeightGet, 
					const double* att,
					const double* ab,
 					double* Vheight)
{
	
	static double GPS_h = 0.1, GPS_h_ = 0.1;	// height_hat for this and last cycle
	static double GPSv_h = 0.0, GPSv_h_ = 0.0;	// v_hat for this and last cycle
	static double diffV=0;//GPS差分得到的速度
	static double GPSa_h = 0.0, GPSa_h_ = 0.0;	// a_hat for this and last cycle
	static double GPSerr_height_h = 0.0, GPSerr_height_h_ = 0.0;	//
	//需要差分
	
	GPS_h_=GPS_h;
	GPS_h=(*HeightGet);

	diffV = (GPS_h - GPS_h_)/DTIME_INNERLOOP;
	
	GPSerr_height_h_ = GPSerr_height_h;
	GPSerr_height_h = (GPSv_h- diffV);
	
	//
	GPSa_h_ = GPSa_h;
	GPSa_h = -sin(att[1]) * ab[0]
			+ sin(att[0]) * cos(att[1]) * ab[1]
			+ cos(att[0])*cos(att[1]) * ab[2];
	GPSa_h = -(GPSa_h + (GRAVITY + biasGravity));
	//

	GPSv_h_ = GPSv_h;
	GPSv_h	= GPSv_h_ + ((GPSerr_height_h_ + GPSerr_height_h) * 2 * kv[2]) * 0.5 * DTIME_INNERLOOP
			+ (GPSa_h + GPSa_h_)* 0.5 * DTIME_INNERLOOP;

	if(Vheight)
	*Vheight = GPSv_h;
}
