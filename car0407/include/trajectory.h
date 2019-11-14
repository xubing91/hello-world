#ifndef TRAJECTORY_H__
#define TRAJECTORY_H__
#include "filter.h"
#define DTIME_TRAJECTORY 0.0025
struct flypoint{// fly point description in NED frame
	double x;	// x(m)
	double y;	// y(m)
	double z;	// z(m)
	double vx;	// speed of x(m/s)
	double vy; // speed of y(m/s)
	double vz;	// speed of z(m/s)
	double v;	// speed(m/s)
	double loop;// number of loops run
	double round;	// number of rounds turned
	double cx;	// x of the center of circle(m)
	double cy;	// y of the center of circle(m)
	double cz;	// z of the center of circle(m)
	double r;	// radius of the circle(m)
	double sita_ori;	// starting angle(rad)
	unsigned char traj;	// trajectory type
};
#define PI 3.1415927

typedef struct{
	struct filter_2o disp_tanx;
	struct filter_2o disp_tany;
	struct filter_2o disp_tanz;
	struct filter_2o disp_tanvx;
	struct filter_2o disp_tanvy;
	struct filter_2o disp_tanvz;
	struct filter_2o disp_tanax;
	struct filter_2o disp_tanay;
	struct filter_2o disp_tanaz;
	unsigned char bTrajRunning;	// if a trajectory is on the run
	volatile struct flypoint pPrev;
	volatile struct flypoint pNext;
	double	timeNextP;
	double  travelangle;//航向角
}Traj;
void SetInitPoint(Traj* traj,double sita_ori);
void SetNextPoint(Traj* traj, const struct flypoint* point_set, unsigned char option, double time);
void TrackReset(Traj* traj);
//重写TrackReset
void TrackReset_V4(Traj* traj);
void TrackSet(Traj* traj, const double* pos_set);
void GetSetValue(Traj* traj, double* pos_set, double* spd_set, double* acc_set,  double time);
//测试增加变化航向角的程序
void GetSetValue_Yaw(Traj* traj_ab, double* yaw_set, double time);
void Traj_Tangential(double t, double dist, double speed, double accl, double* pos_set, double* spd_set, double* acc_set);


#define	ABSOLUTE_LINE	0
#define	RELATIVE_LINE	1
#define	ABSOLUTE_ARC	2  //Wang_应该是圆弧，对应上面直线
#define HALT			3
//#define RELATIVE_ARC	3
#define TRIANGLE_INV	5
#define	ABSOLUTE_EIGHT	6
#define	TRIANGLE_VAR	7
#define BUTTERFLY       8
#define TRIANGLE_ROUND  9
#define SQURAE_SQURE       10
/*
	Trajectory Code
*/
#define TRAJ_LINE	0
#define	TRAJ_ARC	1
#define	TRAJ_TRIANGLE_INV	2
#define	TRAJ_EIGHT	3
#define TRAJ_TRIANGLE_VAR 4
#define TRAJ_BUTTERFLY 5
#define TRAJ_SQURAE_SQURAE 6
#define TRAJ_GUIDANCE 7

#endif
