#include <math.h>
#include "trajectory.h"
#include "mathex.h"
#include "para.h"
#define fTraj	1
Traj traj_ab = {                               //Wang_可能是绝对
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	0,	// if a trajectory is on the run
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	0,
	0
};
Traj traj_ref = {
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	{{0, 0, 0}, {0, 0, 0}, fTraj},
	0,	// if a trajectory is on the run
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	0
};

extern double pos_zero[3];	// NO GOOD!!
extern int	YawSpinRun;
extern int FlagYawRun;

extern char isTakeOff;//加入这个为了区分飞机在地面
//和在空中的状态，在起飞的阶段，偏航值不应该改变，直接起飞
extern double pos[3];

//飞不同航迹
extern double w_wheel;
double sita_ori = 0;
double length =0;
//double length_x = 0;
//double length_y = 0;
double triangle_x, triangle_y;
void SetInitPoint(Traj* traj,double sita_ori)
{
	if(sita_ori>=0 && sita_ori<=PI/3)//0~60
	{length = traj->pNext.r/2 - (sqrt(3)/6*traj->pNext.r*tan(PI/3 - traj->pNext.sita_ori));}
	else if(traj->pNext.sita_ori>PI/3 && traj->pNext.sita_ori<=2/3*PI)//60~120
	{length = traj->pNext.r/2 + (sqrt(3)/6*traj->pNext.r*tan(traj->pNext.sita_ori - PI/3));}
	else if(traj->pNext.sita_ori>PI*2/3 && traj->pNext.sita_ori<=PI)//120~180
	{length = 3/2*traj->pNext.r - sqrt(3)/6*traj->pNext.r*tan(PI - traj->pNext.sita_ori);}
	else if(traj->pNext.sita_ori>=-PI && traj->pNext.sita_ori<=-PI*2/3)//-120~-180
	{length = traj->pNext.r*3/2 + sqrt(3)/6*traj->pNext.r*tan(traj->pNext.sita_ori + PI);}
	else if(traj->pNext.sita_ori>-PI*2/3 && traj->pNext.sita_ori<=-PI/3)//-60~-120
	{length = 5/2*traj->pNext.r - (sqrt(3)/6*traj->pNext.r*tan(-PI/3 - traj->pNext.sita_ori));}
	else if(traj->pNext.sita_ori>-PI/3 && traj->pNext.sita_ori<0)//-60~0
	{length = 5/2*traj->pNext.r + (sqrt(3)/6*traj->pNext.r*tan(PI/3 + traj->pNext.sita_ori));}
}
void SetNextPoint(Traj* traj, const struct flypoint* point_set, unsigned char option, double time)
{

	switch(option)
	{
	case HALT:
		traj->pNext.x = pos[0];
		traj->pNext.y = pos[1];
	case ABSOLUTE_LINE:
		traj->pPrev.x = traj->pNext.x;
		traj->pPrev.y = traj->pNext.y;
		traj->pNext.x = point_set->x;
		traj->pNext.y = point_set->y;
		//traj->pNext.z  = point_set->z;
		traj->pNext.vx = point_set->vx;
		traj->pNext.vy = point_set->vy;
		//traj->pNext.vz = point_set->vz;
		traj->pNext.r = 0;
		traj->pNext.traj = TRAJ_LINE;	// 没有pNext = *point_set这句！！
		FilterReset_2o(&(traj->disp_tanx));
		FilterReset_2o(&(traj->disp_tany));
		FilterReset_2o(&(traj->disp_tanz));
		break;

	case RELATIVE_LINE:
		traj->pPrev.x = traj->pNext.x;
		traj->pPrev.y = traj->pNext.y;
		traj->pNext.x += point_set->x;
		traj->pNext.y += point_set->y;
		traj->pNext.vx = point_set->vx;
		traj->pNext.vy = point_set->vy;
		traj->pNext.traj = TRAJ_LINE;
		FilterReset_2o(&(traj->disp_tanx));
		FilterReset_2o(&(traj->disp_tany));
		FilterReset_2o(&(traj->disp_tanz));
		break;

	case ABSOLUTE_ARC:
		traj->pPrev.x = traj->pNext.x;
		traj->pPrev.y = traj->pNext.y;
		traj->pNext = *point_set;
//		traj->pNext.sita_ori = atan2(traj->pPrev.x - traj->pNext.cx, traj->pPrev.y - traj->pNext.cy)*180/PI;
		traj->pNext.x = traj->pNext.cx + traj->pNext.r*sin(traj->pNext.sita_ori + traj->pNext.loop*2*PI);//这个是终点
		traj->pNext.y = traj->pNext.cy + traj->pNext.r*cos(traj->pNext.sita_ori + traj->pNext.loop*2*PI);
		FilterReset_2o(&(traj->disp_tanx));
		FilterReset_2o(&(traj->disp_tany));
		FilterReset_2o(&(traj->disp_tanz));
		break;

	case ABSOLUTE_EIGHT:
		traj->pPrev = traj->pNext;
		traj->pNext = *point_set;
		
		// pNext.sita_ori has already been set
		traj->pNext.cy = traj->pNext.cy + traj->pNext.r;
		traj->pNext.x = traj->pNext.cx + traj->pNext.r * sin(traj->pNext.sita_ori + traj->pNext.loop*4*PI);
		traj->pNext.y = traj->pNext.cy + traj->pNext.r * signcyc(traj->pNext.sita_ori + traj->pNext.loop*4*PI) * (cos(traj->pNext.sita_ori + traj->pNext.loop*4*PI)-1);

		FilterReset_2o(&(traj->disp_tanx));
		FilterReset_2o(&(traj->disp_tany));
		FilterReset_2o(&(traj->disp_tanz));
		break;

	case BUTTERFLY:
		traj->pPrev = traj->pNext;
		traj->pNext = *point_set;
		// pNext.r: set by ground station
		//traj->pNext.cx += traj->pPrev.x;
		//traj->pNext.cy += traj->pPrev.y;
		//traj->pNext.z += traj->pPrev.z;
		traj->pNext.sita_ori = 0;	// always start from center (?)
		traj->pNext.x = traj->pNext.cx + traj->pNext.r*sin(traj->pNext.loop*2*PI);	// ending point . starting point for next trajectory
		traj->pNext.y = traj->pNext.cy + traj->pNext.r*sin(traj->pNext.loop*4*PI);
		FilterReset_2o(&(traj->disp_tanx));
		FilterReset_2o(&(traj->disp_tany));
		FilterReset_2o(&(traj->disp_tanz));
		break;

	default:
		break;
	}
}
//void SetNextPoint(Traj* traj, const struct flypoint* point_set, unsigned char option, double time)
//{
//
//	switch(option)
//	{
//	case ABSOLUTE_LINE:
//		traj->pPrev.x = traj->pNext.x;
//		traj->pPrev.y = traj->pNext.y;
//		traj->pNext.x = point_set->x;
//		traj->pNext.y = point_set->y;
//		//traj->pNext.z  = point_set->z;
//		traj->pNext.vx = point_set->vx;
//		traj->pNext.vy = point_set->vy;
//		//traj->pNext.vz = point_set->vz;
//		traj->pNext.r = 0;
//		traj->pNext.traj = TRAJ_LINE;	// 没有pNext = *point_set这句！！
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		break;
//
//	case RELATIVE_LINE:
//		traj->pPrev.x = traj->pNext.x;
//		traj->pPrev.y = traj->pNext.y;
//		traj->pNext.x += point_set->x;
//		traj->pNext.y += point_set->y;
//		traj->pNext.vx = point_set->vx;
//		traj->pNext.vy = point_set->vy;
//		traj->pNext.traj = TRAJ_LINE;
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		break;
//
//	case RELATIVE_ARC:
//		traj->pPrev.x = traj->pNext.x;
//	    traj->pPrev.y = traj->pNext.y;
//		//traj->pPrev = traj->pNext;
//		traj->pNext = *point_set;
//		// pNext.r: set by ground station
//		traj->pNext.cx += traj->pPrev.x;
//		traj->pNext.cy += traj->pPrev.y;
//		//traj->pNext.z += traj->pPrev.z;
//		traj->pNext.sita_ori = atan2(traj->pPrev.x - traj->pNext.cx, traj->pPrev.y - traj->pNext.cy)*180/PI;
//		traj->pNext.x = traj->pNext.cx + traj->pNext.r*sin((traj->pNext.sita_ori + traj->pNext.loop*360)*PI/180);//这个是终点
//		traj->pNext.y = traj->pNext.cy + traj->pNext.r*cos((traj->pNext.sita_ori + traj->pNext.loop*360)*PI/180);
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		//w_wheel = traj->pNext.v/traj->pNext.r;
//		break;
//
//	case ABSOLUTE_ARC:
//		traj->pPrev.x = traj->pNext.x;
//		traj->pPrev.y = traj->pNext.y;
//		traj->pNext = *point_set;
////		traj->pNext.sita_ori = atan2(traj->pPrev.x - traj->pNext.cx, traj->pPrev.y - traj->pNext.cy)*180/PI;
//		traj->pNext.x = traj->pNext.cx + traj->pNext.r*sin(traj->pNext.sita_ori + traj->pNext.loop*2*PI);//这个是终点
//		traj->pNext.y = traj->pNext.cy + traj->pNext.r*cos(traj->pNext.sita_ori + traj->pNext.loop*2*PI);
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		break;
//
//	case TRIANGLE_INV:
//		traj->pPrev = traj->pNext;
//		traj->pNext = *point_set;
//#if defined(CAR_BLACK)
//		triangle_x = traj->pNext.cx - traj->pNext.z/2;
//		triangle_y = traj->pNext.cy + traj->pNext.z/2;
//#elif defined(CAR_RED)
//		triangle_x = traj->pNext.cx + traj->pNext.z/2;
//		triangle_y = traj->pNext.cy + traj->pNext.z/2;
//#elif defined(CAR_YELLOW)
//		triangle_x = traj->pNext.cx + traj->pNext.z/2;
//		triangle_y = traj->pNext.cy - traj->pNext.z/2;
//#elif defined(CAR_BLUE)
//		triangle_x = traj->pNext.cx - traj->pNext.z/2;
//		triangle_y = traj->pNext.cy - traj->pNext.z/2;
//#endif
//		traj->pNext.x = triangle_x;
//		traj->pNext.y = triangle_y;
//				// pNext.r: set by ground station
//		/*
//#if defined(CAR_BLACK)
//		triangle_x = traj->pNext.cx;
//		triangle_y = traj->pNext.cy + traj->pNext.z*sqrt(3)/3;
//#elif defined(CAR_YELLOW)
//		triangle_x = traj->pNext.cx - traj->pNext.z*0.5;
//		triangle_y = traj->pNext.cy - traj->pNext.z*sqrt(3)/6;
//#elif defined(CAR_RED)
//		triangle_x = traj->pNext.cx + traj->pNext.z*0.5;
//		triangle_y = traj->pNext.cy - traj->pNext.z*sqrt(3)/6;
//#endif
//*/
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		break;
//
//	case TRIANGLE_ROUND:
//		traj->pPrev = traj->pNext;
//		traj->pNext = *point_set;
//		traj->pNext.cx = traj->pNext.cx;//队形中线终点
//		traj->pNext.cy = traj->pNext.cy - traj->pNext.r;
//#if defined(CAR_BLACK)
//		traj->pNext.cx -= traj->pNext.z/2;
//		traj->pNext.cy += traj->pNext.z/2;
//#elif defined(CAR_RED)
//		traj->pNext.cx += traj->pNext.z/2;
//		traj->pNext.cy += traj->pNext.z/2;
//#elif defined(CAR_YELLOW)
//		traj->pNext.cx += traj->pNext.z/2;
//		traj->pNext.cy -= traj->pNext.z/2;
//#elif defined(CAR_BLUE)
//		traj->pNext.cx -= traj->pNext.z/2;
//		traj->pNext.cy -= traj->pNext.z/2;
//#endif
//		traj->pNext.x = traj->pNext.cx;
//		traj->pNext.y = traj->pNext.cy + traj->pNext.r;
//		/*
//		traj->pPrev = traj->pNext;
//		traj->pNext = *point_set;
//		traj->pNext.sita_ori = 0;
//#if defined(CAR_BLACK)
//		traj->pNext.cx += 0;
//		traj->pNext.cy = traj->pNext.cy - traj->pNext.r + traj->pNext.z*sqrt(3)/3  ;
//#elif defined(CAR_YELLOW)
//		traj->pNext.cx -= traj->pNext.z*0.5;
//		traj->pNext.cy = traj->pNext.cy - traj->pNext.r - traj->pNext.z*sqrt(3)/6;
//#elif defined(CAR_RED)
//		traj->pNext.cx += traj->pNext.z*0.5;
//		traj->pNext.cy  = traj->pNext.cy - traj->pNext.r - traj->pNext.z*sqrt(3)/6;
//#endif
//*/
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		break;
//
//	case ABSOLUTE_EIGHT:
//		traj->pPrev = traj->pNext;
//		traj->pNext = *point_set;
//		// pNext.r: set by ground station
//
//		//- given absolute position in navigation framework
//		//traj->pNext.cx -= pos_zero[0];
//		//traj->pNext.cy -= pos_zero[1];
//		//traj->pNext.z -= pos_zero[2];
//
//		// pNext.sita_ori has already been set
//		traj->pNext.cy = traj->pNext.cy + traj->pNext.r;
//		traj->pNext.x = traj->pNext.cx + traj->pNext.r * sin(traj->pNext.sita_ori + traj->pNext.loop*4*PI);
//		traj->pNext.y = traj->pNext.cy + traj->pNext.r * signcyc(traj->pNext.sita_ori + traj->pNext.loop*4*PI) * (cos(traj->pNext.sita_ori + traj->pNext.loop*4*PI)-1);
//
//		// pNext.z set
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		break;
//
//	case TRIANGLE_VAR:
//		/*
//		traj->pPrev.x = traj->pNext.x;
//	    traj->pPrev.y = traj->pNext.y;
//		traj->pNext  = *point_set;
//
//		// given absolute position in navigation framework
//		traj->pNext.sita_ori = atan2(traj->pPrev.x - traj->pNext.cx, traj->pPrev.y - traj->pNext.cy);
//		sita_ori = traj->pNext.sita_ori;
//		SetInitPoint(traj,sita_ori);//计算出来的是顺时针的长度
//		traj->pNext.sita_ori = traj->pNext.sita_ori + (traj->pNext.loop - (int)(traj->pNext.loop))*2*PI;
//		if(traj->pNext.sita_ori>PI && traj->pNext.sita_ori<=3*PI)
//		{traj->pNext.sita_ori = traj->pNext.sita_ori - 2*PI;}
//		else if(traj->pNext.sita_ori<-PI && traj->pNext.sita_ori>=-3*PI)
//		{traj->pNext.sita_ori = traj->pNext.sita_ori + 2*PI;}
//		if(traj->pNext.sita_ori>=0 && traj->pNext.sita_ori<=PI/3)//0~60
//		{traj->pNext.x = traj->pNext.cx + traj->pNext.r/2 - (sqrt(3)/6*traj->pNext.r*tan(PI/3 - traj->pNext.sita_ori) + traj->pNext.r/2)*cos(PI/3);
//		 traj->pNext.y = traj->pNext.cy + sqrt(3)/3*traj->pNext.r - (traj->pNext.r/2 - sqrt(3)/6*traj->pNext.r*tan(PI/3 - traj->pNext.sita_ori))*cos(PI/6);}
//		else if(traj->pNext.sita_ori>PI/3 && traj->pNext.sita_ori<=2/3*PI)//60~120
//		{traj->pNext.x = traj->pNext.cx + traj->pNext.r/2 - (traj->pNext.r/2 - sqrt(3)/6*traj->pNext.r*tan(traj->pNext.sita_ori - PI/3))*cos(PI/3);
//		 traj->pNext.y = traj->pNext.cy + sqrt(3)/3*traj->pNext.r - (traj->pNext.r/2 + sqrt(3)/6*traj->pNext.r*tan(traj->pNext.sita_ori - PI/3))*cos(PI/6);
//		}
//		else if(traj->pNext.sita_ori>PI*2/3 && traj->pNext.sita_ori<=PI)//120~180
//		{traj->pNext.x = traj->pNext.cx + sqrt(3)/6*traj->pNext.r*tan(PI - traj->pNext.sita_ori);
//		 traj->pNext.y = traj->pNext.cy - sqrt(3)/6*traj->pNext.r;
//		}
//		else if(traj->pNext.sita_ori>=-PI && traj->pNext.sita_ori<=-PI*2/3)//-120~-180
//		{
//			traj->pNext.x = traj->pNext.cx - sqrt(3)/6*traj->pNext.r*tan(traj->pNext.sita_ori + PI);
//			traj->pNext.y = traj->pNext.cy - sqrt(3)/6*traj->pNext.r;
//		}
//		else if(traj->pNext.sita_ori>-PI*2/3 && traj->pNext.sita_ori<=-PI/3)//-60~-120
//		{
//			traj->pNext.x = traj->pNext.cx - (sqrt(3)/6*traj->pNext.r*tan(-PI/3 - traj->pNext.sita_ori) + traj->pNext.r/2)*sin(PI/6);
//		    traj->pNext.y = traj->pNext.cy + sqrt(3)/3*traj->pNext.r - (sqrt(3)/6*traj->pNext.r*tan(-PI/3 - traj->pNext.sita_ori) + traj->pNext.r/2)*cos(PI/6);
//		}
//		else if(traj->pNext.sita_ori>-PI/3 && traj->pNext.sita_ori<0)//-60~0
//		{
//			traj->pNext.x = traj->pNext.cx - (traj->pNext.r/2 - sqrt(3)/6*traj->pNext.r*tan(PI/3 + traj->pNext.sita_ori))*sin(PI/6);
//			traj->pNext.y = traj->pNext.cy + sqrt(3)/3*traj->pNext.r - (traj->pNext.r/2 - sqrt(3)/6*traj->pNext.r*tan(PI/3 + traj->pNext.sita_ori))*cos(PI/6);
//		}
//		//traj->pNext.x = traj->pNext.cx + traj->pNext.r*cos((traj->pNext.sita_ori + traj->pNext.loop*360)*PI/180);//这个是终点
//		//traj->pNext.y = traj->pNext.cy + traj->pNext.r*sin((traj->pNext.sita_ori + traj->pNext.loop*360)*PI/180);
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		FilterReset_2o(&(traj->disp_tanz));
//		*/
//		break;
//
//	case BUTTERFLY:
//		traj->pPrev = traj->pNext;
//		traj->pNext = *point_set;
//		// pNext.r: set by ground station
//		//traj->pNext.cx += traj->pPrev.x;
//		//traj->pNext.cy += traj->pPrev.y;
//		//traj->pNext.z += traj->pPrev.z;
//		traj->pNext.sita_ori = 0;	// always start from center (?)
//		traj->pNext.x = traj->pNext.cx + traj->pNext.r*sin(traj->pNext.loop*2*PI);	// ending point . starting point for next trajectory
//		traj->pNext.y = traj->pNext.cy + traj->pNext.r*sin(traj->pNext.loop*4*PI);
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
//		break;
//
//	case SQURAE_SQURE:
//		traj->pPrev = traj->pNext;
//		traj->pNext = *point_set;
//#if defined(CAR_RED)
//		traj->pNext.y +=  traj->pNext.r*7;
//#elif defined(CAR_BLACK)
//		traj->pNext.y +=  traj->pNext.r*5;
//#elif defined(CAR_YELLOW)
//		traj->pNext.y +=  traj->pNext.r*1;
//#elif defined(CAR_BLUE)
//		traj->pNext.y +=  traj->pNext.r*0;
//#elif defined(CAR_ORANGE)
//		traj->pNext.y +=  traj->pNext.r*2;
//#elif defined(CAR_WHITE)
//		traj->pNext.y +=  traj->pNext.r*6;
//#elif defined(CAR_PURPLE)
//#endif
//		FilterReset_2o(&(traj->disp_tanx));
//		FilterReset_2o(&(traj->disp_tany));
//		FilterReset_2o(&(traj->disp_tanz));
////		traj->pPrev = traj->pNext;
////		traj->pNext = *point_set;
////		/*暂且给定的圈数为整数圈，起始角度为零度*/
////		/*队形的话按照正方形跑，航迹按照菱形跑*/
////		traj->pNext.cx = traj->pNext.cx;//队形中线终点
////		traj->pNext.cy = traj->pNext.cy - traj->pNext.r*sqrt(2)/2;
////#if defined(CAR_BLACK)
////		traj->pNext.cx -= traj->pNext.z/2;
////		traj->pNext.cy += traj->pNext.z/2;
////#elif defined(CAR_RED)
////		traj->pNext.cx += traj->pNext.z/2;
////		traj->pNext.cy += traj->pNext.z/2;
////#elif defined(CAR_YELLOW)
////		traj->pNext.cx += traj->pNext.z/2;
////		traj->pNext.cy -= traj->pNext.z/2;
////#elif defined(CAR_BLUE)
////		traj->pNext.cx -= traj->pNext.z/2;
////		traj->pNext.cy -= traj->pNext.z/2;
////#endif
////		traj->pNext.x = traj->pNext.cx;
////		traj->pNext.y = traj->pNext.cy + traj->pNext.r*sqrt(2)/2;
////		FilterReset_2o(&(traj->disp_tanx));
////		FilterReset_2o(&(traj->disp_tany));
////		FilterReset_2o(&(traj->disp_tanz));
//		break;
//	default:
//		break;
//	}
//}
void TrackReset_V4(Traj* traj)
{
	
	traj->pPrev.x = 0;
	traj->pPrev.y = 0; 
	traj->pPrev.z = 0;
	traj->pPrev.vx = 0;
	traj->pPrev.vy = 0;
	traj->pPrev.vz = 0;
	traj->pPrev.r = 0;
	traj->pPrev.traj = TRAJ_LINE;
	traj->pPrev.loop = 0;
	traj->pNext = traj->pPrev;
	traj->bTrajRunning = 0;
	traj->travelangle = 0;
	YawSpinRun = 0 ;
	//这样它会有一个平滑过渡
	
	FilterReset_2o(&(traj->disp_tanx));
	FilterReset_2o(&(traj->disp_tany));
	FilterReset_2o(&(traj->disp_tanz));
/*
	FilterReset_2o(&(traj->disp_tanvx));
	FilterReset_2o(&(traj->disp_tanvy));
	FilterReset_2o(&(traj->disp_tanvz));
	FilterReset_2o(&(traj->disp_tanax));
	FilterReset_2o(&(traj->disp_tanay));
	FilterReset_2o(&(traj->disp_tanaz));
*/
	
}
void TrackReset(Traj* traj)
{
	
	traj->pPrev.x = 0;
	traj->pPrev.y = 0; 
	traj->pPrev.z = 0;
	traj->pPrev.vx = 0;
	traj->pPrev.vy = 0;
	traj->pPrev.vz = 0;
	traj->pPrev.r = 0;
	traj->pPrev.traj = TRAJ_LINE;
	traj->pPrev.loop = 0;
	traj->pNext = traj->pPrev;
	traj->bTrajRunning = 0;
	traj->travelangle = 0;
	YawSpinRun = 0 ;
	FilterReset_2o(&(traj->disp_tanx));
	FilterReset_2o(&(traj->disp_tany));
	FilterReset_2o(&(traj->disp_tanz));
	FilterReset_2o(&(traj->disp_tanvx));
	FilterReset_2o(&(traj->disp_tanvy));
	FilterReset_2o(&(traj->disp_tanvz));
	FilterReset_2o(&(traj->disp_tanax));
	FilterReset_2o(&(traj->disp_tanay));
	FilterReset_2o(&(traj->disp_tanaz));
}

void TrackSet(Traj* traj, const double* pos_set)
{
	traj->pPrev.x = pos_set[0];
	traj->pPrev.y = pos_set[1]; 
	traj->pPrev.z = pos_set[2];
	traj->pPrev.vx = 0;
	traj->pPrev.vy = 0;
	traj->pPrev.vz = 0;
	traj->pPrev.r = 0;
	traj->pPrev.traj = TRAJ_LINE;
	traj->pPrev.loop = 0;
	traj->pNext = traj->pPrev;
	FilterReset_2o(&(traj->disp_tanx));
	FilterReset_2o(&(traj->disp_tany));
	FilterReset_2o(&(traj->disp_tanz));
	FilterReset_2o(&(traj->disp_tanvx));
	FilterReset_2o(&(traj->disp_tanvy));
	FilterReset_2o(&(traj->disp_tanvz));
	FilterReset_2o(&(traj->disp_tanax));
	FilterReset_2o(&(traj->disp_tanay));
	FilterReset_2o(&(traj->disp_tanaz));
}

// Abstract: generate tangential trajectory。梯形速度
// Input: time, distance, max speed, max acceleration
// Output: position, speed (both tangential)
void Traj_Tangential(double t, double dist, double speed, double accl, double* pos_set, double* spd_set, double* acc_set)
{
	static double vm;

	// t>0, dist>=0, speed>=0, accl>=0
	if(speed * speed < accl * dist)	// 可以加到极速
	{
		if(t * accl < speed)	// 加速阶段
		{
			*pos_set = 0.5 * accl * t * t;
			*spd_set = accl * t;
			*acc_set = accl;
		}
		else if(dist > t * speed)	// 匀速阶段
		{
			*pos_set = speed * (t - 0.5*speed/accl);
			*spd_set = speed;
			*acc_set = 0;
		}
		else if(dist > (t-speed/accl) * speed)	// 减速阶段
		{
			*pos_set = dist - 0.5 * accl * sqr(dist/speed + speed/accl - t);
			*spd_set = speed - accl * (t - dist/speed);
			*acc_set = -accl;
		}
		else
		{
			*pos_set = dist;
			*spd_set = 0;
			*acc_set = 0;
		}
	}
	else	// 不能加到极速
	{
		if(0.5 * accl * t * t < 0.5 * dist)	// 加速阶段
		{
			*pos_set = 0.5 * accl * t * t;
			*spd_set = accl * t;
			*acc_set = accl;
		}
		else if(accl * t * 0.25 * t < dist)	// 减速阶段
		{
			vm = sqrt(accl * dist);
			*pos_set = dist - 0.5 * accl * sqr(2*vm/accl - t);
			*spd_set = vm - accl * (t-vm/accl);
			*acc_set = -accl;
		}
		else	// 停止阶段
		{
			*pos_set = dist;
			*spd_set = 0;
			*acc_set = 0;
		}
	}
}

#define MAX_ACCEL	1.5
#define MAX_ACCELZ	1.2

//按照一圈转12s计算，则每一个周期可以转动0.001314弧度
const double minyawstep=0.00043633;
//按照0.5角度设计误差
const double errYaw=0.00873;


//获取偏航角
extern double vel[3];
extern double acc[3];
extern double pos[3];
void GetSetValue_Yaw(Traj* traj_ab, double* yaw_set, double time)
{
//	double t = time - traj_ab->timeNextP;
	
	//下面只是按照直线的来写

	if(fabs((*yaw_set)-traj_ab->travelangle)>errYaw)
	{
	*yaw_set =  PsaiMdf(*yaw_set + minyawstep);
	}
	else//执行完毕
	{
	traj_ab->timeNextP = time;
	YawSpinRun =0;
	}
	
}

//extern int FlagHalt;//表示航迹中断指令为0，这个时候对于它的pos_set,spd_set以及acc_set需要一定的改变



//走不同轨迹的位移 速度 加速度取值
double disp, spd, acce, sita_t;	// relative displacement
double vmax;
double t_Traj=0;
extern double spd_sett_[3];
extern double kv;
double w_sin = 0;
void GetSetValue(Traj* traj, double* pos_set, double* spd_set, double* acc_set,  double time)
{
	spd_sett_[0]=spd_set[0];
	spd_sett_[1]=spd_set[1];
	t_Traj = time;
	int dir = 0;
	if(traj->pNext.traj == TRAJ_LINE)
	{
        //- X
		Traj_Tangential(t_Traj, fabs(traj->pNext.x - traj->pPrev.x), traj->pNext.vx, MAX_ACCEL, &disp, &spd, &acce);
		disp = FilterRun_2o(&(traj->disp_tanx), disp);
		spd = FilterRun_2o(&(traj->disp_tanvx), spd);
		acce = FilterRun_2o(&(traj->disp_tanax), acce);
		dir = sign(traj->pNext.x - traj->pPrev.x);
		pos_set[0] = dir * disp + traj->pPrev.x;
		spd_set[0] = dir * spd;
		acc_set[0] = dir * acce;
		// - Y
		Traj_Tangential(t_Traj, fabs(traj->pNext.y - traj->pPrev.y), traj->pNext.vy, MAX_ACCEL, &disp, &spd, &acce);
		disp = FilterRun_2o(&(traj->disp_tany), disp);
		spd = FilterRun_2o(&(traj->disp_tanvy), spd);
		acce = FilterRun_2o(&(traj->disp_tanay), acce);
		dir = sign(traj->pNext.y - traj->pPrev.y);
		pos_set[1] = dir * disp + traj->pPrev.y;
		spd_set[1] = dir * spd;
		acc_set[1] = dir * acce;

	}
	else if(traj->pNext.traj == TRAJ_ARC)	// pNext.r > 0
	{
	// 限制径向速度。理论上应该轴向加速度和径向加速度总和受限，此处相当于轴向加速度和径向加速度分别受限
	if(sqr(traj->pNext.v) < MAX_ACCEL * traj->pNext.r)
		vmax = traj->pNext.v;
	else
		vmax = sqrt(MAX_ACCEL * traj->pNext.r);


	dir = sign(traj->pNext.loop);
	    Traj_Tangential(t_Traj, 2*PI*traj->pNext.r*fabs(traj->pNext.loop), vmax, MAX_ACCEL, &disp, &spd, &acce);
		disp = FilterRun_2o(&(traj->disp_tanx), disp);
		spd = FilterRun_2o(&(traj->disp_tanvx), spd);
		acce = FilterRun_2o(&(traj->disp_tanax), acce);

		sita_t = traj->pNext.sita_ori + dir * disp / traj->pNext.r;
		pos_set[0] = traj->pNext.cx + traj->pNext.r * sin(sita_t);
		spd_set[0] = dir * spd * cos(sita_t);
		acc_set[0] = dir * acce * cos(sita_t) + spd * spd / traj->pNext.r * sin(sita_t);//这个向心加速度该有么？？

		pos_set[1] = traj->pNext.cy + traj->pNext.r * cos(sita_t);
		spd_set[1] = - dir * spd * sin(sita_t);
		acc_set[1] = - dir * acce * sin(sita_t) + spd * spd / traj->pNext.r * cos(sita_t);
		if(fabs(traj->pNext.r * sin(sita_t))<=(traj->pNext.r*sin(PI/6)))
		{
			kv = 1.12;
		}
		else if((fabs(traj->pNext.r * sin(sita_t))>(traj->pNext.r*sin(PI/6))) && ((fabs(traj->pNext.r * sin(sita_t))<(traj->pNext.r*sin(PI/4)))))
		{
			kv = 1.14;
		}
		else if(fabs(traj->pNext.r * sin(sita_t))>(traj->pNext.r*sin(PI/3)))
		{
			kv = 1.23;
		}
		else
		{
			kv = 1;
		}
	}

	else if(traj->pNext.traj == TRAJ_EIGHT)
	{
		// 限制径向速度
		if(sqr(traj->pNext.v) < MAX_ACCEL * (traj->pNext.r))
			vmax = traj->pNext.v;
		else
			vmax = sqrt(MAX_ACCEL * (traj->pNext.r));

		dir = sign(traj->pNext.loop);
		Traj_Tangential(t_Traj, 4*PI*(traj->pNext.r)*fabs(traj->pNext.loop), vmax, MAX_ACCEL, &disp, &spd, &acce);	// 4PiR (2loop loops)
		disp = FilterRun_2o(&(traj->disp_tanx), disp);
		spd = FilterRun_2o(&(traj->disp_tanvx), spd);
		acce = FilterRun_2o(&(traj->disp_tanax), acce);
		sita_t = traj->pNext.sita_ori + dir * disp / traj->pNext.r;
		pos_set[0] = traj->pNext.cx + traj->pNext.r * sin(sita_t);
		spd_set[0] = dir * spd * cos(sita_t);
		//acc_set[0] = (-dir * acc * sin(sita_t) - spd * spd / traj->pNext.r * cos(sita_t)) * signcyc(sita_t);

		pos_set[1] = traj->pNext.cy + traj->pNext.r * signcyc(sita_t) * (cos(sita_t) - 1);
		spd_set[1] = -dir * spd * sin(sita_t) * signcyc(sita_t);

	}

	else if(traj->pNext.traj == TRAJ_TRIANGLE_INV)	// 8-shape trajectory
	{
		w_sin = traj->pNext.v/traj->pNext.r;
		if(t_Traj<=traj->pNext.loop*2*PI/w_sin)
		{
		pos_set[0] = triangle_x;
		pos_set[1] = triangle_y + traj->pNext.r*sin(t_Traj*w_sin);
		spd_set[0] = 0;
		spd_set[1] = traj->pNext.v*cos(t_Traj*w_sin);
		}
		else
		{
			pos_set[0] = triangle_x;
			pos_set[1] = triangle_y + traj->pNext.r*sin(traj->pNext.loop*2*PI);
			spd_set[0] = 0;
			spd_set[1] = 0;
		}

	}
	else if(traj->pNext.traj == TRAJ_GUIDANCE)
	{
		;
	}

/*
	else if(traj->pNext.traj == TRAJ_TRIANGLE_VAR)
	{
		dir = sign(traj->pNext.loop);
		acc_set[0] = 0;
		acc_set[1] = 0;
		if((3*traj->pNext.r*traj->pNext.loop - traj->pNext.v*t_Traj)>=1e-2)
		{
	if(dir>0)
	{   disp = fmod((traj->pNext.v*t_Traj + length),traj->pNext.r*3);

		if(disp>=0 && disp<=traj->pNext.r)
		{
		    pos_set[0] = traj->pNext.cx + disp*sin(PI/6);
		    pos_set[1] = traj->pNext.cy + sqrt(3)/3*traj->pNext.r - disp*cos(PI/6);
		    spd_set[0] = traj->pNext.v*cos(PI/3);
		    spd_set[1] = -traj->pNext.v*sin(PI/3);
		}
		else if(disp>traj->pNext.r && disp<=2*traj->pNext.r)
		{
			pos_set[0] = traj->pNext.cx + traj->pNext.r/2 - (disp - traj->pNext.r);
			pos_set[1] = traj->pNext.cy - sqrt(3)/6*traj->pNext.r;
			spd_set[0] = -traj->pNext.v;
			spd_set[1] = 0;
		}
		else if(disp>2*traj->pNext.r && disp<=3*traj->pNext.r)
		{
			pos_set[0] = traj->pNext.cx - traj->pNext.r/2 + (disp - traj->pNext.r*2)*cos(PI/3);
			pos_set[1] = traj->pNext.cy - sqrt(3)/6*traj->pNext.r + (disp - traj->pNext.r*2)*cos(PI/6);
			spd_set[0] = traj->pNext.v*cos(PI/3);
			spd_set[1] = traj->pNext.v*sin(PI/3);
		}
	}
		else if(dir<0)
		{
			disp = fmod((3*traj->pNext.r - length + traj->pNext.v*t_Traj),traj->pNext.r*3);
			if(disp>=0 && disp<=traj->pNext.r)
			{
				pos_set[0] = traj->pNext.cx - disp*sin(PI/6);
				pos_set[1] = traj->pNext.cy + sqrt(3)/3*traj->pNext.r - disp*sin(PI/3);
				spd_set[0] = -traj->pNext.v*cos(PI/3);
				spd_set[1] = -traj->pNext.v*sin(PI/3);
			}
			else if(disp>traj->pNext.r && disp<=2*traj->pNext.r)
		    {
				pos_set[0] = traj->pNext.cx - traj->pNext.r/2 + (disp - traj->pNext.r);
				pos_set[1] = traj->pNext.cy - sqrt(3)/6*traj->pNext.r;
				spd_set[0] = traj->pNext.v;
				spd_set[1] = 0;
		    }
			else if(disp>2*traj->pNext.r && disp<=3*traj->pNext.r)
		    {
				pos_set[0] = traj->pNext.cx + traj->pNext.r/2 - (disp - traj->pNext.r*2)*cos(PI/3);
				pos_set[1] = traj->pNext.cy - sqrt(3)/6*traj->pNext.r + (disp - traj->pNext.r*2)*cos(PI/6);
				spd_set[0] = -traj->pNext.v*cos(PI/3);
				spd_set[1] = traj->pNext.v*sin(PI/3);
		    }
		}
		}
		else
		{
			pos_set[0] = traj->pNext.x;
		    pos_set[1] = traj->pNext.y;
		    spd_set[0] = 0;
		    spd_set[1] = 0;
		}
	}
	*/
	else if(traj->pNext.traj == TRAJ_BUTTERFLY)
	{
		if(sqr(traj->pNext.v) < MAX_ACCEL * traj->pNext.r)
			vmax = traj->pNext.v;
		else
			vmax = sqrt(MAX_ACCEL * traj->pNext.r);

		dir = sign(traj->pNext.loop);
		Traj_Tangential(t_Traj, 2*PI*(traj->pNext.r)*fabs(traj->pNext.loop), vmax/2, MAX_ACCEL/4, &disp, &spd, &acce);	// 4PiR
		disp = FilterRun_2o(&(traj->disp_tanx), disp);
		spd = FilterRun_2o(&(traj->disp_tanvx), spd);
		acce = FilterRun_2o(&(traj->disp_tanax), acce);
		sita_t = traj->pNext.sita_ori + dir * disp / traj->pNext.r;
		pos_set[0] = traj->pNext.cx + traj->pNext.r * sin(sita_t);
		spd_set[0] = dir * spd * cos(sita_t);
		//acc_set[0] = dir * acc * cos(sita_t) - spd * spd / traj->pNext.r * sin(sita_t);
		pos_set[1] = traj->pNext.cy + traj->pNext.r * sin(sita_t * 2);
		spd_set[1] = dir * 2 * spd * cos(sita_t * 2);

	}

	else if(traj->pNext.traj == TRAJ_SQURAE_SQURAE)
	{

//		double duty_length = 0;
//		vmax = traj->pNext.v;
//
//		dir = sign(traj->pNext.loop);
//		Traj_Tangential(t_Traj, 4*(traj->pNext.r)*fabs(traj->pNext.loop), vmax, MAX_ACCEL, &disp, &spd, &acce);	// 4PiR
//		disp = FilterRun_2o(&(traj->disp_tanx), disp);
//		spd = FilterRun_2o(&(traj->disp_tanvx), spd);
//		acce = FilterRun_2o(&(traj->disp_tanax), acce);
//		duty_length = fmod(disp,4*(traj->pNext.r))/traj->pNext.r;
//		if (duty_length>=0 && duty_length<1)
//		{
//			pos_set[0] = traj->pNext.cx + dir*traj->pNext.r*duty_length*cos(PI/4);
//			pos_set[1] = traj->pNext.cy + traj->pNext.r*(1 - duty_length)*cos(PI/4);
//			spd_set[0] = dir*spd*cos(PI/4);
//			spd_set[1] = -spd*cos(PI/4);
//		}
//		else if (duty_length>=1 && duty_length<2)
//		{
//			pos_set[0] = traj->pNext.cx + dir*traj->pNext.r*(2 - duty_length)*cos(PI/4);
//			pos_set[1] = traj->pNext.cy - traj->pNext.r*(duty_length - 1)*cos(PI/4);
//			spd_set[0] = -dir*spd*cos(PI/4);
//			spd_set[1] = -spd*cos(PI/4);
//		}
//		else if (duty_length>=2 && duty_length<3)
//		{
//			pos_set[0] = traj->pNext.cx - dir*traj->pNext.r*(duty_length - 2)*cos(PI/4);
//			pos_set[1] = traj->pNext.cy - traj->pNext.r*(3 - duty_length)*cos(PI/4);
//			spd_set[0] = -dir*spd*cos(PI/4);
//			spd_set[1] = spd*cos(PI/4);
//		}
//		else if (duty_length>=3 && duty_length<=4)
//		{
//			pos_set[0] = traj->pNext.cx - dir*traj->pNext.r*(4 - duty_length)*cos(PI/4);
//			pos_set[1] = traj->pNext.cy + traj->pNext.r*(duty_length - 3)*cos(PI/4);
//			spd_set[0] = dir*spd*cos(PI/4);
//			spd_set[1] = spd*cos(PI/4);
//		}
	}

	traj->bTrajRunning = 0;

}
