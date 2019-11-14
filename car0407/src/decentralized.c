#include "decentralized.h"
#include "para.h"
#include "Fml.h"
#include <math.h>
#include "mathex.h"
#include "trajectory.h"



#if defined(XBEE)
Uint32  addrNeighbors[N_NEIGHBORS][2] = ADDRESS_NEIGHBORS;
#else
Uint16	addrNeighbors[N_NEIGHBORS] = ADDRESS_NEIGHBORS;
#endif
#ifdef	DECENT_SCHEME_1


double vel_f[3];  	//速度传感器当前时刻的值
double pos_f[3]; 	//位移传感器当前时刻的值
     
//double vel_set[3];	//给定速度跟踪值
//double pos_set[3];	//给定位移跟踪值
double u[3]	;	//控制律


double pos_hat_r_[3]={0,0,0};		    //检测器中当前时刻位移估计值
double vel_hat_r_[3]={0,0,0};			//检测器中当前时刻速度估计值
double pos_hat_r[3]={0,0,0};			//检测器中下一时刻位移估计值
double vel_hat_r[3]={0,0,0};			//检测器中下一时刻速度估计值 
double res1[3]={0,0,0};				//检测器残差1
double res2[3]={0,0,0};				//检测器残差2


double pos_hat_e_[3]={0,0,0};			//估计器中当前时刻位移估计值
double vel_hat_e_[3]={0,0,0};			//估计器中当前时刻速度估计值
double pos_hat_e[3]={0,0,0};			//估计器中下一时刻位移估计值
double vel_hat_e[3]={0,0,0};			//估计器中下一时刻速度估计值

double vv[3]={0,0,0};				//定义中间变量下一时刻值
double vv_[3]={0,0,0};				//定义中间变量当前时刻值

double f_hat[3]={0,0,0};				//估计器中下一时刻故障估计值
double f_hat_[3]={0,0,0};			//估计器中当前时刻故障估计值


double qlast[2]={0.2,0.2};
double q[2]={0,0};

//double DTIME=0.0025		//采样时间
double	ux = 0, uy = 0;
double	k11 = -1, k12 = 0;//-1.1
double  k21 = 0.23, k22 = 0;
struct filter_pair	uatt_0[2] = {{0, 0}, {0, 0}};
struct filter_pair	uatt_1[2] = {{0, 0}, {0, 0}};
extern double	spd_sett_[3];
extern double   spd_set[3];
double Kp_Decent = 1;
extern Traj traj_ab;
extern double w_wheel;
extern double n[3];
double kv = 1;
double periodT = 0.0025;
extern double Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Ki_x, Ki_y, Ki_z;
//double R_alarm = Kd_z;
//double r_min = Ki_y;
//double R_alarm = 0.8;
//double r_min = 0.45;
double c = 0.9;
double det_x = 0;
double det_y = 0;
//double kpx = 0, ki = 0, kd = 0;//1.6 1.3
extern Cmd cmdR;
double ax1 = -1, ax2 = -1, av = -1;
extern struct flypoint point_set;
#if defined(Consensus_based)
void DecentPosCtrl(double* pos, double* pos_set, const Agent* self, const double* acc_set, double* v_ctrl, double* vel, double *u, const Agent* neighbors,const double* w)
{
    static double lastError_x = 0;
    static double lastError_y = 0;

    static double Int_Error_x = 0;
    static double Int_Error_y = 0;
//   int i=0;


  // ux = k11 * self->errp[0] + k12 * self->errv[0];
  // uy = k11 * self->errp[1] + k12 * self->errv[1];


   double nowError_x, nowError_y, derivError_x, derivError_y;

   nowError_x = -self->errp[0];
   nowError_y = -self->errp[1];

   derivError_x = (nowError_x - lastError_x) / periodT;
   derivError_y = (nowError_y - lastError_y) / periodT;
   lastError_x = nowError_x;
   lastError_y = nowError_y;
   Int_Error_x +=  nowError_x*periodT;
   Int_Error_y +=  nowError_y*periodT;
   ux = Kp_x*nowError_x + Kd_x*derivError_x + Ki_x*Int_Error_x;
   uy = Kp_y*nowError_y + Kd_y*derivError_y + Ki_y*Int_Error_y;


//	for(i = 0; i< N_SWARM; i++)
//	{
//		ux += (Ki_x * neighbors[i].offsetp[0]) * w[i];
//		uy += (Ki_x * neighbors[i].offsetp[1]) * w[i];
//	}

   u[0] = ux;
   u[1] = uy;

   v_ctrl[0] = spd_set[0] + u[0];//spd_set[0] +
   v_ctrl[1] = spd_set[1] + u[1];//spd_set[1] +
   //w_wheel = sqrt(v_ctrl[0]*v_ctrl[0]+v_ctrl[1]*v_ctrl[1])/traj_ab.pNext.r;

}
#elif defined(PID)
void DecentPosCtrl(double* pos, double* pos_set, const Agent* self, const double* acc_set, double* v_ctrl, double* vel, double *u, const Agent* neighbors,const double* w)
{
    static double lastError_x = 0;
    static double lastError_y = 0;

    static double Int_Error_x = 0;
    static double Int_Error_y = 0;


   double nowError_x, nowError_y, derivError_x, derivError_y;
   nowError_x = -self->errp[0];
   nowError_y = -self->errp[1];
   derivError_x = (nowError_x - lastError_x) / periodT;
   derivError_y = (nowError_y - lastError_y) / periodT;
   lastError_x = nowError_x;
   lastError_y = nowError_y;
   Int_Error_x +=  nowError_x*periodT;
   Int_Error_y +=  nowError_y*periodT;
   ux = Kp_x*nowError_x + Kd_x*derivError_x + Ki_x*Int_Error_x;
   uy = Kp_y*nowError_y + Kd_y*derivError_y + Ki_y*Int_Error_y;

   v_ctrl[0] = ux;
   v_ctrl[1] = uy;


}
#elif defined(Adaptive)
double ax1 = -1, ax2 = -1, av = -1;
void DecentPosCtrl(double* pos, double* pos_set, const Agent* self, const double* acc_set, double* v_ctrl, double* vel, double *u, const Agent* neighbors,const double* w)
{
	double tstep = 0.0025;
	static double velocity[2];
	double k2v[2];
	double k3v[2];
	double k4v[2];
	double u_agent[2];

	u_agent[0] = pos_set[0];
	u_agent[1] = pos_set[0];
    ux = ax1*pos[0]+av*vel[0] + u_agent[0];//ax
    uy = ax2*pos[1]+av*vel[1] + u_agent[1];//ay
    // RK2
    k2v[0] = ax1*pos[0]+av*(vel[0]+tstep*ux/2)+u_agent[0];
    k2v[1] = ax2*pos[1]+av*(vel[1]+tstep*uy/2)+u_agent[1];
    // RK3
    k3v[0] = ax1*pos[0]+av*(vel[0]+tstep*k2v[0]/2)+u_agent[0];
    k3v[1] = ax2*pos[1]+av*(vel[1]+tstep*k2v[1]/2)+u_agent[1];
    // RK4
    k4v[0] = ax1*pos[0]+av*(vel[0]+tstep*k3v[0])+u_agent[0];
    k4v[1] = ax2*pos[1]+av*(vel[1]+tstep*k3v[1])+u_agent[1];

    v_ctrl[0] = velocity[0]+tstep*(ux+2*k2v[0]+2*k3v[0]+k4v[0])/6;// vx cmd
    v_ctrl[1] = velocity[1]+tstep*(uy+2*k2v[1]+2*k3v[1]+k4v[1])/6;// vy cmd

    velocity[0] = v_ctrl[0];// speed need to be iterated
    velocity[1] = v_ctrl[1];// speed need to be iterated
}

#endif
//void DecentPosCtrl(const Agent* self, const double* acc_set, double* v_ctrl, double* vel, double *u, const Agent* neighbors,const double* w)
//{
//	unsigned char i = 0;
//
//#if defined(SWARM_FC_LEADER)
//    static double lastError_x = 0;
//    static double lastError_y = 0;
//	   double nowError_x, nowError_y, derivError_x, derivError_y;
//	   nowError_x = -self->errp[0];
//	   nowError_y = -self->errp[1];
//	   derivError_x = (nowError_x - lastError_x) / periodT;
//	   derivError_y = (nowError_y - lastError_y) / periodT;
//	   lastError_x = nowError_x;
//	   lastError_y = nowError_y;
//	   ux = Kp_x*nowError_x + Kd_x*derivError_x;
//	   uy = Kp_y*nowError_y + Kd_y*derivError_y;
//
//	   u[0] = ux;
//	   u[1] = uy;
//
//	   v_ctrl[0] = spd_set[0] + u[0];
//	   v_ctrl[1] = spd_set[1] + u[1];
//
//#elif	defined(SWARM_FC_FOLLOWER)
//	         det_x = 0;
//	         det_y = 0;
//		for(i = 0; i< N_SWARM; i++)
//		{
//			if (neighbors[i].addr != ADDRESS_BLUE)
//			{
//				det_x += neighbors[i].offsetp[0] * w[i];
//				det_y += neighbors[i].offsetp[1] * w[i];
//			}
//			else
//			{
//				det_x += (self->errp[0] - neighbors[i].p_r[0]) *w[i];
//				det_y += (self->errp[1] - neighbors[i].p_r[1]) *w[i];
//			}
//		}
//		double f_detx = 0;
//		double f_dety = 0;
//		f_detx = (fabs(det_x)>Ki_z ? det_x/fabs(det_x):det_x/Ki_z);
//		f_dety = (fabs(det_y)>Ki_z ? det_y/fabs(det_y):det_y/Ki_z);
//
//		ux = -Kp_x*det_x - Ki_x*f_detx;
//		uy = -Kp_y*det_y - Ki_y*f_dety;
//
//	    u[0] = ux;
//	    u[1] = uy;
//
//	    v_ctrl[0] = Kd_z*spd_set[0] + u[0];
//	    v_ctrl[1] = Kd_z*spd_set[1] + u[1];
//#endif
//
//}

void	DecentPosCtrlReset(const double* att_set)
{
	uatt_0[0].x = att_set[0];
	uatt_1[0].x = att_set[0];
	uatt_0[0].x_ = att_set[0];
	uatt_1[0].x_ = att_set[0];
	uatt_0[1].x = att_set[1];
	uatt_1[1].x = att_set[1];
	uatt_0[1].x_ = att_set[1];
	uatt_1[1].x_ = att_set[1];
}




//计时器达到超时时间，则判定不再是其邻居，如果收到信息在别的函数中会对计时器清零Wang
int16	tmCom[N_SWARM];	// timer for last communication from every potential neighbor
void	ComTimerRun(double* w)
{
	unsigned char i;
	for(i=0; i<N_SWARM; i++)
	{
		if(w[i] > 0)	// considered to be in communication
		{// run timer
			tmCom[i] ++;
			if(tmCom[i] > COM_TIMEOUT)	// time out?
			{
				w[i] = 0;
			}
		}
	}
}
//刚开机时候，对各个计时器清零Wang
void	ComTimerReset()
{
	unsigned char i;
	for(i=0; i<N_SWARM; i++)
	{
		tmCom[i] = 0;
	}
}

double offsetvx=0;
//添加速度误差。主函数中只在x方向添加速度误差，没懂Wang
void Set_offV(double* offsetv,double time)
{
	if(time>50)
		(*offsetv)=3;
	else
		(*offsetv)=0;
}

//xy方向两个维度Wang
void	SetAgent(Agent* agent, const double* pos, const double* vel, const double* pos_set, const double* spd_set)
{
//	agent->errp[0] = pos[0] - pos_set[0];
//	agent->errp[1] = pos[1] - pos_set[1];
	agent->errp[0] = pos[0] - pos_set[0];
	agent->errp[1] = pos[1] - pos_set[1];
	// agent->errp[2] = 
	agent->errv[0] = pos[0];
	agent->errv[1] = pos[1];
	// agent->errv[2] = 
	agent->p_r[0] = pos[0];
	agent->p_r[1] = pos[1];
}
void	AgentStates2Cmd(const Agent* self, Cmd* cmd, const unsigned char iNeighbors)
{
	if(iNeighbors >= N_NEIGHBORS)
		return;	// can't send data to nonexisted neighbor

	cmd->code = FORM_DECENTRALIZED_LINEAR1;
#if defined(XBEE)
    cmd->destH = addrNeighbors[iNeighbors][0];  // OK?
    cmd->destL = addrNeighbors[iNeighbors][1];
	cmd->len = 9;		// 暂定为9
	cmd->para[0] = (int16)(self->errv[0] * 100 + 0.5);
	cmd->para[1] = (int16)(self->errv[1] * 100 + 0.5);
	cmd->para[2] = (int16)(self->p_r[0] * 100 + 0.5);
	cmd->para[3] = (int16)(self->p_r[1]* 100 + 0.5);
	cmd->para[4] = (int16)(self->errv[1] * 100 + 0.5);
	cmd->para[5] = (int16)(self->errv[2] * 100 + 0.5);
	cmd->para[6] = (int16)(ux * 100 + 0.5);
	cmd->para[7] = (int16)(uy * 100 + 0.5);
	cmd->para[8] = 0;
    cmd->srcH = XBEE_ADDRESS_H;
    cmd->srcL = XBEE_ADDRESS_L;
#else
	cmd->dest = addrNeighbors[iNeighbors];	// OK?
	cmd->len = 9;		// 暂定为9
	cmd->para[0] = (int16)(self->errp[0] * 100 + 0.5);
	cmd->para[1] = (int16)(self->errp[1] * 100 + 0.5);
	cmd->para[2] = (int16)(self->p_r[0] * 100 + 0.5);
	cmd->para[3] = (int16)(self->p_r[1]* 100 + 0.5);
	cmd->para[4] = (int16)(self->errv[1] * 100 + 0.5);
	cmd->para[5] = (int16)(self->errv[2] * 100 + 0.5);
	cmd->para[6] = (int16)(ux * 100 + 0.5);
	cmd->para[7] = (int16)(uy * 100 + 0.5);
	cmd->para[8] = 0;
	cmd->src = ZIGBEE_ADDRESS;
#endif
}
void 	Cmd2AgentStates(const  Cmd* cmd, Agent* self, Agent* neighbors, double* w)
{
#if defined(XBEE)
	unsigned char i;
	for(i = 0; i < N_SWARM; i++)
	{
	    if((cmd->srcH == neighbors[i].addr[0]) && (cmd->srcL == neighbors[i].addr[1]))
		{
			switch(cmd->code)
			{
			case FORM_DECENTRALIZED_LINEAR1:
				neighbors[i].errp[0] = cmd->para[0] * 0.01;
				neighbors[i].errp[1] = cmd->para[1] * 0.01;
				neighbors[i].p_r[0] = cmd->para[2] * 0.01;
				neighbors[i].p_r[1] = cmd->para[3] * 0.01;
				neighbors[i].offsetp[0] = neighbors[i].p_r[0] - self->p_r[0];
				neighbors[i].offsetp[1] = neighbors[i].p_r[1] - self->p_r[1];
				neighbors[i].offsetv[0] = neighbors[i].errv[0] - self->errv[0];
				neighbors[i].offsetv[1] = neighbors[i].errv[1] - self->errv[1];

				w[i] = 1;
				tmCom[i] = 0;	// reset its timer
				// absent for Z dimension
				break;
			default:
				break;
			}
			break;	// no need to go on
		}
	}
#else
	unsigned char i;
	for(i = 0; i < N_SWARM; i++)
	{
		if(cmd->src == neighbors[i].addr)
		{
			switch(cmd->code)
			{
			case FORM_DECENTRALIZED_LINEAR1:
				neighbors[i].errp[0] = cmd->para[0] * 0.01;
				neighbors[i].errp[1] = cmd->para[1] * 0.01;
				neighbors[i].p_r[0] = cmd->para[2] * 0.01;
				neighbors[i].p_r[1] = cmd->para[3] * 0.01;
				neighbors[i].offsetp[0] = -neighbors[i].errp[0] + self->errp[0];
				neighbors[i].offsetp[1] = -neighbors[i].errp[1] + self->errp[1];
				neighbors[i].offsetv[0] = neighbors[i].errv[0] - self->errv[0];
				neighbors[i].offsetv[1] = neighbors[i].errv[1] - self->errv[1];

				w[i] = 1;
				tmCom[i] = 0;	// reset its timer
				// absent for Z dimension
				break;
			default:
				break;
			}
			break;	// no need to go on
		}
	}
#endif
}

#endif // DECENT_SCHEME_1

