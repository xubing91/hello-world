#include "decentralized.h"
#include "para.h"
#include "Fml.h"
#include <math.h>
#include "mathex.h"
#include "trajectory.h"

Uint16	addrNeighbors[N_NEIGHBORS] = ADDRESS_NEIGHBORS;
#ifdef	DECENT_SCHEME_1


double vel_f[3];  	//速度传感器当前时刻的值
double pos_f[3]; 	//位移传感器当前时刻的值
     
//double vel_set[3];	//给定速度跟踪值
//double pos_set[3];	//给定位移跟踪值
double u[3]	;	//控制律


double pos_hat_r_[3]={0,0,0};		//检测器中当前时刻位移估计值
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
//double kpx = 0, ki = 0, kd = 0;//1.6 1.3
extern Cmd cmdR;
void DecentPosCtrl(const Agent* self, const double* acc_set, double* v_ctrl, double* vel, double *u, const Agent* neighbors,const double* w)
{
    static double lastError_x = 0;
    static double lastError_y = 0;


   int i=0;


  // ux = k11 * self->errp[0] + k12 * self->errv[0];
  // uy = k11 * self->errp[1] + k12 * self->errv[1];


   double nowError_x, nowError_y, derivError_x, derivError_y;
   nowError_x = -self->errp[0];
   nowError_y = -self->errp[1];
   derivError_x = (nowError_x - lastError_x) / periodT;
   derivError_y = (nowError_y - lastError_y) / periodT;
   lastError_x = nowError_x;
   lastError_y = nowError_y;
   ux = Kp_x*nowError_x + Kd_x*derivError_x;
   uy = Kp_y*nowError_y + Kd_y*derivError_y;


	for(i = 0; i< N_SWARM; i++)
	{
		ux += (Ki_x * neighbors[i].offsetp[0]) * w[i];
		uy += (Ki_x * neighbors[i].offsetp[1]) * w[i];
	}

  // ux += acc_set[0];
   //uy += acc_set[1];

   u[0] = ux;
   u[1] = uy;

   v_ctrl[0] = spd_set[0] + u[0];//spd_set[0] +
   v_ctrl[1] = spd_set[1] + u[1];//spd_set[1] +
   //w_wheel = sqrt(v_ctrl[0]*v_ctrl[0]+v_ctrl[1]*v_ctrl[1])/traj_ab.pNext.r;

}

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
	agent->errp[0] = pos[0] - pos_set[0];
	agent->errp[1] = pos[1] - pos_set[1];
	// agent->errp[2] = 
	agent->errv[0] = vel[0] - spd_set[0];
	agent->errv[1] = vel[1] - spd_set[1];
	// agent->errv[2] = 
}
void	AgentStates2Cmd(const Agent* self, Cmd* cmd, const unsigned char iNeighbors)
{
	if(iNeighbors >= N_NEIGHBORS)
		return;	// can't send data to nonexisted neighbor

	cmd->code = FORM_DECENTRALIZED_LINEAR1;
	cmd->dest = addrNeighbors[iNeighbors];	// OK?
	cmd->len = 9;		// 暂定为9
	cmd->para[0] = (int16)(self->errp[0] * 100 + 0.5);
	cmd->para[1] = (int16)(self->errp[1] * 100 + 0.5);
	cmd->para[2] = (int16)(self->errp[2] * 100 + 0.5);
	cmd->para[3] = (int16)(self->errv[0] * 100 + 0.5);
	cmd->para[4] = (int16)(self->errv[1] * 100 + 0.5);
	cmd->para[5] = (int16)(self->errv[2] * 100 + 0.5);
	cmd->para[6] = (int16)(ux * 100 + 0.5);
	cmd->para[7] = (int16)(uy * 100 + 0.5);
	cmd->para[8] = 0;
	cmd->src = ZIGBEE_ADDRESS;
}
void 	Cmd2AgentStates(const  Cmd* cmd, Agent* self, Agent* neighbors, double* w)
{
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
				neighbors[i].errv[0] = cmd->para[3] * 0.01;
				neighbors[i].errv[1] = cmd->para[4] * 0.01;
				neighbors[i].offsetp[0] = neighbors[i].errp[0] - self->errp[0];
				neighbors[i].offsetp[1] = neighbors[i].errp[1] - self->errp[1];
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
}

#endif // DECENT_SCHEME_1

