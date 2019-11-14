/*
	Decentralized Formation Control, Scheme 2
	in Development and Test
*/


#include "decentralized.h"
#include "para.h"
#include "Fml.h"
#include <math.h>
#include "mathex.h"

#ifdef	DECENT_SCHEME_2
/*
	Decentralized Position Control. For horizonal position now, but can be generalized to 3D.
	Input:	<1> self: my states
			<2> neighbors: neighbors' states
			<3> att: my attitude
			<4> acc_set: my set acceleration
	Output:	<5> att_set: output, which is set attitude
*/
// - K1 K2 K3矩阵，分量1对于位置，分量2对于速度；x,y轴用相同参数
double	k11 = 0, k12 = 0;
double	k21 = -1, k22 = -0.8;
double	k31 = 0.5, k32 = 0;
double	alpha = 1;
double	g_n1[N_SWARM] = {0, 1, 0}; // 根据Formation_Index
// - 算法输出，指令加速度
double	ux = 0, uy = 0;
// - delta 队形偏差量
double	delta_x[5] = {0, 5, 0, -5, 0};
double	delta_y[5] = {0, 0, 5, 0, -5};

struct filter_pair	uatt_0[2] = {{0, 0}, {0, 0}};
struct filter_pair	uatt_1[2] = {{0, 0}, {0, 0}};
void	DecentPosCtrlRef(const Agent* self, const Agent* ref, const Agent* neighbors, const double* acc_set, double* v_ctrl, double* vel, double *spd_set, const double* w, const unsigned char nNeighbors)
{	
	unsigned char i = 0;
	double	yita = 0;
	double	g_ni = 0;
	double	sum_w = 0;
	
	if(g_n1[FORMATION_INDEX] > 0)
		g_ni = g_n1[FORMATION_INDEX];	// normal
	else	// g_n1[FORMATION_INDEX] == 0
	{
		sum_w = 0;
		for(i=0; i<N_SWARM; i++)	// sum_w = sum(w[i])
			sum_w += w[i];	
		if(sum_w < 1e-6)	// no input
			g_ni = 1;		// fake input used
		else g_ni = 0;
		// else	// sum_w > 0
		// 	never mind
	}
		
	// initialization
	//ux = 0;
	//uy = 0;
	// Part I
	ux = g_ni * alpha * (- k31 * (self->errp[0] - ref->errp[0]) - k32 * (self->errv[0] - ref->errv[0]));
	uy = g_ni * alpha * (- k31 * (self->errp[1] - ref->errp[1]) - k32 * (self->errv[1] - ref->errv[1]));
	yita += g_ni * alpha;
	// Part II
	//! for-loop is needed when more than 1 neighbor exists
	//! w_ij are all set as 1 as default
	for(i = 0; i< nNeighbors; i++)
	{
		ux += w[i] * (- k31 * (self->errp[0] - neighbors[i].errp[0]) - k32 * (self->errv[0] - neighbors[i].errv[0]));
		uy += w[i] * (- k31 * (self->errp[1] - neighbors[i].errp[1]) - k32 * (self->errv[1] - neighbors[i].errv[1]));
		yita += w[i];
	}
	// Part III
	if(yita > 1e-6)
	{
		ux /= yita;
		uy /= yita;
	}
	else	// yita == 0
	{
		ux = 0;
		uy = 0;
	}
	v_ctrl[0] = spd_set[0] + ux;
	v_ctrl[1] = spd_set[1] + uy;
	// Part IV
	//ux += acc_set[0];
	//uy += acc_set[1];

	// projection, transform and filtering

}

/*
	Get proper set point when switching from MANUAL control to AUTO control
	Target: continuous output
	(!) ignore other agents' error temporarily
	Input: 	<1> att: attitude
			<2> att_set: attitude set values
			<3> vel: velocity
	Output:	<4> pos_set: position set point revelant to pos_zero (pos_zero set to 0 when switching)
*/
void	DecentPosCtrlGetSetPoint(const double* att, const double* att_set, const double* vel, double* pos_set)
{
	double ox, oy;	// equivalent output
	ox = (-sin(att[2])*att_set[0] - cos(att[2])*att_set[1]) * GRAVITY;
	oy = ( cos(att[2])*att_set[0] - sin(att[2])*att_set[1]) * GRAVITY;
	// notice that k21 < 0!
	pos_set[0] = -(ox + k22 * vel[0]) / k21;
	pos_set[1] = -(oy + k22 * vel[1]) / k21;
}

// Reset for Decentralized Position Controller
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

/*
	Run the timer for communication and act on timeout
	Input:	- time lapse
	Output:	<1> w: weights of neighbors - clear when timeout
	And its reset method.
*/
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

void	ComTimerReset()
{
	unsigned char i;
	for(i=0; i<N_SWARM; i++)
	{
		tmCom[i] = 0;
	}
}

/*
	Unwrap a CMD as an agent's states
	Input:	<1> cmd: coming command about states of a neighbor
			<2> self: states of agent myself
	Output:	<3> neighbor: states of the neighbor agent
			<4> w: weights of neighbors
*/
void 	Cmd2AgentStatesRef(const  Cmd* cmd, Agent* self, Agent* neighbors, double* w)
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
				neighbors[i].erra[0] = cmd->para[6] * 0.01;
				neighbors[i].erra[1] = cmd->para[7] * 0.01;
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

/*
	Set an Agent's states
	Input:	<2> real position [in navigation frame]
			<3> real velosity [in navigation frame]
			<4> position reference
			<5> set position [in navigation frame]
			<6> set speed [in navigation frame]
			<7> set acceleration [in navigation frame]
	Output:	<1> the Agent
*/
void	SetAgent(Agent* agent, const double* pos, const double* vel, const double* pos_set, const double* spd_set)
{
	agent->errp[0] = pos[0] - pos_set[0];
	agent->errp[1] = pos[1] - pos_set[1];
	// agent->errp[2] = 
	agent->errv[0] = vel[0] - spd_set[0];
	agent->errv[1] = vel[1] - spd_set[1];
	// agent->errv[2] = 
}
void	SetRefAgent(Agent* agent, const double* pos, const double* pos_set, const double* spd_set, const double* acc_set)
{
	agent->errp[0] = pos_set[0] - pos[0];
	agent->errp[1] = pos_set[1] - pos[1];
	// agent->errp[2] = 
	agent->errv[0] = spd_set[0];
	agent->errv[1] = spd_set[1];
	// agent->errv[2] = 
	agent->erra[0] = acc_set[0]; // cheated
	agent->erra[1] = acc_set[1];
	// agent->erra[2] = 
}

/*
	Set parameters of decentralized formation control
	Input:	<1> set-parameter command
	| output to the parameters in decentralized.c
*/
void	SetDFPara(const Cmd* cmd)
{
	switch(cmd->code)
	{
	case CMD_SETDFPARA_K:
		if(cmd->para[0] != 0x7FFF) k11 = cmd->para[0] * 0.01;	// k1_p
		if(cmd->para[2] != 0x7FFF) k12 = cmd->para[2] * 0.01;	// k1_d
		if(cmd->para[3] != 0x7FFF) k21 = cmd->para[3] * 0.01;	// k2_p
		if(cmd->para[5] != 0x7FFF) k22 = cmd->para[5] * 0.01;	// k2_d
		if(cmd->para[6] != 0x7FFF) k31 = cmd->para[6] * 0.01;	// k3_p
		if(cmd->para[8] != 0x7FFF) k32 = cmd->para[8] * 0.01;	// k3_d
		// temporarily same for x&y axis. none for z axis
		break;
	default:
		break;
	}
}

extern Uint16	addrNeighbors[N_NEIGHBORS];
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
#endif	// DECENTRALIZED, SCHEME 2

