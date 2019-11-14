/*
	For Decentralized Formation
	Reference: 四旋翼编队飞行算法说明[董希旺]
*/
#ifndef DECENTRALIZED_H__
#define DECENTRALIZED_H__
#include "message.h"

// Agent: describe the states of an agent
// ! Note that error is defined as (x-x_set), which is opposite to normal error
#if defined(XBEE)
typedef struct{
    double  errp[3];    // 3-axis error of position(m)
    double  errv[3];    // 3-axis error of velocity(m/s)
    double  erra[3];    // 3-axis error of acceleration(m/s^2)
    double  p_r[3];     // 3-axis position(m), relative to center point
    double  v_r[3];     // 3-axis velocity(m/s)
    double  offsetp[3]; // 3-axis offset of position: errp-errp_self(m)
    double  offsetv[3]; // 3-axis offset of velocity: errv-errv_self(m/s)
    Uint32  addr[2];        // address
} Agent;
#define AGENT_INIT(x) {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, x}  // used for initialization of an agent
#else
typedef struct{
	double	errp[3];	// 3-axis error of position(m)
	double	errv[3];	// 3-axis error of velocity(m/s)
	double	erra[3];	// 3-axis error of acceleration(m/s^2)
	double	p_r[3];		// 3-axis position(m), relative to center point
	double	v_r[3];		// 3-axis velocity(m/s)
	double	offsetp[3];	// 3-axis offset of position: errp-errp_self(m)
	double	offsetv[3];	// 3-axis offset of velocity: errv-errv_self(m/s)
	Uint16	addr;		// address
} Agent;
#define AGENT_INIT(x) {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, x}	// used for initialization of an agent
#endif
#define	COM_TIMEOUT	400	// timeout for communication: 1000 cycles ~ 2 s
/*
	Decentralized Position Control. For horizonal position now, but can be generalized to 3D.
	Input:	<1> self: my states
			<2> ref: reference track
			<3> neighbors: neighbors' states
			<4> att: my attitude
			<5> acc_set: my set acceleration
			<7> w: weights of neighbors' feedback
			<8> nNeighbors: number of neighbors
	Output:	<6> att_set: output, which is set attitude
*/
void DecentPosCtrl(double* pos, double* pos_set, const Agent* self, const double* acc_set, double* v_ctrl, double* vel, double *u, const Agent* neighbors,const double* w);
void DecentPosCtrlRef(const Agent* self, const Agent* ref, const Agent* neighbors, const double* acc_set, double* v_ctrl, double* vel, double *u, const double* w, const unsigned char nNeighbors);
void	DecentPosCtrlReset(const double* att_set);	// for reset
/*
	Get proper set point when switching from MANUAL control to AUTO control
	Target: continuous output
	(!) ignore other agents' error temporarily
	Input: 	<1> att: attitude
			<2> att_set: attitude set values
			<3> vel: velocity
	Output:	<4> pos_set: position set point revelant to pos_zero (pos_zero set to 0 when switching)
*/
void	DecentPosCtrlGetSetPoint(const double* att, const double* att_set, const double* vel, double* pos_set);

/*
	Wrap this agent's states as a CMD to be sent out
	Input:	<1> self: my states
			<2> dest: neighbor as the destination
	Output:	<3> cmd: wrapped command
			<4> iNeighbors: which neighbor I send error of formation to
*/
void	AgentStates2Cmd(const Agent* self, Cmd* cmd, const unsigned char iNeighbors);

/*
	Unwrap a CMD as an agent's states
	Input:	<1> cmd: coming command about states of a neighbor
			<2> self: states of agent myself
	Output:	<3> neighbor: states of the neighbor agent
			<4> w: weights of neighbors
*/
void 	Cmd2AgentStates(const  Cmd* cmd, Agent* self, Agent* neighbors, double* w);
void 	Cmd2AgentStatesRef(const  Cmd* cmd, Agent* self, Agent* neighbors, double* w);
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
void	SetAgent(Agent* agent, const double* pos, const double* vel, const double* pos_set, const double* spd_set);
void	SetRefAgent(Agent* agent, const double* pos, const double* pos_set, const double* spd_set, const double* acc_set);
/*
	Set parameters of decentralized formation control
	Input:	<1> set-parameter command
	| output to the parameters in decentralized.c
*/
void	SetDFPara(const Cmd* cmd);

/*
	Run the timer for communication and act on timeout
	Input:	- time lapse
	Output:	<1> w: weights of neighbors - clear when timeout
	And its reset method.
*/
void	ComTimerRun(double* w);
void	ComTimerReset();

void	SetComTopo(const Cmd* cmd);


void DecentPosCtrl_new(const Agent* self,const double* att, const double* acc_set, double* att_set, double *u);
void fun2(double u[3], double vel_f[3], double pos_f[3]);
void fun3(double u[3], double vel_f[3], double pos_f[3]);
void Set_offV(double* offsetv,double time);
#endif

