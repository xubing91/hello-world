#ifndef FORMATION_H__
#define FORMATION_H__
#include "message.h"
/* 
	Leader - Follower Formation
*/
void	FormLF_Leader_FormCmd(Cmd* cmd, const Uint16* addr_follower, double* pos, double* vel);
void	SetFormation(const Cmd* cmd);
void	FormSetPosition(double* pos_zero, double* spd_zero);
#endif
