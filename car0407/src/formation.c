#include "formation.h"
#include "para.h"
//void	FormLF_Leader_FormCmd(Cmd* cmd, const Uint16* addr_follower, double* pos, double* vel)
//{
//	cmd->code = FORMLF_LEADER_FORM;
//	cmd->dest = *addr_follower;
//	cmd->len = 9;	// ÔÝ¶¨Îª9
//	cmd->para[0] = (int16)(pos[0] * 100 + 0.5);
//	cmd->para[1] = (int16)(pos[1] * 100 + 0.5);
//	cmd->para[2] = (int16)(pos[2] * 100 + 0.5);
//	cmd->para[3] = (int16)(vel[0] * 100 + 0.5);
//	cmd->para[4] = (int16)(vel[1] * 100 + 0.5);
//	cmd->para[5] = (int16)(vel[2] * 100 + 0.5);
//	cmd->para[6] = 0;
//	cmd->para[7] = 0;
//	cmd->para[8] = 0;
//	cmd->src = ZIGBEE_ADDRESS;
//}

double	pos_leader[3] = {0, 0, 0};
double	vel_leader[3] = {0, 0, 0};
unsigned char bLeader = 0;

