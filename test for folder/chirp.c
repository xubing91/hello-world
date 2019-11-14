#include "chirp.h"
#include "para.h"
#include "mathex.h"
#include <math.h>


struct Chirp chirp = {0, 0, 0, 0, 0, 0};

void	SetChirp(const Cmd* cmd)
{
	switch(cmd->code)
	{
		case CMD_CHIRP_SETPARA:
			if((cmd->para[0] != 0x7fff) && (cmd->para[3] != 0x7fff) && (cmd->para[6] != 0x7fff)
			&& (cmd->para[7] != 0x7fff) && (cmd->para[8] != 0x7fff))
			{
				chirp.A = cmd->para[0];
				chirp.x0 = cmd->para[3];
				chirp.f0 = cmd->para[7] * 0.001;
				chirp.ft = cmd->para[8] * 0.001;
				chirp.kf = cmd->para[6] * 0.001;
				chirp.t = 0;
			}
			break;
		default:
			break;
	}
}

void	ChirpOut(double* x)
{
	double f;	// frequency now
	f = chirp.kf * chirp.t + chirp.f0;
	if(f > chirp.ft)
		chirp.A = 0;	// stop chirping
	*x = chirp.A * sin(2*PI*f*chirp.t) + chirp.x0;
	chirp.t += 0.002;
}
