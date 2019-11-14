#include "flightFSM.h"
#include "trajectory.h"

unsigned int cnt_landing = 0;
unsigned int cnt_idle = 0;
extern double height_US;
extern double output;
// extern unsigned char bTrajRunning;
extern Traj traj_ab;
extern char isTakeOff;
extern ScdFlight scdFlight;


unsigned char FSM_ReadEvent(struct Fsm* fsm)
{
	unsigned char event;
	if(fsm->event != FSM_EVENT_NULL)
	{
		event = fsm->event;
		fsm->event = FSM_EVENT_NULL;
		return event;
	}
	else	// null
		return FSM_EVENT_NULL;
}




