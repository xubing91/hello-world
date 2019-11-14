#ifndef FSM_H__
#define FSM_H__
/************************
 * Finite state machine *
 ************************
*/
#include "DSP2833x_Device.h" 
#include "para.h"
#include "Hal.h"
#include "Fml.h"
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
/*
	Flight Mode
*/
#define	FLIGHTMODE_IDLE		0
#define	FLIGHTMODE_STANDBY	1
#define FLIGHTMODE_AUTO		2
#define	FLIGHTMODE_MANUAL	3
#define	FLIGHTMODE_FLIGHT	4
#define	FLIGHTMODE_LAND		5
#define	FLIGHTMODE_TAKEOFF	6

/*
	Event Code
*/
#define FSM_EVENT_NULL		0
#define	FSM_EVENT_TAKEOFF	1
#define	FSM_EVENT_LAND		2
#define FSM_EVENT_AUTO		3	// manual -> auto flight

struct Fsm{
	unsigned char state;
	unsigned char event;
	unsigned char bGPS;	// GPS OK
	unsigned char cmd;
};
#define FSM_DEFAULT {FLIGHTMODE_IDLE, FSM_EVENT_NULL, 0, 0}

/*
	Update Flight FSM @ every cycle
*/
void FSM_Update(struct Fsm* fsm, unsigned char bShutdown, unsigned char bAuto, int16 nsv);

/*
	Get Triggered Event of FSM. Clear it once get.
*/
unsigned char FSM_ReadEvent(struct Fsm* fsm);

//ÏÔÊ¾×´Ì¬
void LightShow(struct Fsm* fsm);
#endif




