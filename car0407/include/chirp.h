#ifndef CHIRP_H__
#define	CHIRP_H__
#include "message.h"
// x = Asin(2Pi f t) + x0
// f = kf t + f0, till f = ft
struct Chirp{
	double f0;	// 
	double ft;	//
	double kf;	//
	double x0;	//
	double A;	//
	double t;	//
};

void	SetChirp(const Cmd* cmd);
void	ChirpOut(double* x);
#endif
