#include "mathex.h"
#include <math.h>

double	PsaiMdf(double x)
{
	return (x>PI ? (x-2*PI) : (x<=-PI ? x+2*PI : x));	// psai ~ (-pi, pi]
}

double	rad2deg(double x)
{
	return	x/PI*180;
}

double deg2rad(double x)
{
	return	PsaiMdf(x*PI/180);
}

double Saturation(double x, double sat)
{
	return (x>sat)? sat : ((x<-sat)? -sat : x);
}

void	VectorSub(const double* src1, const double* src2, double* dst, int len)
{
	int i=0;
	for(i=0; i<len; i++)
	{
		dst[i] = src1[i] - src2[i];
	}
}

/*
	f(x) = 1, in 2n-1 cycle: 0-2pi, 4pi-6pi, ...
	f(x) = -1, in 2n cycle: 2pi-4pi, 6pi-8pi, ...
*/
// x not too big
double	signcyc(double x)	
{
	int floor = (int)(fabs(x/2/PI));
	if(floor % 2 == 0)	// even
		return 1;
	else	// odd
		return -1;
}

double sign(double x)
{
	return	(x>0 ? 1 : (x<0 ? -1 : 0));
}

double sqr(double x)
{
	return	x * x;
}

double fabs(double x)
{
	return	(x>0 ? x : -(x));
}

