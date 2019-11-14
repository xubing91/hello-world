/*
	Extension for <math.h>. 
	Some other math functions.
*/
#ifndef MATHEX_H__
#define MATHEX_H__
#define	PI	3.1415927
double	PsaiMdf(double x);
double	rad2deg(double x);
double	deg2rad(double x);
double  Saturation(double x, double sat);
void	VectorSub(const double* src1, const double* src2, double* dst, int len);
double	signcyc(double x);
double sign(double x);
double sqr(double x);
double fabs(double x);

#endif
