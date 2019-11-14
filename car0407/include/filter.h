#ifndef FILTER_H__
#define FILTER_H__
/* 
	本文件包含位置滤波
	均为标准单位
*/
#include "Fml.h"
void	SpdFilterUWB(const double* pos_x,
					const double* pos_y,
					double* vx_f, double* vy_f);
void	SpdFilterUWB2(const double* pos_x,
					const double* pos_y,
					double* vx_f, double* vy_f);
void	PosFilter(const double* pi,	// input position, ground frame, NEU
				  const double* vi, // input velocity, ground frame, NEU
				  const double* ab, // input accelaration, body frame, FRD
				  double*	po,		// output position, ground frame, NEU
				  double*	vo, 	// output position, ground frame, NEU
				  const double* att,// attitude
				  unsigned char bUWB);// new UWB data

// maybe shouldn't be here				
struct filter_2o{
	double x[3];
	double x_[3];
	double freq;	// 一阶滤波器的带宽
};
double 	FilterRun_2o(struct filter_2o* fil, double input);
void 	FilterReset_2o(struct filter_2o* fil);

void	HeightFilter(const double HeightUS, const double* acc_f, const double* att, double* height, double* vHeight);
double	HeightFilter_GetBiasA();
#endif

