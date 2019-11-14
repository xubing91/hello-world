#ifndef CONTROLLER_H__
#define CONTROLLER_H__

void Controller(double* state, double* input, int* output, unsigned char* debug);
void Controller_Height(double* height, unsigned char* debug);
unsigned char SetCtrlPara(unsigned char* input);
void SetThro(unsigned int val);
void SetRud(double* yaw_set, unsigned int val);
void GetYaw(double* yaw);
void SetU1();
#endif
