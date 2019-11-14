#include "para.h"
#include "Fml.h"
#include "mathex.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "trajectory.h"
#include "Hal.h"
#include <math.h>
const double Duty_New = 178.5;
const double Duty_New_b = 233.5;

//extern const double v_robot;  //速度
////第一个大圆的圆心和半径
//extern const double cx0;
//extern const double cy0;
//extern const double r0;
////小圆环半径
//extern const double r00;
////分成两个圆时的圆心和半径
//extern const double cx1;
//extern const double cy1;
//extern const double cx2;
//extern const double cy2;
//extern const double r1;
//分成四个圆时的半径
// extern const double r2;
const double pi = 3.14;

/*
	constants in constants.c
*/


void PWMset(double w1,double w2,double w3,double w4)
{
	EPwm1Regs.CMPB = (Uint16)(fabs(w1) * Duty_New + Duty_New_b);
	EPwm2Regs.CMPB = (Uint16)(fabs(w2) * Duty_New + Duty_New_b);
	EPwm3Regs.CMPB = (Uint16)(fabs(w3) * Duty_New + Duty_New_b);
	EPwm4Regs.CMPB = (Uint16)(fabs(w4) * Duty_New + Duty_New_b);
	                                                    if (w1>0)
				       		 		       		        	{AIN1_Head = 0;DELAY_US(1);
				       		 		       					 AIN2_Head = 1;DELAY_US(1);
				       		 		       		        	}
				       		 		       					else if(w1<0)
				       		 		       					{AIN1_Head = 1;DELAY_US(1);
				       		 		       					 AIN2_Head = 0;DELAY_US(1);
				       		 		       					}
				       		 		       					else
				       		 		       					{
				       		 		       						AIN1_Head = 0;DELAY_US(1);
				       		 		       					    AIN2_Head = 0;DELAY_US(1);
				       		 		       					}

				       		      						if (w2>0)
				       		 		       		        	{BIN1_Head = 0;DELAY_US(1);
				       		 		       		             BIN2_Head = 1;DELAY_US(1);
				       		 		       		        	}
				       		 		       					else if(w2<0)
				       		 		       					{BIN1_Head = 1;DELAY_US(1);
				       		 		       				     BIN2_Head = 0;DELAY_US(1);
				       		 		       					}
				       		 		       					else
				       		 		       					{
				       		 		       						BIN1_Head = 0;DELAY_US(1);
				       		 		       						BIN2_Head = 0;DELAY_US(1);
				       		 		       					}
				       		      						if (w3>0)
				       		 		       		        	{AIN1_Tail = 0;DELAY_US(1);
				       		 		       		             AIN2_Tail = 1;DELAY_US(1);
				       		 		       		        	}
				       		 		       					else if(w3<0)
				       		 		       					{AIN1_Tail = 1;DELAY_US(1);
				       		 		       				     AIN2_Tail = 0;DELAY_US(1);
				       		 		       					}
				       		 		       					else
				       		 		       					{
				       		 		       						AIN1_Tail = 0;DELAY_US(1);
				       		 		       						AIN2_Tail = 0;DELAY_US(1);
				       		 		       					}

				       		      						if (w4>0)
				       		 		       		        	{BIN1_Tail = 0;DELAY_US(1);
				       		 		       		             BIN2_Tail = 1;DELAY_US(1);
				       		 		       		        	}
				       		 		       					else if(w4<0)
				       		 		       					{BIN1_Tail = 1;DELAY_US(1);
				       		 		       				     BIN2_Tail = 0;DELAY_US(1);
				       		 		       					}
				       		 		       					else
				       		 		       					{
				       		 		       						BIN1_Tail = 0;DELAY_US(1);
				       		 		       						BIN2_Tail = 0;DELAY_US(1);
				       		 		       					}
}


extern double t;
extern double t_;
void Filter_1LPV(struct filter_pair* input, struct filter_pair* output, double fc)
{
/*
	一阶低通滤波：H(s) = 1/(1+s/fc)
	双线性变换转为数字滤波器
	Y[n](1+2/(Tfc)) + Y[n-1](1-2/(Tfc)) = U[n] + U[n-1]
	时间步长为DTIME=0.002
*/
	double	flp_temp;
	flp_temp = output->x;
	output->x_ = output->x;
	output->x = (input->x + input->x_ - output->x_*(1-2/((t-t_)*fc))) / (1+2/((t-t_)*fc));
	output->x_ = flp_temp;
}
struct UWB_Data UWB;
unsigned char	UWBRxBuffer[39];
unsigned int RX_Count=0;
unsigned char rec_Scib;

unsigned char rdataA[16];    // Received data for SCI-A
extern unsigned char bUWBUpdate;
Uint32 tv = 0;
unsigned int time_count_UWB=0;
extern double t0;
extern double t_;
extern double t;
extern double	uwbpos[3] ;			// position NEU (m) [UWB]
extern double   UWB_angle ;
extern double	uwbvel[3] ;			// velocity NEU (m/s) [filtered by UWB]
extern double   uwbvel2[3];
extern double   uwbvel3[3];
extern double   uwbvel4[3];
extern double	pos_zero[3] ;	// zero of position in GPS's view -- NED(m)
extern double	att2[3] ; 		// attitude-角度

extern Traj traj_ab;
extern Traj traj_ref;
extern double  att_error ;
double t_error = 0;
extern double pos[3];
#ifdef UWB_GUIDANCE
void UWBDataAppend(unsigned char data)
{
	if(data == '@')	// start of a packet
		{
		RX_Count = 0;
		}	// clear all
	if(RX_Count > 39)
		RX_Count = 0;		// for protection

	UWBRxBuffer[RX_Count++] = data;

}

unsigned char UWBPacketReady()
{
	//if(RX_Count <= 2)	// 下面有lenGPS-1 lenGPS-2
	//	return 0;
	if(UWBRxBuffer[0]=='@' && RX_Count==39 && UWBRxBuffer[RX_Count-1] == 0x0A && UWBRxBuffer[RX_Count-2]==0x0D)
		return 1;
    else
	return 0;
}
void UWBPacketRead()
{
	   if(RX_Count == 39)//将从UWB读取到的数据解析后存储起来
	   {
			 UWBRxBuffer[RX_Count] = '\0';//添加结束符
             volatile  unsigned char buf1[1],buf2[6],buf3[6],buf4[6],buf5[1],buf6[2],buf7[4];
             unsigned char i;
             long int bufer_pos[3]={0,0,0};
//           long int bufer_vel[3]={0,0,0};
             long int bufer_angle = 0;
				buf1[0]=UWBRxBuffer[0];//should be @

                buf2[0]=UWBRxBuffer[1]; //x position
				buf2[1]=UWBRxBuffer[2];
				buf2[2]=UWBRxBuffer[3];
				buf2[3]=UWBRxBuffer[4];
				buf2[4]=UWBRxBuffer[5];
				buf2[5]=UWBRxBuffer[6];

			    for( i = 0; i < 6; i++ )
			    {
			    	if( (buf2[i] <= 'f' ) && (buf2[i]>='a') )
			    	  buf2[i] = buf2[i] - 0x60 + 9;
			    	else if( (buf2[i] <= '9' ) && (buf2[i]>='0') )
			    		buf2[i] = buf2[i] - 0x30;
			    	bufer_pos[0] = ( bufer_pos[0] | buf2[i] )<<4;

				}

			    UWB.x = (double)((bufer_pos[0]<<4)>>8) * 0.2 * 0.01;


				buf3[0]=UWBRxBuffer[7]; //y position
				buf3[1]=UWBRxBuffer[8];
				buf3[2]=UWBRxBuffer[9];
				buf3[3]=UWBRxBuffer[10];
				buf3[4]=UWBRxBuffer[11];
				buf3[5]=UWBRxBuffer[12];

			    for( i = 0; i < 6; i++ )
			    {
			    	if( (buf3[i] <= 'f' ) && (buf3[i]>='a') )
			    		buf3[i] = buf3[i] - 0x60 + 9;
			    	else if( (buf3[i] <= '9' ) && (buf3[i]>='0') )
			    		buf3[i] = buf3[i] - 0x30;
			    	bufer_pos[1] = ( bufer_pos[1] | buf3[i] )<<4;

				}
			    UWB.y = (double)((bufer_pos[1]<<4)>>8) * 0.2 * 0.01;

				buf4[0]=UWBRxBuffer[13]; //z position
				buf4[1]=UWBRxBuffer[14];
				buf4[2]=UWBRxBuffer[15];
				buf4[3]=UWBRxBuffer[16];
				buf4[4]=UWBRxBuffer[17];
				buf4[5]=UWBRxBuffer[18];

			    for( i = 0; i < 6; i++ )
			    {
			    	if( (buf4[i] <= 'f' ) && (buf4[i]>='a') )
			    		buf4[i] = buf4[i] - 0x60 + 9;
			    	else if( (buf4[i] <= '9' ) && (buf4[i]>='0') )
			    		buf4[i] = buf4[i] - 0x30;
			    	bufer_pos[2] = (bufer_pos[2] | buf4[i] )<<4;

				}
			    UWB.z = (double)((bufer_pos[2]<<4)>>8) * 0.2 * 0.01;

			    buf5[0] = UWBRxBuffer[30];

			    buf6[0] = UWBRxBuffer[31];
			    buf6[1] = UWBRxBuffer[32];

			    buf7[0] = UWBRxBuffer[33];
			    buf7[1] = UWBRxBuffer[34];
			    buf7[2] = UWBRxBuffer[35];
			    buf7[3] = UWBRxBuffer[36];

			    for( i = 0; i < 4; i++ )
			    {
			    	if( (buf7[i] <= 'f' ) && (buf7[i]>='a') )
			    	  buf7[i] = buf7[i] - 0x60 + 9;
			    	else if( (buf7[i] <= '9' ) && (buf7[i]>='0') )
			    		buf7[i] = buf7[i] - 0x30;
			    	bufer_angle = ( bufer_angle | buf7[i] )<<4;

				}

			    UWB.angle = (double)((bufer_angle<<4)>>8) * 1;
			    if((UWB.angle>180) && (UWB.angle<=360))
			    {
			    	UWB.angle = UWB.angle - 360;
			    }

			    time_count_UWB++;
						    t_=t;
						    t=DTIME*CpuTimer0.InterruptCount;
						    t_error = t - t_;
						    uwbpos[0] = UWB.x;
						    uwbpos[1] = UWB.y;
						    UWB_angle = UWB.angle;
                            //pos[0] = uwbpos[0];
                            //pos[1] = uwbpos[1];

						    SpdFilterUWB(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
						    //SpdFilterUWB2(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
						    //if(tv % 20 == 0)
						   // {
							//    SpdFilterUWB(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
							 //     SpdFilterUWB2(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
						   // }
						    if ((CpuTimer0.InterruptCount>=7000) && (CpuTimer0.InterruptCount<=8000))
					    			{
					    		         pos_zero[0] = uwbpos[0];
					    		         pos_zero[1] = uwbpos[1];
					    		         traj_ab.pNext.x = uwbpos[0];
					    		         traj_ab.pNext.y = uwbpos[1];
					    		         traj_ref.pNext.x = uwbpos[0];
					    		         traj_ref.pNext.y = uwbpos[1];
					    		     }
					    	time_count_UWB=2;

	   }

}
#else
void UWBDataAppend(unsigned char data)
{
	if(data == '@')	// start of a packet
		{
		RX_Count = 0;
		}	// clear all
	if(RX_Count > 30)
		RX_Count = 0;		// for protection

	UWBRxBuffer[RX_Count++] = data;

}

unsigned char UWBPacketReady()
{
	//if(RX_Count <= 2)	// 下面有lenGPS-1 lenGPS-2
	//	return 0;
	if(UWBRxBuffer[0]=='@' && RX_Count==30 && UWBRxBuffer[RX_Count-1] == 0x0A && UWBRxBuffer[RX_Count-2]==0x0D)
		return 1;
    else
	return 0;
}
void UWBPacketRead()
{
	   if(RX_Count == 30)//将从UWB读取到的数据解析后存储起来
	   {
			 UWBRxBuffer[RX_Count] = '\0';//添加结束符
             volatile  unsigned char buf1[1],buf2[6],buf3[6],buf4[6],buf5[2],buf6[2],buf7[2],buf8[1],buf9[2];
             unsigned char i;
             long int bufer_pos[3]={0,0,0};
             long int bufer_vel[3]={0,0,0};
				buf1[0]=UWBRxBuffer[0];//should be @

                buf2[0]=UWBRxBuffer[1]; //x position
				buf2[1]=UWBRxBuffer[2];
				buf2[2]=UWBRxBuffer[3];
				buf2[3]=UWBRxBuffer[4];
				buf2[4]=UWBRxBuffer[5];
				buf2[5]=UWBRxBuffer[6];

			    for( i = 0; i < 6; i++ )
			    {
			    	if( (buf2[i] <= 'f' ) && (buf2[i]>='a') )
			    	  buf2[i] = buf2[i] - 0x60 + 9;
			    	else if( (buf2[i] <= '9' ) && (buf2[i]>='0') )
			    		buf2[i] = buf2[i] - 0x30;
			    	bufer_pos[0] = ( bufer_pos[0] | buf2[i] )<<4;

				}

			    UWB.x = (double)((bufer_pos[0]<<4)>>8) * 0.2 * 0.01;


				buf3[0]=UWBRxBuffer[7]; //y position
				buf3[1]=UWBRxBuffer[8];
				buf3[2]=UWBRxBuffer[9];
				buf3[3]=UWBRxBuffer[10];
				buf3[4]=UWBRxBuffer[11];
				buf3[5]=UWBRxBuffer[12];

			    for( i = 0; i < 6; i++ )
			    {
			    	if( (buf3[i] <= 'f' ) && (buf3[i]>='a') )
			    		buf3[i] = buf3[i] - 0x60 + 9;
			    	else if( (buf3[i] <= '9' ) && (buf3[i]>='0') )
			    		buf3[i] = buf3[i] - 0x30;
			    	bufer_pos[1] = ( bufer_pos[1] | buf3[i] )<<4;

				}
			    UWB.y = (double)((bufer_pos[1]<<4)>>8) * 0.2 * 0.01;

				buf4[0]=UWBRxBuffer[13]; //z position
				buf4[1]=UWBRxBuffer[14];
				buf4[2]=UWBRxBuffer[15];
				buf4[3]=UWBRxBuffer[16];
				buf4[4]=UWBRxBuffer[17];
				buf4[5]=UWBRxBuffer[18];

			    for( i = 0; i < 6; i++ )
			    {
			    	if( (buf4[i] <= 'f' ) && (buf4[i]>='a') )
			    		buf4[i] = buf4[i] - 0x60 + 9;
			    	else if( (buf4[i] <= '9' ) && (buf4[i]>='0') )
			    		buf4[i] = buf4[i] - 0x30;
			    	bufer_pos[2] = (bufer_pos[2] | buf4[i] )<<4;

				}
			    UWB.z = (double)((bufer_pos[2]<<4)>>8) * 0.2 * 0.01;

				buf5[0]=UWBRxBuffer[19]; //x vel
				buf5[1]=UWBRxBuffer[20];

				for(i=0;i<2;i++)
				{
			    	if( (buf5[i] <= 'f' ) && (buf5[i]>='a') )
			    		buf5[i] = buf5[i] - 0x60 + 9;
			    	else if( (buf5[i] <= '9' ) && (buf5[i]>='0') )
			    		buf5[i] = buf5[i] - 0x30;
			    	bufer_vel[0] = (bufer_vel[0] | buf5[i] )<<4;
				}
				UWB.vx = (double)((bufer_vel[0]<<4)>>8) * 0.2 * 0.01;

				for(i=0;i<2;i++)
				{
			    	if( (buf5[i] <= 'f' ) && (buf5[i]>='a') )
			    		buf5[i] = buf5[i] - 0x60 + 9;
			    	else if( (buf5[i] <= '9' ) && (buf5[i]>='0') )
			    		buf5[i] = buf5[i] - 0x30;
			    	bufer_vel[1] = (bufer_vel[1] | buf5[i] )<<4;
				}
				UWB.vy = (double)((bufer_vel[1]<<4)>>8) * 0.2 * 0.01;

				for(i=0;i<2;i++)
				{
			    	if( (buf5[i] <= 'f' ) && (buf5[i]>='a') )
			    		buf5[i] = buf5[i] - 0x60 + 9;
			    	else if( (buf5[i] <= '9' ) && (buf5[i]>='0') )
			    		buf5[i] = buf5[i] - 0x30;
			    	bufer_vel[2] = (bufer_vel[2] | buf5[i] )<<4;
				}
				UWB.vz = (double)((bufer_vel[2]<<4)>>8) * 0.2 * 0.01;
				//buf5[0]=UWBRxBuffer[19];
				//buf6[0]=UWBRxBuffer[20];
					    	time_count_UWB++;
						    t_=t;
						    t=DTIME*CpuTimer0.InterruptCount;
						    t_error = t - t_;
						    uwbpos[0]=UWB.x;
						    uwbpos[1]=UWB.y;
                            //pos[0] = uwbpos[0];
                            //pos[1] = uwbpos[1];

						    SpdFilterUWB(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
						    //SpdFilterUWB2(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
						    //if(tv % 20 == 0)
						   // {
							//    SpdFilterUWB(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
							 //     SpdFilterUWB2(&uwbpos[0],&uwbpos[1],&uwbvel[0], &uwbvel[1]);
						   // }
						    if ((CpuTimer0.InterruptCount>=7000) && (CpuTimer0.InterruptCount<=8000))
					    			{
					    		         pos_zero[0] = uwbpos[0];
					    		         pos_zero[1] = uwbpos[1];
					    		         traj_ab.pNext.x = uwbpos[0];
					    		         traj_ab.pNext.y = uwbpos[1];
					    		         traj_ref.pNext.x = uwbpos[0];
					    		         traj_ref.pNext.y = uwbpos[1];
					    		     }
					    	time_count_UWB=2;

	   }

}
#endif
float Ascll2Float(unsigned char* buf)
{
     float dat=0;
     unsigned long int temp=0;
    // int count1=0;
     int i;
     for(i=0;i<6;i++)
     {
    	 if((buf[i] < 58) && (buf[i] > 47))
	    	 buf[i]=buf[i] - 0x30;
    	 else
    		 buf[i]=buf[i] - 97 + 10;

    	 temp += ((unsigned long int)buf[i])<<((5-i)*4);

	    	// (unsigned int)
     }
//     temp = (unsigned long int)buf1[5] + ((unsigned long int)buf1[4])<<4 + ((unsigned long int)buf1[3])<<8 +
//    		 ((unsigned long int)buf1[2])<<12 + ((unsigned long int)buf1[1])<<16 + ((unsigned long int)buf1[0])<<20;
     dat=(float)temp*0.002;
     return(dat);
}

interrupt void SCIRXINTb_ISR(void)
{
	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP9;
	if(ScibRegs.SCIRXST.bit.RXRDY == 1)
	{
		rec_Scib = ScibRegs.SCIRXBUF.all;
		UWBDataAppend(rec_Scib);
	}
	ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
	ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    if(UWBPacketReady())
    {
    	tv ++;
     UWBPacketRead();//下面要加上速度，现在函数里有traj_ab.pPrev.x=pos[0];
     bUWBUpdate = 1;
    }
    else
    	bUWBUpdate = 0;

}
unsigned char buf_Acc[6];
void GetAccRaw(int16* acc_raw)
{
	HQ7001_Get(buf_Acc);
	acc_raw[0] = (int16)(buf_Acc[0]<<8 | (buf_Acc[1]&0xFF));
	acc_raw[1] = (int16)(buf_Acc[2]<<8 | (buf_Acc[3]&0xFF));
	acc_raw[2] = (int16)(buf_Acc[4]<<8 | (buf_Acc[5]&0xFF));


}

void GetGyroRaw(int16* gyro_raw)	
{
	TLC2543_Read(2, gyro_raw);
	DELAY_US(10);
	TLC2543_Read(1, &(gyro_raw[0]));
	DELAY_US(10);
	TLC2543_Read(0, &(gyro_raw[1]));
	DELAY_US(10);
	TLC2543_Read(0, &(gyro_raw[2]));
	// 说明：按地址，陀螺仪输出次序为ZYX；每次发出的地址指向下次读回的内容
}


const double	K_acc[3] = {0.00179443359375, 0.00179443359375, 0.00179443359375};	// ±6g~32768

// board under bottom
// const double	K_acc[3] = {0.00059814453125, 0.00059814453125, 0.00059814453125};	// ±2g~32768
//const double	K_acc[3] = {0.00179443359375, 0.00179443359375, 0.00179443359375};	// ±6g~32768
#ifdef BIAS_ACC	// significant bias of accelerator exists
	const double	B_acc[3] = {BIASX_ACC, BIASY_ACC, BIASZ_ACC};
#else
	const double	B_acc[3] = {0, 0, 0};
#endif
void	ConvAcc(const int16* acc_raw, double* acc)
{
//此版本是对应580飞机V4版本的，对于其它小板子的方向得飞之前重新修改
//板子X---加速度-y
//板子Y---加速度x
//板子Z---加速度z
	acc[0] =-( acc_raw[1] * K_acc[0] - B_acc[0]);
	acc[1] = acc_raw[0] * K_acc[1] - B_acc[1];
	acc[2] = acc_raw[2] * K_acc[2] - B_acc[2];
//如果之后飞其它板子，重新修改
}

/* borad on top
const double	K_gyro[3] = {+0.01271565755208, +0.01271565755208, +0.01271565755208};	// 2.5V~32768;6mV~deg/s
*/
const double	K_gyro[3] = {+0.01271565755208, 0.01271565755208, 0.01271565755208};
// board under bottom
//const double	K_gyro[3] = {+0.01271565755208, -0.01271565755208, -0.01271565755208};	// 2.5V~32768;6mV~deg/s
const double	B_gyro[3] = {0, 0, 0};	// deg/s
const int16 MEANCYCLE = 5000;	// 用5000个周期做平均
int16 cnt_CG = 0;	// 计数器
double gyro_bias_sum[3] = {0, 0, 0};	// 偏移量之和 (rad/s)
double gyro_bias[3] = {0, 0, 0};		// 偏移量 (rad/s)
void	ConvGyro(const int16* gyro_raw, double* gyro)	//
{
	gyro[0] = (gyro_raw[0] * K_gyro[0] + B_gyro[0]) / 180 * PI - gyro_bias[0];
	gyro[1] = (gyro_raw[1] * K_gyro[1] + B_gyro[1]) / 180 * PI - gyro_bias[1];
	gyro[2] = (gyro_raw[2] * K_gyro[2] + B_gyro[2]) / 180 * PI - gyro_bias[2];
	if (cnt_CG <= MEANCYCLE)	// 第一阶段，累计均值
	{
		gyro_bias_sum[0] += gyro[0];
		gyro_bias_sum[1] += gyro[1];
		gyro_bias_sum[2] += gyro[2];
		gyro[0] = 0;
		gyro[1] = 0;
		gyro[2] = 0;
		cnt_CG++;
	
		if (cnt_CG == MEANCYCLE)	// 计算偏差量
		{
			gyro_bias[0] = gyro_bias_sum[0] / MEANCYCLE;
			gyro_bias[1] = gyro_bias_sum[1] / MEANCYCLE;
			gyro_bias[2] = gyro_bias_sum[2] / MEANCYCLE;
			cnt_CG++;
		}
	}
	else	// 此后，无特殊要求，但要做低通滤波
		GyroFilter(gyro, gyro)
	;		
}

const double	fGYRO = 150;	// 注意：此处为1阶低通的带宽.实际上也不是确切带宽
struct filter_pair gyro_0[3] = {{0, 0}, {0, 0}, {0, 0}};	// 陀螺仪-当前与前一周期
struct filter_pair gyro_1[3] = {{0, 0}, {0, 0}, {0, 0}};	// 陀螺仪一阶滤波后-当前与前一周期
struct filter_pair gyro_2[3] = {{0, 0}, {0, 0}, {0, 0}};	// 陀螺仪二阶滤波后-当前与前一周期
void	GyroFilter(const double* gyro, double* gyro_f)	
{
	// input & last input
/*	gyro_0[0].x_ = gyro_0[0].x;
	gyro_0[0].x = gyro[0];
	gyro_0[1].x_ = gyro_0[1].x;
	gyro_0[1].x = gyro[1];
	gyro_0[2].x_ = gyro_0[2].x;
	gyro_0[2].x = gyro[2];
	// filter twice
	Filter_1LP(&gyro_0[0], &gyro_1[0], fGYRO);
	Filter_1LP(&gyro_1[0], &gyro_2[0], fGYRO);
	Filter_1LP(&gyro_0[1], &gyro_1[1], fGYRO);
	Filter_1LP(&gyro_1[1], &gyro_2[1], fGYRO);
	Filter_1LP(&gyro_0[2], &gyro_1[2], fGYRO);
	Filter_1LP(&gyro_1[2], &gyro_2[2], fGYRO);
	// output
	gyro_f[0] = gyro_2[0].x;
	gyro_f[1] = gyro_2[1].x;
	gyro_f[2] = gyro_2[2].x;
*/
	gyro_f[0] = gyro[0];
	gyro_f[1] = gyro[1];
	gyro_f[2] = gyro[2];
}

const double MAGNETIC_VARIATION = 0.1018; // 5deg50'

void	CalcAtt_a(const double* acc, const double* mag, double* att_a)
{
	att_a[0] = atan(acc[1] / acc[2]);
	att_a[1] = atan(acc[0] / sqrt(acc[1]*acc[1] + acc[2]*acc[2]));
	att_a[2] = atan2(-cos(att_a[0])*mag[1] + sin(att_a[0])*mag[2],
					cos(att_a[1])*mag[0] + sin(att_a[0])*sin(att_a[1])*mag[1] + cos(att_a[0])*sin(att_a[1])*mag[2]);
	att_a[2] = PsaiMdf(att_a[2]  - MAGNETIC_VARIATION); // 地磁偏航角->地理偏航角：反向，补偿
}

void	CalcAcc_bodyframe( const double* acc_f, const double* att, double* acc_bodyframe)
{
	//x-axis
	acc_bodyframe[0] = acc_f[0] - sin(att[1]) * GRAVITY;
	//y-axis
	acc_bodyframe[1] = acc_f[1] - cos(att[1]) * sin(att[0]) * GRAVITY;
	
}

#define FLY_OUTDOOR
static const double kp = 0.4;	// 截止频率
double	Sfai_hat, Cfai_hat, Csita_hat, Tsita_hat;	// 中间变量
double	att_hat[3] = {0, 0, 0};	// attitude-hat, XYZ, rad
double	d_att_hat[3] = {0, 0, 0};	// d-attitude-hat, XYZ, rad/s
double	d_att[3] = {0, 0, 0};	// d-(Eular)attitude, XYZ, rad/s
double	vTemp;	// 缓存变量
void	AttFilter(const double* att_a, const double* gyro, double* att)
{
	// convert angular rate from BODY FRAME -> GROUND FRAME
	Sfai_hat = sin(att_hat[0]);
	Cfai_hat = cos(att_hat[0]);
	Csita_hat = cos(att_hat[1]);
	Tsita_hat = tan(att_hat[1]);
	d_att[0] = (gyro[1]*Sfai_hat + gyro[2]*Cfai_hat)*Tsita_hat + gyro[0];	// d-fai
	d_att[1] = gyro[1]*Cfai_hat - gyro[2]*Sfai_hat;	// d-sita
	d_att[2] = (gyro[1]*Sfai_hat + gyro[2]*Cfai_hat)/Csita_hat;	// d-psai

	// update in the loop
	vTemp = d_att_hat[0];	// Dfai_hat(n-1)
	d_att_hat[0] = d_att[0] - kp * (att_hat[0] - att_a[0]);
	att_hat[0] += DTIME * (vTemp + d_att_hat[0]) / 2;
	vTemp = d_att_hat[1];	// Dsita_hat(n-1)
	d_att_hat[1] = d_att[1] - kp * (att_hat[1] - att_a[1]);
	att_hat[1] += DTIME * (vTemp + d_att_hat[1]) / 2;
	vTemp = d_att_hat[2];	// Dpsai_hat(n-1)
#if defined(FLY_OUTDOOR)
	d_att_hat[2] = d_att[2] - kp * (PsaiMdf(att_hat[2] - att_a[2])); 
#elif defined(FLY_INDOOR)
	d_att_hat[2] = d_att[2];
#else
	#error  WARNING: flying indoor/outdoor not specified!
#endif
	att_hat[2] += DTIME * (vTemp + d_att_hat[2]) / 2;

	att_hat[2] = PsaiMdf(att_hat[2]);
	
	// output
	att[0] = att_hat[0];
	att[1] = att_hat[1];
	att[2] = att_hat[2];
}

const double	fACC = 35;	// 注意：此处为1阶低通的带宽，而下面级联了2个，即，带宽约为 fACC/2
struct filter_pair acc_0[3] = {{0, 0}, {0, 0}, {-GRAVITY, -GRAVITY}};	// 加速度计-当前与前一周期
struct filter_pair acc_1[3] = {{0, 0}, {0, 0}, {-GRAVITY, -GRAVITY}};	// 加速度计一阶滤波后-当前与前一周期
struct filter_pair acc_2[3] = {{0, 0}, {0, 0}, {-GRAVITY, -GRAVITY}};	// 加速度计二阶滤波后-当前与前一周期
void	AccFilter(const double* acc, double* acc_f)	
{
	// input & last input
	acc_0[0].x_ = acc_0[0].x;
	acc_0[0].x = acc[0];
	acc_0[1].x_ = acc_0[1].x;
	acc_0[1].x = acc[1];
	acc_0[2].x_ = acc_0[2].x;
	acc_0[2].x = acc[2];
	// filter twice
	Filter_1LP(&acc_0[0], &acc_1[0], fACC);
	Filter_1LP(&acc_1[0], &acc_2[0], fACC);
	Filter_1LP(&acc_0[1], &acc_1[1], fACC);
	Filter_1LP(&acc_1[1], &acc_2[1], fACC);
	Filter_1LP(&acc_0[2], &acc_1[2], fACC);
	Filter_1LP(&acc_1[2], &acc_2[2], fACC);
	// output
	acc_f[0] = acc_2[0].x;
	acc_f[1] = acc_2[1].x;
	acc_f[2] = acc_2[2].x;
}

const double	fMag = 30;	// 一阶低通的带宽。相当于截止频率15Hz
const double MAG_OFFSET[3] = {BIASX_CMPS, BIASY_CMPS, BIASZ_CMPS}; // CMPS X,Y,Z
#ifdef MAG_SCALING_ON
	const double MAG_SCALE[3] = {SCALEX_CMPS, SCALEY_CMPS, SCALEZ_CMPS};	// CMPS X, Y, Z
#else
	const double MAG_SCALE[3] = {1, 1, 1};
#endif
struct filter_pair mag_0[3] = {{0, 0}, {0, 0}, {0, 0}};
struct filter_pair mag_1[3] = {{0, 0}, {0, 0}, {0, 0}};
struct filter_pair mag_2[3] = {{0, 0}, {0, 0}, {0, 0}};
void	MagFilter(const int16* cmps_raw, double* mag)
{
	// input & last input
	mag_0[0].x_ = mag_0[0].x;
	mag_0[1].x_ = mag_0[1].x;
	mag_0[2].x_ = mag_0[2].x;

	//在这里试图将罗盘修改为倒置，程序中看，mag[0]为y方向的
	//mag[1]为x方向的，mag[2]为z方向的

	mag_0[0].x = (cmps_raw[0] * MAG_SCALE[0] - MAG_OFFSET[0]);
	mag_0[1].x = cmps_raw[1] * MAG_SCALE[1] - MAG_OFFSET[1];
	mag_0[2].x = (cmps_raw[2] * MAG_SCALE[2] - MAG_OFFSET[2]);
	// filter twice
	Filter_1LP(&mag_0[0], &mag_1[0], fMag);
	Filter_1LP(&mag_1[0], &mag_2[0], fMag);
	Filter_1LP(&mag_0[1], &mag_1[1], fMag);
	Filter_1LP(&mag_1[1], &mag_2[1], fMag);
	Filter_1LP(&mag_0[2], &mag_1[2], fMag);
	Filter_1LP(&mag_1[2], &mag_2[2], fMag);
	// output
	mag[0] = mag_2[0].x;
	mag[1] = mag_2[1].x;
	mag[2] = mag_2[2].x;
}

double	flp_temp;
void Filter_1LP(struct filter_pair* input, struct filter_pair* output, double fc)
{
/*
	一阶低通滤波：H(s) = 1/(1+s/fc)
	双线性变换转为数字滤波器
	Y[n](1+2/(Tfc)) + Y[n-1](1-2/(Tfc)) = U[n] + U[n-1]
	时间步长为DTIME=0.002
*/
	flp_temp = output->x;
	output->x_ = output->x;
	output->x = (input->x + input->x_ - output->x_*(1-2/(DTIME*fc))) / (1+2/(DTIME*fc));
	output->x_ = flp_temp;
}


int16	data_temp;
unsigned char cnt_MA;
void	MsgAppend_byte(Msg* msg, unsigned char byte)
{
	msg->frame[msg->len] = byte & 0xFF;
	msg->len ++;
}

void	MsgAppend_float(Msg* msg, const double* data, const unsigned char num)
{
	for(cnt_MA = 0; cnt_MA < num; cnt_MA ++)
	{
		data_temp = (int16)(100 * data[cnt_MA] + 0.5);
		msg->frame[msg->len] = data_temp>>8;
		msg->frame[msg->len+1] = data_temp & 0xff;
		msg->len += 2;
	}
}

void	MsgAppend_rad(Msg* msg, const double* data, const unsigned char num)
{
	for(cnt_MA = 0; cnt_MA < num; cnt_MA ++)
	{
		data_temp = (int16)(100 * rad2deg(data[cnt_MA]) + 0.5);
		msg->frame[msg->len] = data_temp>>8;
		msg->frame[msg->len+1] = data_temp & 0xff;
		msg->len += 2;
	}
}
void	MsgAppend_int(Msg* msg, const int* data, const unsigned char num)
{
	for(cnt_MA = 0; cnt_MA < num; cnt_MA ++)
	{
		data_temp = data[cnt_MA];
		msg->frame[msg->len] = data_temp>>8;
		msg->frame[msg->len+1] = data_temp & 0xff;
		msg->len += 2;
	}
}

#if defined(XBEE)
//const Uint32 net_address = ZIGBEE_ADDRESS;
unsigned char b_send[84];
unsigned char pHead = 0;
unsigned char pTail = 0;
void	MsgSend(Msg* msg)
{
	b_send[0] = 0x7E;

	b_send[1] = (((msg->len)-1) >>8) & 0xFF;
	b_send[2] = ((msg->len) - 1) & 0xFF;
	memcpy((char*)(&(b_send[3])), (char*)(msg->frame), msg->len);
	pTail = (msg->len) + 3;
	pHead = 0;
	ZigbeeSend();
}
#else
const Uint16 net_address = ZIGBEE_ADDRESS;
unsigned char b_send[84];
unsigned char pHead = 0;
unsigned char pTail = 0;
void	MsgSend(Msg* msg)
{
	b_send[0] = 0xFD;
	b_send[1] = msg->len;
	b_send[2] = (msg->dest >> 8) & 0xFF;
	b_send[3] = (msg->dest) & 0xFF;
	memcpy((char*)(&(b_send[4])), (char*)(msg->frame), msg->len);
	pTail = msg->len + 4;
	pHead = 0;
	ZigbeeSend();
}
#endif


void	ZigbeeSend()
{
	while(ScicRegs.SCIFFTX.bit.TXFFST<15 && pHead<pTail)	// FIFO未满 且 队列未发完
	{
		ScicRegs.SCITXBUF = b_send[pHead];
		pHead++;
	}
	// 若已发到队尾，则复位
	if (pHead == pTail)
	{
		pTail = 0;
		pHead = 0;
	}
	// 若发完了，则关闭中断；若还要发，则（保持）开启中断
	if (pTail == 0)
		ScicRegs.SCIFFTX.bit.TXFFIENA = 0;
	else
	{
		ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;	// 清除以前的中断
		ScicRegs.SCIFFTX.bit.TXFFIENA = 1;
	}
}

unsigned char strZigbeeResetAddr[7] = {0xFC, 0x02, 0x91, 0x01, 0xFF, 0xFF, 0x8E};
void	ZigbeeResetAddress()
{
	Scic_msg(strZigbeeResetAddr, 7);
}

unsigned char IsSendOK()
{
	if((pHead == 0) && (pTail == 0))
		return 1;
	else 
		return 0;
}

interrupt void ISRSciTxCFifo(void)
{
	ZigbeeSend();								// 继续MsgSend操作
	ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;	// 清中断位
	PieCtrlRegs.PIEACK.bit.ACK8 = 1;		// 响应
	EINT;
}

char	GPSstring[128];
unsigned char	lenGPS = 0;
inline void GPSDataAppend(unsigned char data)
{	
	if(data == '$')	// start of a packet
		lenGPS = 0;	// clear all
	if(lenGPS >= 127)
		lenGPS = 0;		// for protection
	// else
	GPSstring[lenGPS++] = data;
}

unsigned char GPSPacketReady()
{
	if(lenGPS <= 2)	// 下面有lenGPS-1 lenGPS-2
		return 0;
	if(GPSstring[lenGPS-1] == 0x0A && GPSstring[lenGPS-2]==0x0D)
		return 1;
	// else
	return 0;
}

#ifdef	SENSOR_GPS_UBLOX
struct PUBX GPS = {
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
};
#endif

unsigned char cnt_GPS, cnt_comma;
unsigned char pComma[20];	// 14 for GPHPP, 20 for PUBX 00, 12 for GPRMC


#ifdef	SENSOR_GPS_UBLOX
unsigned char	GPSPacketRead()	// Packet Read for GPS1000
{
	cnt_comma = 0;
	// 预处理
	for(cnt_GPS = 0; cnt_GPS < lenGPS; cnt_GPS++)
	{
		if(GPSstring[cnt_GPS] == ',')
		{
			GPSstring[cnt_GPS] = 0;
			pComma[cnt_comma] = cnt_GPS;
			cnt_comma ++;
		}
	}
	if(cnt_comma != 20)
	{
		lenGPS = 0;
		return 0;	// unavailable
	}
	if(strcmp("$PUBX", GPSstring) != 0)
	{
		lenGPS = 0;
		return 0;	// not PUBX
	}
	// unpacket
	// Field 1 should be '00'
	if(pComma[2] > pComma[1]+1)
	{	// Field 2: UTC Time: hhmmss.ss
		GPS.GPSTime = 0;
		GPS.GPSTime += atof(GPSstring+pComma[1]+5);
		*(GPSstring+pComma[1]+5) = ' ';
		GPS.GPSTime += atof(GPSstring+pComma[1]+3) * 60;
		*(GPSstring+pComma[1]+3) = ' ';
		GPS.GPSTime += atof(GPSstring+pComma[1]+1) * 3600;
	}
	if(pComma[3] > pComma[2]+1)
	{	// Field 3: Latitude-ddmm.mmmmmm 40.xxx / 39.xxx
		GPS.Latitude = atof(GPSstring+pComma[2]+5) / 60;
		*(GPSstring+pComma[2]+5) = ' ';
		GPS.Latitude += atof(GPSstring+pComma[2]+3) / 60;
		*(GPSstring+pComma[2]+3) = ' ';
		GPS.Latitude += atof(GPSstring+pComma[2]+1);
	}
	// Field 4 should be 'N', at northern hemisphere
	if(pComma[5] > pComma[4]+1)
	{	// Field 5: Longitude-dddmm.mmmmmm 116.xxx
		GPS.Longitude = atof(GPSstring+pComma[4]+6) / 60;
		*(GPSstring+pComma[4]+6) = ' ';
		GPS.Longitude += atof(GPSstring+pComma[4]+4) / 60;
		*(GPSstring+pComma[4]+4) = ' ';
		GPS.Longitude += atof(GPSstring+pComma[4]+1);
	}
	// Field 6 should be 'E', at northern hemisphere
	if(pComma[7] > pComma[6]+1)	GPS.Altitude = atof(GPSstring+pComma[6]+1);	// Field 7: Altitude
	if(pComma[8] > pComma[7]+1)
	{	// Field 8: Navigation Status
		GPS.NavStat[0] = *(GPSstring+pComma[7]+1);
		GPS.NavStat[1] = *(GPSstring+pComma[7]+2);
	}
	if(pComma[9] > pComma[8]+1)
	{	// Field 9: Horizontal Accuracy Estimate(m)
		GPS.Hacc = atof(GPSstring+pComma[8]+1);
	}
	if(pComma[10] > pComma[9]+1)
	{	// Field 10: Vertical Accuracy Estimate(m)
		GPS.Vacc = atof(GPSstring+pComma[9]+1);
	}
	if(pComma[11] > pComma[10]+1)
	{	// Field 11: Speed over Ground(km/h -> m/s)
		GPS.GSpeed = atof(GPSstring+pComma[10]+1) / 3.6;
	}
	if(pComma[12] > pComma[11]+1)
	{	// Field 12: Course over Ground(deg, NED frame)
		GPS.Track = atof(GPSstring+pComma[11]+1);
		if(GPS.Track > 180)
			GPS.Track -= 360;
		GPS.Ve = sin(GPS.Track*PI/180) * GPS.GSpeed;
		GPS.Vn = cos(GPS.Track*PI/180) * GPS.GSpeed;
	}
	if(pComma[13] > pComma[12]+1)
	{	// Field 13: Vertical Velocity(downward positive in protocal)
		GPS.Vu = -atof(GPSstring+pComma[12]+1);
	}
	// Field 14: ageC, probably empty
	if(pComma[15] > pComma[14]+1)
	{	// Field 15: HDOP
		GPS.HDOP = atof(GPSstring+pComma[14]+1);
	}
	if(pComma[16] > pComma[15]+1)
	{
		// Field 16: VDOP
		GPS.VDOP = atof(GPSstring+pComma[15]+1);
	}
	if(pComma[17] > pComma[16]+1)
	{	// Field 17: TDOP
		GPS.TDOP = atof(GPSstring+pComma[16]+1);
	}
	if(pComma[18] > pComma[17]+1)
	{	// Field 18: GU, Number of GPS Satellites used / NSV
		GPS.NSV = atoi(GPSstring+pComma[17]+1);
	}

	lenGPS = 0;	// not to unpacket again
	return 1;
}
#endif

#define	EARTH_RADIUS 6378140	// (m)

int16	BiasGPS_X=16;
int16	BiasGPS_Y=16;

long double	Latitude_zero=0;
long double	Longitude_zero=0;
long double	Atitude_zero=0;
long double Cos_Latitude_zero = 1;
double 	KLati2Meter = 0;
double  KLong2Meter = 0;
char	isSure=0;//表示飞机是否已经确认自己的零点位置


//在这里使用一种当计次大于一定次数的时候设置home point
int countHomePoint = 0;
int FlagSetHP =0 ;//为0表示没有设置
double homepoint[3] = {0, 0, 0};//归航点，感觉相当于位置零点

void	GetGPSPosition(double* p)	// position
{	// transform needed
	#if defined(HOME_DUGUG)
		p[0]=-10;
		p[1]=-10;
		p[2]=40;
		homepoint[0] = 0;
		homepoint[1] = 0;
		homepoint[2] = 40;

	#else
	if(!isSure) return;
	else
	{
	p[0] = (GPS.Latitude - Latitude_zero) * KLati2Meter - BiasGPS_X;
	p[1] = (GPS.Longitude - Longitude_zero) * KLong2Meter - BiasGPS_Y;
	p[2] = GPS.Altitude;


	if(!FlagSetHP)
		{
	countHomePoint = countHomePoint+1;
	if(countHomePoint==100)
			{

		homepoint[0] = p[0];
		homepoint[1] = p[1];
		homepoint[2] = p[2];
		FlagSetHP = 1;
		countHomePoint = 0;
			}
		}
	}
	#endif
}

double posGPS_[3]={0, 0, 0};//last period
void	GetGPSSpeed_V4(double* v, double* posGPS, double tGPS)
{
if(!isSure) return;
else
{
	v[0] =(posGPS[0]-posGPS_[0])/tGPS;
	v[1] =(posGPS[1]-posGPS_[1])/tGPS;
	v[2] =(posGPS[2]-posGPS_[2])/tGPS;
	posGPS_[0]=posGPS[0];
	posGPS_[1]=posGPS[1];
	posGPS_[2]=posGPS[2];
	}
}
void	GetGPSSpeed(double* v)		// speed
{
if(!isSure) return;
else
{

	v[0] = GPS.Vn;
	v[1] = GPS.Ve;
	v[2] = GPS.Vu;	// -?
}
}

void	GetGPSNSV(int16* n)
{
	*n = GPS.NSV;
	//>=6
	if((GPS.NSV >=8)&&(isSure==0))
	{
		Latitude_zero   = GPS.Latitude;
		Longitude_zero	= GPS.Longitude;
		Atitude_zero	= GPS.Altitude;
		Cos_Latitude_zero = cos(Latitude_zero/180 * PI);
		isSure =1;
		KLati2Meter = PI / 180 * EARTH_RADIUS;
		KLong2Meter = PI / 180 * EARTH_RADIUS * Cos_Latitude_zero;
	}




}

void	GetGPSTime(long double* t)
{
	*t = GPS.GPSTime;
}

void	GetGPSTrack(double* trackGPS)
{
	*trackGPS = GPS.Track;
}

void	GetGPSGSpeed(double* gspeedGPS)
{
	*gspeedGPS = GPS.GSpeed;
}

unsigned char rec_Scia;
interrupt void ISRSciRxAFifo(void)
{
	while(SciaRegs.SCIFFRX.bit.RXFFST > 0)
	{
		rec_Scia = SciaRegs.SCIRXBUF.all;
		GPSDataAppend(rec_Scia);
		// if(bAutoMode)	直接从Scic发出收到的数据
		//	ScicRegs.SCITXBUF = rec_Scia;
	}
	SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// 清中断位
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;		// 响应
	EINT;
}


void GPSPacketClean()
{
	lenGPS = 0;
}

unsigned char rec_Scic;
interrupt void ISRSciRxCFifo(void)
{
	while(ScicRegs.SCIFFRX.bit.RXFFST > 0)
	{
		rec_Scic = ScicRegs.SCIRXBUF.all;
		WirelessDataAppend(rec_Scic);
	}
	ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
	ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;	// 清中断位
	PieCtrlRegs.PIEACK.bit.ACK8 = 1;		// 响应
	EINT;
}

char	Wirelessstring[32];
unsigned char	lenWRL = 0;
Msg msgRecv;
Msg msgRead;
void WirelessDataAppend(unsigned char data)
{
#if defined(XBEE)
	lenWRL++;
	if ((data != 0x7E && lenWRL == 1)	// the first letter is not 0x7E!
		//|| (lenWRL >= 32)					// for protection//不明白
		|| (lenWRL > (msgRecv.len) + 3))	// more than specified
	{
		MsgReset(&msgRecv);
		lenWRL = 0;
		return;
	}

	if (lenWRL == 1)	// 0x7E, continue
		;
	else if (lenWRL == 2)	// length
	{
		msgRecv.len &= 0x00FF;
		msgRecv.len |= (data & 0xFF) << 8;
	}
	else if (lenWRL == 3)	// length
	{
		msgRecv.len &= 0xFF00;
		msgRecv.len |= data & 0xFF;
		//msgRecv.len -= 8;
		msgRecv.len += 1;

	}
	else if (lenWRL <= msgRecv.len + 3)	// payload
	{
		msgRecv.frame[lenWRL - 4] = data;
		if (lenWRL == msgRecv.len + 3)
		{
			msgRecv.srcH &= 0x00FFFFFF;
			msgRecv.srcH |= ((Uint32)msgRecv.frame[1] & 0xFF) << 24;
			msgRecv.srcH &= 0xFF00FFFF;
			msgRecv.srcH |= ((Uint32)msgRecv.frame[2] & 0xFF) << 16;
			msgRecv.srcH &= 0xFFFF00FF;
			msgRecv.srcH |= (msgRecv.frame[3] & 0xFF) << 8;
			msgRecv.srcH &= 0xFFFFFF00;
			msgRecv.srcH |= (msgRecv.frame[4] & 0xFF);

			msgRecv.srcL&= 0x00FFFFFF;
			msgRecv.srcL |= ((Uint32)msgRecv.frame[5] & 0xFF) << 24;
			msgRecv.srcL &= 0xFF00FFFF;
			msgRecv.srcL |= ((Uint32)msgRecv.frame[6] & 0xFF) << 16;
			msgRecv.srcL &= 0xFFFF00FF;
			msgRecv.srcL |= (msgRecv.frame[7] & 0xFF) << 8;
			msgRecv.srcL &= 0xFFFFFF00;
			msgRecv.srcL |= (msgRecv.frame[8] & 0xFF);
			// packet finished
			msgRead = msgRecv;
			MsgReset(&msgRecv);
			lenWRL = 0;
		}
	}

#else
	lenWRL ++;
	if(	(data != 0xFD && lenWRL == 1)	// the first letter is not 0xFD!
	||	(lenWRL >= 32)					// for protection
	||	(lenWRL > (msgRecv.len)+6) )	// more than specified
	{
		MsgReset(&msgRecv);
		lenWRL = 0;
		return;
	}

	if(lenWRL == 1)	// 0xFD, continue
		;
	else if(lenWRL == 2)	// length
		msgRecv.len = data;
	else if(lenWRL == 3)	// DEST address
	{
		msgRecv.dest &= 0x00FF;
		msgRecv.dest |= (data & 0xFF) << 8;
	}
	else if(lenWRL == 4)
	{
		msgRecv.dest &= 0xFF00;
		msgRecv.dest |= data & 0xFF;
	}
	else if(lenWRL <= msgRecv.len + 4)	// payload
		msgRecv.frame[lenWRL-5] = data;
	else if(lenWRL == msgRecv.len + 5)	// SRC address
	{
		msgRecv.src &= 0x00FF;
		msgRecv.src |= (data & 0xFF) << 8;
	}
	else if(lenWRL == msgRecv.len + 6)
	{
		msgRecv.src &= 0xFF00;
		msgRecv.src |= data & 0xFF;
		// packet finished
		msgRead = msgRecv;
		MsgReset(&msgRecv);
		lenWRL = 0;
	}
#endif
}

Msg* IsMsgReady()
{
	if(IsMsgEmpty(&msgRead))
		return 0;
	else
		return &msgRead;
}


/*
	About Radio Controller
*/
#define ECAP_MS2COUNT 135000
unsigned char cnt_GRC = 0;
double	RC_width[6] = {1.5, 1.5, 1, 1.5, 1.5, 1.5};
unsigned char cnt_RC6 = 0;
unsigned char bRC6high = 0;
void	GetRC(double* pulsewidth)
{	
	for (cnt_GRC = 0; cnt_GRC < 6; cnt_GRC++)
	{
		if(RC_width[cnt_GRC] < 0.8 || RC_width[cnt_GRC] > 2.2) // abnormal
			continue;
		else	// refresh
		{
			if(cnt_GRC == 5) // RC_6
			{// two samples make it valid
				if(bRC6high)
				{
					if(RC_width[5] < 1.5)
					{
						bRC6high = 0;
						cnt_RC6 = 0;
					}
					else  // >1.5
					{
						cnt_RC6 ++;
					}
				}
				else // bRC6 low
				{
					if(RC_width[5] < 1.5)
					{
						cnt_RC6 ++;
					}
					else // >1.5
					{
						bRC6high = 1;
						cnt_RC6 = 0;
					}
				}
				if(cnt_RC6 > 21)	// 2 cycles
				{
					pulsewidth[cnt_GRC] = RC_width[cnt_GRC];
					cnt_RC6 = 21;
				}
			}
			else	// normal case
				pulsewidth[cnt_GRC] = RC_width[cnt_GRC];
		}
	}
}	

// RCx_ISR指定从ECAPx口读入，RC_width[x]指定是x通道
unsigned char	bShutdown = 1;	// shutdown or not (Ch3 < 1.05) - about shutdown switch
unsigned char	bThroOff = 1;	// throttle off - about output throttle
interrupt void RC3_ISR(void)
{
	if(ECap3Regs.ECFLG.bit.CEVT2 == 1)
		RC_width[2] = 1.0 * ECap3Regs.CAP2 / ECAP_MS2COUNT;	// (ms)
	else if (ECap3Regs.ECFLG.bit.CEVT4 == 1)
		RC_width[2] = 1.0 * ECap3Regs.CAP4 / ECAP_MS2COUNT;	// (ms)

	if(RC_width[2] < 1.05)
		bShutdown = 1;
	else if(RC_width[2] <2.1)
		bShutdown = 0;

	ECap3Regs.ECCLR.bit.INT = 0x01;	// 只清除中断标记，但CET1-CET4可能继续产生中断
	ECap3Regs.ECCLR.all = 0x1F;	// 把CET1-CET4都清理掉

	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP4;
}

interrupt void RC1_ISR(void)
{
	if(ECap1Regs.ECFLG.bit.CEVT2 == 1)

		RC_width[0] = 1.0 * ECap1Regs.CAP2 / ECAP_MS2COUNT;	// (ms)
	else if (ECap1Regs.ECFLG.bit.CEVT4 == 1)
		RC_width[0] = 1.0 * ECap1Regs.CAP4 / ECAP_MS2COUNT;	// (ms)

	ECap1Regs.ECCLR.bit.INT = 0x01;	// 只清除中断标记，但CET1-CET4可能继续产生中断
	ECap1Regs.ECCLR.all = 0x1F;	// 把CET1-CET4都清理掉

	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP4;
}

interrupt void RC2_ISR(void)
{
	if(ECap2Regs.ECFLG.bit.CEVT2 == 1)
		RC_width[1] = 1.0 * ECap2Regs.CAP2 / ECAP_MS2COUNT;	// (ms)
	else if (ECap2Regs.ECFLG.bit.CEVT4 == 1)
		RC_width[1] = 1.0 * ECap2Regs.CAP4 / ECAP_MS2COUNT;	// (ms)

	ECap2Regs.ECCLR.bit.INT = 0x01;	// 只清除中断标记，但CET1-CET4可能继续产生中断
	ECap2Regs.ECCLR.all = 0x1F;	// 把CET1-CET4都清理掉

	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP4;
}

interrupt void RC4_ISR(void)
{
	if(ECap4Regs.ECFLG.bit.CEVT2 == 1)
		RC_width[3] = 1.0 * ECap4Regs.CAP2 / ECAP_MS2COUNT;	// (ms)
	else if (ECap4Regs.ECFLG.bit.CEVT4 == 1)
		RC_width[3] = 1.0 * ECap4Regs.CAP4 / ECAP_MS2COUNT;	// (ms)

	ECap4Regs.ECCLR.bit.INT = 0x01;	// 只清除中断标记，但CET1-CET4可能继续产生中断
	ECap4Regs.ECCLR.all = 0x1F;	// 把CET1-CET4都清理掉

	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP4;
}

interrupt void RC6_ISR(void)
{
	if(ECap6Regs.ECFLG.bit.CEVT2 == 1)
		RC_width[5] = 1.0 * ECap6Regs.CAP2 / ECAP_MS2COUNT;	// (ms)
	else if (ECap6Regs.ECFLG.bit.CEVT4 == 1)
		RC_width[5] = 1.0 * ECap6Regs.CAP4 / ECAP_MS2COUNT;	// (ms)

	ECap6Regs.ECCLR.bit.INT = 0x01;	// 只清除中断标记，但CET1-CET4可能继续产生中断
	ECap6Regs.ECCLR.all = 0x1F;	// 把CET1-CET4都清理掉

	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP4;
}

unsigned char bUSIdle = 1;	// is URM37 idle?
double	Height_URM37 = 0.05;
interrupt void USonic_ISR(void)
{
	if(ECap5Regs.ECFLG.bit.CEVT1 == 1)
	{
		Height_URM37 = (Uint16)(1.0 * ECap5Regs.CAP1 * 20 / ECAP_MS2COUNT + 0.5) * 0.01; 	// (m; 精度到cm)
		bUSIdle = 1;
	}
	else if (ECap5Regs.ECFLG.bit.CEVT3 == 1)
	{
		Height_URM37 = (Uint16)(1.0 * ECap5Regs.CAP3 * 20 / ECAP_MS2COUNT + 0.5) * 0.01; 	// (m; 精度到cm)
		bUSIdle = 1;
	}
	ECap5Regs.ECCLR.bit.INT = 0x01;	// 只清除中断标记，但CET1-CET4可能继续产生中断
	ECap5Regs.ECCLR.all = 0x1F;	// 把CET1-CET4都清理掉

	PieCtrlRegs.PIEACK.all|=PIEACK_GROUP4;
}

void	GetHeightURM37(double* height)
{
	*height = Height_URM37;
}

int16	cnt_URMTrigger = 0;
void	URM_Trigger()	// 应改为上次测量结束才发起下一次测量
{
	if(cnt_URMTrigger == 1)
	{
		URM37_TRIG = 0;
		bUSIdle = 0;	// Triggered, URM37 not idle
	}
	else if(cnt_URMTrigger == 3)
	{
		URM37_TRIG = 1;
	}
	else if(cnt_URMTrigger*DTIME >= 0.2 - 1e-5)	// 5Hz
	{	// TIME OUT?
		cnt_URMTrigger = 0;
		bUSIdle = 1;
	}
	else if(cnt_URMTrigger*DTIME >= 0.05 - 1e-5) // 20Hz
	{
		if(bUSIdle)
			cnt_URMTrigger = 0;
		// else, continue to wait
	}
	cnt_URMTrigger ++;
}

#define	RC_MAXPITCH	(PI/4)		// rad
#define	RC_MAXYAWSPEED (PI/2)	// rad/s
double	RCInput;
double 	sensitivity[3]= {1, 1, 1};//sensitivity of 3 axises.
double  RC_alfa=0.95;//cut fre---cof relative
void	RCAttSet(double* rcctrl, double* att_set, const double* att)
{	// rcctrl(ms), att_set(rad)
	if(!att_set)	// att_set == NULL, run for filter
		return;

	//- Ch1 - roll
	RCInput =	sensitivity[0] * ( rcctrl[0] - 1.5 );
	if(RCInput < 0.005 && RCInput > -0.005)
		att_set[0] = att_set[0] * RC_alfa + 0;
	else
		att_set[0] = att_set[0] * RC_alfa + RCInput / 0.5 * RC_MAXPITCH * (1-RC_alfa);	// ~100Hz

	//- Ch2 - pitch
	RCInput = - sensitivity[1] * ( rcctrl[1] - 1.5 );
	if(RCInput < 0.005 && RCInput > -0.005)
		att_set[1] = att_set[1] * RC_alfa + 0;
	else
		att_set[1] = att_set[1] * RC_alfa + RCInput / 0.5 * RC_MAXPITCH * (1-RC_alfa);	// ~100Hz
	
	//- Ch4 - yaw(speed)
	RCInput = - sensitivity[2] * (rcctrl[3] - 1.5);
	if(RCInput < 0.01 && RCInput > -0.01)	// 死区，略大一些，因为是控制速度
		RCInput = 0;
	att_set[2] = att_set[2] + RCInput / 0.5 * RC_MAXYAWSPEED * DTIME;
	att_set[2] = PsaiMdf(att_set[2]);
	if(bThroOff)
		att_set[2] = att[2];
}
//super simple mode

double RCinputSM[2]={1.5,1.5};
const double RCinputDZ[3]={0.1,0.1,0.1};//dead zone, Unit:ms
const double KitoVel[2]={5,5};//遥控器脉宽给定的速度比例系数
double vel1_set[2]={0,0};
double pos1_set[2]={0,0};
//including the home point
double anglenow=0;


void	RCPosSet(double* rcctrl, double* pos, double*RCpos_set, double* RCspd_set, double*RCacc_set)
{
//得到和home点之间方位角
		double tempvel[2]={0,0};
		
		anglenow = atan2((pos[1]-homepoint[1]),(pos[0]-homepoint[0]));
	//RCinput[0];
		RCinputSM[0] = sensitivity[0] * ( rcctrl[0] - 1.5 );
		if(fabs(RCinputSM[0])>RCinputDZ[0])
		{
			tempvel[0] = (RCinputSM[0]>0 ? (RCinputSM[0]-RCinputDZ[0]):(RCinputSM[0]+RCinputDZ[0]));
			vel1_set[0] = 0.95 * vel1_set[0] + (1-0.95) * KitoVel[0]* tempvel[0];

		}
		
		else
		{
			vel1_set[0] = 0.95 * vel1_set[0];
		}
			pos1_set[0] = pos1_set[0] + DTIME * vel1_set[0];

	// RCinput[1];
		RCinputSM[1] = - sensitivity[1] * ( rcctrl[1] - 1.5 );
		if(fabs(RCinputSM[1])>RCinputDZ[1])
		{
			tempvel[1] = (RCinputSM[1]>0 ? (RCinputSM[1]-RCinputDZ[1]):(RCinputSM[1]+RCinputDZ[1]));
			vel1_set[1] = 0.95 * vel1_set[1] + (1-0.95) * KitoVel[1]* tempvel[1];

		}
		
		else
		{
			vel1_set[1] = 0.95 * vel1_set[1];
		}
			pos1_set[1] = pos1_set[1] + DTIME * vel1_set[1];

		RCspd_set[0]= - vel1_set[1] * cos(anglenow) -  vel1_set[0] * sin(anglenow);
		RCpos_set[0]= - pos1_set[1] * cos(anglenow) -  pos1_set[0] * sin(anglenow);

		RCspd_set[1]= - vel1_set[1] * sin(anglenow) +  vel1_set[0] * cos(anglenow);
		RCpos_set[1]= - pos1_set[1] * sin(anglenow) +  pos1_set[0] * cos(anglenow);
	//double homepoint[3] = {0, 0, 0};
}

void	RCYawSet(double* rcctrl,double* yaw_set)
{
		//- Ch4 - yaw(speed)
	double tempyaw=0;
	RCInput = - sensitivity[2] * (rcctrl[3] - 1.5);
	if(fabs(RCInput) <RCinputDZ[2])	// 死区，略大一些，因为是控制速度
		tempyaw = 0;
	else
		tempyaw=(RCInput>0 ? (RCInput-RCinputDZ[2]) : (RCInput+RCinputDZ[2]));
	(*yaw_set) = (*yaw_set)  + tempyaw / 0.5 * RC_MAXYAWSPEED * DTIME;
	(*yaw_set)  = PsaiMdf((*yaw_set));
//	if(bThroOff)
//		att_set[2] = att[2];
}

unsigned char IsAutoMode(double ChannelSwitch)
{
	if(ChannelSwitch > 1.5)
		return 1;
	else
		return 0;
}


const double PULSE_MIN = 0.99;	// 0.15-0.2之间部分为启动，可能不稳定
const double PULSE_MAX = 1.97 - 0.0005;



#define	MAX_THRUST	50
void GetThrottle(double * throttle, double * output)	// throtle(ms); output(N)
{
	if (*throttle < 1)	// 排除少量越界的干扰
		*output = 0;	
	else if (*throttle > 2)
		*output = MAX_THRUST;
	else 
		*output = (*throttle - 1) * MAX_THRUST;	// max: 32N
}
// 最外层的控制器，输入state姿态/位置/etc，输入input设定姿态/位置/etc，输出四个脉宽



// 控制参数
#if defined(ROTOR_30A_580)
double Kp_roll = 8;
double Kd_roll = 3;	// available: 1.8
double Ki_roll = 0;
double Kp_pitch = 8;
double Kd_pitch = 3;
double Ki_pitch = 0;
double Kp_yaw = 1;  //0.1468;
double Kd_yaw = 0.4;  //0.2;
double Ki_yaw = 0.1;
double K_x_acc=0;//对于姿态环控制X轴方向
double K_y_acc=0;
double Kp_x_roll=0;//对于陀螺仪角速度的反馈
double Kq_y_pitch=0;
double Kr_z_yaw=0;
double fre_anglerate=10;//角速度方向的低通参数
double alfa_accfeedback=0.97;//速度方向的滤波系数
#elif defined(ROTOR_X650)
double Kp_roll = 6.5;
double Kd_roll = 1.5;	// available: 1.8
double Ki_roll = 0;
double Kp_pitch = 6.5;
double Kd_pitch = 1.5;
double Ki_pitch = 0;
double Kp_yaw = 0.5;  //0.1468;
double Kd_yaw = 0.2;  //0.2;
double Ki_yaw = 0;
#elif defined(ROTOR_X650V)
double Kp_roll = 10;
double Kd_roll = 4;	// available: (2.2, 1.4/1.8)
double Ki_roll = 0;
double Kp_pitch = 10;
double Kd_pitch = 4;
double Ki_pitch = 0;
double Kp_yaw = 0.5;
double Kd_yaw = 0.2;
double Ki_yaw = 0.1;
#endif
double Ts_roll = 0.01;
double Ts_pitch = 0.01;
double Ts_yaw = 0.01;
// 限幅用于姿态控制的差分量（脉宽ms）
//const double U2_sat = 0.6;
//const double U3_sat = 0.6;
//const double U4_sat = 0.8;
const double U2_sat = 0.6;
const double U3_sat = 0.6;
const double U4_sat = 0.8;
// 差分输出限幅值（相当于每个因素对电机脉宽的调整量只有300us以内[200 for yaw]）- cancelled
const double U2d_sat = 1e5;
const double U3d_sat = 1e5;
const double U4d_sat = 1e5;
// 积分器及其限幅
double	U2i = 0;
double	U3i = 0;
double 	U4i = 0;
//const double U2i_sat = 0.3;
//const double U3i_sat = 0.3;
//const double U4i_sat = 0.4;
const double U2i_sat = 0.3;
const double U3i_sat = 0.3;
const double U4i_sat = 0.4;
// robust compensation
const double	a_model[3] = {0.275/0.029, 0.275/0.029, 1/6.255e-2};
const double	T_model[3] = {0.1, 0.1, 0.1};
const double	RC_ATT_SAT[3] = {0.6, 0.6, 0.8};
	// - x''=au -- a[0, 1]=l/I, a[2]=1/I_z
double	rc_att_z[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};	
double	rc_att_z_[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};	
double	rc_att_out[3] = {0, 0, 0};
double	rc_att_u_[3] = {0, 0, 0};
double	f[3] = {0, 0, 0};//{20, 20, 0};		//30
double	g[3] = {0, 0, 0};//{4, 4, 0};		//8
//- z[i][j]: z_(i+1) for channel j(roll, pitch, yaw); _ indicates last cycle

// 变量
struct filter_pair err_roll = {0, 0};
struct filter_pair err_pitch = {0, 0};
struct filter_pair err_yaw = {0, 0};
struct filter_pair err_roll_f = {0, 0};
struct filter_pair err_pitch_f = {0, 0};
struct filter_pair err_yaw_f = {0, 0};
double U2_= 0, U3_ = 0, U4_ = 0;	// 上一周期的控制量
double	U2 = 0;
double	U3 = 0;
double	U4 = 0;// 中间四位控制量，前2个升力和/差，最后一个扭矩差
double	U2d = 0, U3d = 0, U4d = 0;	// 速度反馈项

double att_set_V4[3] = {0,0,0};
double gyro_V4[3] = {0,0,0};
double Uout_gyro_[3] = {0,0,0};
double Uout_gyro[3] = {0,0,0};

double Uout_V4plus_[3] = {0,0,0};
double Uout_V4plus[3] = {0,0,0};
double accx_feed_=0;
double accy_feed_=0;
double accx_feed=0;
double accy_feed=0;


double Kp_x = 0.8;
double Kp_y = 0.8;
double Kp_z = 0.025;
double Kd_x = 0.15;
double Kd_y = 0.15;
double Kd_z = 1;
double Ki_x = 0;
double Ki_y = 0;
double Ki_z = 0.2;
double att_set_NAV[2] = {0, 0};	// att_set in navigation frame. (rad; X, Y)
struct filter_pair att_0[2] = {{0, 0}, {0, 0}};	// 给定姿态角 - 当前与前一周期
struct filter_pair att_1[2] = {{0, 0}, {0, 0}}; // 给定姿态角一阶滤波后 - 当前与前一周期
double intgX = 0;
double intgY = 0;
unsigned char bTurnAuto = 0;
double pdX = 0;	// non-integral part
double pdY = 0;
const double INTG_POS_SAT = 0.15;	// 8.6 deg 
// unsigned char bIntgOn = 1;
#define	POS_TIME 0.0025

double err_height = 0;
double err_height_ = 0;	// previous err_height_
double height_ = 0;		// previous height
double err_vheight = 0;
double err_vheight_ = 0;	// previous err_vheight
double vheight_ = 0;	// previous vheight
double intg_height = 0;	// integrator of height
//const double basicOutFly=33.5;	//正常飞机在空中时候的值
//const double basicOutGround=38; //地面上快速起来时候的值
const double basicOutFly=25.2;	//正常飞机在空中时候的值
const double basicOutGround=27; //地面上快速起来时候的值
//double basicOut=33.5; //最终给高度控制的值
double basicOut=25.2; //最终给高度控制的值
char isTakeOff = 1;//表示是否起飞的标识，1表示起飞状态，包括之前的状态，0表示再正常飞行或者着陆的状态

unsigned char cnt_SV = 0;
void SetVector(const double* vsrc, double* vdest, const unsigned char n)
{
	for(cnt_SV = 0; cnt_SV < n; cnt_SV++)
		vdest[cnt_SV] = vsrc[cnt_SV];
}
extern double ax1, ax2, av;
void	SetCtrlPara(Cmd* cmd)
{
	switch(cmd->code)
	{
	// GROUP 0x3* Set Parameter
	case CMD_SETPARA_PID:
		if(cmd->para[0] != 0x7FFF) Kp_x = cmd->para[0] * 0.001;
		if(cmd->para[1] != 0x7FFF) Ki_x = cmd->para[1] * 0.001;
		if(cmd->para[2] != 0x7FFF) Kd_x = cmd->para[2] * 0.001;
		if(cmd->para[3] != 0x7FFF) Kp_y = cmd->para[3] * 0.001;
		if(cmd->para[4] != 0x7FFF) Ki_y = cmd->para[4] * 0.001;
		if(cmd->para[5] != 0x7FFF) Kd_y = cmd->para[5] * 0.001;
		if(cmd->para[6] != 0x7FFF) Kp_z = cmd->para[6] * 0.001;
		if(cmd->para[7] != 0x7FFF) Ki_z = cmd->para[7] * 0.001;
		if(cmd->para[8] != 0x7FFF) Kd_z = cmd->para[8] * 0.001;
		break;
	case CMD_SETPARA_CONSENSUS:
		if(cmd->para[0] != 0x7FFF) Kp_x = cmd->para[0] * 0.001;
		if(cmd->para[1] != 0x7FFF) Ki_x = cmd->para[1] * 0.001;
		if(cmd->para[2] != 0x7FFF) Kd_x = cmd->para[2] * 0.001;
		if(cmd->para[3] != 0x7FFF) Kp_y = cmd->para[3] * 0.001;
		if(cmd->para[4] != 0x7FFF) Ki_y = cmd->para[4] * 0.001;
		if(cmd->para[5] != 0x7FFF) Kd_y = cmd->para[5] * 0.001;
		if(cmd->para[6] != 0x7FFF) Kp_z = cmd->para[6] * 0.001;
		if(cmd->para[7] != 0x7FFF) Ki_z = cmd->para[7] * 0.001;
		if(cmd->para[8] != 0x7FFF) Kd_z = cmd->para[8] * 0.001;
		break;
	case CMD_SETPARA_ADAPTIVE:	// inner loop robust compensation
		if(cmd->para[0] != 0x7FFF) ax1 = cmd->para[0] * 0.001;
		if(cmd->para[1] != 0x7FFF) ax2 = cmd->para[1] * 0.001;
		if(cmd->para[2] != 0x7FFF) av = cmd->para[2] * 0.001;
		break;
	default:
		break;
	}
}



//extern unsigned char bTrajRunning;	// if a trajectory is on the run
/*
void	SetRefDestPos(Cmd* cmd)
{
	static struct flypoint point_set = {0, 0, 0, 0, 0, 0};
	 t0=CpuTimer0.InterruptCount*DTIME;
	switch(cmd->code)
	{

	// GROUP 0xD* Set Reference Track (and speeds)
	case CMD_SETREF_LINEAB:	// line, absolute position
		point_set.x = cmd->para[0] * 0.01;
		point_set.y = cmd->para[1] * 0.01;
		point_set.z = cmd->para[2] * 0.01;
		point_set.vx = cmd->para[3] * 0.01;
		point_set.vy = cmd->para[4] * 0.01;
		point_set.vz = cmd->para[5] * 0.01;
		point_set.traj = TRAJ_LINE;
		SetNextPoint(&traj_ref, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);
		break;

	case CMD_SETREF_LINERE:	// line, relative position
		point_set.x = cmd->para[0] * 0.01;
		point_set.y = cmd->para[1] * 0.01;
		//point_set.z = cmd->para[2] * 0.01;v
		point_set.vx = cmd->para[3] * 0.01;
		point_set.vy = cmd->para[4] * 0.01;
		//point_set.vz = cmd->para[5] * 0.01;
		point_set.traj = TRAJ_LINE;
		SetNextPoint(&traj_ref, &point_set, RELATIVE_LINE, CpuTimer0.InterruptCount*DTIME);
		break;
	case CMD_SETREF_ARCAB:	// arc, absolute position
		point_set.cx = cmd->para[0] * 0.01;
		point_set.cy = cmd->para[1] * 0.01;
		point_set.z = cmd->para[2] * 0.01;
		point_set.v = cmd->para[3] * 0.01;
		point_set.r = cmd->para[6] * 0.01;
		point_set.loop = cmd->para[7] * 0.01;
		point_set.round = cmd->para[8] * 0.01;
		point_set.traj = TRAJ_ARC;
		if(point_set.r > 0)
			SetNextPoint(&traj_ref, &point_set, ABSOLUTE_ARC, CpuTimer0.InterruptCount*DTIME);
		break;

	case CMD_SETREF_ARCRE:	// arc, relative position
		point_set.cx = cmd->para[0] * 0.01;
		point_set.cy = cmd->para[1] * 0.01;
		point_set.z = cmd->para[2] * 0.01;
		point_set.v = cmd->para[3] * 0.01;
		point_set.r = cmd->para[6] * 0.01;
		point_set.loop = cmd->para[7] * 0.01;
		point_set.round = cmd->para[8] * 0.01;
		point_set.traj = TRAJ_ARC;
		if(point_set.r > 0)
			SetNextPoint(&traj_ref, &point_set, RELATIVE_ARC, CpuTimer0.InterruptCount*DTIME);
		break;
	case CMD_SETREF_BUTTERFLYRE:	// butterfly(8), relative position
		point_set.cx = cmd->para[0] * 0.01;
		point_set.cy = cmd->para[1] * 0.01;
		point_set.z = cmd->para[2] * 0.01;
		point_set.v = cmd->para[3] * 0.01;
		point_set.r = cmd->para[6] * 0.01;
		point_set.loop = cmd->para[7] * 0.01;
		point_set.traj = TRAJ_BUTTERFLY;
		if(point_set.r > 0)
			SetNextPoint(&traj_ref, &point_set, RELATIVE_BUTTERFLY, CpuTimer0.InterruptCount*DTIME);
		break;
	case CMD_SETREF_EIGHTAB:	// 8, absolute position
		if(cmd->para[6] * 0.01 < 1)	// too close, dangerous
			break;
		if(cmd->para[8] * 0.01 * FORMATION_INDEX > 360)	// too many agents in this set formation
			break;
		point_set.cx = cmd->para[0] * 0.01;
		point_set.cy = cmd->para[1] * 0.01;
		point_set.z = cmd->para[2] * 0.01;
		point_set.v = cmd->para[3] * 0.01;
		point_set.r = cmd->para[6] * 0.01;
		point_set.loop = cmd->para[7] * 0.01;
		point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;
		point_set.traj = TRAJ_EIGHT;
		if(point_set.r > 0)
			SetNextPoint(&traj_ref, &point_set, ABSOLUTE_EIGHT, CpuTimer0.InterruptCount*DTIME);
		break;

	case CMD_SETREF_EIGHTRE:	// 8, relative position
		point_set.cx = cmd->para[0] * 0.01;
		point_set.cy = cmd->para[1] * 0.01;
		point_set.z = cmd->para[2] * 0.01;
		point_set.v = cmd->para[3] * 0.01;
		point_set.r = cmd->para[6] * 0.01;
		point_set.loop = cmd->para[7] * 0.01;
		point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;
		point_set.traj = TRAJ_EIGHT;
		if(point_set.r > 0)
			SetNextPoint(&traj_ref, &point_set, RELATIVE_EIGHT, CpuTimer0.InterruptCount*DTIME);
		break;
	default:
		break;
	}
}
*/
extern double pos[3], pos_zero[3];
extern Traj traj_ab;
extern	double	p_c[3];
extern double w1,w2,w3,w4;
//extern double k21;
extern int16 sendType;
extern ScdFlight scdFlight;
struct flypoint point_set = {0, 0, 0, 0, 0, 0};
void	SetDestPos(Cmd* cmd)
{

//	static struct flypoint point_set = {0, 0, 0, 0, 0, 0};

	// can't interrupt a trajectory except HALT command -- not a good design but makes sense
	if(cmd->code != CMD_SETSTATE_HALT)	// for normal trajectories
	{
		if(traj_ab.bTrajRunning == 1)	// then ignore incoming trajectory
			return;
		else
			traj_ab.bTrajRunning = 1;
		    t0=CpuTimer0.InterruptCount*DTIME;
	}
	switch(cmd->code)
	{
		case CMD_SETDEST_SWARMAB:	// form into a swarm, absolute position
				sendType = 72;
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6] * 0.01 * sin(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6] * 0.01 * cos(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);

				point_set.vx = cmd->para[3] * 0.01;
				point_set.vy = cmd->para[4] * 0.01;

				point_set.traj = TRAJ_LINE;
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
				break;
		case CMD_SETDEST_SWARM_RING://形成两个交错的圆
				sendType = 74;
				if (FORMATION_INDEX % 2 == 0)
				{
					point_set.x = cmd->para[0] * 0.01 + cmd->para[6] * 0.01 * sin(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
					point_set.y = cmd->para[1] * 0.01 + cmd->para[6] * 0.01 * cos(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
				}
				else
				{
					point_set.x = cmd->para[0] * 0.01 + cmd->para[2] * 0.01 * sin(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
					point_set.y = cmd->para[1] * 0.01 + cmd->para[2] * 0.01 * cos(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
				}
				point_set.vx = cmd->para[3] * 0.01;
				point_set.vy = cmd->para[4] * 0.01;

				point_set.traj = TRAJ_LINE;
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
				break;
		case CMD_SETDEST_SWARM_DOUBLE:
				sendType = 75;
				if (FORMATION_INDEX <= 3)
				{
					point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01*3/2 + cmd->para[6] * 0.01 * sin(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
					point_set.y = cmd->para[1] * 0.01 + cmd->para[6] * 0.01 * cos(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
				}
				else
				{
					point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01*3/2 + cmd->para[6] * 0.01 * sin(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
					point_set.y = cmd->para[1] * 0.01 + cmd->para[6] * 0.01 * cos(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
				}
				point_set.vx = cmd->para[3] * 0.01;
				point_set.vy = cmd->para[4] * 0.01;
				point_set.traj = TRAJ_LINE;
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
				break;
		case CMD_SETDEST_SWARM_ONE://form into “一”
			sendType = 76;
			#if defined (CAR_1)
			{
				point_set.x = cmd->para[0] * 0.01;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_2)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01*2/3;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_3)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01*4/3;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_4)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_5)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01*1/3;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_8)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01*1/3;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_9)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_10)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01*2/3;
				point_set.y = cmd->para[1] * 0.01;
			}
			#endif

			point_set.vx = cmd->para[3] * 0.01;
			point_set.vy = cmd->para[4] * 0.01;

			point_set.traj = TRAJ_LINE;
			SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
			break;

		case CMD_SETDEST_SWARM_TWO://form into “二”
			sendType = 77;
			#if defined (CAR_1)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01*1/3;
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6]*0.01;
			}
			#elif defined(CAR_2)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01*1/3;
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6]*0.01;
			}
			#elif defined(CAR_3)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6]*0.01;
			}
			#elif defined(CAR_4)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 - cmd->para[6]*0.01;
			}
			#elif defined(CAR_5)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01*1/3;
				point_set.y = cmd->para[1] * 0.01 - cmd->para[6]*0.01;
			}
			#elif defined(CAR_8)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01*1/3;
				point_set.y = cmd->para[1] * 0.01 - cmd->para[6]*0.01;
			}
			#elif defined(CAR_9)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 - cmd->para[6]*0.01;
			}
			#elif defined(CAR_10)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6]*0.01;
			}
			#endif
			point_set.vx = cmd->para[3] * 0.01;
			point_set.vy = cmd->para[4] * 0.01;

			point_set.traj = TRAJ_LINE;
			SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
			break;

		case CMD_SETDEST_SWARM_THREE://form into “三”
			sendType = 78;
			#if defined (CAR_1)
			{
				point_set.x = cmd->para[0] * 0.01;
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6]*0.01;
			}
			#elif defined(CAR_2)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6]*0.01;
			}
			#elif defined(CAR_3)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01*1/2;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_4)
			{
				point_set.x = cmd->para[0] * 0.01 + cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 - cmd->para[6]*0.01;
			}
			#elif defined(CAR_5)
			{
				point_set.x = cmd->para[0] * 0.01;
				point_set.y = cmd->para[1] * 0.01 - cmd->para[6]*0.01;
			}
			#elif defined(CAR_8)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 - cmd->para[6]*0.01;
			}
			#elif defined(CAR_9)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01*1/2;
				point_set.y = cmd->para[1] * 0.01;
			}
			#elif defined(CAR_10)
			{
				point_set.x = cmd->para[0] * 0.01 - cmd->para[6]*0.01;
				point_set.y = cmd->para[1] * 0.01 + cmd->para[6]*0.01;
			}
			#endif
			point_set.vx = cmd->para[3] * 0.01;
			point_set.vy = cmd->para[4] * 0.01;

			point_set.traj = TRAJ_LINE;
			SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
			break;
	    case CMD_SETDEST_LINEAB:	// line, absolute position
				sendType = 64;
#if defined (CAR_0)
				point_set.x = cmd->para[0] * 0.01;
				point_set.y = cmd->para[1] * 0.01;
				point_set.vx = cmd->para[3] * 0.01;
				point_set.vy = cmd->para[4] * 0.01;
				point_set.traj = TRAJ_LINE;
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);
#elif defined(CAR_1)
				point_set.x = cmd->para[0] * 0.01;
				point_set.y = cmd->para[1] * 0.01 - 0.8;
				point_set.vx = cmd->para[3] * 0.01;
				point_set.vy = cmd->para[4] * 0.01;
				point_set.traj = TRAJ_LINE;
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);
#elif defined(CAR_2)
				point_set.x = cmd->para[0] * 0.01;
				point_set.y = cmd->para[1] * 0.01 - 1.6;
				point_set.vx = cmd->para[3] * 0.01;
				point_set.vy = cmd->para[4] * 0.01;
				point_set.traj = TRAJ_LINE;
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);
#endif
				break;
	    case CMD_SETDEST_LINERE:	// line, relative position
				sendType = 65;
				point_set.x = cmd->para[0] * 0.01;
				point_set.y = cmd->para[1] * 0.01;
				point_set.vx = cmd->para[3] * 0.01;
				point_set.vy = cmd->para[4] * 0.01;
				point_set.traj = TRAJ_LINE;
				SetNextPoint(&traj_ab, &point_set, RELATIVE_LINE, CpuTimer0.InterruptCount*DTIME);
				break;
	   case CMD_SETDEST_ARCAB:	// arc, absolute position
				sendType = 66;
				point_set.cx = cmd->para[0] * 0.01;
				point_set.cy = cmd->para[1] * 0.01;

				point_set.v = cmd->para[3] * 0.01;
				point_set.r = cmd->para[6] * 0.01;
				point_set.loop = cmd->para[7] * 0.01;
				point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;

				point_set.traj = TRAJ_ARC;
				if(point_set.r > 0)
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_ARC, CpuTimer0.InterruptCount*DTIME);
				break;
	   case CMD_SETDEST_ARC_RING:
		   	   	sendType = 67;
				point_set.cx = cmd->para[0] * 0.01;
				point_set.cy = cmd->para[1] * 0.01;
				point_set.v = cmd->para[3] * 0.01;
				if (FORMATION_INDEX % 2 == 0)
				{
					point_set.r = cmd->para[6] * 0.01;
					point_set.loop = cmd->para[7] * 0.01;
				}
				else
				{
					point_set.r = cmd->para[2] * 0.01;
					point_set.loop = cmd->para[5] * 0.01;
				}
				point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;
				point_set.traj = TRAJ_ARC;
				if(point_set.r > 0)
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_ARC, CpuTimer0.InterruptCount*DTIME);
				break;
	   case CMD_SETDEST_ARC_DOUBLE:
		   	   	sendType = 68;
				if (FORMATION_INDEX <= 3)
				{
					point_set.cx = cmd->para[0] * 0.01 + cmd->para[6] * 0.01*3/2;
					point_set.cy = cmd->para[1] * 0.01;
					point_set.loop = cmd->para[7] * 0.01;
				}
				else
				{
					point_set.cx = cmd->para[0] * 0.01 - cmd->para[6] * 0.01*3/2;
					point_set.cy = cmd->para[1] * 0.01;
					point_set.loop = cmd->para[5] * 0.01;
				}
				point_set.v = cmd->para[3] * 0.01;
				point_set.r = cmd->para[6] * 0.01;
				point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;
				point_set.traj = TRAJ_ARC;
				if(point_set.r > 0)
				SetNextPoint(&traj_ab, &point_set, ABSOLUTE_ARC, CpuTimer0.InterruptCount*DTIME);
				break;

	   case CMD_SETDEST_EIGHTAB:	// 8, absolute position//跑的时候先给一个圆心，作为swarm的圆心，之后圆心cy加上r作为8字形中心的cy，cx则不变
				sendType = 70;
				if(cmd->para[8] * 0.01 * FORMATION_INDEX > 360)	// too many agents in this set formation
					break;
				point_set.cx = cmd->para[0] * 0.01;
				point_set.cy = cmd->para[1] * 0.01;
				//point_set.cx += p_c[0];
				//point_set.cy += p_c[1];
				point_set.z = cmd->para[2] * 0.01;
				point_set.v = cmd->para[3] * 0.01;
				point_set.r = cmd->para[6] * 0.01;
				point_set.loop = cmd->para[7] * 0.01;
				point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;
				point_set.traj = TRAJ_EIGHT;
				if(point_set.r > 0)
					SetNextPoint(&traj_ab, &point_set, ABSOLUTE_EIGHT, CpuTimer0.InterruptCount*DTIME);
				break;

	   case CMD_SETDEST_BUTTERFLY:
				sendType = 73;
				point_set.cx = cmd->para[0] * 0.01;
				point_set.cy = cmd->para[1] * 0.01;
				//point_set.z = cmd->para[2] * 0.01;
				point_set.v = cmd->para[3] * 0.01;
				point_set.r = cmd->para[6] * 0.01;
				point_set.loop = cmd->para[7] * 0.01;
				point_set.traj = TRAJ_BUTTERFLY;
				if(point_set.r > 0)
					SetNextPoint(&traj_ab, &point_set, BUTTERFLY, CpuTimer0.InterruptCount*DTIME);
				break;
	   case CMD_SETSTATE_HALT:
				 sendType = 81;

				  w1 = 0;
				  w2 = 0;
				  w3 = 0;
				  w4 = 0;
				  t0 = 0;
				  scdFlight.iSchedule = N_SCHEDULE;
				  SetNextPoint(&traj_ab, &point_set, HALT, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
				  break;

		default:
			break;
	}
}
//void	SetDestPos(Cmd* cmd)
//{
//
//	static struct flypoint point_set = {0, 0, 0, 0, 0, 0};
//
//	// can't interrupt a trajectory except HALT command -- not a good design but makes sense
//	if(cmd->code != CMD_SETSTATE_HALT)	// for normal trajectories
//	{
//		if(traj_ab.bTrajRunning == 1)	// then ignore incoming trajectory
//			return;
//		else
//			traj_ab.bTrajRunning = 1;
//		    t0=CpuTimer0.InterruptCount*DTIME;
//	}
//	else
//	{
//		// bTrajRunning will be 0
//	}
//    if((cmd->code == CMD_SETDEST_SWARMAB) || (cmd->code == CMD_SETDEST_SWARM_SQURAE))
//    {
//    	k21 = 0;
//    }
//    else
//
//    {
//    	k21 = 0.11;
//    }
//	switch(cmd->code)
//	{
//	// GROUP 0x4* Set Destination Points (and speeds)
//		case CMD_SETDEST_SWARMAB:	// form into a swarm, absolute position
//		//if(cmd->para[6] < 1)	// too close, dangerous
//		//	break;
//			sendType = 72;
////		if(cmd->para[8] * 0.01 * FORMATION_INDEX > 360)	// too many agents in this set formation
////			break;
//		point_set.x = cmd->para[0] * 0.01 + cmd->para[6] * 0.01 * sin(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
//		point_set.y = cmd->para[1] * 0.01 + cmd->para[6] * 0.01 * cos(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX);
//		//point_set.z = cmd->para[2] * 0.01;
//		//point_set.x += p_c[0];
//		//point_set.y += p_c[1];
//		point_set.vx = cmd->para[3] * 0.01;
//		point_set.vy = cmd->para[4] * 0.01;
//		//point_set.vz = cmd->para[5] * 0.01;
//		point_set.traj = TRAJ_LINE;
//		SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
//		break;
//		case CMD_SETDEST_SWARM_SQURAE:
//			sendType = 76;
//			if(cmd->para[8] * 0.01 * FORMATION_INDEX > 360)	// too many agents in this set formation
//				break;
//			cmd->para[6] *= sqrt(2)/2;
//			point_set.x = cmd->para[0] * 0.01 + cmd->para[6] * 0.01 * sin(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX + PI/4);
//			point_set.y = cmd->para[1] * 0.01 + cmd->para[6] * 0.01 * cos(cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX + PI/4);
//			//point_set.z = cmd->para[2] * 0.01;
//			//point_set.x += p_c[0];
//			//point_set.y += p_c[1];
//			point_set.vx = cmd->para[3] * 0.01;
//			point_set.vy = cmd->para[4] * 0.01;
//			//point_set.vz = cmd->para[5] * 0.01;
//			point_set.traj = TRAJ_LINE;
//			SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
//			break;
//		case CMD_SETDEST_SWARM_LINE:
//			 sendType = 75;
//			 point_set.x = cmd->para[0] * 0.01;
//			 point_set.y = cmd->para[1] * 0.01;
//			 point_set.vx = cmd->para[3] * 0.01;
//			 point_set.vy = cmd->para[4] * 0.01;
//			 point_set.r = cmd->para[6]*0.01;
//			 point_set.traj = TRAJ_LINE;
//			 SetNextPoint(&traj_ab, &point_set, SQURAE_SQURE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
//			 break;
////		case CMD_SETDEST_SQURAE_SQURAE:
////			sendType = 75;
////			point_set.cx = cmd->para[0]*0.01;
////			point_set.cy = cmd->para[1]*0.01;
////			point_set.z = cmd->para[2]*0.01;
////			point_set.v = cmd->para[3]*0.01;
////			point_set.r = cmd->para[6]*0.01;
////			point_set.loop = cmd->para[7]*0.01;
////			point_set.traj = TRAJ_SQURAE_SQURAE;
////			SetNextPoint(&traj_ab, &point_set, SQURAE_SQURE, CpuTimer0.InterruptCount*DTIME);//using ABSOLUTE_LINE
////			break;
//
//	    case CMD_SETSTATE_HALT:
//	    	 sendType = 81;
//
//			  w1 = 0;
//			  w2 = 0;
//			  w3 = 0;
//			  w4 = 0;
//			  t0 = 0;
//		      break;
//	    case CMD_SETDEST_LINEAB:	// line, absolute position
//	    	sendType = 64;
//		    point_set.x = cmd->para[0] * 0.01;
//		    point_set.y = cmd->para[1] * 0.01;
//		    point_set.vx = cmd->para[3] * 0.01;
//		    point_set.vy = cmd->para[4] * 0.01;
//		    point_set.traj = TRAJ_LINE;
//		    SetNextPoint(&traj_ab, &point_set, ABSOLUTE_LINE, CpuTimer0.InterruptCount*DTIME);
//		    break;
//	    case CMD_SETDEST_LINERE:	// line, relative position
//	    	sendType = 65;
//		    point_set.x = cmd->para[0] * 0.01;
//		    point_set.y = cmd->para[1] * 0.01;
//		    point_set.vx = cmd->para[3] * 0.01;
//		    point_set.vy = cmd->para[4] * 0.01;
//		    point_set.traj = TRAJ_LINE;
//		    SetNextPoint(&traj_ab, &point_set, RELATIVE_LINE, CpuTimer0.InterruptCount*DTIME);
//		    break;
//
//
//	   case CMD_SETDEST_ARCAB:	// arc, absolute position
//		    sendType = 66;
//	 	    point_set.cx = cmd->para[0] * 0.01;
//		    point_set.cy = cmd->para[1] * 0.01;
//
//		    point_set.v = cmd->para[3] * 0.01;
//		    point_set.r = cmd->para[6] * 0.01;
//		    point_set.loop = cmd->para[7] * 0.01;
//		    point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;
//
//		    point_set.traj = TRAJ_ARC;
//		    if(point_set.r > 0)
//			SetNextPoint(&traj_ab, &point_set, ABSOLUTE_ARC, CpuTimer0.InterruptCount*DTIME);
//		    break;
//	    case CMD_SETDEST_ARCRE:	// arc, relative position
//	    	sendType = 67;
//		    point_set.cx = cmd->para[0] * 0.01;
//		    point_set.cy = cmd->para[1] * 0.01;
//		    //point_set.z = cmd->para[2] * 0.01;
//		    point_set.v = cmd->para[3] * 0.01;
//		    point_set.r = cmd->para[6] * 0.01;
//		    point_set.loop = cmd->para[7] * 0.01;
//		    //point_set.round = cmd->para[8] * 0.01;
//		    point_set.traj = TRAJ_ARC;
//		    if(point_set.r > 0)
//			SetNextPoint(&traj_ab, &point_set, RELATIVE_ARC, CpuTimer0.InterruptCount*DTIME);
//		    break;
//
//
//	case CMD_SETDEST_TRIANGLE_INV:	// butterfly(8), relative position
//		sendType = 69;
//		point_set.cx = cmd->para[0] * 0.01;
//		point_set.cy = cmd->para[1] * 0.01;
//		point_set.z = cmd->para[2] * 0.01;//length of side of triangle
//		point_set.v = cmd->para[3] * 0.01;
//		point_set.r = cmd->para[6] * 0.01;
//		point_set.loop = cmd->para[7] * 0.01;
//		point_set.traj = TRAJ_TRIANGLE_INV;
//		SetNextPoint(&traj_ab, &point_set, TRIANGLE_INV, CpuTimer0.InterruptCount*DTIME);
//		break;
//
//	case CMD_SETDEST_EIGHTAB:	// 8, absolute position//跑的时候先给一个圆心，作为swarm的圆心，之后圆心cy加上r作为8字形中心的cy，cx则不变
//		//if(cmd->para[6] * 0.01 < 1)	// too close, dangerous
//		//	break;
//		sendType = 70;
//		if(cmd->para[8] * 0.01 * FORMATION_INDEX > 360)	// too many agents in this set formation
//			break;
//		point_set.cx = cmd->para[0] * 0.01;
//		point_set.cy = cmd->para[1] * 0.01;
//		//point_set.cx += p_c[0];
//		//point_set.cy += p_c[1];
//		point_set.z = cmd->para[2] * 0.01;
//		point_set.v = cmd->para[3] * 0.01;
//		point_set.r = cmd->para[6] * 0.01;
//		point_set.loop = cmd->para[7] * 0.01;
//		point_set.sita_ori = cmd->para[8] * 0.01 * PI / 180 * FORMATION_INDEX;
//		point_set.traj = TRAJ_EIGHT;
//		if(point_set.r > 0)
//			SetNextPoint(&traj_ab, &point_set, ABSOLUTE_EIGHT, CpuTimer0.InterruptCount*DTIME);
//		break;
//	case CMD_SETDEST_TRIANGLE_VAR:	//
//		sendType = 71;
//		point_set.cx = cmd->para[0] * 0.01;
//		point_set.cy = cmd->para[1] * 0.01;
//
//		point_set.v = cmd->para[3] * 0.01;
//		point_set.r = cmd->para[6] * 0.01;
//		point_set.loop = cmd->para[7] * 0.01;
//		point_set.traj = TRAJ_TRIANGLE_VAR;
//		SetNextPoint(&traj_ab, &point_set, TRIANGLE_VAR, CpuTimer0.InterruptCount*DTIME);
//		break;
//	case CMD_SETDEST_BUTTERFLY:
//		sendType = 73;
//		point_set.cx = cmd->para[0] * 0.01;
//		point_set.cy = cmd->para[1] * 0.01;
//		//point_set.z = cmd->para[2] * 0.01;
//		point_set.v = cmd->para[3] * 0.01;
//		point_set.r = cmd->para[6] * 0.01;
//		point_set.loop = cmd->para[7] * 0.01;
//		point_set.traj = TRAJ_BUTTERFLY;
//		if(point_set.r > 0)
//			SetNextPoint(&traj_ab, &point_set, BUTTERFLY, CpuTimer0.InterruptCount*DTIME);
//		break;
//	case CMD_SETDEST_TRIANGLE_ROUND:
//		sendType = 74;
//		point_set.cx = cmd->para[0] * 0.01;
//		point_set.cy = cmd->para[1] * 0.01;
//		point_set.z  = cmd->para[2] * 0.01;//length of side of triangle
//		point_set.v  = cmd->para[3] * 0.01;
//		point_set.r  = cmd->para[6] * 0.01;//整体走圆的半径
//		point_set.loop = cmd->para[7] * 0.01;
//
//		point_set.traj = TRAJ_ARC;
//		SetNextPoint(&traj_ab, &point_set, TRIANGLE_ROUND, CpuTimer0.InterruptCount*DTIME);
//		break;
//	//case CMD_SETDEST_ROTATE:
//
//
//	default:
//		break;
//	}
//}
int16 cnt_SDR;
unsigned char retSD;
#define	SECTORLEN	256	// 256 words ~ 512 bytes
#define	BUFFERLEN	0x400
#pragma DATA_SECTION(SD_buffer,"SD_BUFFER");
int16	offBufHead = 0;
int16	offBufTail = 0;
Uint16 	SD_buffer[0x400];//1024*1
Uint16*	Sector_buffer = SD_buffer;
void SD_ReadBlock(Uint32 SectorAddress)
{
	SD_CS = 0;
	SectorAddress <<= 1;
	SD_Ex(0x51);	// ReadBlock Command
	SD_Ex(SectorAddress >> 16);
	SD_Ex(SectorAddress >> 8);
	SD_Ex(SectorAddress & 0xFF);
	SD_Ex(0x00);
	SD_Ex(0xFF);
	
	do{
		retSD = SD_Ex(0xFF);
	}while(retSD != 0x00);	// 等到?x00

	do{
		retSD = SD_Ex(0xFF);
	}while(retSD != 0xFE);	// 等到起始字节0xFE
	
//	for(cnt_SDR = 0; cnt_SDR<512; cnt_SDR++)
//		SD_buffer[cnt_SDR] = SD_Ex(0xff);
	for(cnt_SDR = 0; cnt_SDR<256; cnt_SDR++)
	{
		SD_buffer[cnt_SDR] = 0;
		SD_buffer[cnt_SDR] |= SD_Ex(0xFF) & 0xFF;
		SD_buffer[cnt_SDR] |= SD_Ex(0xFF) << 8;
	}

	SD_Ex(0xFF); SD_Ex(0xFF); // 两字节CRC
	SD_CS = 1;
	SD_Ex(0xFF);	// 空时钟
}

int16 cnt_SDW;
int16 lenBuffer = 0;
int16 debugonly = 0;
unsigned char pWrite = 0;
void SD_WriteBlock(Uint16* data, Uint32 SectorAddress)	// data of a sector
{
	// wait till the writing process ends
//	do{	retSD = SD_Ex(0xff);
//		debugonly++;
//		DELAY_US(20);
//	}while(retSD != 0xff);

	SD_Ex(0xFF);

	/* Step 1 */
	SD_CS = 0;
	SectorAddress <<= 1;
	SD_Ex(0x58);	// CMD24 WRITE SINGLE BLOCK
	SD_Ex(SectorAddress >> 16);
	SD_Ex(SectorAddress >> 8);
	SD_Ex(SectorAddress & 0xFF);
	SD_Ex(0x00);
	SD_Ex(0xFF);

	do{
		retSD = SD_Ex(0xFF);
	}while(retSD != 0x00); 	// 等到返回0x00
	SD_Ex(0xFF);	SD_Ex(0xFF);	SD_Ex(0xFF);
	SD_Ex(0xFE);	// 起始位

	/* Step 2 */
//	for(cnt_SDW = 0; cnt_SDW<512; cnt_SDW++)
//	{
//		SD_Ex(data[cnt_SDW]);
//	}
	// for uint16*
	for(cnt_SDW = 0; cnt_SDW<256; cnt_SDW++)
	{
		SD_Ex(data[cnt_SDW] & 0xFF);
		SD_Ex((data[cnt_SDW] >> 8) & 0xFF);
	}

	do{
		retSD = SD_Ex(0xff);
	}while( (retSD & 0x1F) != 0x05 );// Ψ祷豿xx00101

	// 要先发一个CLK再等待。不知为何
	SD_Ex(0xff); SD_Ex(0xff);

	/* Step 3 */
	// DELAY_US(5000);
	do{	// 为什么强行延时没有效果而且始终快慢影响也不大？
		DELAY_US(20);
		retSD = SD_Ex(0xff);

	}while(retSD != 0xff);	// 等到SD卡不忙 事实上数据线转为高电平即不忙，不过等到0xff也没错

	
	SD_CS = 1;
	SD_Ex(0xFF);
}

void	WriteNextBlock(Uint32 SectorAddress)
{
	while(offBufHead == offBufTail)
		continue;
	// There is data available
	SD_WriteBlock(SD_buffer + offBufTail, SectorAddress);
	offBufTail = (offBufTail + SECTORLEN) % BUFFERLEN;	// to get from buffer and write to SD Card
}

#define	SD_CONFIG_SECTOR	20000
extern unsigned char bSDExist;
void	SD_SetStartSector(Uint32* StartSector)	// Set Start Sector, and write it to the SD Card
{
	if(!bSDExist)
		return;

	SD_ReadBlock(SD_CONFIG_SECTOR);
	// - low byte first
	*StartSector = SD_buffer[1];
	*StartSector = (*StartSector<<16) | SD_buffer[0];
	*StartSector += 120000;
	SD_buffer[0] = (*StartSector) & 0xFFFF;
	SD_buffer[1] = ((*StartSector)>>16) & 0xFFFF;
	SD_WriteBlock(SD_buffer, SD_CONFIG_SECTOR);
}

void	SDBufReset()	// Reset to a new sector
{	
	if((offBufHead + SECTORLEN)%BUFFERLEN != offBufTail)	// if equals: throw the new sector
	{
		offBufHead = (offBufHead + SECTORLEN) % BUFFERLEN;
		Sector_buffer = SD_buffer + offBufHead;	// to write in buffer
	}
	lenBuffer = 0;

}

unsigned char cnt_SDBA = 0;

void	SDBufAppend_int(int* data, unsigned char num)
{
	for(cnt_SDBA = 0; cnt_SDBA < num; cnt_SDBA++)
	{
		Sector_buffer[lenBuffer + cnt_SDBA] = data[cnt_SDBA];
	}
	lenBuffer += num;
}

void	SDBufAppend_float(double* data, unsigned char num)
{
	for(cnt_SDBA = 0; cnt_SDBA < num; cnt_SDBA++)
	{
		Sector_buffer[lenBuffer + cnt_SDBA] = (int16)(100*data[cnt_SDBA] + 0.5);
	}
	lenBuffer += num;
}

int32	bufI32;	
void	SDBufAppend_float2(long double* data, unsigned char num)
{
	for(cnt_SDBA = 0; cnt_SDBA < num; cnt_SDBA++)
	{
		bufI32 = (int32)(100*data[cnt_SDBA] + 0.5);
		Sector_buffer[lenBuffer + (cnt_SDBA<<1)] = bufI32 & 0xFFFF;
		Sector_buffer[lenBuffer + (cnt_SDBA<<1) + 1] = (bufI32 >> 16) & 0xFFFF;
	}
	lenBuffer += num<<1;
}

void	SDBufAppend_rad(double* data, unsigned char num)
{
	for(cnt_SDBA = 0; cnt_SDBA < num; cnt_SDBA++)
	{
		Sector_buffer[lenBuffer + cnt_SDBA] = (int16)(100*data[cnt_SDBA]/PI*180 + 0.5);
	}
	lenBuffer += num;
}

void	SDBufAppend_zeros(unsigned char num)
{
	for(cnt_SDBA = 0; cnt_SDBA < num; cnt_SDBA++)
	{
		Sector_buffer[lenBuffer + cnt_SDBA] = 0;
	}
	lenBuffer += num;
}

void	SDBufAppend_buflen()
{
	Sector_buffer[lenBuffer] = offBufHead - offBufTail;
	lenBuffer ++;
}

void	SetVar(const Cmd* cmd, double* var)
{
	switch(cmd->code)
	{
	case CMD_SETVAR_ROTOR:	// 4-rotor
		if(cmd->para[0] != 0x7FFF) var[0] = cmd->para[0];
		if(cmd->para[1] != 0x7FFF) var[1] = cmd->para[1];
		if(cmd->para[2] != 0x7FFF) var[2] = cmd->para[2];
		if(cmd->para[3] != 0x7FFF) var[3] = cmd->para[3];
		break;
	default:
		break;
	}
}
void	FlightScdRun(ScdFlight* scdFlight)
{
	if(scdFlight->isOn)
	{
		//scdFlight->t += 0.002;
		scdFlight->t += DTIME;
	}
	else
		return;

	if(scdFlight->iSchedule >= N_SCHEDULE)	// schedule all loaded
		return;	// no need to check next schedule

	if(scdFlight->isCmd)	// command not executed
		return;

	if(scdFlight->t >= scdFlight->rawSchedule[scdFlight->iSchedule+1][0])
	{	// should begin next schedule
		scdFlight->isCmd = 1;
		scdFlight->iSchedule++;
	}
}
void	FlightScdCmd(ScdFlight* scdFlight, Cmd* cmd)
{
	unsigned char i;
	cmd->code = (unsigned char)(scdFlight->rawSchedule[scdFlight->iSchedule][1]);
	for(i=0; i<9; i++)
	{
		cmd->para[i] = scdFlight->rawSchedule[scdFlight->iSchedule][i+2];
	}
	// show center
	/*
	if((cmd->code & 0x0001) == 0x0000 && GetCmdType(cmd) == CMD_SETDEST_TYPE)	// last bit - 0: AB
	{
		cmd->para[0] += xc;
		cmd->para[1] += yc;
		// zc
	}
	*/
#if defined(XBEE)

    cmd->destH = 0xFFFF;
    cmd->destL = 0xFFFF;
    cmd->srcH = 0x0000;
    cmd->srcL = 0x0000;
#else
	cmd->dest = 0xFFFF;
	cmd->src = 0x0000;
#endif
	cmd->len = 9;
	scdFlight->isCmd = 0;
//	FlagYawRun = 0;
}
//5.4*7.2
//int16	arraySchedule[N_SCHEDULE][11] = {
//
//
//{2,				CMD_SETDEST_SWARM_THREE,	270,	280,		0,		38,		38,		0,		100,		0,		0},
//
//{10, 			CMD_SETDEST_SWARM_TWO,		270,	280,		0,		38,		38,		0,		100,		0,		0},
//
//{16,			CMD_SETDEST_SWARM_ONE,		270,	280,		0,		38,		38,		0,		120,		0,		0},
//
//{25,			CMD_SETDEST_SWARMAB,		270,	280,		0,		38,		38,		0,		120,		0,		4500},
//
//{30,			CMD_SETDEST_SWARMAB,		270,	280,		0,		38,		38,		0,		100,		0,		4500},
//
//{33,			CMD_SETDEST_EIGHTAB,		270,	280,		0,		38,		38,		0,		100,	    100,	4500},
//
//{73,			CMD_SETDEST_SWARMAB,		270,	360,		0,		38,		38,		0,		120,		 0,		4500},
//
//{82,			CMD_SETDEST_ARCAB,			270,	360,		0,		38,		38,		0,		120,		-100,	4500},
//
//{107,			CMD_SETDEST_SWARM_RING,		270,	360,		60,		38,		38,		0,		180,		0,		4500},
//
//{112,			CMD_SETDEST_ARC_RING,		270,	360,		60,		38,		38,		300,	180,		-100,	4500},
//
//{149,			CMD_SETDEST_SWARMAB,		270,	360,		0,		38,		38,		0,		180,		0,		4500},
//
//{158,			CMD_SETDEST_SWARM_DOUBLE,	270,	360,		0,		38,		38,		0,		90,			0,		9000},
//
//{169,			CMD_SETDEST_ARC_DOUBLE,		270,	360,		0,		38,		38,		100,	80,			-100,	9000},
//
//};

#if defined (CAR_0)
int16   arraySchedule[N_SCHEDULE][11] = {

{ 2,            CMD_SETDEST_SWARMAB,        180,    210,        0,      38,     38,     0,      100,          0,    12000},

{10,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,        100,    12000},

{35,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,       -100,    12000},

{60,            CMD_SETDEST_SWARMAB,        180,    150,        0,      38,     38,     0,       50,          0,    12000},

{70,            CMD_SETDEST_EIGHTAB,        180,    150,        0,      38,     38,     0,       50,        100,    12000},

{95,           CMD_SETDEST_EIGHTAB,        180,    150,        0,      38,     38,     0,       50,       -100,    12000},

{120,           CMD_SETDEST_LINEAB,         180,    210,        0,      38,     38,     0,        0,          0,        0},

{125,           CMD_SETDEST_LINEAB,         180,    210,        0,      38,     38,     0,        0,          0,        0},

{150,           CMD_SETDEST_LINEAB,         180,    210,        0,      38,     38,     0,        0,          0,        0},

{175,           CMD_SETDEST_SWARM_ONE,      180,    210,        0,      38,     38,     0,      100,          0,        0}

};

#elif defined (CAR_1)
int16   arraySchedule[N_SCHEDULE][11] = {

{ 2,            CMD_SETDEST_SWARMAB,        180,    210,        0,      38,     38,     0,      100,          0,    12000},

{10,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,        100,    12000},

{35,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,       -100,    12000},

{60,            CMD_SETDEST_SWARMAB,        180,    150,        0,      38,     38,     0,       50,          0,    12000},

{70,            CMD_SETDEST_EIGHTAB,        180,    150,        0,      38,     38,     0,      50,         100,    12000},

{95,           CMD_SETDEST_EIGHTAB,        180,    150,        0,      38,     38,     0,      50,        -100,    12000},

{120,           CMD_SETDEST_SWARMAB,        180,    210,        0,      38,     38,     0,      100,          0,    18000},

{125,           CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,        100,    18000},

{150,           CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,       -100,    18000},

{175,           CMD_SETDEST_SWARM_ONE,      180,    210,        0,      38,     38,     0,      100,          0,        0}

};

#elif defined (CAR_2)
int16   arraySchedule[N_SCHEDULE][11] = {

{ 2,            CMD_SETDEST_SWARMAB,        180,    210,        0,      38,     38,     0,      100,          0,    12000},

{10,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,        100,    12000},

{35,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,       -100,    12000},

{60,            CMD_SETDEST_SWARMAB,        180,    150,        0,      38,     38,     0,       50,          0,    12000},

{70,            CMD_SETDEST_EIGHTAB,        180,    150,        0,      38,     38,     0,      50,         100,    12000},

{95,            CMD_SETDEST_EIGHTAB,        180,    150,        0,      38,     38,     0,      50,        -100,    12000},

{120,            CMD_SETDEST_SWARMAB,        180,    210,        0,      38,     38,     0,      100,          0,    18000},

{125,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,        100,    18000},

{150,            CMD_SETDEST_ARCAB,          180,    210,        0,      38,     38,     0,      100,       -100,    18000},

{175,            CMD_SETDEST_SWARM_ONE,      180,    210,        0,      38,     38,     0,      100,          0,        0}

};
#endif


void	FlightScdDesign(ScdFlight* scdFlight)
{
	unsigned char i, j;
	scdFlight->isOn = 0;
	scdFlight->t = 0;
	scdFlight->isCmd = 0;
	scdFlight->iSchedule = -1;	// haven't begun the 1st schedule
	for(i=0; i<N_SCHEDULE; i++)
	for(j=0; j<11;j++)
	{
		scdFlight->rawSchedule[i][j] = arraySchedule[i][j];
	}
}
