#include "remote.h"

unsigned int channels[4] = {7500, 7500, 5000, 7500};
double dt_r = 0.01;

unsigned int channel;
void SetChannel(unsigned char NoCh, double percent)
{
	channel = percent * 100000;
	if(channel>4500 && channel<10500) // 否则不处理。只能容忍短暂的坏数
	{
		if(channel < 5000)
			channel = 5000;
		if(channel > 10000)
			channel = 10000;
		
		if(NoCh >= 1 && NoCh <= 4)
			channels[NoCh-1] = channel;
	}
	

}

void GetChannel(unsigned char NoCh, unsigned int* ch)
{
	if(NoCh >= 1 && NoCh <= 4)
		*ch = channels[NoCh-1];
}
