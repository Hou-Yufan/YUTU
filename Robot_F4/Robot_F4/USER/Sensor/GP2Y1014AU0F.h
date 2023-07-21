#ifndef __GP2Y1014AU0F_H__
#define __GP2Y1014AU0F_H__

#include "main.h"

#define GP2Y1010AU0F_Ready	0
#define GP2Y1010AU0F_Wait		1
#define GP2Y1010AU0F_Read		2
#define GP2Y1010AU0F_Done		3

struct GP2Y1010AU0F
{
	float data;	//0-0.2 Y; 0.2-0.4 L; 0.4-0.6 Z; 0.6-0.8 C
	int16_t adc;
	uint16_t time;
	uint8_t run_flag;		//0：可以打开红外，1：等待数据稳定，2：ADC采集，3关闭红外
};

extern struct GP2Y1010AU0F PM2_5;

void GP2Y1010AU0F_Run(void);
void GP2Y1010AU0F_IRQ(void);

#endif


