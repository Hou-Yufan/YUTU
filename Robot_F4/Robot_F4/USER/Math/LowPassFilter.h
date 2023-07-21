/*
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-30     LSY       the first version
 */

#ifndef		LOWPASSFILTER_H_
#define		LOWPASSFILTER_H_

typedef struct //二阶低通滤波器
{
	float a1;//
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;
	float delay_element_2;
} lpf2pData;

void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq);
float lpf2pApply(lpf2pData* lpfData, float sample);

#endif
