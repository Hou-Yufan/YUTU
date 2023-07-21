#ifndef __POWER_H__
#define __POWER_H__
#include "main.h"

struct PM{
	float vbat;
	float current;
	float bat;
	double mah;
	double max_mah;
	uint32_t last_time;
	uint8_t power_state;	//0：上电  1：开机状态  2：按键按下了	 3：AP关机	4：AP确认关机
	uint8_t Count;				//按键按下计数，单位10ms
	uint8_t last_key;			//上一次的按键键值
	uint8_t last_power;		//上一次的电源状态，用于还原电源键按下后的情况
};

extern struct PM Power_management;

void Power_Control(void);
void BAT_Init(void);
void BAT_Update(void);
#endif
