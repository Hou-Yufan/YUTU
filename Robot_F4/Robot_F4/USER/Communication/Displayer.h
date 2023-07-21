#ifndef __DISPLAYER_H__
#define __DISPLAYER_H__

#include "main.h"

struct Displayer_Data
{
	uint16_t V;
	uint16_t I;
	float angle;
	float speed;
	int16_t out;
	uint8_t control_mode;
	uint8_t Regs[5];
	uint8_t state;
	uint8_t t;
};

extern struct Displayer_Data Displayer;

void Disp_RX_Motor_Data(uint8_t *buf);
void Disp_RX_Sensor_Data(uint8_t *buf);
void Disp_RX_Regs_Data(uint8_t *buf);
void Disp_Send_AP_Data(CAN_HandleTypeDef *can_dev, uint8_t TX_ADDR, uint8_t RX_ADDR);
#endif

