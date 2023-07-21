#ifndef _VL53L0_H_
#define _VL53L0_H_

#include "main.h"

#define VL53L0_RUNING		0
#define VL53L0_ERROR		1
#define VL53L0_DISABLE	2

struct VL53L0
{
	uint16_t distance;
	uint8_t state;
	uint8_t t;			//接收间隔，单位500ms
};

extern struct VL53L0 VL53L0_Dev[2];

void VL53L0_Init(void);
void VL53L0_Dev_Check(void);
void VL53L0_UART_IRQ(UART_HandleTypeDef *huart, uint16_t Size);
void VL53L0_CAN_IRQ(uint8_t *buf);
#endif


