#ifndef DYPA02_H
#define DYPA02_H

#include "main.h"

#define DYPA02_RUNING		0
#define DYPA02_ERROR		1
#define DYPA02_DISABLE	2

struct DYPA02
{
	uint16_t distance;
	uint8_t state;
	uint8_t t;			//接收间隔，单位500ms
};

extern struct DYPA02 DYPA02_Dev[3];

void DYPA02_Init(void);
void DYPA02_Dev_Check(void);
void DYPA02_IRQ(UART_HandleTypeDef *huart, uint16_t Size);
#endif


