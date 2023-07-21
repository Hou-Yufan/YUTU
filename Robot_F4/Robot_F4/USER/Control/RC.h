#ifndef __RC_H__
#define __RC_H__

#include "main.h"

#define RC_TYPE_CAN					1
#define RC_TYPE_ANO					2
#define RC_TYPE_YUTU				3
#define RC_TYPE_NONE				255

typedef struct RC_s{
	uint8_t Type;
	uint8_t Mode;
	float X;
	float Y;
	uint32_t last_time;
}RC_s;

extern RC_s rc;

void RC_WatchDog(void);
void RC_Init(void);

void RC_CAN_RX(uint8_t *buf);

#endif
