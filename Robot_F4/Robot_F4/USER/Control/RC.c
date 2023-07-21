#include "RC.h"

RC_s rc;

void RC_Reset()
{
	rc.Mode = 0;
	rc.X = 0;
	rc.Y = 0;
	rc.last_time = HAL_GetTick();
}

void RC_WatchDog()
{
	uint32_t tnow = HAL_GetTick();
	
	if(((tnow > rc.last_time) && (tnow - rc.last_time) > 1000) || ((tnow < rc.last_time) && (rc.last_time - tnow) > 1000))
	{
		rc.Type = RC_TYPE_NONE;
		RC_Reset();
	}
}

void RC_CAN_RX(uint8_t *buf)
{
	rc.Mode = buf[0];
	rc.Type = 0;
	rc.X = (int16_t)(((uint16_t)buf[1]<<8) | buf[2]);
	rc.Y = (int16_t)(((uint16_t)buf[3]<<8) | buf[4]);
	rc.last_time = HAL_GetTick();
}

void RC_Init()
{
	rc.Type = RC_TYPE_NONE;
	RC_Reset();
}

