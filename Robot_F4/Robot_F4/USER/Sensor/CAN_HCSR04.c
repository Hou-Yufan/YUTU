/**
  ******************************************************************************
  * @file           : CAN_HCSR04.c
  * @brief          : 超声波扩展板通信
  ******************************************************************************
  * @attention		
  *
  * @Logs:
  * Date           Author       Notes
  * 2023-03-18     李树益      第一个版本
  ******************************************************************************
  */

#include "CAN_HCSR04.h"
	
CAN_HCSR_Hub hcsr_hub;

void CAN_HCSR04_RX(uint16_t id, uint8_t *buf)
{
	switch(id)
	{
		case 0:
			hcsr_hub.Distence[0] = (((uint16_t)buf[0])<<8 | buf[1])/10;
			hcsr_hub.Distence[1] = (((uint16_t)buf[2])<<8 | buf[3])/10;
			hcsr_hub.Distence[2] = (((uint16_t)buf[4])<<8 | buf[5])/10;
			hcsr_hub.Distence[3] = (((uint16_t)buf[6])<<8 | buf[7])/10;
			hcsr_hub.last_time = HAL_GetTick();
			break;
		case 1:
			hcsr_hub.Distence[4] = (((uint16_t)buf[0])<<8 | buf[1])/10;
			hcsr_hub.Distence[5] = (((uint16_t)buf[2])<<8 | buf[3])/10;
			hcsr_hub.Distence[6] = (((uint16_t)buf[4])<<8 | buf[5])/10;
			hcsr_hub.Distence[7] = (((uint16_t)buf[6])<<8 | buf[7])/10;
			hcsr_hub.last_time = HAL_GetTick();
			break;
		default:
			break;
	}
}

