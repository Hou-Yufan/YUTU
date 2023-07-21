/**
  ******************************************************************************
  * @file           : CAN_HCSR04.h
  * @brief          : 超声波扩展板通信
  ******************************************************************************
  * @attention		
  *
  * @Logs:
  * Date           Author       Notes
  * 2023-03-18     李树益      第一个版本
  ******************************************************************************
  */
	
#ifndef __CAN_HCSR04_H__
#define __CAN_HCSR04_H__

#include "main.h"

typedef struct CAN_HCSR_Hub
{
	uint32_t last_time;
	uint16_t Distence[8];
}CAN_HCSR_Hub;

extern CAN_HCSR_Hub hcsr_hub;

void CAN_HCSR04_RX(uint16_t id, uint8_t *buf);
#endif
