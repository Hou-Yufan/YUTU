/**
  ******************************************************************************
  * @file           : CAN_ADC.h
  * @brief          : ADC扩展板通信
  ******************************************************************************
  * @attention		
  *
  * @Logs:
  * Date           Author       Notes
  * 2023-03-18     李树益      第一个版本
  ******************************************************************************
  */
	
#ifndef __CAN_ADC_H__
#define __CAN_ADC_H__

#include "main.h"

typedef struct CAN_Rangefinder_Hub
{
	uint32_t last_time;
	uint8_t Distence[6];
}CAN_Rangefinder_Hub;

typedef struct CAN_IR_Hub
{
	uint32_t last_time;
	uint16_t adc[3];
}CAN_IR_Hub;

extern CAN_Rangefinder_Hub Rangefinder_hub;
extern CAN_IR_Hub IR_hub;
extern uint8_t Touch_Key;


void CAN_ADCHub_Data0_RX(uint8_t *buf);
void CAN_ADCHub_Data1_RX(uint8_t *buf);
#endif
