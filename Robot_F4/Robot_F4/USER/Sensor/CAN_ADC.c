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

#include "CAN_ADC.h"
	
CAN_Rangefinder_Hub Rangefinder_hub;
CAN_IR_Hub IR_hub;
uint8_t Touch_Key;


void CAN_ADCHub_Data0_RX(uint8_t *buf)
{
	Rangefinder_hub.last_time = HAL_GetTick();
	Rangefinder_hub.Distence[0] = buf[0];
	Rangefinder_hub.Distence[1] = buf[1];
	Rangefinder_hub.Distence[2] = buf[2];
	Rangefinder_hub.Distence[3] = buf[3];
	Rangefinder_hub.Distence[4] = buf[4];
	Rangefinder_hub.Distence[5] = buf[5];
	Touch_Key = buf[6];
}

void CAN_ADCHub_Data1_RX(uint8_t *buf)
{
	IR_hub.last_time = HAL_GetTick();
	IR_hub.adc[0] = ((uint16_t)buf[0])<<8 | buf[1];
	IR_hub.adc[1] = ((uint16_t)buf[2])<<8 | buf[3];
	IR_hub.adc[2] = ((uint16_t)buf[4])<<8 | buf[5];
}

