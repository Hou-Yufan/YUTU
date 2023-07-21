#include "BSP_TIM.h"
#include "tim.h"
#include "Motor.h"

void delay_us(uint16_t us)
{
	TIM9->CNT = 0;
	HAL_TIM_Base_Start(&htim9);
	while(TIM9->CNT < us);
	HAL_TIM_Base_Stop(&htim9);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	if(htim == &Modbus_Tim)
//	{
////		ModBus_TIM_IRQ(htim);
//	}
	if(htim == &htim14)
	{
		Motor_TIM_IRQ();
	}
}



