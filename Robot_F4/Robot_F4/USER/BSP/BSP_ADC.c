#include "BSP_ADC.h"
#include "adc.h"
#include "GP2Y1014AU0F.h"
#include "Motor.h"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc3)
	{
		Motor_ADC_IRQ();
	}
	else if(hadc == &hadc2)
	{
		GP2Y1010AU0F_IRQ();
	}
}


