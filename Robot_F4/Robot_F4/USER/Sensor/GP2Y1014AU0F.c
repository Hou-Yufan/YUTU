#include "GP2Y1014AU0F.h"
#include "adc.h"
#include "gpio.h"

#define PM25_ADC_BUF_LEN	50
/*
PM2.5
*/
struct GP2Y1010AU0F PM2_5;
uint16_t ADC_Value[PM25_ADC_BUF_LEN];

void GP2Y1010AU0F_Run(void)			//1ms调用一次
{
	PM2_5.time++;
	if(PM2_5.run_flag == GP2Y1010AU0F_Ready)
	{
		HAL_GPIO_WritePin(PM_LED_GPIO_Port,PM_LED_Pin,GPIO_PIN_RESET);
		PM2_5.run_flag = GP2Y1010AU0F_Wait;
	}
	else if(PM2_5.run_flag == GP2Y1010AU0F_Wait)
	{
		PM2_5.run_flag = GP2Y1010AU0F_Read;
	}
	else if(PM2_5.run_flag == GP2Y1010AU0F_Read)
	{
		HAL_ADC_Start_DMA(&hadc2,(uint32_t*)&ADC_Value, PM25_ADC_BUF_LEN);
	}
	else if(PM2_5.run_flag == GP2Y1010AU0F_Done)
	{
		HAL_GPIO_WritePin(PM_LED_GPIO_Port,PM_LED_Pin,GPIO_PIN_SET);
		if(PM2_5.time == 50)	//100ms读一次PM2.5
		{
			PM2_5.run_flag = GP2Y1010AU0F_Ready;
			PM2_5.time = 0;
		}
	}
}

void GP2Y1010AU0F_IRQ()
{
	uint32_t AD_PM = 0;
	uint8_t i;
	for(i = 0; i< PM25_ADC_BUF_LEN; i++)
	{
		AD_PM += ADC_Value[i];
	}
	AD_PM /= PM25_ADC_BUF_LEN;
	PM2_5.adc = AD_PM;
	PM2_5.data = 0.17f*AD_PM-0.1f; //转换公式
	PM2_5.run_flag = GP2Y1010AU0F_Done;
}

