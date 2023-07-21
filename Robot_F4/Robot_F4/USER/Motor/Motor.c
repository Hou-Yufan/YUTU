#include "Motor.h"
#include "tim.h"
#include "adc.h"
#include "LowPassFilter.h"
#include "Chassis.h"

#define Motor_ADC_BUF						300

struct Motor ChassisMotor_L = {0};
struct Motor ChassisMotor_R = {0};
struct Motor LifterMotor = {0};

uint16_t adc_motor[Motor_ADC_BUF] = {0};

void Motor_Init()
{
	HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&adc_motor, Motor_ADC_BUF);
	HAL_TIM_Base_Start_IT(&htim14);		//定时读取编码器脉冲
	
	HAL_TIM_Base_Start(&htim3);				//左边轮子电机编码器
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start(&htim1);			//左边轮子电机驱动
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	HAL_TIM_Base_Start(&htim8);				//右边轮子电机编码器
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start(&htim4);			//右边轮子电机驱动
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void Motor_Set(struct Motor* m, int16_t pwm)
{
	if(pwm > 100)
		pwm = 100;
	else if(pwm < -100)
		pwm = -100;
	
	if( (m->pwm_set > 0 && pwm < 0) || (m->pwm_set < 0 && pwm > 0) )
	{
		if(m->zone == 0)
		{
			m->zone = 1;		//一个时间片的死区
		}
	}
	m->pwm_set = pwm;
}

void Motor_PWM_Update(void)
{
	int16_t pwm;
	/*---------------------------
	底盘左电机输出
	*/
	pwm = ChassisMotor_L.pwm_set;
	if(ChassisMotor_L.zone > 0)
	{
		ChassisMotor_L.zone--;
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
	}
	else
	{
		if(pwm > 0)
		{
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
			ChassisMotor_L.last = 1;
		}
		else if(pwm < 0)
		{
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, -pwm);
			ChassisMotor_L.last = 2;
		}
		else
		{
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
			ChassisMotor_L.last = 0;
		}
	}
	/*---------------------------
	底盘右电机输出
	*/
	pwm = ChassisMotor_R.pwm_set;
	if(ChassisMotor_R.zone > 0)
	{
		ChassisMotor_R.zone--;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
	}
	else
	{
		if(pwm > 0)
		{
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
			ChassisMotor_R.last = 1;
		}
		else if(pwm < 0)
		{
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, -pwm);
			ChassisMotor_R.last = 2;
		}
		else
		{
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
			ChassisMotor_R.last = 0;
		}
	}
}

void Motor_TIM_IRQ(void)
{
	//获取电机脉冲数
	ChassisMotor_L.encode =  (int16_t)__HAL_TIM_GET_COUNTER(&htim3);		//左边反馈
	ChassisMotor_R.encode =  (int16_t)__HAL_TIM_GET_COUNTER(&htim8);		//右边反馈

	//清空电机脉冲计数
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim8,0);

	//计算电机转速，单位圈/10ms
	ChassisMotor_L.rpm = ChassisMotor_L.encode/(float)(ChassisMotor_ratio*ChassisMotor_Reduction*4.0f);
	ChassisMotor_R.rpm = ChassisMotor_R.encode/(float)(ChassisMotor_ratio*ChassisMotor_Reduction*4.0f);
	
	//计算电机输出轴转动圈数
	ChassisMotor_L.circle += ChassisMotor_L.rpm;
	ChassisMotor_R.circle += ChassisMotor_R.rpm;
	
	//转化电机输出轴转速，最终单位rpm
	ChassisMotor_L.rpm *= 60000;
	ChassisMotor_R.rpm *= 60000;
	Chassis_Run();
	Motor_PWM_Update();
}

void Motor_ADC_IRQ(void)
{
	uint16_t i;
	uint32_t adc_avg[3] = {0};
	for(i=0;i<Motor_ADC_BUF;)
	{
		adc_avg[0] += adc_motor[i++];
		adc_avg[1] += adc_motor[i++];
		adc_avg[2] += adc_motor[i++];
	}
	adc_avg[0]/=(Motor_ADC_BUF/3);
	adc_avg[1]/=(Motor_ADC_BUF/3);
	adc_avg[2]/=(Motor_ADC_BUF/3);
	
	ChassisMotor_R.current = adc_avg[0]*0.80586f;
	LifterMotor.current = adc_avg[1]*0.80586f;
	ChassisMotor_L.current = adc_avg[2]*0.80586f;
}

