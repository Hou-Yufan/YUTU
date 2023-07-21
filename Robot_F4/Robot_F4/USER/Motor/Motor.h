#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#define ChassisMotor_ratio			500	//底盘电机转子转动一圈的脉冲数量
#define ChassisMotor_Reduction	71	//底盘电机减速比

struct Motor
{

	int16_t encode;		//编码器
	int16_t current;	//电机电流
	float rpm;				//电机转速
	double	circle;		//电机转动圈数
	int16_t pwm_set;	//PWM输出
	uint8_t zone;			//反转死区
	uint8_t last;			//上一次的PWM输出，0:之前是静止；1:之前是正转；2:之前是反转
};

extern struct Motor ChassisMotor_L;
extern struct Motor ChassisMotor_R;
extern struct Motor LifterMotor;

void Motor_Init(void);
void Motor_Set(struct Motor* m, int16_t pwm);
void Motor_PWM_Update(void);
void Motor_TIM_IRQ(void);
void Motor_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Motor_ADC_IRQ(void);
#endif
