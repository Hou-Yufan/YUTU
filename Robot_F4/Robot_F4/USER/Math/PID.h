/**
  ******************************************************************************
  * @file           : PID.h
  * @brief          : PID计算过程
  ******************************************************************************
  * @attention		
  *
	* @Logs:
	* Date           Author       Notes
	* 2021-12-25     李树益      第一个版本
  ******************************************************************************
  */
	
#ifndef _PID_H_
#define _PID_H_

#include "main.h"

typedef struct
{
// PID参数
	float Kp;
	float Ki;
	float Kd;
// PID输出
	float P_OUT;
	float I_OUT;
	float D_OUT;
	float OUT;
// 限幅
	float OUT_MAX;
	float OUT_MIN;
	float I_MAX;
	float I_MIN;
//////////////////////
	float last_delta;	//上一次的误差
	uint32_t last_time;
}PID;

//-----------------------------------------------
//	PID变量
extern PID pid_motor_L;
extern PID pid_motor_R;
extern PID pid_yaw_speed;
extern PID pid_yaw_angle;
extern PID pid_charge_angle;
extern PID pid_charge_speed;

//-----------------------------------------------
//	PID函数
void PID_Creat(PID *pid, float Kp, float Ki, float Kd, float I_MAX, float I_MIN, float OUT_MAX, float OUT_MIN);
void PID_Cale_DELTA(PID *pid, float delta, float freq);
void PID_Cale_POSITION(PID *pid, float this_pos, float next_pos, float freq);
void PID_resize(PID *pid);
void PID_Init(void);
#endif
