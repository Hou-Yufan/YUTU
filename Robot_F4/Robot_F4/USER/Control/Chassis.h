#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"

extern float chassis_speed[2];
extern float yaw_Set;							//期望的yaw轴位置
void Chassis_Init(void);
void Chassis_Run(void);
float Get_Chassis_Angle_Err(float angle);
#endif
