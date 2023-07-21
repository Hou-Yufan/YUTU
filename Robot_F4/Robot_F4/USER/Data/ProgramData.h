#ifndef _PROGRAM_DATA_H_
#define _PROGRAM_DATA_H_

#include "main.h"

#define PID_MOTOR_L					0	//左电机速度
#define PID_MOTOR_R					1	//右电机速度
#define PID_REVOLVE					2	//底盘旋转速度
#define PID_DISPLAY					3	//屏幕上下

typedef struct
{
//-----------------------------------------------
//	flash数据校验
	uint8_t flag;				//写入标志	准备写入 aa，写入完成0。从Flash里读取到aa，说明有数据，否则没有数据
	uint8_t  check_bcc;	//异或校验
	uint16_t check_sum;	//和校验
//-----------------------------------------------
//	陀螺仪校准数据
	float Gyro_Offset[3]; 	//x,y,z偏移
	float Acc_Offset[3]; 		//x,y,z偏移
	float Acc_Offset_Scale[3]; 			//x,y,z偏移比例
//-----------------------------------------------
//	PID参数
	float pid_kp[4];
	float pid_ki[4];
	float pid_kd[4];
	
	uint16_t Modify_cnt;
}Program_Data_set;

typedef union
{
	Program_Data_set set;
	uint8_t buf[sizeof(Program_Data_set)];
}Program_Data;

extern Program_Data program_data;

void Program_Data_Init(void);
void Program_Data_Modify(void);
void Program_Data_Save(void);
void Program_Data_Reset(void);
#endif

