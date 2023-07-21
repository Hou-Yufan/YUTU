#ifndef __IMU_DEV_H__
#define __IMU_DEV_H__

#include "main.h"
#include "LowPassFilter.h"

typedef struct
{
	//陀螺仪旋转矩阵
	int8_t Gyro_Rotation[3][3];
	int8_t Acc_Rotation[3][3];
	//旋转后的原始数据
	int16_t Gyro_Raw[3];    //x,y,z原始数据
	int16_t Acc_Raw[3];     //x,y,z原始数据
	//陀螺仪校准
	uint16_t motionless;		//静止的时间
	double gyro_sum[3];			//静止时的零飘和
	float Gyro_Offset[3]; 	//x,y,z偏移
	float Acc_Offset[3]; 		//x,y,z偏移
	float Acc_Offset_Scale[3]; 			//x,y,z偏移比例
//		陀螺仪数据
	float Gyro_Deg[3];  //角速度，单位：°/s
	float Acc_G[3];     //加速度，单位：G
#ifdef		LOWPASSFILTER_H_
//		滤波器
	lpf2pData Acc_lpfilter[3];
	lpf2pData Gyro_lpfilter[3];
#endif
//		滤波后的数据
	float Acc_LPF_G[3];
	float Gyro_LPF_Deg[3];
//		陀螺仪刻度
	float Gyro_Scale_Factor;
	float Acc_Scale_Factor;
	uint8_t gyro_calibrate;
	uint8_t acc_calibrate;
}IMU_Dev;

extern IMU_Dev imu;

void IMU_Init(void);
void IMU_Update(void);
void IMU_GYRO_Fast_calibrate(void);
void IMU_GYRO_Auto_calibrate(void);
void IMU_ACC_Fast_calibrate(void);
void IMU_ACC_calibrate(void);
#endif
