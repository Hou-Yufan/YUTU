//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)		// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

Quaternion quaternion;	//自定义的四元数结构体
float ahrs_yaw, ahrs_pitch, ahrs_roll;

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS reset
void MahonyAHRSInit(Quaternion *q, unsigned int time_ms)
{
	q->q[0] = 1.0f;
	q->q[1] = 0.0f;
	q->q[2] = 0.0f;
	q->q[3] = 0.0f;
	q->integralFBx = 0.0f;
  q->integralFBy = 0.0f;
	q->integralFBz = 0.0f;
	q->time_ms = time_ms;
	q->twoKp = twoKpDef;
	q->twoKi = twoKiDef;
}

void Quaternion_To_Euler_angle(Quaternion *q, float *yaw, float *pitch, float *roll)
{
	//四元数转化成欧拉角
	ahrs_yaw = atan2f(	2.0f*(q->q[0] * q->q[3] + q->q[1] * q->q[2]),
									2.0f*(q->q[0] * q->q[0] + q->q[1] * q->q[1]) -1.0f ) * 57.3f;
	
	ahrs_pitch = asinf(-2.0f*(q->q[1] * q->q[3] - q->q[0] * q->q[2])) * 57.3f;
	
	ahrs_roll = atan2f(	2.0f*(q->q[0] * q->q[1] + q->q[2] * q->q[3]),
									2.0f*(q->q[0] * q->q[0] + q->q[3] * q->q[3])-1.0f) * 57.3f;
}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(Quaternion *q, unsigned int time_ms, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float dt;
	
	//磁强计测量无效时的IMU算法（避免磁强计标准化中的NaN）更新出四元数
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q, time_ms, gx, gy, gz, ax, ay, az);
		return;
	}
	//计算更新频率
	if(time_ms > q->time_ms)
	{
		dt = (time_ms - q->time_ms)/1000.0f;
	}
	else if(time_ms < q->time_ms)	//溢出了
	{
		dt = (4294967295U - q->time_ms + time_ms + 1)/1000.0f;
	}
	else
	{
		dt = 1.0f/sampleFreqDef;
	}
	q->time_ms = time_ms;
	//仅当加速计测量有效时计算反馈（避免加速计正常化中的NaN
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		//角度转换为弧度1/57.3
		gx*=0.017453293f;
		gy*=0.017453293f;
		gz*=0.017453293f;
		 
		//标准化加速度计测量
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm; 		
		
		//正态磁强计测量
		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

				//避免重复运算的辅助变量
        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q->q[0] * q->q[0];
        q0q1 = q->q[0] * q->q[1];
        q0q2 = q->q[0] * q->q[2];
        q0q3 = q->q[0] * q->q[3];
        q1q1 = q->q[1] * q->q[1];
        q1q2 = q->q[1] * q->q[2];
        q1q3 = q->q[1] * q->q[3];
        q2q2 = q->q[2] * q->q[2];
        q2q3 = q->q[2] * q->q[3];
        q3q3 = q->q[3] * q->q[3];   

				//地球磁场参考方向
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		//估计重磁场方向
		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		//误差是估计方向与实测场矢量方向之间的交叉积之和
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		//计算并应用积分反馈（如果启用）
		// Compute and apply integral feedback if enabled
		if(q->twoKi > 0.0f) {
			q->integralFBx += q->twoKi * halfex * dt;	// integral error scaled by Ki
			q->integralFBy += q->twoKi * halfey * dt;	//用Ki标定的积分误差
			q->integralFBz += q->twoKi * halfez * dt;
			gx += q->integralFBx;	// apply integral feedback
			gy += q->integralFBy;	// 应用积分反馈
			gz += q->integralFBz;
		}
		else {	//防止整体上卷
			q->integralFBx = 0.0f;	// prevent integral windup
			q->integralFBy = 0.0f;
			q->integralFBz = 0.0f;
		}
		// 应用比例反馈
		// 调整后的陀螺仪测量
		// Apply proportional feedback
		gx += q->twoKp * halfex;
		gy += q->twoKp * halfey;
		gz += q->twoKp * halfez;
	}
	
	// 四元数积分变化率// 整合四元数率和正常化
	//下面进行姿态的更新，也就是四元数微分方程的求解
	//采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q->q[0];//q0;
	qb = q->q[1];//q1;
	qc = q->q[2];//q2;
	q->q[0] += (-qb * gx - qc * gy - q->q[3] * gz);
	q->q[1] += (qa * gx + qc * gz - q->q[3] * gy);
	q->q[2] += (qa * gy - qb * gz + q->q[3] * gx);
	q->q[3] += (qa * gz + qb * gy - qc * gx); 
	
	//正规化四元数
	//单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
	// Normalise quaternion
	recipNorm = invSqrt(q->q[0] * q->q[0] + q->q[1] * q->q[1] + q->q[2] * q->q[2] + q->q[3] * q->q[3]);
	q->q[0] *= recipNorm;
	q->q[1] *= recipNorm;
	q->q[2] *= recipNorm;
	q->q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(Quaternion *q, unsigned int time_ms, float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float dt;		//时间差，单位秒
	//计算更新频率
	if(time_ms > q->time_ms)
	{
		dt = (time_ms - q->time_ms)/1000.0f;
	}
	else if(time_ms < q->time_ms)	//溢出了
	{
		dt = (4294967295U - q->time_ms + time_ms + 1)/1000.0f;
	}
	else
	{
		dt = 1.0f/sampleFreqDef;
	}
	q->time_ms = time_ms;
	//仅当加速计测量有效时计算反馈（避免加速计正常化中的NaN
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//		rt_kprintf("dt = %d\n",(int)(dt*1000));
		//角度转换为弧度1/57.3
		gx*=0.017453293f;
		gy*=0.017453293f;
		gz*=0.017453293f;
		
		//标准化加速度计测量
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        
		
		//估计 重力方向 和 垂直于磁通量 的矢量
		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q->q[1] * q->q[3] - q->q[0] * q->q[2];//q1 * q3 - q0 * q2;
		halfvy = q->q[0] * q->q[1] + q->q[2] * q->q[3];//q0 * q1 + q2 * q3;
		halfvz = q->q[0] * q->q[0] -0.5f + q->q[3] * q->q[3];//q0 * q0 - 0.5f + q3 * q3;
	
		//误差是 重力方向估计值 和 测量值 的乘积之和
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		//计算并应用积分反馈（如果启用）
		// Compute and apply integral feedback if enabled
		if(q->twoKi > 0.0f) {	//用Ki标定的积分误差
			q->integralFBx += q->twoKi * halfex * dt;	// integral error scaled by Ki
			q->integralFBy += q->twoKi * halfey * dt;
			q->integralFBz += q->twoKi * halfez * dt;
			gx += q->integralFBx;	// apply integral feedback
			gy += q->integralFBy;	// 应用积分反馈//将误差PI后补偿到陀螺仪，即补偿零点漂移
			gz += q->integralFBz;
		}
		else {
			q->integralFBx = 0.0f;	// prevent integral windup
			q->integralFBy = 0.0f;
			q->integralFBz = 0.0f;
		}

		// 应用比例反馈
		// Apply proportional feedback
		gx += q->twoKp * halfex;
		gy += q->twoKp * halfey;
		gz += q->twoKp * halfez;
	}
	
	// 四元数积分变化率// 整合四元数率和正常化
	//下面进行姿态的更新，也就是四元数微分方程的求解
	//采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q->q[0];//q0;
	qb = q->q[1];//q1;
	qc = q->q[2];//q2;
	q->q[0] += (-qb * gx - qc * gy - q->q[3] * gz);
	q->q[1] += (qa * gx + qc * gz - q->q[3] * gy);
	q->q[2] += (qa * gy - qb * gz + q->q[3] * gx);
	q->q[3] += (qa * gz + qb * gy - qc * gx); 
	
	//正规化四元数
	//单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
	// Normalise quaternion
	recipNorm = invSqrt(q->q[0] * q->q[0] + q->q[1] * q->q[1] + q->q[2] * q->q[2] + q->q[3] * q->q[3]);
	q->q[0] *= recipNorm;
	q->q[1] *= recipNorm;
	q->q[2] *= recipNorm;
	q->q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
