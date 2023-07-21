//=====================================================================================================
// MahonyAHRS.h
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
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration
typedef struct
{
	float q[4];		// quaternion of sensor frame relative to auxiliary frame
	float twoKp;	// 2 * proportional gain (Kp)
	float twoKi;	// 2 * integral gain (Ki)
	float integralFBx,  integralFBy, integralFBz;	// integral error terms scaled by Ki
	unsigned int time_ms;		//上一次更新的时间ms部分
}Quaternion;

extern Quaternion quaternion;	//自定义的四元数结构体
extern float ahrs_yaw, ahrs_pitch, ahrs_roll;
//---------------------------------------------------------------------------------------------------
// Function declarations
void MahonyAHRSupdate(Quaternion *q, unsigned int time_ms, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(Quaternion *q, unsigned int time_ms, float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSInit(Quaternion *q, unsigned int time_ms);
void Quaternion_To_Euler_angle(Quaternion *q, float *yaw, float *pitch, float *roll);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
