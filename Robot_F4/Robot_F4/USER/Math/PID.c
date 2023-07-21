/**
  ******************************************************************************
  * @file           : PID.c
  * @brief          : PID计算过程
  ******************************************************************************
  * @attention		
  *
	* @Logs:
	* Date           Author       Notes
	* 2021-12-25     李树益      第一个版本
	* 2022-01-07     李树益      修正了PID的计算公式
	* 2022-10-04     李树益      修正了PID的计算公式
  ******************************************************************************
  */
	
#include "PID.h"
#include "ProgramData.h"

#define sampleFreqDef	1000.0f			// 默认PID频率

//底盘电机速度PID
PID pid_motor_L;
PID pid_motor_R;

//底盘YAW轴旋转速度PID
PID pid_yaw_speed;
PID pid_yaw_angle;

//回充pid
PID pid_charge_angle;
PID pid_charge_speed;


/**
 * @brief 初始化PID参数
 *
 * @param pid   		PID结构体地址
 * @param Kp   			比例参数
 * @param Ki   			积分参数
 * @param Kd   			微分参数
 * @param I_MAX   	积分最大值
 * @param I_MIN   	积分最小值
 * @param OUT_MAX   输出最大值
 * @param OUT_MIN   输出最小值
 *
 * @return none
 */
void PID_Creat(PID *pid, float Kp, float Ki, float Kd, float I_MAX, float I_MIN, float OUT_MAX, float OUT_MIN)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	
	pid->P_OUT = 0;
	pid->I_OUT = 0;
	pid->D_OUT = 0;
	pid->OUT = 0;
	
	pid->I_MAX = I_MAX;
	pid->I_MIN = I_MIN;
	pid->OUT_MAX = OUT_MAX;
	pid->OUT_MIN = OUT_MIN;
	
	pid->last_delta = 0;
}

/**
 * @brief 计算PID
 *
 * @param pid   		PID结构体地址
 * @param delta   	误差
 * @param time   		当前时间
 *
 * @return none
 */
void PID_Cale_DELTA(PID *pid, float delta, float freq)
{
	uint32_t tnow = HAL_GetTick();
	if( ! (freq > 0.0f) )	//频率不能小于或等于0
	{
		freq = sampleFreqDef;
	}
	
	if(tnow != pid->last_time)
	{
		freq = 1000.0f/(tnow - pid->last_time);
		pid->last_time = tnow;
	}

	//计数PID各个部分的输出
	pid->P_OUT = delta * pid->Kp;
	pid->I_OUT += delta / freq * pid->Ki;
	pid->D_OUT = (pid->last_delta - delta) * freq * pid->Kd;
	pid->last_delta = delta;
	//积分限幅
	if(pid->I_OUT > pid->I_MAX)
	{
		pid->I_OUT = pid->I_MAX;
	}
	else if(pid->I_OUT < pid->I_MIN)
	{
		pid->I_OUT = pid->I_MIN;
	}
	//计算总输出
	pid->OUT = pid->P_OUT + pid->I_OUT + pid->D_OUT;
	if(pid->OUT > pid->OUT_MAX)	//输出限幅
	{
		pid->OUT = pid->OUT_MAX;
	}
	else if(pid->OUT < pid->OUT_MIN)
	{
		pid->OUT = pid->OUT_MIN;
	}
}

/**
 * @brief 计算PID
 *
 * @param pid   		PID结构体地址
 * @param this_pos  现在的位置
 * @param next_pos  期望的位置
 * @param time   		当前时间
 *
 * @return none
 */
void PID_Cale_POSITION(PID *pid, float this_pos, float next_pos, float freq)
{
	float delta = next_pos - this_pos;
	
	if( ! (freq > 0.0f) )	//频率不能小于或等于0
	{
		freq = sampleFreqDef;
	}

	PID_Cale_DELTA(pid, delta, freq);
}

/**
 * @brief 重置PID
 *
 * @param pid   		PID结构体地址
 *
 * @return none
 */
void PID_resize(PID *pid)
{
	pid->P_OUT = 0;
	pid->I_OUT = 0;
	pid->D_OUT = 0;
	pid->OUT = 0;
	pid->last_delta = 0;
}

/**
 * @brief 初始化PID
 *
 * @return none
 */
void PID_Init()
{
	//底盘电机速度PID
	PID_Creat(&pid_motor_L,
//						program_data.set.pid_kp[PID_MOTOR_L],
//						program_data.set.pid_ki[PID_MOTOR_L],
//						program_data.set.pid_kd[PID_MOTOR_L],							//P, I, D
						5.0f, 20.0f, 0,
						100, -100, 100, -100);			//Imax, Imin, OutMax, OutMin
	PID_Creat(&pid_motor_R,
//						program_data.set.pid_kp[PID_MOTOR_R],
//						program_data.set.pid_ki[PID_MOTOR_R],
//						program_data.set.pid_kd[PID_MOTOR_R],							//P, I, D
						5.0f, 20.0f, 0,
						100, -100, 100, -100);			//Imax, Imin, OutMax, OutMin
	//底盘YAW轴PID
	PID_Creat(&pid_yaw_speed,
//						program_data.set.pid_kp[PID_REVOLVE],
//						program_data.set.pid_ki[PID_REVOLVE],
//						program_data.set.pid_kd[PID_REVOLVE],							//P, I, D
						1.0f, 0.3f, 0,
						30, -30, 90, -90);			//Imax, Imin, OutMax, OutMin
	PID_Creat(&pid_yaw_angle,
						0.8f, 0.0f, 0,							//P, I, D
						10, -10, 360, -360);			//Imax, Imin, OutMax, OutMin
						
	//红外回充PID
	PID_Creat(&pid_charge_angle,
						//P, I, D
						0.3, 0.02, 0.03,
						5.0f, -5.0f, 8, -8);		//Imax, Imin, OutMax, OutMin
						//0.5, 0.01, 0.01,		//P, I, D
						//7.5, -7.5, 10, -10	//Imax, Imin, OutMax, OutMin
	PID_Creat(&pid_charge_speed,
						//P, I, D
						0.5, 0, 1,
						35, -35, 35, -35);			//Imax, Imin, OutMax, OutMin
}
