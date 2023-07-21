#include "Chassis.h"
#include "Motor.h"
#include "MahonyAHRS.h"
#include "IMU_Dev.h"
#include "PID.h"
#include "math.h"
#include "CAN_ADC.h"
#include "VL53L0.h"
#include "INA226.h"
#include "DYPA02.h"
#include "AutoCharge.h"
#include "RC.h"
#include "CAN_HCSR04.h"

#define PI 3.1415926f
#define WheelDiameter				21.0f

#define SPEED_SET_MAX				100.0f			//改这个可以调整最大摇杆输出（前后速度），单位cm/s
#define Revolve_Speed				70.0f			//期望的旋转速度，单位°/s
#define Reverse_Speed				0.4f			//倒车速度上限是前进的Reverse_Speed倍。例如0.5f就是满速的50%

#define Downhill_Braking_Speed		0.6f			//下坡刹车速度，越大刹车踩得越凶，最终会降低到joystick_y * Speed_Downhill
#define Speed_Downhill				0.4f			//下坡减速比例，速度会降低到SPEED_SET_MAX * Speed_Downhill以内。例如0.4f就是满速的40%

#define DYPA02_Front_Start			1200.0f			//前方超声波开始减速的距离，单位mm
#define DYPA02_Front_End			320.0f			//前方超声波停止的距离

#define DYPA02_Behind_Start			600.0f			//后方超声波开始减速的距离
#define DYPA02_Behind_End			300.0f			//后方超声波停止的距离
#define CONTROL_HZ 1000

uint8_t Normal_Run = 0x00;	//0x00 异常状态，0x01 正常工作
float chassis_speed[2];
float Last_Moving;			//上次期望移动速度
float yaw_Set;				//期望的yaw轴位置

/**
 * @brief 计算电机输出
 *
 * @param Speed_L   	左轮子转速，单位rpm
 * @param Speed_R   	右轮子转速，单位rpm
 *
 * @return none
 */
void Chassis_Control_RPM(float Speed_L, float Speed_R)
{
	float bat = INA_BAT.Bus_V/1000.0f;
	int16_t speed[2];
	chassis_speed[0] = Speed_L;
	chassis_speed[1] = Speed_R;
//电机PID控制，使得电机转速和期望一致
	PID_Cale_POSITION(&pid_motor_L, ChassisMotor_L.rpm, Speed_L, CONTROL_HZ);
	PID_Cale_POSITION(&pid_motor_R, ChassisMotor_R.rpm, Speed_R, CONTROL_HZ);

//使用PID输出
	speed[0] = pid_motor_L.OUT;
	speed[1] = pid_motor_R.OUT;

	//电池电压补偿
	if(bat > 0)
	{
		speed[0] *= 8.7f / bat;
		speed[1] *= 8.7f / bat;
	}

	Motor_Set(&ChassisMotor_L, speed[0]);
	Motor_Set(&ChassisMotor_R, speed[1]);
}


/**
 * @brief 计算电机输出
 *
 * @param Moving_set   		期望移动速度，单位cm/s
 * @param Revolve_set   	期望旋转速度，单位°/s
 *
 * @return none
 */
void Chassis_Control_Angle(float Moving_set, float Revolve_set)
{
//单位转化，把期望速度转化成对应的电机转速
	//移动期望转速 = 期望移动速度/轮子周长*60
	Moving_set = Moving_set/(PI*WheelDiameter)*60.0f;

//期望电机转速PID计算，得到值是满足旋转速度需求的电机转速
	PID_Cale_POSITION(&pid_yaw_speed, imu.Gyro_Deg[2], Revolve_set, 200);
//使用电机速度环+角度控制
	Chassis_Control_RPM(Moving_set + pid_yaw_speed.OUT,  pid_yaw_speed.OUT - Moving_set);
}


void Chassis_Init()
{

}

float DYPA02_Decelerate(float speed);

float Get_Chassis_Angle_Err(float angle)
{
	float set_yaw_temp;
	if(yaw_Set > 180)
		yaw_Set -= 360;
	if(yaw_Set < -180)
		yaw_Set += 360;
	set_yaw_temp = yaw_Set;
	if(angle > 90 && set_yaw_temp < -90)
	{
		set_yaw_temp += 360;
	}
	else if(set_yaw_temp > 90 && angle < -90)
	{
		set_yaw_temp -= 360;
	}
	return set_yaw_temp;
}

/**
 * @brief 底盘控制
 *
 * @return none
 */
void Chassis_Run()
{
	float Moving = 0;		//期望移动速度
	float Revolve = 0;		//期望旋转速度
	float set_yaw_temp = 0;	//设置yaw轴
	static uint32_t last_run_time = 0;	//上次使用角度控制的时间
	
	//回充检测
	AutoCharge();
	
	if(rc.Mode == 0 && AutoCharge_Mode == 0)	
	{
		Last_Moving = 0;
		yaw_Set = ahrs_yaw;
	}
	else if(rc.Mode == 2 && AutoCharge_Mode == 0)	//仅保留电机PID（摇杆混控）
	{
		Normal_Run = 1;
		yaw_Set = ahrs_yaw;
		Chassis_Control_RPM(rc.Y + rc.X,\
							rc.Y - rc.X);
		return;
	}
	else if(rc.Mode == 3 && AutoCharge_Mode == 0)	//直接调整PWM（摇杆混控）
	{
		Normal_Run = 1;
		yaw_Set = ahrs_yaw;
		Motor_Set(&ChassisMotor_L, rc.Y + rc.X);
		Motor_Set(&ChassisMotor_R, rc.Y - rc.X);
		return;
	}
	else if(rc.Mode == 4 && AutoCharge_Mode == 0)	//仅保留电机PID（双轮独立）
	{
		Normal_Run = 1;
		yaw_Set = ahrs_yaw;
		Chassis_Control_RPM(rc.Y, rc.X);
		return;
	}
	else if(rc.Mode == 5 && AutoCharge_Mode == 0)	//直接调整PWM（双轮独立）
	{
		Normal_Run = 1;
		yaw_Set = ahrs_yaw;
		Motor_Set(&ChassisMotor_L, rc.Y);
		Motor_Set(&ChassisMotor_R, rc.X);
		return;
	}

	if(!(rc.Mode == 0 && AutoCharge_Mode == 0))	//非待机状态
	{
		Moving = (rc.Y - rc.X)/100.0f*SPEED_SET_MAX;
		Revolve = (rc.Y + rc.X)/100.0f*Revolve_Speed;
	}
	
	//红外回充数据
	Revolve += IR_Angel;
	Moving += IR_Speed;
	
	if(Moving == 0 && Revolve == 0)	//静止
	{
		yaw_Set = ahrs_yaw;
		PID_resize(&pid_yaw_speed);
		PID_resize(&pid_yaw_angle);
	}
//	if(rc.Mode == 0x01)	//正常模式（加入传感器）
//	{
//		Normal_Run = 0x01;
//		if(Moving < 0)	//倒车减速
//			Moving *= Reverse_Speed;

//		//前方悬崖检测
//		if(((Rangefinder_hub.Distence[1] > 18) || (Rangefinder_hub.Distence[2] > 18)))
//		{
//			Revolve = 0;    // 有悬崖，禁止旋转
//			if(Moving > 0)  // 前方有悬崖，禁止向前
//			{
//				Normal_Run = 0x00;
//				Moving = 0;
//			}
//		}
//		//后方悬崖检测
//		if(Rangefinder_hub.Distence[0] > 34)
//		{
//			Revolve = 0;      // 有悬崖，禁止旋转
//			if(Moving < 0)    // 后方有悬崖，禁止向前
//			{
//				Normal_Run = 0x00;
//				Moving = 0;
//			}
//		}

//		//前上方有障碍物检测，禁止前进
//		if(Moving > 0 && VL53L0_Dev[1].distance >5 && VL53L0_Dev[1].distance < 42)
//		{
//			Normal_Run = 0x00;
//			Moving = 0;
//		}
//		else
//			Moving = DYPA02_Decelerate(Moving);

//		//减速下坡倒车（车头向前）
//		if(ahrs_pitch < -4.5f && Moving > 0)		//车头朝下，下坡
//		{
//			if(Last_Moving > SPEED_SET_MAX*Speed_Downhill && Moving > Downhill_Braking_Speed)
//				Moving = Last_Moving - Downhill_Braking_Speed;	//车头向前下坡，不能急刹车
//			else if(Moving > SPEED_SET_MAX*Speed_Downhill)
//				Moving *= Speed_Downhill;
//		}
//	}
	
	float freq = 1000;
	uint32_t tnow = HAL_GetTick();
	if(tnow != last_run_time)
	{
		freq = 1000.0f / (tnow - last_run_time);
	}
	last_run_time = tnow;
	yaw_Set += Revolve/freq;
	set_yaw_temp = Get_Chassis_Angle_Err(ahrs_yaw);
	PID_Cale_POSITION(&pid_yaw_angle, ahrs_yaw, set_yaw_temp, 200);
	Last_Moving = Moving;
	Chassis_Control_Angle(Moving, pid_yaw_angle.OUT);
}


/**
 * @brief  超声波减速
 *
 * @param  speed 移动的速度
 *
 * @return 修改后的移动速度
 */
float DYPA02_Decelerate(float speed)
{
	float Min_Distence;	//最短超声波距离
	//前后障碍物检测
	if(speed > 0 && (ahrs_pitch < 4.5f && ahrs_pitch > -4.5f) &&\
		(Rangefinder_hub.Distence[1] < 10 ||
		Rangefinder_hub.Distence[2] < 10 ||
		hcsr_hub.Distence[0] < 60 ||
		hcsr_hub.Distence[1] < 60 || 
		hcsr_hub.Distence[7] < 60))	//向前移动，但不是从平地开始爬坡
	{
		if(hcsr_hub.Distence[0] > hcsr_hub.Distence[1] && hcsr_hub.Distence[1] > hcsr_hub.Distence[7])
			Min_Distence = hcsr_hub.Distence[7];
		else if(hcsr_hub.Distence[7] > hcsr_hub.Distence[0] && hcsr_hub.Distence[1] > hcsr_hub.Distence[0])
			Min_Distence = hcsr_hub.Distence[0];
		else
			Min_Distence = hcsr_hub.Distence[1];

		if(Min_Distence < DYPA02_Front_Start)	//前方100cm有障碍物，开始减速，25cm时停止
		{
			if(Min_Distence < DYPA02_Front_End)
			{
				Normal_Run = 0x00;
				speed = 0;
			}
			else
				speed *= 0.4f;
		}
	}
	else if(speed < 0)								//向后移动，静止不用管，后方20cm有物体时停止
	{
		Min_Distence = hcsr_hub.Distence[5];
		//需要判断后方的超声波
		if(Min_Distence < DYPA02_Behind_Start)
		{
			if(Min_Distence < DYPA02_Behind_End)
			{
				Normal_Run = 0x00;
				speed = 0;
			}
			else
				speed *= 0.4f;
		}
	}
	return speed;
}


