#include "Scheduler.h"
#include "MahonyAHRS.h"
#include "ano_dt.h"
#include "IMU_Dev.h"
#include "ProgramData.h"
#include "Displayer.h"
#include "Power.h"
#include "DYPA02.h"
#include "VL53L0.h"
#include "SGP30.h"
#include "SHT20.h"
#include "GP2Y1014AU0F.h"
#include "ADS1115.h"
#include "Motor.h"
#include "Chassis.h"
#include "Power.h"
#include "BSP_CAN.h"
#include "RC.h"
#include "YuTu.h"
#include "INA226.h"

static void Loop_1000Hz(void)	//1ms执行一次
{
	/*更新IMU数据*/
	IMU_Update();
	/*姿态解算更新*/
	MahonyAHRSupdateIMU(&quaternion, HAL_GetTick(),
												imu.Gyro_LPF_Deg[0],	imu.Gyro_LPF_Deg[1],	imu.Gyro_LPF_Deg[2],
												imu.Acc_LPF_G[0],			imu.Acc_LPF_G[1],			imu.Acc_LPF_G[2]);
	/*获取欧拉角*/
	Quaternion_To_Euler_angle(&quaternion, &ahrs_yaw, &ahrs_pitch, &ahrs_roll);

	/*红外回充检测*/
	ADS1115_Run();	//没接这个设备时，不要打开
	
	/*上报调试状态数据*/
	CAN_Auto_Send_Task();
	YuTu_DT_Data_Exchange();
	ANO_DT_Data_Exchange();
	ANO_TX_Task();
}

static void Loop_500Hz(void)	//2ms执行一次
{
	/*PM2.5检测*/
	GP2Y1010AU0F_Run();
}

static void Loop_200Hz(void)	//5ms执行一次
{
	RC_WatchDog();
}

static void Loop_100Hz(void)	//10ms执行一次
{
	
	/*更新电机PWM输出*/
	
	/*延时保存flash*/
	Program_Data_Save();
	Power_Control();
}

static void Loop_50Hz(void)	//20ms执行一次
{
	INA226_Read(&INA_BAT);
	INA226_Read(&INA_CHARGE);
}

static void Loop_20Hz(void)	//50ms执行一次
{
	BAT_Update();
	if(Power_management.bat < 0.2f){	// 电量低于20%
		if(Power_management.current < 200){	// 充电电流低于200ma
			HAL_GPIO_TogglePin(CHARG_LED_GPIO_Port, CHARG_LED_Pin);
		}
		else{
			HAL_GPIO_WritePin(CHARG_LED_GPIO_Port, CHARG_LED_Pin, GPIO_PIN_RESET);
		}
	}
	else {
		HAL_GPIO_WritePin(CHARG_LED_GPIO_Port, CHARG_LED_Pin, GPIO_PIN_SET);
	}
}

static void Loop_2Hz(void)	//500ms执行一次
{	
//	Read_SHT20();
//	SGP30_Read();
	HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
	VL53L0_Dev_Check();			//VL53L0激光掉线检测
	DYPA02_Dev_Check();			//DYPA02超声波掉线检测
	if(imu.gyro_calibrate){	//校准角速度计
		IMU_GYRO_Fast_calibrate();
	}
	if(imu.acc_calibrate){		//校准加速度计
		IMU_ACC_Fast_calibrate();
	}

	LOG("PM2_5 = %.2f\r\n",PM2_5.data);
	LOG("BAT = %.2fv %.2fma\r\n",Power_management.vbat, Power_management.current);
}

//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},
	{Loop_500Hz , 500,  0, 0},
	{Loop_200Hz , 200,  0, 0},
	{Loop_100Hz , 100,  0, 0},
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for(index=0;index < TASK_NUM;index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND/sched_tasks[index].rate_hz;
		//最短周期为1，也就是1ms
		if(sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = HAL_GetTick();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}

