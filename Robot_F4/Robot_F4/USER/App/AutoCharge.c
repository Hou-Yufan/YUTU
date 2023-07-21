#include "AutoCharge.h"
#include "Motor.h"
#include "ADS1115.h"
#include "MahonyAHRS.h"
#include "Displayer.h"
#include "INA226.h"
#include "DYPA02.h"
#include "math.h"
#include "PID.h"
#include "Chassis.h"
#include "RC.h"
#include "Displayer.h"
#include "CAN_HCSR04.h"

#define REVOLE_TIME		10000	//左右扫描的时间
#define CHARGE_TIME		1000	//进入充电桩时间(1000=1s)
#define HOLD_TIME		500		//维持状态稳定的时间

#define CHARGE_REVOLE_SPEED	30.0f	//寻找充电桩时的旋转速度，单位°/s
#define CHARGE_MOVE_SPEED_0	12.0f	//寻找充电桩时的移动速度（1.2m内无障碍），单位cm/s
#define CHARGE_MOVE_SPEED_1	8.0f	//寻找充电桩时的移动速度（1.2m内有障碍）
#define CHARGE_MOVE_SPEED_2	5.0f	//寻找充电桩时的移动速度（0.5m内有障碍）

float IR_Angel = 0;
float IR_Speed = 0;

uint8_t AutoCharge_Mode = 0;
uint16_t Stop_Delay = 500;		//红外死区延时
uint16_t IR_Delay = REVOLE_TIME;
uint16_t IR_Min = 4096;							//红外最强的位置
float Charger_angle = 0;						//充电器方向
float Charger_Pitch = 0;

uint8_t find_charger = 0;	//发现充电器
uint8_t charger_set = 0;	//回充步骤

//寻找红外充电桩
void Seek_Charger()
{
	IR_Speed = 0;
	if(IR_Delay > REVOLE_TIME*3/4 || IR_Delay < REVOLE_TIME/4)
		IR_Angel = CHARGE_REVOLE_SPEED;
	else if(IR_Delay < REVOLE_TIME*3/4 && IR_Delay > REVOLE_TIME/4)
		IR_Angel = -CHARGE_REVOLE_SPEED;
	else
		IR_Angel = 0;
	
	if(charge_ir.adc[IR_M] < IR_Min)
	{
		IR_Min = charge_ir.adc[IR_M];
		Charger_angle = ahrs_yaw;
		find_charger = 1;
	}
	
	Charger_Pitch += ahrs_pitch/REVOLE_TIME;
	
	IR_Delay--;
	if(IR_Delay == 0)
	{
		if(find_charger == 1)
		{
			find_charger = 2;
		}
		else	//失败，不在充电桩附近
		{
			AutoCharge_Mode = 0;
		}
	}
}

// 计算角度输出 误差+-0.5°
float Get_Angle_Error(float angle_temp)
{
	if(ahrs_yaw > 90 && angle_temp < -90)
	{
		angle_temp += 360;
	}
	else if(angle_temp > 90 && ahrs_yaw < -90)
	{
		angle_temp -= 360;
	}
	
	angle_temp = angle_temp - ahrs_yaw;
	if(angle_temp > CHARGE_REVOLE_SPEED)
		angle_temp = CHARGE_REVOLE_SPEED;
	else if(angle_temp < -CHARGE_REVOLE_SPEED)
		angle_temp = -CHARGE_REVOLE_SPEED;
	if(angle_temp > -0.5f && angle_temp < 0.5f)
	{
		if(IR_Delay)
			IR_Delay--;
		angle_temp = 0;
	}
	return angle_temp;
}

void AutoCharge(void)
{	
	static uint16_t key_cnt = 0;
	if(rc.Mode == 7)
		AutoCharge_Mode = 4;
	
	if((Displayer.Regs[1] & 0x01) == 1)
	{
		key_cnt++;
		if(key_cnt == 3000)	// 1000 = 1s
		{
			key_cnt = 0;
			if(AutoCharge_Mode == 0)
				AutoCharge_Mode = 4;
			else
				AutoCharge_Mode = 0;
		}
	}
	else
	{
		key_cnt = 0;
	}
	
	if(AutoCharge_Mode == 0)	//不需要进入充电
	{
		IR_Angel = 0;
		IR_Speed = 0;
		IR_Min = 4096;
		find_charger = 0;
		charger_set = 0;
		Charger_Pitch = 0;
		IR_Delay = REVOLE_TIME;	//30%速度旋转一圈需要的时间
		//0x00：未进入回充；0x01：左；0x02右；0x03：中间；0x04：寻找充电桩 0x05：在充电
		return;
	}
	
	if(INA_CHARGE.Power > 200 || hcsr_hub.Distence[5] < 20)// 	//在充电
	{
		if(IR_Delay)
		{
			IR_Delay--;
		}
		else
		{
			yaw_Set = ahrs_yaw;
			AutoCharge_Mode = 0;
			IR_Angel = 0;
			IR_Speed = 0;
			IR_Min = 4096;
			find_charger = 0;
			charger_set = 0;
			IR_Delay = REVOLE_TIME;
			PID_resize(&pid_motor_L);
			PID_resize(&pid_motor_R);	
			return;
		}
	}
	
	if(charger_set == 0)			//寻找充电桩
	{
		AutoCharge_Mode = 0x04;
		Seek_Charger();
		if(find_charger == 2)
		{
			IR_Angel = 0;
			IR_Speed = 0;
			find_charger = 0;
			charger_set = 1;
			IR_Delay = HOLD_TIME;
			Stop_Delay = HOLD_TIME;
		}
		return;
	}
	else if(charger_set == 1)	//对准充电桩
	{
		IR_Angel = Get_Angle_Error(Charger_angle);
		if(IR_Angel == 0 && IR_Delay == 0)
		{
			IR_Speed = 0;
			IR_Min = 4096;
			IR_Delay = HOLD_TIME;
			Stop_Delay = HOLD_TIME;
			charger_set = 2;
		}
		return;
	}
	else if(charger_set == 2)	//移动到里充电桩 52.5 +- 2.5 cm的位置
	{
		IR_Angel = Get_Angle_Error(Charger_angle);
		if(hcsr_hub.Distence[5] > 550)
		{
			if(hcsr_hub.Distence[5] > 1200)
			{
				IR_Speed = -CHARGE_MOVE_SPEED_0;
			}
			else
			{
				IR_Speed = -CHARGE_MOVE_SPEED_1;
			}
		}
		else if(hcsr_hub.Distence[5] < 500)
		{
			IR_Speed = CHARGE_MOVE_SPEED_2;
		}
		else
		{
			IR_Speed = 0;
			if(Stop_Delay)
				Stop_Delay--;
			else
			{
				IR_Angel = 0;
				IR_Min = 4096;
				Stop_Delay = HOLD_TIME;
				IR_Delay = CHARGE_TIME;
				PID_resize(&pid_charge_angle);
				charger_set = 3;	//
			}
		}
		return;
	}

	//PID校准方向
	if(hcsr_hub.Distence[5] > 55)
	{
		PID_Cale_POSITION(&pid_charge_angle, charge_ir.adc[IR_L], charge_ir.adc[IR_R], 200);
		//P限幅
		if(pid_charge_angle.P_OUT > pid_charge_angle.OUT_MAX*0.5f)
		{	pid_charge_angle.P_OUT = pid_charge_angle.OUT_MAX*0.5f;	}
		else if(pid_charge_angle.P_OUT < pid_charge_angle.OUT_MIN*0.5f)
		{	pid_charge_angle.P_OUT = pid_charge_angle.OUT_MIN*0.5f;	}
		//计算PI控制
		IR_Angel = pid_charge_angle.P_OUT + pid_charge_angle.I_OUT;
		//D限幅
		if(IR_Angel > 0)
		{
			if(pid_charge_angle.D_OUT > IR_Angel)
				pid_charge_angle.D_OUT = IR_Angel;
			else if(pid_charge_angle.D_OUT < -IR_Angel)
				pid_charge_angle.D_OUT = -IR_Angel;
		}
		else if(IR_Angel < 0)
		{
			if(pid_charge_angle.D_OUT < IR_Angel)
				pid_charge_angle.D_OUT = IR_Angel;
			else if(pid_charge_angle.D_OUT > -IR_Angel)
				pid_charge_angle.D_OUT = -IR_Angel;
		}
		else
		{
			if(pid_charge_angle.D_OUT > pid_charge_angle.OUT_MAX)
				pid_charge_angle.D_OUT = pid_charge_angle.OUT_MAX;
			else if(pid_charge_angle.D_OUT < pid_charge_angle.OUT_MIN)
				pid_charge_angle.D_OUT = pid_charge_angle.OUT_MIN;
		}
		//加上D输出
		IR_Angel += pid_charge_angle.D_OUT;
		//输出限幅
		if(IR_Angel > pid_charge_angle.OUT_MAX)
		{	IR_Angel = pid_charge_angle.OUT_MAX;	}
		else if(IR_Angel < pid_charge_angle.OUT_MIN)
		{	IR_Angel = pid_charge_angle.OUT_MIN;	}
		
		if(IR_Angel > 0)
		{
			AutoCharge_Mode = 0x02;
		}
		else if(IR_Angel < 0)
		{
			AutoCharge_Mode = 0x01;
		}
		
		//前后方向调整（如果方向盘打到最大，且很靠近充电桩，往外走；否则继续倒车）
//		if(hcsr_hub.Distence[5] < 100 && (IR_Angel == pid_charge_angle.OUT_MAX || IR_Angel == pid_charge_angle.OUT_MIN))
//		{
//			IR_Angel /= 2;
//			IR_Speed = CHARGE_MOVE_SPEED_1;	//10
//		}
//		else
			IR_Speed = -CHARGE_MOVE_SPEED_0;	//-15
	}
	else
	{
		IR_Angel = 0;
		IR_Speed = -CHARGE_MOVE_SPEED_0;	//-15
		AutoCharge_Mode = 0x03;
	}
	if(ahrs_pitch < Charger_Pitch - 2.0f)	//倾斜-2.5°，可能开到充电桩上了
	{
		IR_Speed = CHARGE_MOVE_SPEED_0;		//不一定能向前走（其他传感器限制），但能停下
	}
}

