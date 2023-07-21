#include "Power.h"
#include "BSP_UART.h"
#include "INA226.h"
#include "YuTu.h"

struct PM Power_management;

uint8_t Power_flag = 0;	

void Power_Control(void)
{
	uint8_t key;
	key = HAL_GPIO_ReadPin(POWER_B_GPIO_Port,POWER_B_Pin);	//检测到电源键按下,0按下，1弹起
	if(key == 0 && key == Power_management.last_key)
	{
		if(Power_management.Count == 10)
		{
			Power_management.last_power = Power_management.power_state;	//记录当前的电源键状态
			Power_management.power_state = 2;						//电源键按下100ms以上，上报有按键按下
			LOG("key down\r\n");
		}
		if(Power_management.Count < 200)
		{
			Power_management.Count++;	//单位10ms
		}
		else
		{
//			LOG("long key down\r\n");
			HAL_GPIO_WritePin(POWER_C_GPIO_Port,POWER_C_Pin,GPIO_PIN_SET);	//按键松开后保持电源打开状态
		}
	}
	else	//按键松开
	{
		Power_management.power_state = Power_management.last_power;
		if(Power_management.power_state >= 3)		//已经发送过关机指令，强制关机
		{
			if(Power_management.Count == 200)
			{
				if(Power_management.power_state < 255)
					Power_management.power_state++;
			}
			if(Power_management.power_state >= 4 || HAL_GetTick() - yutu_last > 15000)
			{
				HAL_GPIO_WritePin(CHARE_C_GPIO_Port,CHARE_C_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(POWER_C_GPIO_Port,POWER_C_Pin,GPIO_PIN_RESET);	//关机
				LOG("power off\r\n");
			}

			Power_management.Count = 0;
			Power_management.last_key = key;
			Power_management.last_power = Power_management.power_state;
			return;
		}
		if(Power_management.Count == 200)			//之前长按电源键
		{
			if(Power_management.power_state == 0)	//上电 》》》 确认要开机
			{
				LOG("Power ON!!!\r\n");
				Power_management.power_state = 1;				//电源状态改成开机
				Power_management.last_power = Power_management.power_state;
			}
			else								//已开机 》》》 要关机
			{
				LOG("shutdown!!!\r\n");
				Power_management.power_state = 3;			// 发送AP关机命令
				Power_management.last_power = Power_management.power_state;
			}
		}
		Power_management.Count = 0;
	}
	Power_management.last_key = key;
}


void BAT_Init()
{
	uint8_t i;
	INA226_Init(&INA_BAT, INA_ADDR_0);
	INA226_Init(&INA_CHARGE, INA_ADDR_3);
	for(i=0; i<10; i++)
	{
		HAL_Delay(9);
		INA226_Read(&INA_BAT);
		INA226_Read(&INA_CHARGE);
		Power_management.vbat += INA_BAT.Bus_V/1000.0f;
		Power_management.current += (INA_CHARGE.Power - INA_BAT.Current)/1000.0f;
//		Power_management.current += (INA_CHARGE.Current - INA_BAT.Current)/1000.0f;
	}
	Power_management.vbat /= 10.0f;
	Power_management.current /= 10.0f;
	Power_management.max_mah = 140000.0f;	// 电池额定容量
	Power_management.mah = (Power_management.vbat - 9.1f - Power_management.current * 0.1f) / 3.4f * Power_management.max_mah;
	Power_management.last_time = HAL_GetTick();
}


void BAT_Update()
{
	uint32_t tnow = HAL_GetTick();
	uint32_t dt;

	Power_management.vbat = INA_BAT.Bus_V / 1000.0f;	// 单位V
	
	Power_management.current = (INA_CHARGE.Power - INA_BAT.Current)/1000.0f;
//	Power_management.current = (INA_CHARGE.Current - INA_BAT.Current)/1000.0f;
	
	if(tnow > Power_management.last_time)
		dt = tnow - Power_management.last_time;
	else
		dt = Power_management.last_time - tnow;
	Power_management.last_time = tnow;
	
	Power_management.mah += Power_management.current * dt / 360.0f;
//	Power_management.mah = (Power_management.vbat - 9.0f - Power_management.current) / 3.4f * Power_management.max_mah;
	Power_management.bat = Power_management.mah/Power_management.max_mah;
	if(Power_management.bat > 1)
		Power_management.bat = 1;
	else if(Power_management.bat < 0)
		Power_management.bat = 0;
}

