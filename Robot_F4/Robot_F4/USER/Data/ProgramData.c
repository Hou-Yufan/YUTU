#include "ProgramData.h"
#include "Flash.h"
#include "string.h"
#include "usart.h"

Program_Data program_data;

/**
 * FLASH参数恢复默认值
 *
 * @param  none
 *
 * @return none
 */
void Program_Data_Reset()
{
//----------------------------------------------
//角速度计校准值
	program_data.set.Gyro_Offset[0] = 0;
	program_data.set.Gyro_Offset[1] = 0;
	program_data.set.Gyro_Offset[2] = 0;
//----------------------------------------------
//加速度计校准值
	program_data.set.Acc_Offset[0] = 0;
	program_data.set.Acc_Offset[1] = 0;
	program_data.set.Acc_Offset[2] = 0;
	
	program_data.set.Acc_Offset_Scale[0] = 1;
	program_data.set.Acc_Offset_Scale[1] = 1;
	program_data.set.Acc_Offset_Scale[2] = 1;
//----------------------------------------------
//左电机速度PID
	program_data.set.pid_kp[PID_MOTOR_L] = 0.0f;
	program_data.set.pid_ki[PID_MOTOR_L] = 1.0f;
	program_data.set.pid_kd[PID_MOTOR_L] = 0;
//右电机速度PID
	program_data.set.pid_kp[PID_MOTOR_R] = 0.0f;
	program_data.set.pid_ki[PID_MOTOR_R] = 1.0f;
	program_data.set.pid_kd[PID_MOTOR_R] = 0;
//底盘旋转速度PID
	program_data.set.pid_kp[PID_REVOLVE] = 1;
	program_data.set.pid_ki[PID_REVOLVE] = 0;
	program_data.set.pid_kd[PID_REVOLVE] = 0;
//屏幕上下PID
	program_data.set.pid_kp[PID_DISPLAY] = 10.5;
	program_data.set.pid_ki[PID_DISPLAY] = 200;
	program_data.set.pid_kd[PID_DISPLAY] = 1;
//修改结束
	program_data.set.Modify_cnt = 0;
	program_data.set.flag = 0x01;
}


/**
 * 校验FLASH参数
 *
 * @param check_sum		合校验结果
 *				check_bcc		异或校验结果
 *
 * @return none
 */
void Program_Data_Check(uint16_t *check_sum, uint8_t *check_bcc)
{
	uint16_t len = sizeof(Program_Data);
	uint16_t i = 0;
	*check_sum = 0;
	*check_bcc = 0;
	for(i = 4; i < len; i++)
	{
		*check_sum += program_data.buf[i];
		*check_bcc ^= program_data.buf[i];
	}
}
 
 
/**
 * 初始化FLASH，从FLASH中读取飞控参数
 *
 * @param none
 *
 * @return none
 */
void Program_Data_Init()
{
	uint16_t check_sum;
	uint8_t check_bcc;
	FLASH_Read((uint32_t *)program_data.buf, sizeof(program_data));	//从FLASH中读取参数
	if(program_data.set.flag != 0x64)	//查看数据保存标志
	{
		LOG("Program flag error: %d\r\n",program_data.set.flag);
		Program_Data_Reset();
		return;
	}
	Program_Data_Check(&check_sum, &check_bcc);	//数据校验
	if(program_data.set.check_sum != check_sum || program_data.set.check_bcc != check_bcc)
	{
		LOG("Program check error: %d, %d; %d, %d\r\n",check_sum, check_bcc, program_data.set.check_sum, program_data.set.check_bcc);
		Program_Data_Reset();
		return;
	}
	program_data.set.flag = 0x00;
	LOG("Program ready\r\n");
}

void Program_Data_Modify()
{
	program_data.set.flag = 0x01;
	program_data.set.Modify_cnt++;
}

/**
 * 保存参数到FLASH
 *
 * @param none
 *
 * @return none
 */
void Program_Data_Save(void)
{
	if(program_data.set.flag > 0)
	{
		__disable_irq();
		program_data.set.flag ++;
		if(program_data.set.flag == 0x64)//program_data.set.flag == 100
		{
			//数据校验
			Program_Data_Check(&program_data.set.check_sum, &program_data.set.check_bcc);
			//准备写入FLASH
			FLASH_Write((uint32_t *)program_data.buf, sizeof(program_data));
			Program_Data_Init();
			LOG("Program save successfully\r\n");
		}
		__enable_irq();
	}
}


