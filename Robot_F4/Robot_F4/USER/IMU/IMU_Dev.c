/**
  ******************************************************************************
  * @file           : IMU_Dev.c
  * @brief          : IMU模块抽象层
  ******************************************************************************
  * @attention		
  *
	* @Logs:
	* Date           Author       Notes
	* 2022-08-21     李树益      第一个版本，暂未实现加速度计六面校准
  ******************************************************************************
  */
#include "IMU_Dev.h"
#include "ICM42605.h"
#include "spi.h"
#include "ProgramData.h"
#include "usart.h"

IMU_Dev imu = {
	.Gyro_Rotation = {{-1,0,0}, 
										{0,-1,0}, 
										{0,0,1}},
	
	.Acc_Rotation = { {1,0,0}, 
										{0,1,0}, 
										{0,0,-1}},
	.Acc_Offset_Scale = {1,1,1},
};

struct icm42605 icm = {
	.hw = &hspi1,
	.CS_GPIO_Port = ICM_CS_GPIO_Port,
	.CS_Pin = ICM_CS_Pin,
	.HW_Type = ICM42605_HW_SPI,
};

/**
 * @brief 初始化IMU抽象层
 *
 * @param none
 *
 * @return none
 */
void IMU_Init()
{
	do{
		LOG("IMU Init\r\n");
		HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
	}while(ICM42605_Init(&icm, AFS_8G, GFS_2000DPS, 1000));
	
	imu.Acc_Scale_Factor = icm.Acc_Scale_Factor;
	imu.Gyro_Scale_Factor = icm.Gyro_Scale_Factor;
	
	for(uint8_t i=0;i<3;i++)	
	{
		//初始化校准参数
		imu.Acc_Offset_Scale[i] = program_data.set.Acc_Offset_Scale[i];
		imu.Acc_Offset[i] = program_data.set.Acc_Offset[i];
		imu.Gyro_Offset[i] = program_data.set.Gyro_Offset[i];
#ifdef		LOWPASSFILTER_H_
		//初始化陀螺仪的二阶低通滤波器
		lpf2pSetCutoffFreq(&imu.Acc_lpfilter[i],1000,250);
		lpf2pSetCutoffFreq(&imu.Gyro_lpfilter[i],1000,150);
#endif
	}
}

/**
 * @brief 更新IMU数据
 *
 * @param none
 *
 * @return none
 */
void IMU_Update()
{
	int16_t accel[3];
	int16_t gyro[3];
	uint8_t i;

	ICM42605_GetAccelerometer(&icm, accel);
	ICM42605_GetGyroscope(&icm, gyro);
	for (i = 0; i < 3; ++i) {   //将陀螺仪的数据旋转到相应的坐标系
			imu.Acc_Raw[i] = accel[0]*imu.Acc_Rotation[i][0] + accel[1]*imu.Acc_Rotation[i][1] + accel[2]*imu.Acc_Rotation[i][2];
			imu.Gyro_Raw[i] = gyro[0]*imu.Gyro_Rotation[i][0] + gyro[1]*imu.Gyro_Rotation[i][1] + gyro[2]*imu.Gyro_Rotation[i][2] ;
		
			imu.Acc_G[i] = (imu.Acc_Raw[i]/imu.Acc_Scale_Factor + imu.Acc_Offset[i]) * imu.Acc_Offset_Scale[i];
			imu.Gyro_Deg[i] = imu.Gyro_Raw[i]/imu.Gyro_Scale_Factor + imu.Gyro_Offset[i];
			
			if(imu.gyro_calibrate == 0 && imu.Gyro_Deg[i] < 0.2f && imu.Gyro_Deg[i] > -0.2f)
			{
				imu.Gyro_Deg[i] = 0;
			}
			
			imu.Acc_Raw[i] = imu.Acc_G[i]*imu.Acc_Scale_Factor;
			imu.Gyro_Raw[i] = imu.Gyro_Deg[i]*imu.Gyro_Scale_Factor;
		
#ifdef		LOWPASSFILTER_H_
			imu.Acc_LPF_G[i] = lpf2pApply(&imu.Acc_lpfilter[i],imu.Acc_G[i]);
			imu.Gyro_LPF_Deg[i] = lpf2pApply(&imu.Gyro_lpfilter[i],imu.Gyro_Deg[i]);
#else
			imu.Acc_LPF_G[i] = imu.Acc_G[i];
			imu.Gyro_LPF_Deg[i] = imu.Gyro_Deg[i];
#endif
	}
	if(imu.gyro_calibrate == 0)
	{
		IMU_GYRO_Auto_calibrate();
	}
}

/**
 * @brief 角速度计静止校准（手动）
 *
 * @param none
 *
 * @return none
 */
void IMU_GYRO_Fast_calibrate()
{
	float diff = 10;		//静止时的误差
	uint16_t a = 500;		//单轮读取的次数
	uint16_t i = 0;			//记录有多少次数据是稳定的
	uint16_t k = 0;			//记录读取了多少次数据
	int16_t t = 0;			//两次陀螺仪误差暂存值
	uint8_t j = 0;			//用来刷0到2
	int8_t n = 0;				//重复校准的次数
	
	int16_t gyro_temp[3] = {0};
	LOG("Gyro calibrate\r\n");
	//重复校准几次，使得误差最小
	for(n = 0; n < 4; n++)
	{
		k = 0;
		i = 0;
		while(1)
		{
			IMU_Update();
			for(j=0;j<3;j++)
			{
				t = imu.Gyro_Raw[j] - gyro_temp[j];
				if(t > -diff && t < diff)
				{
					i++;
				}
				gyro_temp[j] = imu.Gyro_Raw[j];
				imu.gyro_sum[j] += imu.Gyro_Deg[j];
			}
			k++;
			if(i > 0)
				i--;
			HAL_Delay(2);
			if(i > a)
			{
				diff /= 2;	//允许的误差减小
				a *= 2;			//增加累计误差的周期增加
				break;
			}
			if(k > a*2.0f)	//一次失败的校准
			{
				if(i<k)
					if(n>0)
						n--;
				break;
			}
		}
//////////  每次认为静止时间足够长的时候都校准一次  /////////////
		for (j = 0; j < 3; j++) 
		{
			imu.Gyro_Offset[j] -= imu.gyro_sum[j]/k;
			imu.gyro_sum[j] = 0;
    }
	}
	for (j = 0; j < 3; j++) 
	{
		program_data.set.Gyro_Offset[j] = imu.Gyro_Offset[j];
	}
	Program_Data_Modify();
	imu.gyro_calibrate = 0;
}

/**
 * @brief 角速度计零飘校准（自动）
 *
 * @param none
 *
 * @return none
 */
void IMU_GYRO_Auto_calibrate()
{
	uint8_t i = 0;
	if(imu.motionless == 0)
	{
		for(i = 0; i<3; i++)
			imu.gyro_sum[i] = 0;
	}
	
	if(imu.Gyro_Raw[0] <= 10  && 
		 imu.Gyro_Raw[0] >= -10 &&
	
		 imu.Gyro_Raw[1] <= 10  && 
		 imu.Gyro_Raw[1] >= -10 &&
	
		 imu.Gyro_Raw[2] <= 10  && 
		 imu.Gyro_Raw[2] >= -10 )
	{
		imu.motionless++;
		for(i = 0; i<3; i++)
		{
			imu.gyro_sum[i] += imu.Gyro_Deg[i];
		}
		if(imu.motionless == 1000)
		{
			imu.motionless = 0;
			for (i = 0; i < 3; i++) 
			{
				imu.Gyro_Offset[i] -= imu.gyro_sum[i]/1000;
				imu.gyro_sum[i] = 0;
			}
		}
	}
}

/**
 * @brief 加速度计静止校准
 *
 * @param none
 *
 * @return none
 */
void IMU_ACC_Fast_calibrate()
{
	uint16_t i;
	float acc_sum[3];
	LOG("Acc calibrate\r\n");
	for(i = 0; i < 1000; i++)
	{
		IMU_Update();
		acc_sum[0] += imu.Acc_G[0];
		acc_sum[1] += imu.Acc_G[1];
		acc_sum[2] += imu.Acc_G[2];
		HAL_Delay(2);
	}
	imu.Acc_Offset[0] -= acc_sum[0]/1000.0f;
	imu.Acc_Offset[1] -= acc_sum[1]/1000.0f;
	imu.Acc_Offset[2] -= acc_sum[2]/1000.0f;
	imu.Acc_Offset[2] += 1;

	program_data.set.Acc_Offset[0] = imu.Acc_Offset[0];
	program_data.set.Acc_Offset[1] = imu.Acc_Offset[1];
	program_data.set.Acc_Offset[2] = imu.Acc_Offset[2];
	Program_Data_Modify();
	imu.acc_calibrate = 0;
}

/**
 * @brief 加速度计六面校准
 *
 * @param none
 *
 * @return none
 */
void IMU_ACC_calibrate()
{

}


