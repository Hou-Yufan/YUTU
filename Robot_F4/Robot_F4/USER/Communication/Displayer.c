#include "Displayer.h"
#include "VL53L0.h"
#include "can.h"
#include "BSP_CAN.h"
#include "Power.h"

struct Displayer_Data Displayer;


void Disp_RX_Motor_Data(uint8_t *buf)
{
	int16_t angle;
	int16_t speed;
	
	angle = ((uint16_t)buf[0]) << 8 | buf[1];
	speed = ((uint16_t)buf[2]) << 8 | buf[3];
	
	Displayer.angle = angle/100.0f;
	Displayer.speed = speed/100.0f;
	Displayer.out = ((uint16_t)buf[4]) << 8 | buf[5];
	Displayer.control_mode = buf[7];
}

void Disp_RX_Sensor_Data(uint8_t *buf)
{
	Displayer.V = ((uint16_t)buf[0]) << 8 | buf[1];
	Displayer.I = ((uint16_t)buf[2]) << 8 | buf[3];
	VL53L0_CAN_IRQ(buf+4);
}


void Disp_RX_Regs_Data(uint8_t *buf)
{	
	Displayer.Regs[0] = buf[0];		// 光照强度传感器
	Displayer.Regs[1] = buf[1];		// 按键状态 bit0为1代表右按键短按，bit2为1代表左按键短按
	if(HAL_GetTick() < 10000)			// 上电前10秒记录屏幕的默认设置
	{
		Displayer.Regs[2] = buf[2];
		Displayer.Regs[3] = buf[3];
		Displayer.Regs[4] = buf[4];
	}
}

void Disp_Send_AP_Data(CAN_HandleTypeDef *can_dev, uint8_t TX_ADDR, uint8_t RX_ADDR)
{
	uint32_t TxMailbox; 								//发送邮箱
	CAN_TxHeaderTypeDef     tx_message;	//定义发送结构体
	uint8_t tx_buf[6];									//发送缓存
	
	CAN_ExtId extid;
	extid.ExtId.Type = CAN_TYPE_DATA_MARK | 0x0C;
	extid.ExtId.TX_ADDR = TX_ADDR;
	extid.ExtId.RX_ADDR = RX_ADDR;

	tx_message.ExtId = extid.ID;
	tx_message.IDE   = CAN_ID_EXT;
	tx_message.RTR   = CAN_RTR_DATA;
	tx_message.DLC   = 0x06;
	
	tx_buf[0] = Displayer.Regs[0];	// 无意义
	tx_buf[1] = Displayer.Regs[1];	// 无意义

// 按键指示灯状态
/*
【状态】1：红常亮，2：绿常亮，3：蓝常亮4：红快呼吸，5：绿快呼吸，
6：蓝快呼吸7：红慢呼吸，8：绿慢呼吸，9：蓝慢呼吸，10：白快呼吸，
11：白慢呼吸，12：白超慢呼吸，13：红转圈，14：绿转圈，15：蓝转圈
【高4位优先有效，用于按键按下时的状态显示】
（4-5位控制按键灯颜色）
【状态】1:红，2:绿，3:蓝，0:白
（6-7位按键强制亮起）  
1左按键亮，2中间亮，3右侧亮
*/
	tx_buf[2] = Displayer.Regs[2];	
	
// 环形灯光状态 
/*
【状态】1：红常亮，2：绿常亮，3：蓝常亮4：红快呼吸，5：绿快呼吸，
6：蓝快呼吸7：红慢呼吸，8：绿慢呼吸，9：蓝慢呼吸10：白快呼吸，
11：白慢呼吸，12：白超慢呼吸13：红转圈，14：绿转圈，15：蓝转圈	
*/
//	tx_buf[3] = 12;	// 开机默认白色
//	if(0)
//		tx_buf[3] = 3;	// 导航模式
//	if(Power_management.bat < 0.2f)
//		tx_buf[3] = 4;	// 低电量
//	else if(Power_management.current > 200.0f)
//		tx_buf[3] = 2;	// 充电中
//	
//	tx_buf[4] = 80;	// 屏幕背光亮度 0~100，默认80
	
	tx_buf[3] = Displayer.Regs[3];	
	tx_buf[4] = Displayer.Regs[4];	
	
	tx_buf[5] = 0;

	if(HAL_GetTick() > 10000)		// 上电10s后才开始控制屏幕
		HAL_CAN_AddTxMessage(can_dev,&tx_message,tx_buf,&TxMailbox);//发送
}



