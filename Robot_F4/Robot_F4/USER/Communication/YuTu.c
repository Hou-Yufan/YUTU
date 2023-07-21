#include "YuTu.h"
#include "ano_dt.h"
#include "usart.h"
#include "stdio.h"
#include "MahonyAHRS.h"
#include "IMU_Dev.h"
#include "usbd_cdc_if.h"
#include "ProgramData.h"
#include "PID.h"
#include "INA226.h"
#include "Displayer.h"
#include "Motor.h"
#include "VL53L0.h"
#include "ADS1115.h"
#include "DYPA02.h"
#include "GP2Y1014AU0F.h"
#include "SHT20.h"
#include "SGP30.h"
#include "Chassis.h"
#include "RC.h"
#include "RingBuffer.h"
#include "CAN_HCSR04.h"
#include "CAN_ADC.h"
#include "Displayer.h"
#include "AutoCharge.h"
#include "Power.h"

static uint8_t rx_buf[YuTu_BUF_SIZE];
static uint8_t send_buf[YuTu_BUF_SIZE];
uint32_t yutu_last;
uint32_t uart3_rx_last;

//初始化
void YuTu_DT_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, YuTu_BUF_SIZE);
	yutu_last = HAL_GetTick();
	uart3_rx_last = HAL_GetTick();
}

//发送结构体
uint8_t YuTu_DT_Send_Struct(uint8_t *Struct_Data,uint32_t length, uint8_t function)
{
	static uint8_t data_to_send[YuTu_BUF_SIZE];
	uint8_t _cnt=0;
	uint8_t sum = 0;
  uint8_t i=0;

	//memset(data_to_send_buf,0,100);
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=function;
	data_to_send[3]=length;
  memcpy(&data_to_send[4], Struct_Data,length);
  _cnt=length+4;
	for(;i<4;i++)
		sum += data_to_send[i];
	for(i=0;i<length;i++)
	sum += Struct_Data[i];
	data_to_send[_cnt++]=sum;
	if(huart3.gState == HAL_UART_STATE_BUSY_TX){
		return 0;
	}
	memcpy(send_buf,data_to_send,_cnt);
	HAL_UART_Transmit_DMA(&huart3, send_buf, _cnt);
	return _cnt;
}

//定时上报
#define DT_FREQ(x)	(cnt%(1000/x) == 0)
void YuTu_DT_Data_Exchange(void)
{
	static uint16_t cnt = 0;
	//数据上报
	static uint8_t send_senser = 0;
	static uint8_t send_ultrasonic = 0;
	static uint8_t send_Rangefinder = 0;
	static uint8_t send_charge_ir = 0;
	static uint8_t send_air_sensor = 0;
	static uint8_t send_vl53l0 = 0;
	static uint8_t send_bat = 0;
	static uint8_t send_key = 0;
	static uint8_t send_light = 0;
	
	cnt++;

	if(DT_FREQ(100))	// 100Hz
	{
		send_senser = 1;
		send_charge_ir = 1;
		send_Rangefinder = 1;
		
		if(DT_FREQ(10))	// 10Hz
		{
			send_ultrasonic = 1;
			send_vl53l0 = 1;
			send_bat = 1;
			send_key = 1;
			if(DT_FREQ(1))	// 1Hz
			{
				send_air_sensor = 1;
				send_light = 1;
			}
		}
	}
	
// imu、轮子数据
	if(send_senser)		
	{
		sensor_data_all_t sensor;
		sensor.Acc[0] = imu.Acc_LPF_G[0];
		sensor.Acc[1] = imu.Acc_LPF_G[1];
		sensor.Acc[2] = imu.Acc_LPF_G[2];
		sensor.Gyro[0] = imu.Gyro_LPF_Deg[0];
		sensor.Gyro[1] = imu.Gyro_LPF_Deg[1];
		sensor.Gyro[2] = imu.Gyro_LPF_Deg[2];
		sensor.angle[0] = ahrs_roll;
		sensor.angle[1] = ahrs_pitch;
		sensor.angle[2] = ahrs_yaw;
		sensor.wheel_circle[0] = ChassisMotor_L.circle;
		sensor.wheel_circle[1] = ChassisMotor_R.circle;
		sensor.wheel_rpm[0] = ChassisMotor_L.rpm;
		sensor.wheel_rpm[1] = ChassisMotor_R.rpm;
		
		if(YuTu_DT_Send_Struct((uint8_t *)&sensor, sizeof(sensor), 0x0A))	//如果没发送成功，下次再发送
			send_senser = 0;
	}
// 红外数据
	if(send_charge_ir)
	{
		if(YuTu_DT_Send_Struct((uint8_t *)charge_ir.adc, 6, 0x0B))	//如果没发送成功，下次再发送
			send_charge_ir = 0;
	}
// 悬崖检测数据
	if(send_Rangefinder)
	{
		if(YuTu_DT_Send_Struct(Rangefinder_hub.Distence, 8, 0x0C))	//如果没发送成功，下次再发送
			send_Rangefinder = 0;
	}
// 超声波数据
	if(send_ultrasonic)		
	{
		if(YuTu_DT_Send_Struct((uint8_t *)hcsr_hub.Distence, 16, 0x0D))	//如果没发送成功，下次再发送
			send_ultrasonic = 0;
	}
// 空气检测数据
	if(send_air_sensor)		
	{
		air_sensor_s air;
		air.pm25 = PM2_5.data * 100;
		air.TVOC = sgp.SGP30_TVOC;
		air.CO2 = sgp.SGP30_CO2;
		air.RH = SHT20.RH * 100;
		air.Temp = SHT20.Tem * 100;
		if(YuTu_DT_Send_Struct((uint8_t *)&air, 10, 0x0E))	//如果没发送成功，下次再发送
			send_air_sensor = 0;
	}
// 激光测距数据
	if(send_vl53l0)		
	{
		uint16_t d[2];
		d[0] = VL53L0_Dev[0].distance;
		d[1] = VL53L0_Dev[1].distance;
		if(YuTu_DT_Send_Struct((uint8_t *)d, 4, 0x0F))	//如果没发送成功，下次再发送
			send_vl53l0 = 0;
	}
// 电池数据
	if(send_bat)		
	{
		batt_s batt;
		batt.vbat = Power_management.vbat;
		batt.current = Power_management.current;
		batt.bat = Power_management.bat;
		batt.mah = Power_management.mah;
		batt.max_mah = Power_management.max_mah;
		if(YuTu_DT_Send_Struct((uint8_t *)&batt, sizeof(batt_s), 0x10))	//如果没发送成功，下次再发送
			send_bat = 0;
	}
// 按键数据
	if(send_key)		
	{
		uint8_t key[4];
		key[0] = Displayer.Regs[1] & 0x01;		// 按键状态 bit0为1代表右按键短按，bit2为1代表左按键短按
		key[1] = (Displayer.Regs[1] >> 2) & 0x01;
		key[2] = Touch_Key;
		key[3] = Power_management.power_state;
		if(YuTu_DT_Send_Struct(key, 4, 0x11))	//如果没发送成功，下次再发送
			send_key = 0;
	}
// 光照强度数据
	if(send_light)		
	{
		uint8_t send_light;
		send_light = Displayer.Regs[0];
		if(YuTu_DT_Send_Struct(&send_light, 1, 0x11))	//如果没发送成功，下次再发送
			send_light = 0;
	}
	
	//串口接收中断未开启检测
	if(huart3.RxState == HAL_UART_STATE_READY && \
		 huart3.ReceptionType == HAL_UART_RECEPTION_STANDARD )
	{
		while(HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, YuTu_BUF_SIZE) != HAL_OK)
		{
			huart3.RxState = HAL_UART_STATE_READY;
			__HAL_UNLOCK(&huart3);
		}
	}
}

void YuTu_DT_Data_Receive(uint8_t *data_buf,uint16_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	yutu_last = HAL_GetTick();
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)//加速度校准
			imu.acc_calibrate = 1;
		if(*(data_buf+4)==0X02)//陀螺仪校准
		{
			imu.gyro_calibrate=1;
		}
		if(*(data_buf+4)==0X03)
		{
			imu.acc_calibrate = 1;	
			imu.gyro_calibrate=1;	
		}
	}
	else if(*(data_buf+2)==0X02)
	{

	}
	else if(*(data_buf+2)==0X03)	//上位机遥控器
	{
		YuTu_RC_s yutu_rc;
		memcpy(&yutu_rc,data_buf+4,sizeof(yutu_rc));
		rc.Mode = yutu_rc.Mode;
		rc.X = yutu_rc.X;
		rc.Y = yutu_rc.Y;
		rc.Type = RC_TYPE_YUTU;
		rc.last_time = HAL_GetTick();
	}
}

void YuTu_DT_Data_Receive_Prepare(uint8_t data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		YuTu_DT_Data_Receive(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

void YuTu_RX_IRQ(void *hw, uint16_t size)
{
	uint16_t i;
	
	if(hw != &huart3)
		return;
	uart3_rx_last = HAL_GetTick();
	for(i=0; i< size; i++)
	{
		YuTu_DT_Data_Receive_Prepare(rx_buf[i]);
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, YuTu_BUF_SIZE);
}




