#include "ano_dt.h"
#include "usart.h"
#include "stdio.h"
#include "MahonyAHRS.h"
#include "IMU_Dev.h"
#include "usbd_cdc_if.h"
#include "ProgramData.h"
#include "PID.h"
#include "Power.h"
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
#include "INA226.h"
#include "YuTu.h"

#define PID_SCALE		100.0f
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTEM0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTEM1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTEM2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTEM3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
#define BYTEM4(dwTemp)       ( *( (char *)(&dwTemp)	+ 4) )
#define BYTEM5(dwTemp)       ( *( (char *)(&dwTemp) + 5) )
#define BYTEM6(dwTemp)       ( *( (char *)(&dwTemp) + 6) )
#define BYTEM7(dwTemp)       ( *( (char *)(&dwTemp) + 7) )
	
RingBuffer ano_tx_fifo[ANO_PORT_NUM];
uint8_t ano_tx_buf[ANO_PORT_NUM][ANO_FIFO_SIZE];

ano_port_s ano_port[ANO_PORT_NUM] = {
	{
		.hw = &USBD_CDC,
		.HW_Type = ANO_CDC,
	},
	
	{
		.hw = &huart1,
		.HW_Type = ANO_UART,
	},
};

dt_flag_t f;							//需要发送数据的标志
uint8_t data_to_send[260];	//发送数据缓存

void ANO_Init(void)
{
	uint8_t i = 0;
	for(; i<ANO_PORT_NUM; i++)
	{
		ring_buffer_creat(&ano_tx_fifo[i], ano_tx_buf[i], ANO_FIFO_SIZE);
		if(ano_port[i].HW_Type == ANO_UART)
		{
			if(((UART_HandleTypeDef *)(ano_port[i].hw))->hdmarx)
				HAL_UARTEx_ReceiveToIdle_DMA(ano_port[i].hw, ano_port[i].rx_buf, ANO_BUF_SIZE);
			else
				HAL_UARTEx_ReceiveToIdle_IT(ano_port[i].hw, ano_port[i].rx_buf, ANO_BUF_SIZE);
			if(((UART_HandleTypeDef *)(ano_port[i].hw))->hdmatx)
				ano_port[i].TX_FIFO = &ano_tx_fifo[i];
		}
		else if(ano_port[i].HW_Type == ANO_CDC)
		{
			ano_port[i].TX_FIFO = &ano_tx_fifo[i];
		}
	}
}

void ANO_TX_Task(void)
{
	uint8_t i;
	uint16_t len;
	for(i=0; i<ANO_PORT_NUM; i++)
	{
		if(ano_port[i].HW_Type == ANO_UART)
		{
			//串口接收中断未开启检测
			if(((UART_HandleTypeDef *)(ano_port[i].hw))->RxState == HAL_UART_STATE_READY && \
				 ((UART_HandleTypeDef *)(ano_port[i].hw))->ReceptionType == HAL_UART_RECEPTION_STANDARD )
			{
				while(HAL_UARTEx_ReceiveToIdle_DMA(ano_port[i].hw, ano_port[i].rx_buf, ANO_BUF_SIZE) != HAL_OK)
				{
					((UART_HandleTypeDef *)ano_port[i].hw)->RxState = HAL_UART_STATE_READY;
					__HAL_UNLOCK((UART_HandleTypeDef *)ano_port[i].hw);
				}
			}
			
			//串口发送
			if((ano_port[i].hw && \
				(((UART_HandleTypeDef *)(ano_port[i].hw))->gState & HAL_UART_STATE_BUSY_TX) != HAL_UART_STATE_BUSY_TX) && \
				 (ano_port[i].TX_FIFO && ((RingBuffer *)ano_port[i].TX_FIFO)->data_len))
			{
				len = ring_buffer_read(ano_port[i].TX_FIFO, ano_port[i].tx_buf, ANO_BUF_SIZE);
				if(((UART_HandleTypeDef *)(ano_port[i].hw))->hdmatx)
					HAL_UART_Transmit_DMA(ano_port[i].hw, ano_port[i].tx_buf, len);
				else
					HAL_UART_Transmit_IT(ano_port[i].hw, ano_port[i].tx_buf, len);
			}
		}
		if(ano_port[i].HW_Type == ANO_CDC)
		{
			if(ano_port[i].TX_FIFO && ((RingBuffer *)ano_port[i].TX_FIFO)->data_len)
			{
				len = ring_buffer_read(ano_port[i].TX_FIFO, ano_port[i].tx_buf, ANO_BUF_SIZE);
//				CDC_Transmit_FS(ano_port[i].tx_buf, len);
			}
		}
	}
}

void ANO_RX_IRQ(void *hw, uint16_t size)
{
	uint16_t i,port;
	for(port=0; port<ANO_PORT_NUM; port++)
	{
		if(hw == ano_port[port].hw)
			break;
	}
	if(port == ANO_PORT_NUM)
		return;
	for(i=0; i< size; i++)
	{
		ANO_DT_Data_Receive_Prepare(ano_port[port].rx_buf[i]);
	}
	if(ano_port[port].HW_Type == ANO_UART)	//重新打开串口接收
	{
		i = 0;
		if(((UART_HandleTypeDef *)(ano_port[port].hw))->hdmarx)
			HAL_UARTEx_ReceiveToIdle_DMA(ano_port[port].hw, ano_port[port].rx_buf, ANO_BUF_SIZE);
		else
			HAL_UARTEx_ReceiveToIdle_IT(ano_port[port].hw, ano_port[port].rx_buf, ANO_BUF_SIZE);
	}
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
#define DT_FREQ(x)	(1000/x)
void ANO_DT_Data_Exchange(void)
{
	static uint8_t cnt = 0;
	const uint16_t senser_cnt 	= DT_FREQ(100);
	const uint16_t ultrasonic_cnt 	= DT_FREQ(10);
	const uint16_t status_cnt 	= DT_FREQ(100);
	const uint16_t rcdata_cnt 	= DT_FREQ(10);
	const uint16_t motopwm_cnt	= DT_FREQ(100);
	const uint16_t power_cnt	=	DT_FREQ(10);
	const uint16_t air_cnt		=	DT_FREQ(1);

	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;		
	
	if((cnt % air_cnt) == (air_cnt-1))
		f.send_air_sensor = 1;	
	
	if((cnt % ultrasonic_cnt) == (ultrasonic_cnt-1))
		f.send_ultrasonic = 1;		
	cnt++;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,100,100,434,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_status)
	{
		f.send_status = 0;
		ANO_DT_Send_Status(ahrs_roll, ahrs_pitch, ahrs_yaw,
							0, rc.Mode, rc.Mode>0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_senser)
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser(	imu.Acc_LPF_G[0] * 980.0f,		imu.Acc_LPF_G[1] * 980.0f,		imu.Acc_LPF_G[2] * 980.0f,
							imu.Gyro_LPF_Deg[0] * 10.0f,	imu.Gyro_LPF_Deg[1] * 10.0f,	imu.Gyro_LPF_Deg[2] * 10.0f,
							0,	0,	0,
							0);
		
		ANO_DT_Send_IRCharger(charge_ir.adc[0], charge_ir.adc[1], charge_ir.adc[2], AutoCharge_Mode);
//		ANO_DT_Send_IRadc(IR_hub.adc[0], IR_hub.adc[1], IR_hub.adc[2]);
		ANO_DT_Send_IRdis(Rangefinder_hub.Distence, Touch_Key);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_ultrasonic)
	{
		f.send_ultrasonic = 0;
		ANO_DT_Send_Ultrasonic(hcsr_hub.Distence[0],// 前面
							 hcsr_hub.Distence[1],	// 左前
							 hcsr_hub.Distence[2],	// 无
							 hcsr_hub.Distence[3],	// 无
							 hcsr_hub.Distence[4],	// 右后
							 hcsr_hub.Distence[5],	// 后超声波
							 hcsr_hub.Distence[6],	// 左后
							 hcsr_hub.Distence[7]); // 右前

		ANO_DT_Send_USER(4, Displayer.Regs, 5);
		ANO_DT_Send_Laser(VL53L0_Dev[0].distance, VL53L0_Dev[1].distance);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_air_sensor)
	{
		f.send_air_sensor = 0;
		ANO_DT_Send_Air(PM2_5.data * 100, sgp.SGP30_TVOC, sgp.SGP30_CO2, SHT20.RH * 100, SHT20.Tem * 100);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		ANO_DT_Send_RCData(	rc.X*5 + 1500 , 					//thr  >>> 左轮
							rc.Y*5 + 1500 ,						//yaw  >>> 右轮
							0,												//roll >>> 
							0,												//pit  >>> 
							rc.Mode*200 + 1000,				//aux1 >>> 控制模式
							rc.last_time - uart3_rx_last,		//aux2 >>>
							rc.last_time - yutu_last,		//aux3 >>>
							0,		//aux4 >>> 
							0,		//aux5 >>> 
							0);		//aux6 >>>
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(//左电机
							ChassisMotor_L.pwm_set, 			//MOTOR1>>>PWM输出
							ChassisMotor_L.circle*100.0f,	//MOTOR2>>>转轴转速
							chassis_speed[0]*100.0f,			//MOTOR3>>>期望转速		
							ChassisMotor_L.current,				//MOTOR4>>>电机电流
							//右边电机
							ChassisMotor_R.pwm_set, 			//MOTOR5>>>PWM输出
							ChassisMotor_R.circle*100.0f,	//MOTOR6>>>转轴转速
							chassis_speed[1]*100.0f,			//MOTOR7>>>期望转速		
							ChassisMotor_R.current);			//MOTOR8>>>电机电流
		ANO_DT_Send_Speed(ChassisMotor_R.rpm*100.0f, ChassisMotor_L.rpm*100.0f, 100.0f*Power_management.bat);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power(Power_management.vbat*100.0f, Power_management.current*100.0f);
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,0,0,0,	//1
											0,0,0,	//2
											0,0,0);	//3
	}	
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid2)
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,0,0,0,	//4
											0,0,0,	//5
											0,0,0);	//6
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid3)
	{
		f.send_pid3 = 0;
		ANO_DT_Send_PID(3,0,0,0,	//7
											0,0,0,	//8
											0,0,0);	//9
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid4)
	{
		f.send_pid4= 0;
		ANO_DT_Send_PID(4,0,0,0,	//10
											program_data.set.pid_kp[PID_MOTOR_L], program_data.set.pid_ki[PID_MOTOR_L],program_data.set.pid_kd[PID_MOTOR_L],		//PID11
											program_data.set.pid_kp[PID_MOTOR_R], program_data.set.pid_ki[PID_MOTOR_R],program_data.set.pid_kd[PID_MOTOR_R]);		//PID12
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid5)
	{
		f.send_pid5= 0;
		ANO_DT_Send_PID(5,program_data.set.pid_kp[PID_REVOLVE], program_data.set.pid_ki[PID_REVOLVE],program_data.set.pid_kd[PID_REVOLVE],		//PID13
											0,0,0,		//PID14
											0,0,0);		//PID15
	}
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid6)
	{
		f.send_pid6= 0;
		ANO_DT_Send_PID(6,0,0,0,
											0,0,0,
											0,0,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
}


/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
	uint8_t i = 0;
	for(; i<ANO_PORT_NUM; i++)
	{
		if(ano_port[i].TX_FIFO != NULL)		//使用FIFO
		{
			//采用轮询fifo的形式发送
			ring_buffer_write(ano_port[i].TX_FIFO, dataToSend, length);
		}
		else	//直接发送使用DMA或者中断会导致丢数据
		{
			if(length > ANO_BUF_SIZE)
				length = ANO_BUF_SIZE;
			
			if(ano_port[i].HW_Type == ANO_UART)
			{
				HAL_UART_Transmit(ano_port[i].hw, dataToSend, length, 0xFF);
			}
			else if(ano_port[i].HW_Type == ANO_CDC)
			{
				CDC_Transmit_FS(dataToSend, length);
			}
		}
	}
}

static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
	sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(uint8_t data)
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
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
//		usart_printf("cmd=%d",*(data_buf+4));
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
//			if(*(data_buf+4)==0X05)//气压校准
//		if(*(data_buf+4)==0X20)//退出六面校准
	}
	else if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
			Program_Data_Reset();
		}
	}
	else if(*(data_buf+2)==0X03)	//上位机遥控器
	{
		rc.Type = RC_TYPE_ANO;
		rc.X = (( (int16_t)(*(data_buf+4)<<8)|*(data_buf+5) ) - 1500)/5.0f;
		rc.Y = (( (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) ) - 1500)/5.0f;
		rc.Mode = (( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) ) - 1000)/200;
		rc.last_time = HAL_GetTick();
	}
	else if(*(data_buf+2)==0X10)								//PID1
	{
		Displayer.Regs[3] = ( (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
		Displayer.Regs[4] = ( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
		ANO_DT_Send_Check(*(data_buf+2),sum);
//		Program_Data_Modify();
	}
	else if(*(data_buf+2)==0X11)								//PID2
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);	  
//		Program_Data_Modify();
	}
	else if(*(data_buf+2)==0X12)								//PID3
	{
			ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	else if(*(data_buf+2)==0X13)								//PID4
	{	
//		pid_motor_L.Kp = program_data.set.pid_kp[PID_MOTOR_L] = ( (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) )/PID_SCALE;
//		pid_motor_L.Ki = program_data.set.pid_kp[PID_MOTOR_L] = ( (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) )/PID_SCALE;
//		pid_motor_L.Kd = program_data.set.pid_kp[PID_MOTOR_L]= ( (int16_t)(*(data_buf+14)<<8)|*(data_buf+15) )/PID_SCALE;
//		PID_resize(&pid_motor_L);
//		
//		pid_motor_R.Kp = program_data.set.pid_kp[PID_MOTOR_R] 	= ( (int16_t)(*(data_buf+16)<<8)|*(data_buf+17) )/PID_SCALE;
//		pid_motor_R.Ki = program_data.set.pid_kp[PID_MOTOR_R] 	= ( (int16_t)(*(data_buf+18)<<8)|*(data_buf+19) )/PID_SCALE;
//		pid_motor_R.Kd = program_data.set.pid_kp[PID_MOTOR_R] 	= ( (int16_t)(*(data_buf+20)<<8)|*(data_buf+21) )/PID_SCALE;
//		PID_resize(&pid_motor_R);
//		
		ANO_DT_Send_Check(*(data_buf+2),sum);
//		Program_Data_Modify();
	}
	else if(*(data_buf+2)==0X14)								//PID5
	{
//		pid_yaw_speed.Kp = program_data.set.pid_kp[PID_REVOLVE]  = ( (int16_t)(*(data_buf+4)<<8)|*(data_buf+5) )/PID_SCALE;
//		pid_yaw_speed.Ki = program_data.set.pid_kp[PID_REVOLVE]  = ( (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) )/PID_SCALE;
//		pid_yaw_speed.Kd = program_data.set.pid_kp[PID_REVOLVE]  = ( (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) )/PID_SCALE;
//		PID_resize(&pid_yaw_speed);
		
		ANO_DT_Send_Check(*(data_buf+2),sum);
//		Program_Data_Modify();
	}
	else if(*(data_buf+2)==0X15)								//PID6
	{

		ANO_DT_Send_Check(*(data_buf+2),sum);
//		Program_Data_Modify();
	}
}

void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTEM1(hardware_ver);
	data_to_send[_cnt++]=BYTEM0(hardware_ver);
	data_to_send[_cnt++]=BYTEM1(software_ver);
	data_to_send[_cnt++]=BYTEM0(software_ver);
	data_to_send[_cnt++]=BYTEM1(protocol_ver);
	data_to_send[_cnt++]=BYTEM0(protocol_ver);
	data_to_send[_cnt++]=BYTEM1(bootloader_ver);
	data_to_send[_cnt++]=BYTEM0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	int16_t _temp;
	int32_t _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	
	data_to_send[_cnt++]=BYTEM3(_temp2);
	data_to_send[_cnt++]=BYTEM2(_temp2);
	data_to_send[_cnt++]=BYTEM1(_temp2);
	data_to_send[_cnt++]=BYTEM0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar)
{
	uint8_t _cnt=0;
	int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTEM1(thr);
	data_to_send[_cnt++]=BYTEM0(thr);
	data_to_send[_cnt++]=BYTEM1(yaw);
	data_to_send[_cnt++]=BYTEM0(yaw);
	data_to_send[_cnt++]=BYTEM1(rol);
	data_to_send[_cnt++]=BYTEM0(rol);
	data_to_send[_cnt++]=BYTEM1(pit);
	data_to_send[_cnt++]=BYTEM0(pit);
	data_to_send[_cnt++]=BYTEM1(aux1);
	data_to_send[_cnt++]=BYTEM0(aux1);
	data_to_send[_cnt++]=BYTEM1(aux2);
	data_to_send[_cnt++]=BYTEM0(aux2);
	data_to_send[_cnt++]=BYTEM1(aux3);
	data_to_send[_cnt++]=BYTEM0(aux3);
	data_to_send[_cnt++]=BYTEM1(aux4);
	data_to_send[_cnt++]=BYTEM0(aux4);
	data_to_send[_cnt++]=BYTEM1(aux5);
	data_to_send[_cnt++]=BYTEM0(aux5);
	data_to_send[_cnt++]=BYTEM1(aux6);
	data_to_send[_cnt++]=BYTEM0(aux6);

	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(uint16_t votage, int16_t current)
{
	uint8_t _cnt=0;
	uint16_t temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTEM1(temp);
	data_to_send[_cnt++]=BYTEM0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTEM1(temp);
	data_to_send[_cnt++]=BYTEM0(temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(int16_t m_1,int16_t m_2,int16_t m_3,int16_t m_4,int16_t m_5,int16_t m_6,int16_t m_7,int16_t m_8)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTEM1(m_1);
	data_to_send[_cnt++]=BYTEM0(m_1);
	data_to_send[_cnt++]=BYTEM1(m_2);
	data_to_send[_cnt++]=BYTEM0(m_2);
	data_to_send[_cnt++]=BYTEM1(m_3);
	data_to_send[_cnt++]=BYTEM0(m_3);
	data_to_send[_cnt++]=BYTEM1(m_4);
	data_to_send[_cnt++]=BYTEM0(m_4);
	data_to_send[_cnt++]=BYTEM1(m_5);
	data_to_send[_cnt++]=BYTEM0(m_5);
	data_to_send[_cnt++]=BYTEM1(m_6);
	data_to_send[_cnt++]=BYTEM0(m_6);
	data_to_send[_cnt++]=BYTEM1(m_7);
	data_to_send[_cnt++]=BYTEM0(m_7);
	data_to_send[_cnt++]=BYTEM1(m_8);
	data_to_send[_cnt++]=BYTEM0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Speed(int16_t x, int16_t y, int16_t z)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTEM1(x);
	data_to_send[_cnt++]=BYTEM0(x);
	data_to_send[_cnt++]=BYTEM1(y);
	data_to_send[_cnt++]=BYTEM0(y);
	data_to_send[_cnt++]=BYTEM1(z);
	data_to_send[_cnt++]=BYTEM0(z);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0;
	int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p1_i  * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p1_d  * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p2_p  * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p2_i  * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p2_d * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p3_p  * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p3_i  * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	_temp = p3_d * PID_SCALE;
	data_to_send[_cnt++]=BYTEM1(_temp);
	data_to_send[_cnt++]=BYTEM0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_USER(uint8_t group, uint8_t *data, uint8_t len)
{
	uint8_t _cnt=0;
	uint8_t i;
	uint8_t sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1+group;
	data_to_send[_cnt++]=len;
  memcpy(&data_to_send[4], data,len);
  _cnt=len+4;
	for(;i<4;i++)
		sum += data_to_send[i];
	for(i=0;i<len;i++)
		sum += data[i];
//	sum = data_to_send[0] + data_to_send[1] + data_to_send[2] + data_to_send[3];
//	for(i=0; i<len; i++)
//	{
//		data_to_send[_cnt++] = data[i];
//		sum += data[i];
//	}
	data_to_send[_cnt++]=sum;
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Ultrasonic(uint16_t d1, uint16_t d2, uint16_t d3, uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7, uint16_t d8)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;	//用户数据1
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTEM1(d1);
	data_to_send[_cnt++]=BYTEM0(d1);
	data_to_send[_cnt++]=BYTEM1(d2);
	data_to_send[_cnt++]=BYTEM0(d2);
	data_to_send[_cnt++]=BYTEM1(d3);
	data_to_send[_cnt++]=BYTEM0(d3);
	data_to_send[_cnt++]=BYTEM1(d4);
	data_to_send[_cnt++]=BYTEM0(d4);
	data_to_send[_cnt++]=BYTEM1(d5);
	data_to_send[_cnt++]=BYTEM0(d5);
	data_to_send[_cnt++]=BYTEM1(d6);
	data_to_send[_cnt++]=BYTEM0(d6);
	data_to_send[_cnt++]=BYTEM1(d7);
	data_to_send[_cnt++]=BYTEM0(d7);
	data_to_send[_cnt++]=BYTEM1(d8);
	data_to_send[_cnt++]=BYTEM0(d8);
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_IRCharger(uint16_t a1, uint16_t a2, uint16_t a3, uint8_t mode)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF2;	//用户数据2
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTEM1(a1);
	data_to_send[_cnt++]=BYTEM0(a1);
	
	data_to_send[_cnt++]=BYTEM1(a2);
	data_to_send[_cnt++]=BYTEM0(a2);
	
	data_to_send[_cnt++]=BYTEM1(a3);
	data_to_send[_cnt++]=BYTEM0(a3);
	
	data_to_send[_cnt++]=mode;
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_IRdis(uint8_t *dis, uint8_t key)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF3;	//用户数据3
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=dis[0];
	data_to_send[_cnt++]=dis[1];
	data_to_send[_cnt++]=dis[2];
	data_to_send[_cnt++]=dis[3];
	data_to_send[_cnt++]=dis[4];
	data_to_send[_cnt++]=key;

	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Air(uint16_t pm, uint16_t TVOC, uint16_t CO2, uint16_t RH, uint16_t Temp)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF4;	//用户数据4
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTEM1(pm);
	data_to_send[_cnt++]=BYTEM0(pm);
	
	data_to_send[_cnt++]=BYTEM1(TVOC);
	data_to_send[_cnt++]=BYTEM0(TVOC);
	
	data_to_send[_cnt++]=BYTEM1(CO2);
	data_to_send[_cnt++]=BYTEM0(CO2);
	
	data_to_send[_cnt++]=BYTEM1(RH);
	data_to_send[_cnt++]=BYTEM0(RH);
	
	data_to_send[_cnt++]=BYTEM1(Temp);
	data_to_send[_cnt++]=BYTEM0(Temp);

	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Laser(uint16_t d1, uint16_t d2)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF6;	//用户数据6
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTEM1(d1);
	data_to_send[_cnt++]=BYTEM0(d1);
	
	data_to_send[_cnt++]=BYTEM1(d2);
	data_to_send[_cnt++]=BYTEM0(d2);

	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
