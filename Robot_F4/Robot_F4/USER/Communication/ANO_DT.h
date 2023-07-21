#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "main.h"
#define ANO_PORT_CDC	0
#define ANO_PORT_NUM	2
#define ANO_BUF_SIZE	128
#define ANO_FIFO_SIZE	(ANO_BUF_SIZE*8)

#define ANO_UART			0
#define ANO_CDC				1
typedef struct 
{
		uint8_t send_version;
		uint8_t send_status;
		uint8_t send_senser;
		uint8_t send_air_sensor;
		uint8_t send_ultrasonic;
		uint8_t send_pid1;
		uint8_t send_pid2;
		uint8_t send_pid3;
		uint8_t send_pid4;
		uint8_t send_pid5;
		uint8_t send_pid6;
		uint8_t send_rcdata;
		uint8_t send_offset;
		uint8_t send_motopwm;
		uint8_t send_power;
}dt_flag_t;

typedef struct _ano_port{
	void *hw;
	uint8_t tx_buf[ANO_BUF_SIZE];
	uint8_t rx_buf[ANO_BUF_SIZE];
	void *TX_FIFO;
	void *RX_FIFO;
	uint8_t HW_Type;
}ano_port_s;

extern ano_port_s ano_port[ANO_PORT_NUM];
extern dt_flag_t f;

void ANO_Init(void);
void ANO_TX_Task(void);
void ANO_TX_IRQ(void *hw);
void ANO_RX_IRQ(void *hw, uint16_t size);
void ANO_DT_Data_Exchange(void);
void ANO_DT_Data_Receive_Prepare(uint8_t data);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar);
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
void ANO_DT_Send_Power(uint16_t votage, int16_t current);
void ANO_DT_Send_MotoPWM(int16_t m_1,int16_t m_2,int16_t m_3,int16_t m_4,int16_t m_5,int16_t m_6,int16_t m_7,int16_t m_8);
void ANO_DT_Send_Speed(int16_t x, int16_t y, int16_t z);

void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_USER(uint8_t group, uint8_t *data, uint8_t len);
void ANO_DT_Send_Struct(uint8_t *Struct_Data,uint32_t length, uint8_t function);

void ANO_DT_Send_Ultrasonic(uint16_t d1, uint16_t d2, uint16_t d3, uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7, uint16_t d8);	//用户数据1
void ANO_DT_Send_IRCharger(uint16_t a1, uint16_t a2, uint16_t a3, uint8_t mode);	//用户数据2
void ANO_DT_Send_IRdis(uint8_t *dis, uint8_t key);	//用户数据3
void ANO_DT_Send_Air(uint16_t pm, uint16_t TVOC, uint16_t CO2, uint16_t RH, uint16_t Temp);	//用户数据4
void ANO_DT_Send_Laser(uint16_t d1, uint16_t d2);	//用户数据6
#endif

