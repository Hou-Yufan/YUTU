#ifndef __YUTU_H__
#define __YUTU_H__

#include "main.h"

#define YuTu_BUF_SIZE		128

typedef struct 
{
	float angle[3];
	float Acc[3];
	float Gyro[3];
	float wheel_circle[2];
	float wheel_rpm[2];
} sensor_data_all_t;

typedef struct YuTu_RC_s{
	uint8_t Type;
	uint8_t Mode;
	float X;
	float Y;
}YuTu_RC_s;

typedef struct air_sensor_s{
	uint16_t pm25;
	uint16_t TVOC;
	uint16_t CO2;
	uint16_t RH;
	uint16_t Temp;
}air_sensor_s;

typedef struct
{
	float vbat;
	float current;
	float bat;
	double mah;
	double max_mah;
}batt_s;

extern uint32_t yutu_last;
extern uint32_t uart3_rx_last;

void YuTu_DT_Init(void);
uint8_t YuTu_DT_Send_Struct(uint8_t *Struct_Data,uint32_t length, uint8_t function);
void YuTu_DT_Data_Exchange(void);
void YuTu_DT_Data_Receive(uint8_t *data_buf,uint16_t num);
void YuTu_DT_Data_Receive_Prepare(uint8_t data);
void YuTu_RX_IRQ(void *hw, uint16_t size);
#endif
