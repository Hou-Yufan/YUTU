#ifndef INA226_H
#define INA226_H

#include "main.h"

//A1=GND,A2=GND // R=1, W=0 //0x80
#define INA_ADDR_0                 0x80	//0x81

//A1=VCC,A2=VCC // R=1, W=0 //0x8A
#define INA_ADDR_3                 0x8A		//0x80

#define Config_Reg                 0x00
#define Shunt_V_Reg                0x01		//分流电压， 此处分流电阻为 0.1欧
#define Bus_V_Reg                  0x02		//总线电压
#define Power_Reg                  0x03		//电源功率
#define Current_Reg                0x04		//电流
#define Calib_Reg                  0x05		//校准，设定满量程范围以及电流和功率测数的 
#define Mask_En_Reg                0x06		//屏蔽 使能 警报配置和转换准备就绪
#define Alert_Reg                  0x07		//包含与所选警报功能相比较的限定值
#define Man_ID_Reg                 0xFE  	//0x5449
#define ID_Reg                     0xFF  	//包含唯一的芯片标识号

typedef struct
{
	uint8_t chip_addr;
	uint8_t ready;
	float Bus_V;
	float Shunt_V;
	float Current;
	float Power;
	uint8_t error_cnt;
	uint8_t update_cnt;
}INA_Drv;

extern INA_Drv INA_BAT;
extern INA_Drv INA_CHARGE;

uint8_t INA226_Init(INA_Drv *drv, uint8_t chip_addr);
void INA226_Read(INA_Drv *drv);

#endif



