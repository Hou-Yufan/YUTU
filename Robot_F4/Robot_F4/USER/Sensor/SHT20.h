#ifndef _SHT20_H_
#define _SHT20_H_

#include "main.h"

#define SHT20_I2C hi2c1

#define SHT20_ADDR_WR				0x80
#define SHT20_ADDR_RD				0x81

#define Trig_T_Addr					0xE3
#define Trig_RH_Addr				0xE5

typedef struct
{
	uint16_t Tem_Value;
	uint16_t RH_Value;
	
	float Tem;
	float RH;
	
	uint8_t error_cnt;
}SHT20_T;

extern SHT20_T SHT20;;

extern void Read_SHT20(void);

#endif


