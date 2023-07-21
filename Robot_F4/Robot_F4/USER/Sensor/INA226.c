#include "INA226.h"
#include "Soft_I2C.h"
#include "i2c.h"

INA_Drv INA_BAT;
INA_Drv INA_CHARGE;

uint16_t INA226_Read2Byte(INA_Drv *drv, uint8_t reg_addr)
{
	uint8_t data[2] = {0,0};
	if (HAL_I2C_Mem_Read(&hi2c2, drv->chip_addr, reg_addr, 1, data, 2, 0x10) == HAL_OK)
	{
		if (drv->error_cnt>0)
			drv->error_cnt--;
	}
	else
	{
		if (drv->error_cnt<255)
			drv->error_cnt++;
	}
//	Soft_I2C_Read_Regs(&swi2c2, drv->chip_addr, reg_addr, 2, data);
	return (uint16_t)((data[0]<<8) | data[1]);
}

uint8_t INA226_Write2Byte(INA_Drv *drv, uint8_t reg_addr,uint16_t reg_data)
{        
	uint8_t data[2];
	data[0] =(uint8_t)((reg_data&0xFF00)>>8);		//存放高八位数据
	data[1] =(uint8_t)reg_data&0x00FF;					//存放低八位数据
	return HAL_I2C_Mem_Write(&hi2c2, drv->chip_addr, reg_addr, 1, data, 2, 0x10);
//	return Soft_I2C_Send_Regs(&swi2c2, drv->chip_addr, reg_addr, 2, data);
}

uint8_t INA226_Init(INA_Drv *drv, uint8_t chip_addr)
{
	drv->chip_addr = chip_addr;
	uint16_t ID = INA226_Read2Byte(drv, ID_Reg);
	uint16_t Man_ID = INA226_Read2Byte(drv, Man_ID_Reg);
	INA226_Write2Byte(drv, Config_Reg, 0x4527);	//0100_010_100_100_111 //16次平均,1.1ms,1.1ms,连续测量分流电压和总线电压
	INA226_Write2Byte(drv, Calib_Reg, 0xa00);	// 分辨率0.2mA 采样电阻0.01r	CAL = 0.00512/(0.2*0.01)*1000 = 2560= 0xa00
	drv->ready = 1;
	return 0;
}


void INA226_Read(INA_Drv *drv)
{
	if(drv->ready == 0)
		return;
	
	uint16_t Bus_V = INA226_Read2Byte(drv, Bus_V_Reg);
//	uint16_t Shunt_V = INA226_Read2Byte(drv, Shunt_V_Reg);
	uint16_t Current = INA226_Read2Byte(drv, Current_Reg);
	uint16_t Power = INA226_Read2Byte(drv, Power_Reg);
//	INA226_Init(drv, drv->chip_addr);
	drv->Bus_V = Bus_V*1.25f;
//	drv->Shunt_V = Shunt_V*2.5f*0.001f;
	drv->Current = Current*0.2f;
	drv->Power = Power*0.02f*25;
}



