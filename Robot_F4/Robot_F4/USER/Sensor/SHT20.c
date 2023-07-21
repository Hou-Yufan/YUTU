#include "SHT20.h"
#include "i2c.h"
#include "BSP_UART.h"
/*
*温湿度
*湿度;45%-65%
*/

SHT20_T SHT20;

uint8_t DataBuf_SHT20[4];

void Read_SHT20(void)
{
	if(SHT20.error_cnt > 10)
			return;
	if(HAL_I2C_Mem_Read(&SHT20_I2C, SHT20_ADDR_RD, Trig_T_Addr, I2C_MEMADD_SIZE_8BIT, &DataBuf_SHT20[0], 2, 0x100) == HAL_OK)
	{
		SHT20.Tem_Value = ((uint16_t)DataBuf_SHT20[0] << 8) + (DataBuf_SHT20[1] & 0xfe);
		SHT20.Tem = SHT20.Tem_Value * 175.72f / 65535 - 46.85f;
		if(SHT20.error_cnt > 0)
			SHT20.error_cnt --;
	}
	else
	{
		if(SHT20.error_cnt < 255)
			SHT20.error_cnt ++;
		LOG("HT20 I2C Master Receive TEM fail\r\n");
	}
	if(HAL_I2C_Mem_Read(&SHT20_I2C, SHT20_ADDR_RD, Trig_RH_Addr, I2C_MEMADD_SIZE_8BIT, &DataBuf_SHT20[2], 2, 0x100) == HAL_OK)
	{
		SHT20.RH_Value = ((uint16_t)DataBuf_SHT20[2] << 8) + (DataBuf_SHT20[3] & 0xfc);
		SHT20.RH = SHT20.RH_Value * 125.00f / 65535 - 6.00f;
		if(SHT20.error_cnt > 0)
			SHT20.error_cnt --;
	}
	else
	{
		if(SHT20.error_cnt < 255)
			SHT20.error_cnt ++;
		LOG("HT20 I2C Master Receive RH fail\r\n");
	}
}

