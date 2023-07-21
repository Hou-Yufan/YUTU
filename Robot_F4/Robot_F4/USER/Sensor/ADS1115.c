#include "ADS1115.h"
#ifdef ADS_USE_HEAD_IIC
#include "i2c.h"
#endif
#ifdef ADS_USE_SOFT_IIC
#include "Soft_I2C.h"
#endif

struct ads1115 charge_ir = {
	.hw = &hi2c1,
	.HW_Type = ADS_HEAD_IIC,	//ADS_SOFT_IIC
	.addr = ADS1x15_WR_REG,
};

void ADS1115_Start(struct ads1115* ads, uint8_t channel, uint16_t gain, uint16_t data_rate, uint16_t mode)
{
	uint8_t buf[2];
	uint16_t mux = channel + 0x04;
	//Go out of power-down mode for conversion
	uint16_t config = ADS1x15_CONFIG_OS_SINGLE;
	//Specify mux value
	config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET;
	//设置增益
	config |= gain;
	//设置模式(continuous or single shot)
  config |= mode;
	//设置采样速度
	config |= data_rate;
	//disable 比较器模式
  config |= ADS1x15_CONFIG_COMP_QUE_DISABLE;
	
	buf[0] = (config >> 8) & 0xFF;
	buf[1] = config & 0x00FF;
	
	switch(ads->HW_Type)
	{
#ifdef ADS_USE_HEAD_IIC
		case ADS_HEAD_IIC:
			ads->state = HAL_I2C_Mem_Write(ads->hw,ads->addr, ADS1x15_POINTER_CONFIG, 1, buf, 2, 10);
			break;
#endif
#ifdef ADS_USE_SOFT_IIC
		case ADS_SOFT_IIC:
			Soft_I2C_Send_Regs(ads->hw,ads->addr, ADS1x15_POINTER_CONFIG, 2, buf);
			break;
#endif
		default:
			break;
	}
	if (ads->state != HAL_OK)
	{
		if (ads->error_cnt < 255)
			ads->error_cnt++;
		LOG("ads write error %d\r\n", ads->error_cnt);

	}
	else
	{
		if (ads->error_cnt > 0)
			ads->error_cnt--;
	}
}

uint16_t ADS1115_Read(struct ads1115* ads)
{
	uint16_t adc_val;
	uint8_t buf[2] = {0,0};
	switch(ads->HW_Type)
	{
#ifdef ADS_USE_HEAD_IIC
		case ADS_HEAD_IIC:
			ads->state = HAL_I2C_Mem_Read(ads->hw,ads->addr, ADS1x15_POINTER_CONVERSION, 1, buf, 2, 10);
			break;
#endif
#ifdef ADS_USE_SOFT_IIC
		case ADS_SOFT_IIC:
			Soft_I2C_Read_Regs(ads->hw,ads->addr, ADS1x15_POINTER_CONVERSION, 2, buf);
			break;
#endif
		default:
			break;
	}
	if(ads->state != HAL_OK)
		LOG("ads read error\r\n");
	adc_val = ((uint16_t)buf[0])<<8 | buf[1];
	adc_val = adc_val >> 4;	//转化成12位adc
	return adc_val;
}

void ADS1115_Run()
{
	static uint8_t t = 0;
	static uint8_t ch = 0;
	if(charge_ir.error_cnt > 100)
		return;
	t++;
	if(t == 1)
	{
		ADS1115_Start(&charge_ir, ch, ADS1x15_CONFIG_GAIN_1, ADS1115_CONFIG_RATE_860, ADS1x15_CONFIG_MODE_SINGLE);
	}
	else if(t == 4)
	{
		charge_ir.adc[ch] = ADS1115_Read(&charge_ir);
		ch++;
		if(ch == 3)
			ch = 0;
		t = 0;
	}
}


