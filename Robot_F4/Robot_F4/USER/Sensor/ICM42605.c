#include "ICM42605.h"
#ifdef ICM_USE_HARD_SPI
#include "spi.h"
#endif
#ifdef ICM_USE_HARD_I2C
#include "i2c.h"
#endif
#ifdef ICM_USE_SOFT_I2C
#include "Soft_I2C.h"
#endif

/*ICM42605使用的ms级延时函数，须由用户提供。*/
#define ICM42605DelayMs(_nms)  HAL_Delay(_nms)

#define ICM_PORT_SPIX_CS	 	 icm->CS_GPIO_Port
#define ICM_PIN_SPIX_CS	     icm->CS_Pin
#define ICM_SPI_CS_LOW()     HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_RESET)
#define ICM_SPI_CS_HIGH()    HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_SET)

uint8_t icm42605_read_reg(struct icm42605* icm, uint8_t reg)
{
	uint8_t regval = 0;
	switch(icm->HW_Type)
	{
#if defined(ICM_USE_HARD_SPI)
		case ICM42605_HW_SPI:
		{
			ICM_SPI_CS_LOW();
			reg |= 0x80;
			HAL_SPI_Transmit(icm->hw, &reg, 1, 5);
			HAL_SPI_Receive(icm->hw, &regval, 1, 5);
			ICM_SPI_CS_HIGH();
			break;
		}
#endif
#if defined(ICM_USE_HARD_I2C)
		case ICM42605_HW_IIC:
		{
			HAL_I2C_Mem_Read(icm->hw, icm->I2C_ADDR, reg, 1, &regval, 1, 5);
			break;
		}
#endif
#if defined(ICM_USE_SOFT_I2C)
		case ICM42605_SW_IIC:
		{
			Soft_I2C_Read_Regs(icm->hw, icm->I2C_ADDR, reg, 1, &regval);
			break;
		}
#endif
		default:
			break;
	}
	return regval;
}

static void icm42605_read_regs(struct icm42605* icm, uint8_t reg, uint8_t* buf, uint16_t len)
{
	switch(icm->HW_Type)
	{
#if defined(ICM_USE_HARD_SPI)
		case ICM42605_HW_SPI:
		{
			ICM_SPI_CS_LOW();
			reg |= 0x80;
			HAL_SPI_Transmit(icm->hw, &reg, 1, 5);
			HAL_SPI_Receive(icm->hw, buf, len, 5);
			ICM_SPI_CS_HIGH();
			break;
		}
#endif
#if defined(ICM_USE_HARD_I2C)
		case ICM42605_HW_IIC:
		{
			HAL_I2C_Mem_Read(icm->hw, icm->I2C_ADDR, reg, 1, buf, len, 5);
			break;
		}
#endif
#if defined(ICM_USE_SOFT_I2C)
		case ICM42605_SW_IIC:
		{
			Soft_I2C_Read_Regs(icm->hw, icm->I2C_ADDR, reg, len, buf);
			break;
		}
#endif
		default:
			break;
	}
}

static uint8_t icm42605_write_reg(struct icm42605* icm, uint8_t reg, uint8_t value)
{
	switch(icm->HW_Type)
	{
#if defined(ICM_USE_HARD_SPI)
		case ICM42605_HW_SPI:
		{
			ICM_SPI_CS_LOW();
			HAL_SPI_Transmit(icm->hw, &reg, 1, 5);
			HAL_SPI_Transmit(icm->hw, &value, 1, 5);
			ICM_SPI_CS_HIGH();
			break;
		}
#endif
#if defined(ICM_USE_HARD_I2C)
		case ICM42605_HW_IIC:
		{
			HAL_I2C_Mem_Write(icm->hw, icm->I2C_ADDR, reg, 1, &value, 1, 5);
			break;
		}
#endif
#if defined(ICM_USE_SOFT_I2C)
		case ICM42605_SW_IIC:
		{
			Soft_I2C_Send_Regs(icm->hw, icm->I2C_ADDR, reg, 1, &value);
			break;
		}
#endif
		default:
			break;
	}
	return 0;
}

/*******************************************************************************
* 名    称： bsp_Icm42605Init
* 功    能： Icm42605 传感器初始化
* 入口参数： 无
* 出口参数： 0: 初始化成功  其他值: 初始化失败
* 作　　者： Roger-WY.
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
int8_t ICM42605_Init(struct icm42605* icm, uint8_t acc, uint8_t gyro, uint16_t freq)
{
	uint8_t reg_val = 0;
	uint8_t odr_freq;
	/* 读取 who am i 寄存器 */
	reg_val = icm42605_read_reg(icm, ICM42605_WHO_AM_I);
	if(reg_val == ICM42605_ID)
	{
		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0); //设置bank 0区域寄存器
		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0x01); //软复位传感器
		ICM42605DelayMs(100);
	
		if(icm->HW_Type == ICM42605_HW_SPI)
		{
			icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 1); 		//设置bank 1区域寄存器
			icm42605_write_reg(icm, ICM42605_INTF_CONFIG4, 0x02); //设置为4线SPI通信
		}
		
		if(acc > AFS_2G)
			acc = AFS_2G;
		
		switch(acc)
		{
			case AFS_2G:
				icm->Acc_Scale_Factor = 16384.0f;
				break;
			case AFS_4G:
				icm->Acc_Scale_Factor = 8192.0f;
				break;
			case AFS_8G:
				icm->Acc_Scale_Factor = 4096.0f;
				break;
			case AFS_16G:
				icm->Acc_Scale_Factor = 2048.0f;
				break;
		}
		
		if(gyro > GFS_15_125DPS)
			gyro = GFS_15_125DPS;
		
		switch(gyro)
		{
			case GFS_15_125DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 15.125f;
				break;
			case GFS_31_25DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 31.25f;
				break;
			case GFS_62_5DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 62.5f;
				break;
			case GFS_125DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 125.0f;
				break;
			case GFS_250DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 250.0f;
				break;
			case GFS_500DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 500.0f;
				break;
			case GFS_1000DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 1000.0f;
				break;
			case GFS_2000DPS:
				icm->Gyro_Scale_Factor = 32768.0f / 2000.0f;
				break;
		}
		
		if(freq < 13)
			odr_freq = AODR_12_5Hz;
		else if(freq < 26)
			odr_freq = AODR_25Hz;
		else if(freq < 51)
			odr_freq = AODR_50Hz;
		else if(freq < 101)
			odr_freq = AODR_100Hz;
		else if(freq < 201)
			odr_freq = AODR_200Hz;
		else if(freq < 501)
			odr_freq = AODR_500Hz;
		else if(freq < 1001)
			odr_freq = AODR_1000Hz;
		else if(freq < 2000)
			odr_freq = AODR_2000Hz;
		else if(freq < 4000)
			odr_freq = AODR_4000Hz;
		else
			odr_freq = AODR_8000Hz;
		
		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0); 		//设置bank 0区域寄存器
		icm42605_write_reg(icm, ICM42605_FIFO_CONFIG, 0x40); 	//Stream-to-FIFO Mode(page61)
		
		reg_val = icm42605_read_reg(icm, ICM42605_INT_SOURCE0);
		icm42605_write_reg(icm, ICM42605_INT_SOURCE0, 0x00);
		icm42605_write_reg(icm, ICM42605_FIFO_CONFIG2, 0x00); // watermark
		icm42605_write_reg(icm, ICM42605_FIFO_CONFIG3, 0x02); // watermark
		icm42605_write_reg(icm, ICM42605_INT_SOURCE0, reg_val);
		icm42605_write_reg(icm, ICM42605_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0x00);
		icm42605_write_reg(icm, ICM42605_INT_CONFIG, 0x36);

		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0x00);
		reg_val = icm42605_read_reg(icm, ICM42605_INT_SOURCE0);
		reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
		icm42605_write_reg(icm, ICM42605_INT_SOURCE0, reg_val);

		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0x00);
		reg_val = icm42605_read_reg(icm, ICM42605_ACCEL_CONFIG0);//page74
		reg_val |= (acc << 5);   			//量程
		reg_val |= (odr_freq);     		//输出速率
		icm42605_write_reg(icm, ICM42605_ACCEL_CONFIG0, reg_val);

		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0x00);
		reg_val = icm42605_read_reg(icm, ICM42605_GYRO_CONFIG0);//page73
		reg_val |= (gyro << 5);   		//量程
		reg_val |= (odr_freq);     		//输出速率
		icm42605_write_reg(icm, ICM42605_GYRO_CONFIG0, reg_val);

		icm42605_write_reg(icm, ICM42605_REG_BANK_SEL, 0x00);
		reg_val = icm42605_read_reg(icm, ICM42605_PWR_MGMT0); //读取PWR—MGMT0当前寄存器的值(page72)
		reg_val &= ~(1 << 5);	//使能温度测量
		reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
		reg_val |= (3);				//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
		icm42605_write_reg(icm, ICM42605_PWR_MGMT0, reg_val);
		ICM42605DelayMs(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作
		return 0;
	}
	return -1;
}

/*******************************************************************************
* 名    称： bsp_IcmGetTemperature
* 功    能： 读取Icm42605 内部传感器温度
* 入口参数： 无
* 出口参数： 无
* 作　　者： Roger-WY.
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t ICM42605_GetTemperature(struct icm42605* icm, int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    icm42605_read_regs(icm, ICM42605_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}

/*******************************************************************************
* 名    称： bsp_IcmGetAccelerometer
* 功    能： 读取Icm42605 加速度的值
* 入口参数： 三轴加速度的值
* 出口参数： 无
* 作　　者： Roger-WY.
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t ICM42605_GetAccelerometer(struct icm42605* icm, int16_t *accData)
{
    uint8_t buffer[6] = {0};

    icm42605_read_regs(icm, ICM42605_ACCEL_DATA_X1, buffer, 6);

    accData[0] = ((uint16_t)buffer[0] << 8) | buffer[1];
    accData[1] = ((uint16_t)buffer[2] << 8) | buffer[3];
    accData[2] = ((uint16_t)buffer[4] << 8) | buffer[5];

    return 0;
}

/*******************************************************************************
* 名    称： bsp_IcmGetGyroscope
* 功    能： 读取Icm42605 陀螺仪的值
* 入口参数： 三轴陀螺仪的值
* 出口参数： 无
* 作　　者： Roger-WY.
* 创建日期： 2021-05-21
* 修    改：
* 修改日期：
* 备    注： datasheet page63
*******************************************************************************/
int8_t ICM42605_GetGyroscope(struct icm42605* icm, int16_t *GyroData)
{
    uint8_t buffer[6] = {0};

    icm42605_read_regs(icm, ICM42605_GYRO_DATA_X1, buffer, 6);

    GyroData[0] = ((uint16_t)buffer[0] << 8) | buffer[1];
    GyroData[1] = ((uint16_t)buffer[2] << 8) | buffer[3];
    GyroData[2] = ((uint16_t)buffer[4] << 8) | buffer[5];

    return 0;
}


