#include "SGP30.h"
#include "BSP_UART.h"

/*
*甲醛TVOC，二氧化碳CO2
*TVOC:  0ppb-2008ppb		1ppb;  2008ppb-11110ppb		6ppb;  11110ppb-60000ppb		32ppb
*TVOC:  < 0.6 mg/m3	 
*CO2:  400ppm-1479ppm  1ppm;  1479ppm-5144ppm  3ppm;  5144ppm-17597ppm  9ppm;  17597ppm-60000ppm  31ppm;  
*CO2:  < 960 mg/m3 :优；  >960 < 1440 mg/m3 :良； >1440mg/m3 :差；
*100ppb=0.1ppm=0.12mg/m3 
*/

struct SGP30 sgp;

/**
 * @brief	向SGP30发送一条指令(16bit)
 * @param	cmd SGP30指令
 * @retval	成功返回HAL_OK
*/
static uint8_t sgp30_send_cmd(sgp30_cmd_t cmd)
{
    uint8_t cmd_buffer[2];
    cmd_buffer[0] = cmd >> 8;
    cmd_buffer[1] = cmd;
    return HAL_I2C_Master_Transmit(&SGP30_I2C_Handle_Name, SGP30_ADDR_WRITE, (uint8_t*) cmd_buffer, 2, 10);
}

/**
 * @brief	软复位SGP30
 * @param	none
 * @retval	成功返回HAL_OK
*/
static int sgp30_soft_reset(void)
{
    uint8_t cmd = 0x06;
    return HAL_I2C_Master_Transmit(&SGP30_I2C_Handle_Name, 0x00, &cmd, 1, 10);
}

/**
 * @brief	初始化SGP30空气质量测量模式
 * @param	none
 * @retval	成功返回0，失败返回-1
*/
int SGP30_Init(void)
{
    int status;
    
    status = sgp30_soft_reset();
    if (status != HAL_OK) {
				sgp.error_cnt = 255;
        return -1;
    }
    
    HAL_Delay(200);
    
    status = sgp30_send_cmd(INIT_AIR_QUALITY);
    
    HAL_Delay(200);
    sgp30_start();
    return status == 0 ? 0 : -1;
}

/**
 * @brief	初始化SGP30空气质量测量模式
 * @param	none
 * @retval	成功返回HAL_OK
*/
int sgp30_start(void)
{
    return sgp30_send_cmd(MEASURE_AIR_QUALITY);
}

#define CRC8_POLYNOMIAL 0x31

static uint8_t CheckCrc8(uint8_t* const message, uint8_t initial_value)
{
    uint8_t  remainder;	    //余数
    uint8_t  i = 0, j = 0;  //循环变量

    /* 初始化 */
    remainder = initial_value;

    for(j = 0; j < 2;j++)
    {
        remainder ^= message[j];

        /* 从最高位开始依次计算  */
        for (i = 0; i < 8; i++)
        {
            if (remainder & 0x80)
            {
                remainder = (remainder << 1)^CRC8_POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }

    /* 返回计算的CRC码 */
    return remainder;
}

/**
 * @brief	读取一次空气质量数据
 * @param	none
 * @retval	成功返回0，失败返回-1
*/
int SGP30_Read(void)
{
    int status;
    uint8_t recv_buffer[6]={0};
    
		if(sgp.error_cnt > 10)
			return -1;
		
    /* 读取测量数据 */
    status = HAL_I2C_Master_Receive(&SGP30_I2C_Handle_Name, SGP30_ADDR_READ, (uint8_t*)recv_buffer, 6, 10);
    if (status != HAL_OK) {
				if(sgp.error_cnt < 255)
					sgp.error_cnt++;
        LOG("SGP30 I2C Master Receive fail\r\n");
        return -1;
    }
    
    /* 校验接收的测量数据 */
    if (CheckCrc8(&recv_buffer[0], 0xFF) != recv_buffer[2]) {
				if(sgp.error_cnt < 255)
					sgp.error_cnt++;
        LOG("SGP30 co2 recv data crc check fail\r\n");
        return -1;
    }
		/* 转换测量数据 */
    sgp.SGP30_TVOC = (uint16_t)(recv_buffer[3] << 8 | recv_buffer[4]);
		
    if (CheckCrc8(&recv_buffer[3], 0xFF) != recv_buffer[5]) {
				if(sgp.error_cnt < 255)
					sgp.error_cnt++;
        LOG("SGP30 tvoc recv data crc check fail\r\n");
        return -1;
    }
		
		if(sgp.error_cnt > 0)
					sgp.error_cnt--;
    /* 转换测量数据 */
    sgp.SGP30_CO2  = (uint16_t)(recv_buffer[0] << 8 | recv_buffer[1]);
    return 0;
}
