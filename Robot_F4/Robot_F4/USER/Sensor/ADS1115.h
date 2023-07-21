//i2c_ads1115.h
#ifndef __ADS1115_H__
#define __ADS1115_H__

#define ADS_USE_HEAD_IIC
//#define ADS_USE_SOFT_IIC

#define ADS_HEAD_IIC	0
#define ADS_SOFT_IIC	1

#include "main.h"

#define IR_L	2
#define IR_R	1
#define IR_M	0

/***************/
//配置寄存器说明

//config register
/*CRH[15:8](R/W)
   BIT      15      14      13      12      11      10      9       8
   NAME     OS      MUX2    MUX1    MUX0    PGA2    PGA1    PGA0    MODE
CRL[7:0] (R/W)
   BIT      7       6       5       4       3       2       1       0
   NAME    DR0     DR1     DR0   COM_MODE COM_POL COM_LAT COM_QUE1 COM_QUE0


   -----------------------------------------------------------------------------------
 * 15    | OS             |  运行状态会单词转换开始
 *       |                | 写时:
 *       |                | 0   : 无效
 *       |                | 1   : 开始单次转换处于掉电状态时
 *       |                | 读时:
 *       |                | 0   : 正在转换
 *       |                | 1   : 未执行转换
 * -----------------------------------------------------------------------------------
 * 14:12 | MUX [2:0]      | 输入复用多路配置
 *       |                | 000 : AINP = AIN0 and AINN = AIN1 (default)
 *       |                | 001 : AINP = AIN0 and AINN = AIN3
 *       |                | 010 : AINP = AIN1 and AINN = AIN3
 *       |                | 011 : AINP = AIN2 and AINN = AIN3
 *       |                | 100 : AINP = AIN0 and AINN = GND
 *       |                | 101 : AINP = AIN1 and AINN = GND
 *       |                | 110 : AINP = AIN2 and AINN = GND
 *       |                | 111 : AINP = AIN3 and AINN = GND
 * -----------------------------------------------------------------------------------
 * 11:9  | PGA [2:0]      | 可编程增益放大器配置(FSR  full scale range)
 *       |                | 000 : FSR = В±6.144 V
 *       |                | 001 : FSR = В±4.096 V
 *       |                | 010 : FSR = В±2.048 V (默认)
 *       |                | 011 : FSR = В±1.024 V
 *       |                | 100 : FSR = В±0.512 V
 *       |                | 101 : FSR = В±0.256 V
 *       |                | 110 : FSR = В±0.256 V
 *       |                | 111 : FSR = В±0.256 V
 * -----------------------------------------------------------------------------------
 * 8     | MODE           | 工作模式
 *       |                | 0   : 连续转换
 *       |                | 1   : 单词转换
 * -----------------------------------------------------------------------------------
 * 7:5   | DR [2:0]       | 采样频率
 *       |                | 000 : 8 SPS
 *       |                | 001 : 16 SPS
 *       |                | 010 : 32 SPS
 *       |                | 011 : 64 SPS
 *       |                | 100 : 128 SPS (默认)
 *       |                | 101 : 250 SPS
 *       |                | 110 : 475 SPS
 *       |                | 111 : 860 SPS
 * -----------------------------------------------------------------------------------
 * 4     | COMP_MODE      | 比较器模式
 *       |                | 0   : 传统比较器 (default)
 *       |                | 1   : 窗口比较器
 * -----------------------------------------------------------------------------------
 * 3     | COMP_POL       | Comparator polarity
 *       |                | 0   : 低电平有效 (default)
 *       |                | 1   : 高电平有效
 * -----------------------------------------------------------------------------------
 * 2     | COMP_LAT       | Latching comparator
 *       |                | 0   : 非锁存比较器. (default)
 *       |                | 1   : 锁存比较器.
 * -----------------------------------------------------------------------------------
 * 1:0   | COMP_QUE [1:0] | Comparator queue and disable
 *       |                | 00  : Assert after one conversion
 *       |                | 01  : Assert after two conversions
 *       |                | 10  : Assert after four conversions
 *       |                | 11  : 禁用比较器并将ALERT/RDY设置为高阻抗 (default)
 * -----------------------------------------------------------------------------------
*/


#define ADS1x15_DEFAULT_ADDRESS        0x48
#define ADS1x15_POINTER_CONVERSION     0x00		//ADC转化值的寄存器
#define ADS1x15_POINTER_CONFIG         0x01		//ADC配置寄存器
#define ADS1x15_POINTER_LOW_THRESHOLD  0x02		//ADC低阈值寄存器
#define ADS1x15_POINTER_HIGH_THRESHOLD 0x03		//ADC高阈值寄存器
#define ADS1x15_CONFIG_OS_SINGLE       0x8000
#define ADS1x15_CONFIG_MUX_OFFSET      12

#define  ADS1x15_WR_REG 0x90       //器件地址 0x48<<1
//#define  ADS1x15_WR_REG 0x92       //器件地址 0x49<<1

//增益
#define ADS1x15_CONFIG_GAIN_0 	0x0000
#define ADS1x15_CONFIG_GAIN_1		0x0200
#define ADS1x15_CONFIG_GAIN_2 	0x0400
#define ADS1x15_CONFIG_GAIN_4   0x0600
#define ADS1x15_CONFIG_GAIN_8   0x0800
#define ADS1x15_CONFIG_GAIN_16  0x0A00
/*
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
*/

//模式
#define ADS1x15_CONFIG_MODE_CONTINUOUS  0x0000
#define ADS1x15_CONFIG_MODE_SINGLE      0x0100

//采样速度
#define ADS1115_CONFIG_RATE_8    0x0000
#define ADS1115_CONFIG_RATE_16   0x0020
#define ADS1115_CONFIG_RATE_32   0x0040
#define ADS1115_CONFIG_RATE_64   0x0060
#define ADS1115_CONFIG_RATE_128  0x0080
#define ADS1115_CONFIG_RATE_250  0x00A0
#define ADS1115_CONFIG_RATE_475  0x00C0
#define ADS1115_CONFIG_RATE_860  0x00E0

#define ADS1x15_CONFIG_COMP_QUE_DISABLE 0x0003

struct ads1115
{
	void* hw;
	uint8_t HW_Type;	//0:硬件IIC，1:软件IIC
	uint8_t state;
	uint8_t addr;
	uint16_t adc[4];	//adc原始值
	uint8_t error_cnt;
};

extern struct ads1115 charge_ir;
void ADS1115_Run(void);

#endif

