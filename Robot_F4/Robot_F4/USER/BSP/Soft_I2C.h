#ifndef __SOFT_I2C_H_
#define __SOFT_I2C_H_

#include "main.h"

#define I2C_OK		HAL_OK
#define I2C_ERR		HAL_ERROR

struct sotf_i2c
{
	GPIO_TypeDef* SDA_GPIO_Port;
	GPIO_TypeDef* SCL_GPIO_Port; 
	uint16_t SDA_Pin;
	uint16_t SCL_Pin;
	float freq;
	uint8_t delay_us;
};

extern struct sotf_i2c swi2c1;
extern struct sotf_i2c swi2c2;

void Soft_I2C_Init(void);
void Soft_I2C_MspInit(struct sotf_i2c* i2c, GPIO_TypeDef* SDA_GPIO_Port, uint16_t SDA_Pin, GPIO_TypeDef* SCL_GPIO_Port, uint16_t SCL_Pin, uint32_t freq);
int Soft_I2C_Start(struct sotf_i2c* i2c);
void Soft_I2C_Stop(struct sotf_i2c* i2c);
int Soft_I2C_Wait_Ack(struct sotf_i2c* i2c);
void Soft_I2C_Ack(struct sotf_i2c* i2c);
void Soft_I2C_NAck(struct sotf_i2c* i2c);
void Soft_I2C_Send_Byte(struct sotf_i2c* i2c, uint8_t txd);
uint8_t Soft_I2C_Read_Byte(struct sotf_i2c* i2c, unsigned char ack);
int Soft_I2C_Send_Regs(struct sotf_i2c* i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int Soft_I2C_Read_Regs(struct sotf_i2c* i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
#endif
