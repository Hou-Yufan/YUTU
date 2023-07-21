#include "Soft_I2C.h"
#include "BSP_TIM.h"


#define IIC_TIMEOUT	500

struct sotf_i2c swi2c1;
struct sotf_i2c swi2c2;

void Soft_I2C_Init()
{
	Soft_I2C_MspInit(&swi2c1, GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_6, 200000);
	Soft_I2C_MspInit(&swi2c2, GPIOF, I2C2_SDA_Pin, GPIOF, I2C2_SCL_Pin, 200000);
}

void Soft_I2C_MspInit(struct sotf_i2c* i2c, GPIO_TypeDef* SDA_GPIO_Port, uint16_t SDA_Pin, GPIO_TypeDef* SCL_GPIO_Port, uint16_t SCL_Pin, uint32_t freq)
{
	assert_param((i2c != (void*)0));
	assert_param(IS_GPIO_ALL_INSTANCE(SDA_GPIO_Port));
	assert_param(IS_GPIO_ALL_INSTANCE(SCL_GPIO_Port));
	
	i2c->SDA_GPIO_Port = SDA_GPIO_Port;
	i2c->SDA_Pin = SDA_Pin;
	i2c->SCL_GPIO_Port = SCL_GPIO_Port;
	i2c->SCL_Pin = SCL_Pin;
	
	if(freq > 400000)			//最高400KHz
		freq = 400000;
	else if(freq < 50000)	//最低50KHz
		freq = 50000;
	
	i2c->delay_us = 500000/freq;				//clk延时，一个clk两次delay
	i2c->freq = 500000/i2c->delay_us;		//实际频率
}

void Soft_I2C_Delay(struct sotf_i2c* i2c)
{
	delay_us(i2c->delay_us);
}

void Soft_I2C_SDA_OUT(GPIO_TypeDef* SDA_GPIO_Port, uint16_t SDA_GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = SDA_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);	
}

void Soft_I2C_SDA_IN(GPIO_TypeDef* SDA_GPIO_Port, uint16_t SDA_GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = SDA_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_SPEED_FREQ_HIGH;//GPIO_NOPULL;
  HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
}

int Soft_I2C_Start(struct sotf_i2c* i2c)
{
	Soft_I2C_SDA_OUT(i2c->SDA_GPIO_Port, i2c->SDA_Pin);     //sda线输出
	
	HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_SET);
	if(!HAL_GPIO_ReadPin(i2c->SDA_GPIO_Port, i2c->SDA_Pin))return I2C_ERR;	
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_SET);
	Soft_I2C_Delay(i2c);
 	HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_RESET);//START:when CLK is high,DATA change form high to low 
	if(HAL_GPIO_ReadPin(i2c->SDA_GPIO_Port, i2c->SDA_Pin))return I2C_ERR;
	Soft_I2C_Delay(i2c);
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin,GPIO_PIN_RESET);//钳住I2C总线，准备发送或接收数据 
	return I2C_OK;
}

void Soft_I2C_Stop(struct sotf_i2c* i2c)
{
	Soft_I2C_SDA_OUT(i2c->SDA_GPIO_Port, i2c->SDA_Pin);//sda线输出
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_RESET);//STOP:when CLK is high DATA change form low to high
 	Soft_I2C_Delay(i2c);
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_SET); 
	Soft_I2C_Delay(i2c);	
	HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_SET);//发送I2C总线结束信号
							   	
}

int Soft_I2C_Wait_Ack(struct sotf_i2c* i2c)
{
	uint16_t ucErrTime=0;
	Soft_I2C_SDA_IN(i2c->SDA_GPIO_Port, i2c->SDA_Pin);      //SDA设置为输入  
	HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_SET);
	Soft_I2C_Delay(i2c);	   
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_SET);
	Soft_I2C_Delay(i2c);	 
	while(HAL_GPIO_ReadPin(i2c->SDA_GPIO_Port, i2c->SDA_Pin))
	{
		ucErrTime++;
		if(ucErrTime>IIC_TIMEOUT)
		{
			Soft_I2C_Stop(i2c);
			return I2C_ERR;
		}
	}
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);//时钟输出0 	   
	return I2C_OK;  
} 

void Soft_I2C_Ack(struct sotf_i2c* i2c)
{
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);
	Soft_I2C_SDA_OUT(i2c->SDA_GPIO_Port, i2c->SDA_Pin);
	HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_RESET);
	Soft_I2C_Delay(i2c);
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_SET);
	Soft_I2C_Delay(i2c);
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);
}

void Soft_I2C_NAck(struct sotf_i2c* i2c)
{
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);
	Soft_I2C_SDA_OUT(i2c->SDA_GPIO_Port, i2c->SDA_Pin);
	HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_SET);
	Soft_I2C_Delay(i2c);
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_SET);
	Soft_I2C_Delay(i2c);
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);
}
	  
void Soft_I2C_Send_Byte(struct sotf_i2c* i2c, uint8_t txd)
{                        
	uint8_t t;   
	Soft_I2C_SDA_OUT(i2c->SDA_GPIO_Port, i2c->SDA_Pin); 	    
	HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		if((txd&0x80)>>7)
			{HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_SET);}
		else
			{HAL_GPIO_WritePin(i2c->SDA_GPIO_Port, i2c->SDA_Pin, GPIO_PIN_RESET);}
		txd<<=1;
		Soft_I2C_Delay(i2c);   
		HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_SET);
		Soft_I2C_Delay(i2c); 
		HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET);	
		Soft_I2C_Delay(i2c);
	}
}

uint8_t Soft_I2C_Read_Byte(struct sotf_i2c* i2c, unsigned char ack)
{
	unsigned char i,receive=0;
	Soft_I2C_SDA_IN(i2c->SDA_GPIO_Port, i2c->SDA_Pin);//SDA设置为输入
    for(i=0;i<8;i++ )
	{
		HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_RESET); 
		Soft_I2C_Delay(i2c);
		HAL_GPIO_WritePin(i2c->SCL_GPIO_Port, i2c->SCL_Pin, GPIO_PIN_SET);
		receive<<=1;
		if(HAL_GPIO_ReadPin(i2c->SDA_GPIO_Port, i2c->SDA_Pin))receive++;   
		Soft_I2C_Delay(i2c);
  }
	if (ack)
			Soft_I2C_Ack(i2c); //发送ACK 
	else
			Soft_I2C_NAck(i2c);//发送nACK  
	return receive;
}

int Soft_I2C_Send_Regs(struct sotf_i2c* i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (Soft_I2C_Start(i2c) == I2C_ERR)
        return I2C_ERR;
    Soft_I2C_Send_Byte(i2c, addr);
    if (Soft_I2C_Wait_Ack(i2c) == I2C_ERR) {
        Soft_I2C_Stop(i2c);
        return I2C_ERR;
    }
    Soft_I2C_Send_Byte(i2c,reg);
    Soft_I2C_Wait_Ack(i2c);
		for (i = 0; i < len; i++) {
        Soft_I2C_Send_Byte(i2c,data[i]);
        if (Soft_I2C_Wait_Ack(i2c) == I2C_ERR) {
            Soft_I2C_Stop(i2c);
            return I2C_ERR;
        }
    }
    Soft_I2C_Stop(i2c);
    return I2C_OK;
}

int Soft_I2C_Read_Regs(struct sotf_i2c* i2c, uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (Soft_I2C_Start(i2c) == I2C_ERR)
        return I2C_ERR;
    Soft_I2C_Send_Byte(i2c, addr);
    if (Soft_I2C_Wait_Ack(i2c) == I2C_ERR) {
        Soft_I2C_Stop(i2c);
        return I2C_ERR;
    }
    Soft_I2C_Send_Byte(i2c, reg);
    Soft_I2C_Wait_Ack(i2c);
    Soft_I2C_Start(i2c);
    Soft_I2C_Send_Byte(i2c,addr|0x01);
    Soft_I2C_Wait_Ack(i2c);
    while (len) {
        if (len == 1)
            *buf = Soft_I2C_Read_Byte(i2c,0);
        else
            *buf = Soft_I2C_Read_Byte(i2c,1);
        buf++;
        len--;
    }
    Soft_I2C_Stop(i2c);
    return I2C_OK;
}



