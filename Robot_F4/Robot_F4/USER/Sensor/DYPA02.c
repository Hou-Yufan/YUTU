#include "DYPA02.h"
#include "usart.h"

/*
超声波测距
*/
#define DYPA02_1_Usart huart4
#define DYPA02_2_Usart huart7
#define DYPA02_3_Usart huart8

#define DYPA02_FIFO_LEN	5

uint8_t DYPA02_Data[3][8];
struct DYPA02 DYPA02_Dev[3];

void DYPA02_Init(void)
{
	LOG("DYPA02 Init\r\n");

	HAL_UARTEx_ReceiveToIdle_DMA(&DYPA02_1_Usart,DYPA02_Data[2],8);//开始接收（使用DMA）
	__HAL_DMA_DISABLE_IT(DYPA02_1_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	DYPA02_Dev[2].distance = 4500;
	DYPA02_Dev[2].state = DYPA02_RUNING;
	DYPA02_Dev[2].t	= 0;

	HAL_UARTEx_ReceiveToIdle_DMA(&DYPA02_2_Usart,DYPA02_Data[1],8);//开始接收（使用DMA）
	__HAL_DMA_DISABLE_IT(DYPA02_2_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	DYPA02_Dev[1].distance = 4500;
	DYPA02_Dev[1].state = DYPA02_RUNING;
	DYPA02_Dev[1].t	= 0;
	
	HAL_UARTEx_ReceiveToIdle_DMA(&DYPA02_3_Usart,DYPA02_Data[0],8);//开始接收（使用DMA）
	__HAL_DMA_DISABLE_IT(DYPA02_3_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	DYPA02_Dev[0].distance = 4500;
	DYPA02_Dev[0].state = DYPA02_RUNING;
	DYPA02_Dev[0].t	= 0;
}

void DYPA02_Dev_Check()
{
	uint8_t i;
	for(i = 0; i < 3; i++)
	{
		if(DYPA02_Dev[i].state == DYPA02_RUNING)
		{
			LOG("DYPA02_Dev[%d] = %d\r\n",i,DYPA02_Dev[i].distance);
			DYPA02_Dev[i].t ++;
			if(DYPA02_Dev[i].t > 10)	//5s没收到正确的超声波数据
			{
				DYPA02_Dev[i].state = DYPA02_ERROR;
				LOG("DYPA02 %d Error\r\n",i);
			}
		}
	}
}


uint16_t DYPA02_fifo[3][DYPA02_FIFO_LEN] = {0};

uint16_t DYPA02_filter(uint16_t *fifo, uint16_t data)
{
	uint16_t temp[DYPA02_FIFO_LEN];
	uint16_t max = 0;
	uint8_t i,j;
	for(i = 0; i < DYPA02_FIFO_LEN-1; i++)
	{
		fifo[i] = fifo[i+1];	//整体往前移动
		temp[i] = fifo[i];		//拷贝
	}
	fifo[DYPA02_FIFO_LEN-1] = data;					//插入队尾
	temp[DYPA02_FIFO_LEN-1] = data;
	for(i = 0; i < DYPA02_FIFO_LEN; i++)
	{
		for(j = i; j < DYPA02_FIFO_LEN; j++)
		{
			if(max < temp[j])
			{
				data = max;
				max = temp[j];
				temp[j] = data;
			}
		}
	}
	return temp[(int)(DYPA02_FIFO_LEN/2)];
}

uint8_t DYPA02_Check(uint8_t *buf)
{
	uint8_t temp = 0;
	uint8_t i;
	for (i = 0; i < 3; i++)
	{
		temp += buf[i];
	}
	return temp;
}


void DYPA02_Read_Data(uint8_t id, uint16_t Size)
{
	uint16_t data = 0;
	uint8_t offset = 0;
	if(DYPA02_Dev[id].state == DYPA02_DISABLE)
	{
		DYPA02_Dev[id].t	= 0;
		return;
	}
	for(offset = 0; offset+3 < Size; offset++)
	{
		if(DYPA02_Data[id][offset] == 0xFF)
		{
			if(DYPA02_Data[id][offset+3] == DYPA02_Check(DYPA02_Data[id]+offset))
			{
				data = (uint16_t)(DYPA02_Data[id][offset+1]*256 + DYPA02_Data[id][offset+2]);
				DYPA02_Dev[id].distance	= DYPA02_filter(DYPA02_fifo[id], data);	//输出滤波后的距离
				if(DYPA02_Dev[id].distance == 0)
					DYPA02_Dev[id].distance = 4500;
				DYPA02_Dev[id].t	= 0;
				DYPA02_Dev[id].state = DYPA02_RUNING;
				break;
			}
		}
	}
}

void DYPA02_IRQ(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &DYPA02_1_Usart)
	{
		DYPA02_Read_Data(2, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&DYPA02_1_Usart,DYPA02_Data[2],8);//开始接收（使用DMA）
		__HAL_DMA_DISABLE_IT(DYPA02_1_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	}
	else if(huart == &DYPA02_2_Usart)
	{
		DYPA02_Read_Data(1, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&DYPA02_2_Usart,DYPA02_Data[1],8);//开始接收（使用DMA）
		__HAL_DMA_DISABLE_IT(DYPA02_2_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	}
	else if(huart == &DYPA02_3_Usart)
	{
		DYPA02_Read_Data(0, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&DYPA02_3_Usart,DYPA02_Data[0],8);//开始接收（使用DMA）
		__HAL_DMA_DISABLE_IT(DYPA02_3_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	}
}

