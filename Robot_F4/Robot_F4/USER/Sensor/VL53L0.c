#include "VL53L0.h"
#include "usart.h"

#define VL53L0_Usart huart2
/*
载物激光测距
*/
struct VL53L0 VL53L0_Dev[2];
uint8_t VL53L0_Data[64];

void VL53L0_Init(void)
{
	LOG("VL53L0 Init\r\n");
//  __HAL_UART_ENABLE_IT(&VL53L0_Usart, UART_IT_IDLE);  				//空闲中断
	HAL_UARTEx_ReceiveToIdle_DMA(&VL53L0_Usart,VL53L0_Data,64);//开始接收（使用DMA）
	__HAL_DMA_DISABLE_IT(VL53L0_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	VL53L0_Dev[0].distance = 0;
	VL53L0_Dev[0].state = VL53L0_RUNING;
	VL53L0_Dev[0].t	= 0;
}

void VL53L0_Read_Data(uint8_t *Read_Data, uint16_t Size)
{
	uint8_t offset;
	uint8_t len;
	uint16_t d = 0;

	if(Read_Data == NULL)
	{
		return;
	}
	for(offset = 0; offset < Size; offset++)
	{
		if(Read_Data[offset] == 'm')	//末尾
		{
			len = offset;
			for(;offset > 0;offset--)
			{
				if(Read_Data[offset] == ':')
				{
					offset += 1;
					for(len = 0; len<9; len++)
					{
						if(Read_Data[offset+len] == 'm')	//解析完成
						{
							VL53L0_Dev[0].distance = d/10;
							if(VL53L0_Dev[0].distance < 4)
								VL53L0_Dev[0].distance = 2000;
							else
								VL53L0_Dev[0].distance -= 3;
							VL53L0_Dev[0].t = 0;
							break;
						}
						if(Read_Data[offset+len] > ('0'-1) && Read_Data[offset+len] < ('9'+1))	//48~57,数字的ASCII码
						{
							d *= 10;
							d += (Read_Data[offset+len] - '0');
						}
					}
					break;
				}
			}
			break;
		}
	}
}

void VL53L0_Dev_Check()
{
	uint8_t i;
	for(i = 0; i < 2; i++)
	{
		if(VL53L0_Dev[i].state == VL53L0_RUNING || VL53L0_Dev[i].state == VL53L0_ERROR)
		{
			LOG("VL53L0[%d] = %d\r\n",i,VL53L0_Dev[i].distance);
			if(VL53L0_Dev[i].t < 255)
				VL53L0_Dev[i].t ++;
			if(VL53L0_Dev[i].t > 10)	//5s没收到正确的超声波数据
			{
				VL53L0_Dev[i].state = VL53L0_ERROR;
				LOG("VL53L0 %d Error\r\n",i);
			}
			else
				VL53L0_Dev[i].state = VL53L0_RUNING;
		}
	}
}

void VL53L0_UART_IRQ(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &VL53L0_Usart)
	{
		VL53L0_Dev[0].t = 0;
		VL53L0_Read_Data(VL53L0_Data,Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&VL53L0_Usart,VL53L0_Data,64);//开始接收（使用DMA）
		__HAL_DMA_DISABLE_IT(VL53L0_Usart.hdmarx, DMA_IT_HT);				//关闭DMA中断
	}
}

void VL53L0_CAN_IRQ(uint8_t *buf)
{
	VL53L0_Dev[1].state = buf[2];
	if(VL53L0_Dev[1].state == VL53L0_DISABLE)
	{
		VL53L0_Dev[1].t = 0;
		return;
	}
	VL53L0_Dev[1].distance = ((uint16_t)buf[0]) << 8 | buf[1];
	if(VL53L0_Dev[1].distance > 3)
		VL53L0_Dev[1].distance -= 3;
	VL53L0_Dev[1].t = buf[3];
}
