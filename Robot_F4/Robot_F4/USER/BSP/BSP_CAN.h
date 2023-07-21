#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "main.h"
#include "can.h"

#define BROAD_ID_RK							0x01
#define BROAD_ID_F4							0x02
#define BROAD_ID_ADC						0x03
#define BROAD_ID_DISPLAY				0x04
#define BROAD_ID_HCSR						0x05

#define My_CAN_ID								BROAD_ID_F4
#define My_DATA_LEN							13

#define CAN_TYPE_OTA_MARK				0x0400		//OTA命令从0x400开始
#define CAN_TYPE_CMD_MARK				0x0200		//数据控制命令
#define CAN_TYPE_DATA_MARK			0x0100		//每个板子最多可以有255种自动上报数据

#define CAN_OTA_ERASE						(CAN_TYPE_OTA_MARK|0x0000)			//擦除APP区域数据
#define CAN_OTA_WRITE_INFO			(CAN_TYPE_OTA_MARK|0x0001)			//设置多字节写数据相关参数（写起始地址，数据量）
#define CAN_OTA_WRITE						(CAN_TYPE_OTA_MARK|0x0020)			//以多字节形式写数据
#define CAN_OTA_CHECK						(CAN_TYPE_OTA_MARK|0x0003)			//检测节点是否在线，同时返回固件信息
#define CAN_OTA_SET_BAUD_RATE		(CAN_TYPE_OTA_MARK|0x0004)			//设置节点波特率
#define CAN_OTA_EXCUTE					(CAN_TYPE_OTA_MARK|0x0005)			//执行固件
#define CAN_OTA_SUCCSEE					(CAN_TYPE_OTA_MARK|0x0008)			//命令执行成功
#define CAN_OTA_FAILD						(CAN_TYPE_OTA_MARK|0x0009)			//命令执行失败

#define CAN_CMD_SET_FREQ				(CAN_TYPE_CMD_MARK|0x0000)			//设置自动上报数据频率
#define CAN_CMD_SLEEP						(CAN_TYPE_CMD_MARK|0x0001)			//设置电源状态
#define CAN_CMD_GET_DATA				(CAN_TYPE_CMD_MARK|0x0002)			//主动读取数据
#define CAN_CMD_SET_ADDR				(CAN_TYPE_CMD_MARK|0x0003)			//设置数据上报位置
#define CAN_CMD_EX							(CAN_TYPE_CMD_MARK|0x0004)			//扩展命令

typedef struct CAN_DATA_Config_s{
	uint8_t INDEX;
	uint8_t RX_ADDR;
	uint16_t Freq;
	uint32_t last_time;
	CAN_HandleTypeDef *can_dev;
	void(*send_func)(CAN_HandleTypeDef *can_dev, uint8_t TX_ADDR, uint8_t RX_ADDR);
}CAN_DATA_Config_s;

struct CAN_ExtId_s{
	uint16_t Type;
	uint8_t TX_ADDR;
	uint8_t RX_ADDR;
};

typedef union CAN_ExtId{
	struct CAN_ExtId_s ExtId;
	uint32_t ID;
}CAN_ExtId;

extern CAN_DATA_Config_s can_data[My_DATA_LEN];

void CAN_Filter_Init(void);
void CAN_Auto_Send_Task(void);
#endif
