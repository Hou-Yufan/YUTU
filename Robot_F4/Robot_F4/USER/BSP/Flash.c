/**
  ******************************************************************************
  * @file           : Flash.c
  * @brief          : 内部Flash读写
  ******************************************************************************
  * @attention		
  *
	* @Logs:
	* Date           Author       Notes
	* 2022-09-04     李树益      第一个版本
  ******************************************************************************
  */
#include "Flash.h"
#include "string.h"
 
/**
 * @brief 从FLASH中读取参数
 *
 * @param none
 *
 * @return none
 */
void FLASH_Read(uint32_t *buf, uint16_t len)
{
	memcpy(buf, (void*)USER_FLASH_ADDRESS, len);	//从FLASH中读取参数
}

/**
 * @brief 保存参数到FLASH
 *
 * @param none
 *
 * @return none
 */
void FLASH_Write(uint32_t *buf, uint32_t len)
{
	FLASH_EraseInitTypeDef flash_erase;
	uint32_t error;

	//FLASH擦除方式
	flash_erase.Sector = USER_FLASH_SECTOR;
	flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	flash_erase.NbSectors = 1;
	
	uint16_t data_len = 0;
	
	//准备写入FLASH
	HAL_FLASH_Unlock();	//先解锁
	do{
	HAL_FLASHEx_Erase(&flash_erase, &error);	//再擦除
	}while(error != 0xFFFFFFFF);
	for(data_len = 0; data_len < len; data_len += 4)	//最后四个字节四个字节写入
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, data_len + USER_FLASH_ADDRESS, *buf);
		buf++;
	}
	HAL_FLASH_Lock();	//写完锁定FLASH
}

