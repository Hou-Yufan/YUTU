/**
  ******************************************************************************
  * @file           : Flash.h
  * @brief          : 内部Flash读写
  ******************************************************************************
  * @attention		
  *
	* @Logs:
	* Date           Author       Notes
	* 2022-09-04     李树益      第一个版本
  ******************************************************************************
  */
#ifndef _FLASH_H_
#define _FLASH_H_

#include "main.h"

#define USER_FLASH_ADDRESS ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */
#define USER_FLASH_SECTOR	11U

void FLASH_Read(uint32_t *buf, uint16_t len);
void FLASH_Write(uint32_t *buf, uint32_t len);

#endif

