/**
  ******************************************************************************
  * @file           : RingBuffer.h
  * @brief          : 环形缓冲
  ******************************************************************************
  * @attention		
  *
  * @Logs:
  * Date           Author       Notes
  * 2022-01-22	   李树益      第一个版本
  ******************************************************************************
  */
	
#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

typedef struct
{
	unsigned short wrtie_index;		//写位置
	unsigned short read_index;		//读位置
	unsigned short data_len;			//缓冲区数据长度	
	
	unsigned short buffer_max;		//缓冲区大小	最大64K byte
	unsigned char *buffer;				//数据
}RingBuffer;

void ring_buffer_creat(RingBuffer *fifo, unsigned char *buffer, unsigned short size);
unsigned short ring_buffer_write(RingBuffer *fifo, unsigned char *data, unsigned short size);
unsigned short ring_buffer_read(RingBuffer *fifo, unsigned char *data, unsigned short size);

#endif

