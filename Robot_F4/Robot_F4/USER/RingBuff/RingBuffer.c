/**
  ******************************************************************************
  * @file           : RingBuffer.c
  * @brief          : 环形缓冲
  ******************************************************************************
  * @attention		
  *
  * @Logs:
  * Date           Author       Notes
  * 2022-01-22	   李树益      第一个版本
  ******************************************************************************
  */

#include "RingBuffer.h"
#include "string.h"

/**
 * @brief 创建环形缓冲
 *
 * @param fifo		环形缓冲
 * @param buffer	缓冲区
 * @param size		缓冲区大小
 *
 * @return none
 */
void ring_buffer_creat(RingBuffer *fifo, unsigned char *buffer, unsigned short size)
{
	if(fifo == NULL || buffer == NULL)	
		return;
	
	fifo->buffer = buffer;
	fifo->buffer_max = size;
	fifo->wrtie_index = 0;
	fifo->read_index = 0;
	fifo->data_len = 0;
}

/**
 * @brief 写入数据
 *
 * @param fifo		环形缓冲
 * @param data		写入的数据
 * @param size		写入量
 *
 * @return 最终写入的数据量
 */
unsigned short ring_buffer_write(RingBuffer *fifo, unsigned char *data, unsigned short size)
{
	unsigned short len;
	unsigned char *pdata;				//指向data的指针
	
	if(fifo == NULL || data == NULL)	//指针为空，不能写入
		return 0;

	if(size > fifo->buffer_max)	//能写爆环形缓冲
	{
		pdata = data + (size - fifo->buffer_max);
		size = fifo->buffer_max;			//最后能够写入的部分
	}
	else
	{
		pdata = data;
	}
	
	len = fifo->buffer_max - fifo->data_len;	//计算环形缓冲区的空闲长度
	if(len < size)	//会覆盖之前的内容
	{
		fifo->data_len = fifo->buffer_max;	//存满了
		fifo->read_index += size - len;		//要覆盖部分之前的内容
		if( fifo->read_index > fifo->buffer_max)	//越界了，往回走
			fifo->read_index -= fifo->buffer_max;
	}
	else
	{
		fifo->data_len += size;
	}
	
	len = fifo->buffer_max - fifo->wrtie_index;	//计算距离数组末尾还有多少
	if(len < size)	//需要拆分成两次写
	{
		//存入前半部分
		memcpy(fifo->buffer + fifo->wrtie_index, pdata, len);
		//存入后半部分
		fifo->wrtie_index = size - len;	//计算后半部分数据的长度
		memcpy(fifo->buffer, pdata + len, fifo->wrtie_index);
	}
	else
	{
		memcpy(fifo->buffer + fifo->wrtie_index, pdata, size);
		fifo->wrtie_index += size;
		if(fifo->wrtie_index >= fifo->buffer_max)
			fifo->wrtie_index -= fifo->buffer_max;
	}
	
	return size;
}

/**
 * @brief 读取数据
 *
 * @param fifo		环形缓冲
 * @param data		读取的数据存储位置
 * @param size		读取量
 *
 * @return 最终读取到的数据量
 */
unsigned short ring_buffer_read(RingBuffer *fifo, unsigned char *data, unsigned short size)
{
	unsigned short len;
	
	if(fifo == NULL || data == NULL)	//指针为空，不能写入
		return 0;
	
	if(size > fifo->data_len)	//缓冲区的数据小于需要读取的数据
	{
		size = fifo->data_len;			//最后能够的读出的部分
	}
	
	len = fifo->buffer_max - fifo->read_index;	//计算距离数组末尾还有多少
	if(len < size)	//需要拆成两次读
	{
		//存入前半部分
		memcpy(data, fifo->buffer + fifo->read_index, len);
		//存入后半部分
		fifo->read_index = size - len;	//计算后半部分数据的长度
		memcpy(data + len, fifo->buffer, fifo->read_index);
	}
	else
	{
		memcpy(data, fifo->buffer + fifo->read_index, size);
		fifo->read_index += size;
		if(fifo->read_index >= fifo->buffer_max)
			fifo->read_index -= fifo->buffer_max;
	}
	
	fifo->data_len -= size;
	
	return size;
}


