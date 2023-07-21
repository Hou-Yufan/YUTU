/**
  ******************************************************************************
  * @file           : WorkQueue.c/WorkQueue.h
  * @brief          : 工作队列的实现
  ******************************************************************************
  * @attention		
  *
	* @Logs:
	* Date           Author       Notes
	* 2022-01-28	 李树益      第一个版本
	* 2022-03-24     李树益      整理代码，移除RT-Thread依赖
	* 2022-04-05     李树益      添加线程唤醒
  ******************************************************************************
  */
	
#ifndef _WORK_QUEUE_H_
#define _WORK_QUEUE_H_

#define WORK_NAME_LEN   0			//如果希望工作项有名称，WORK_NAME_LEN>0
#define WORK_IN_THREAD 	0			//如果工作队列有线程相关的API支持，WORK_IN_THREAD = 1

#define WORK_ITEM_WAIT 	  'W'
#define WORK_ITEM_RUNING 	'R'
#define WORK_ITEM_SLEEP 	'S'

struct work_item					//工作项
{
#if WORK_NAME_LEN > 0
    char name[WORK_NAME_LEN];		    //工作项名称
#endif
    unsigned char press;			      //当前工作压力
    unsigned char press_max;		    //最大压力
    char state;            			    //工作项状态
    int work_data;                  //工作项私有数据
    void (*func)(int parameter);    //工作函数
    struct work_item *next;
    struct work_item *last;
		void *this_work_queue;
};

struct work_queue					//工作队列
{
    struct work_item work_list;		//等待处理工作的列表
    struct work_item sleep_list;	//无工作的列表
    struct work_item *run_end;		//等待处理工作列表的末尾
    struct work_item *sleep_end;	//无工作列表的末尾
		unsigned int work_num;		    //工作队列中的工作项数量
#if WORK_IN_THREAD
    void *this_thread;				    //工作队列所在的线程
#endif
};

extern struct work_queue WorkQueue;
	
#if WORK_NAME_LEN > 0
struct work_item * work_creat(char name[WORK_NAME_LEN], void (*func)(int parameter), int work_data, unsigned char press_max);
#else
struct work_item * work_creat(void (*func)(int parameter), int work_data, unsigned char press_max);
#endif
int work_add(struct work_queue *work_queue, struct work_item *work_item);
void work_del(struct work_item *work_item);
int work_start(struct work_item *work_item, char p);
void work_schudule(struct work_queue *work_queue);

#if WORK_IN_THREAD
void work_queue_install(struct work_queue *work_queue);
#define WORK_QUEUE_INSTALL()	work_queue_install(&WorkQueue)
#endif

#define WORK_INIT(item) 	work_add(&WorkQueue, item)
#define WORK_QUEUE_RUN()	work_schudule(&WorkQueue)

#endif

