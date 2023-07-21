/**
  ******************************************************************************
  * @file           : WorkQueue.c/WorkQueue.h
  * @brief          : 工作队列的实现
  ******************************************************************************
  * @attention		
  *
  * @Logs:
  * Date			Author       Notes
  * 2022-01-28		李树益      第一个版本
  * 2022-03-24		李树益      整理代码，移除RT-Thread依赖
  * 2022-04-09		李树益		修复存在的BUG，新增快速移植接口
  ******************************************************************************
  */
	
#include "WorkQueue.h"
#include <stdlib.h>



/*默认的工作队列*/
struct work_queue WorkQueue = {
    .work_list = {
#if WORK_NAME_LEN > 0
        .name = "Run",
#endif
        .press = 0,
        .press_max = 0,
        .state = 0,
        .work_data = 0,
        .func = NULL,
        .next = NULL,
        .last = NULL,
		.this_work_queue = &WorkQueue,
    },
    .sleep_list = {
#if WORK_NAME_LEN > 0
        .name = "Sleep",
#endif
        .press = 0,
        .press_max = 0,
        .state = 0,
        .work_data = 0,
        .func = NULL,
        .next = NULL,
        .last = NULL,
		.this_work_queue = &WorkQueue,
    },
    .run_end = &WorkQueue.work_list,
    .sleep_end = &WorkQueue.sleep_list,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		移植请按一下函数介绍实现相应功能
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 还原中断状态
 *
 * @param irq_flag	复原成什么状态
 *
 * @return 无
 */
void resize_irq(int irq_flag)
{

}

/**
 * @brief 关闭中断
 *
 * @return 之前的中断状态
 */
int disable_irq()
{
	return 0;
}

#if WORK_IN_THREAD
/**
 * @brief 安装工作队列到线程中
 *
 * @param work_queue	安装的工作队列（获取工作队列所在的线程）
 *
 * @return 无
 */
void work_queue_install(struct work_queue *work_queue)
{
	//请在这里获取当前线程，并把它保存到work_queue->this_thread中
}

/**
 * @brief 唤醒工作队列
 *
 * @param work_queue	需要唤醒的工作队列（线程）
 *
 * @return 无
 */
void work_queue_wake_up(struct work_queue *work_queue)
{
	if (work_queue->this_thread == NULL)
		return;
	//请在这里唤醒work_queue->this_thread
}

/**
 * @brief 工作队列休眠
 *
 * @param work_queue	需要休眠的工作队列（线程）
 *
 * @return 无
 */
void work_queue_sleep(struct work_queue *work_queue)
{
	if (work_queue->this_thread == NULL)
		return;
	//请在这里使work_queue->this_thread休眠
}
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		移植完毕
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * @brief 创建工作项
 *
 * @param name			工作项名称
 * @param func			工作函数
 * @param work_data		工作项私有参数
 * @param press_max		工作压力忍耐度
 *
 * @return 工作项
 */
#if WORK_NAME_LEN > 0
struct work_item * work_creat(char name[WORK_NAME_LEN], void (*func)(int parameter), int work_data, unsigned char press_max)
#else
struct work_item * work_creat(void (*func)(int parameter), int work_data, unsigned char press_max)
#endif
{
    struct work_item *work = malloc(sizeof(struct work_item));
    work->func = func;
    work->work_data = work_data;
    work->press = 0;
    work->press_max = press_max;
#if WORK_NAME_LEN > 0
    unsigned int i;
    for(i = 0; i < WORK_NAME_LEN; i++)
    {
        work->name[i] = name[i];
    }
#endif
		return work;
}

/**
 * @brief 添加工作项到工作队列
 *
 * @param work_queue		工作队列
 * @param work_item			工作项
 *
 * @return 成功：0，失败：1
 */
int work_add(struct work_queue *work_queue, struct work_item *work_item)
{
	int irq_flag;
    if(	work_queue == NULL || \
		work_item == NULL || \
		work_item->last != NULL || \
		work_item-> next != NULL || \
		work_item->state != 0)
	{
		return 1;
	}
	irq_flag = disable_irq();
	work_item->last  = work_queue->sleep_end;
    work_queue->sleep_end->next = work_item;
    work_queue->sleep_end = work_item;
	work_item->this_work_queue = work_queue;
	work_item->state = WORK_ITEM_SLEEP;
	work_queue->work_num ++;
	resize_irq(irq_flag);
	return 0;
}


/**
 * @brief 删除工作项
 *
 * @param work_item			工作项
 *
 * @return 无
 */
void work_del(struct work_item *work_item)
{
	int irq_flag;
    if(work_item == NULL)
        return;
	irq_flag = disable_irq();
    if(work_item->last != NULL)
    {
        work_item->last->next = work_item->next;
    }
    if(work_item->next != NULL)
    {
        work_item->next->last = work_item->last;
    }
		else
		{
			if(work_item->this_work_queue != NULL)
			{
				if(work_item == ((struct work_queue *)(work_item->this_work_queue))->run_end)
				{
					((struct work_queue *)(work_item->this_work_queue))->run_end = work_item->next->last;
				}
				if(work_item == ((struct work_queue *)(work_item->this_work_queue))->sleep_end)
				{
					((struct work_queue *)(work_item->this_work_queue))->sleep_end = work_item->next->last;
				}
			}
		}
	if(work_item->this_work_queue != NULL)
	{
		((struct work_queue *)(work_item->this_work_queue))->work_num --;
	}
	resize_irq(irq_flag);
    free(work_item);
}

/**
 * @brief 准备处理工作项
 *
 * @param work_item		工作项
 * @param p				允许提前工作
 *
 * @return 成功：0，提前处理：1，其它：2
 */
int work_start(struct work_item *work_item, char p)
{
	int irq_flag;
	if(work_item == NULL || \
		 work_item->state == 0 || \
		 work_item->last == NULL || \
		 work_item->func == NULL)
		return 2;

	if(work_item->state == WORK_ITEM_WAIT)		//判断工作是否在等待处理
	{
		work_item->press ++;
		if(work_item->press > work_item->press_max && p == 1)	//判断工作压力是否超出
		{
			irq_flag = disable_irq();
			//从等待处理列表移除
			work_item->last->next = work_item->next;
			if(work_item->next == NULL)
			{
				((struct work_queue *)(work_item->this_work_queue))->run_end = work_item->last;
			}
			else
			{
				work_item->next->last = work_item->last;
				((struct work_queue *)(work_item->this_work_queue))->run_end = work_item->next;
			}
			work_item->next = NULL;
			//改变状态为处理中
			work_item->state = WORK_ITEM_RUNING;
			work_item->press = 0;
			//添加到无工作列表中
			((struct work_queue *)(work_item->this_work_queue))->sleep_end->next = work_item;
			work_item->last = ((struct work_queue *)(work_item->this_work_queue))->sleep_end;
			((struct work_queue *)(work_item->this_work_queue))->sleep_end = work_item;
			resize_irq(irq_flag);
			//处理工作
			work_item->func(work_item->work_data);
			work_item->state = WORK_ITEM_SLEEP;
			return 1;
		}
		return 2;
	}
	irq_flag = disable_irq();
	//从休眠工作列表中移除
	work_item->last->next = work_item->next;
	if(work_item->next == NULL)
	{
		((struct work_queue *)(work_item->this_work_queue))->sleep_end = work_item->last;
	}
	else
	{
		work_item->next->last = work_item->last;
		((struct work_queue *)(work_item->this_work_queue))->sleep_end = work_item->next;
	}
	work_item->next = NULL;
	//更改状态为待处理
	work_item->state = WORK_ITEM_WAIT;
	work_item->press = 1;
	//添加到等待处理列表
	((struct work_queue *)(work_item->this_work_queue))->run_end->next = work_item;
	work_item->last = ((struct work_queue *)(work_item->this_work_queue))->run_end;
	((struct work_queue *)(work_item->this_work_queue))->run_end = work_item;
	resize_irq(irq_flag);
#if WORK_IN_THREAD
	work_queue_wake_up(work_item->this_work_queue);
#endif

	return 0;
}

/**
 * @brief 处理工作队列中的工作项
 *
 * @param work_queue 工作队列
 *
 * @return 无
 */
void work_schudule(struct work_queue *work_queue)
{
	struct work_item *run_item;
	int irq_flag;
	work_queue->run_end = &work_queue->work_list;
	for(run_item = &work_queue->work_list; run_item != NULL; run_item = run_item->next)
	{
		
		irq_flag = disable_irq();
		if(run_item->func == NULL || run_item->state != WORK_ITEM_WAIT)
		{
			resize_irq(irq_flag);
			continue;
		}
		//改变状态为处理中
		run_item->state = WORK_ITEM_RUNING;
		run_item->press = 0;
		//从等待处理列表移除
		run_item->last->next = run_item->next;
		run_item->next->last = run_item->last;
		run_item->next = NULL;
		//添加到无工作列表中
		work_queue->sleep_end->next = run_item;
		run_item->last = work_queue->sleep_end;
		work_queue->sleep_end = run_item;
		resize_irq(irq_flag);
		//处理工作
		run_item->func(run_item->work_data);
		run_item->state = WORK_ITEM_SLEEP;
	}

#if WORK_IN_THREAD
	work_queue_sleep(work_queue);
#endif
}

#if WORK_NAME_LEN > 0
void show_work_queue(struct work_queue *work_queue)
{
	struct work_item *run_item;
	printf("========== Wait =========\n");
	printf("[name]\t[press]\n");
	for(run_item = &work_queue->work_list; run_item != NULL; run_item = run_item->next)
	{
		printf("  %s\t  %d\n",run_item->name,run_item->press);
	}
	printf("Run list End = %s\n",work_queue->run_end->name);
	printf("========== Sleep =========\n");
	printf("[name]\t[press]\n");
	for(run_item = &work_queue->sleep_list; run_item != NULL; run_item = run_item->next)
	{
		printf("  %s\t  %d\n",run_item->name,run_item->press);
	}
	printf("Sleep list End = %s\n",work_queue->sleep_end->name);
}
#endif
