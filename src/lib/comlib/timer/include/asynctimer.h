#ifndef __ASYNC_TIMER_H__
#define __ASYNC_TIMER_H__

#include <stdint.h>
#include <stdlib.h>

typedef int (*task_func)();
typedef struct _async_timer
{
	_async_timer(uint32_t msec, uint32_t cotcnt, task_func tf, bool imstart=true):
		milli_sec(msec),conti_cnt(cotcnt),time_task(tf),imme_start(imstart)
	{

	}
	uint32_t	milli_sec;
	uint32_t 	conti_cnt;
	task_func	time_task;
	bool		imme_start;
}AsyncTimer;

extern void timer_init(AsyncTimer* at);

#endif
