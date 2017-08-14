#include "asynctimer.h"
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include "ulog.h"

void fixed_task(const boost::system::error_code& e,
           boost::asio::deadline_timer* t,
		   AsyncTimer* a)
{
	if(a->conti_cnt != 0)
	{
		a->time_task();
		t->expires_at(t->expires_at()+ boost::posix_time::millisec(a->milli_sec));
		t->async_wait(boost::bind(fixed_task,boost::asio::placeholders::error,t,a));
		--a->conti_cnt;
	}
}

void thread_start(AsyncTimer* at)
{
	boost::asio::io_service ios;
	if(at->imme_start)
		at->time_task();
	boost::asio::deadline_timer t(ios, boost::posix_time::millisec(at->milli_sec));
	t.async_wait(boost::bind(fixed_task,boost::asio::placeholders::error,&t,at));
	ios.run();
}

void timer_init(AsyncTimer* at)
{
   UDEBUG("asynchronous timer init.");
   boost::thread(boost::bind(thread_start, at));
}
