#include "asynctimer.h"
#include <boost/thread.hpp>
#include <ros/ros.h>
#include "ulog.h"

void timer_task(const ros::WallTimerEvent& event)
{
	//dealPros();
}

void timer_init(AsyncTimer* at)
{
	UDEBUG("ros asynchronous timer init.");
//	ros::NodeHandle* nh = (ros::NodeHandle*)at->ros_nh;
//	bool oneshot = (at->conti_cnt == 1) ? true : false;
//	at->ros_wt = new ros::WallTimer();
//	*((ros::WallTimer*)at->ros_wt) = nh->createWallTimer(ros::WallDuration(((double)at->milli_sec)/1000), timer_task, oneshot, at->imme_start);
}
