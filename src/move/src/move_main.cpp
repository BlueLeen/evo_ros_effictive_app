#include <stdio.h>
#include "def.h"
#include "comlib/ulog.h"
#include "motor2local.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "comlib/serialinit.h"

boost::mutex g_mtMutex;

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODENAME, ros::init_options::NoRosout);
	ULogger::setType(ULogger::kTypeConsole, NULL, NODENAME);
	UINFO("move init.");
	MotorToLocal* mtol = new MotorToLocal();
	SerialInit ser_init(E_MOTORBOARD, NODENAME, mtol);
	serial_init(&ser_init);
    while(ros::ok())
    {
    	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    	spinner.spin();
    }
}
