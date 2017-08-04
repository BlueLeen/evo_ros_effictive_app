#include <stdio.h>
#include "comlib/ulog.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "comlib/serialinit.h"

boost::mutex g_mtMutex;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move", ros::init_options::NoRosout);
	ULogger::setType(ULogger::kTypeConsole, NULL, "core_serial");
	UINFO("move init.");
	SerialInit ser_init(E_MOTORBOARD, 'm');
	serial_init(&ser_init);
    while(ros::ok())
    {
    	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    	spinner.spin();
    }
}
