#include <stdio.h>
#include "def.h"
#include "comlib/ulog.h"
#include "motor2local.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "comlib/serialinit.h"

MOVE_INFO* g_pMi = NULL;
boost::mutex g_mtMutex;

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODENAME, ros::init_options::NoRosout);
	ULogger::setType(ULogger::kTypeConsole, NULL, NODENAME);
	UDEBUG("move init.");
	g_pMi = new MOVE_INFO();
	MotorToLocal* mtol = new MotorToLocal();
	g_pMi->mtol = mtol;
	SerialInit ser_init(E_MOTORBOARD, NODENAME, mtol, g_pMi);
	serial_init(&ser_init);
    while(ros::ok())
    {
    	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    	spinner.spin();
    }
}
