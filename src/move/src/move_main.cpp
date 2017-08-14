#include <stdio.h>
#include "def.h"
#include "comlib/ulog.h"
#include "motor2local.h"
#include "local2motor.h"
#include "movecenter.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

MOVE_INFO* g_pMi = NULL;

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODENAME, ros::init_options::NoRosout);
	ULogger::setType(ULogger::kTypeConsole, NULL, NODENAME);
	UDEBUG("move init.");
	g_pMi = (MOVE_INFO*)malloc(sizeof(MOVE_INFO));
	memset(g_pMi, 0, sizeof(MOVE_INFO));
	g_pMi->mtol = new MotorToLocal();
	g_pMi->ltom = new LocalToMotor();
	MoveCenter::getInstance(g_pMi);
	SerialInit ser_init(E_MOTORBOARD, NODENAME, g_pMi->mtol, g_pMi);
	serial_init(&ser_init);
	g_pMi->si = ser_init;
    while(ros::ok())
    {
    	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    	spinner.spin();
    }
    MoveCenter::freeInstance();
}
