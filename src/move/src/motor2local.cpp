#include "motor2local.h"
#include <comlib/ulog.h>

MotorToLocal::MotorToLocal()
{
	add_cmd_function(M2LCMD1, boost::bind(&MotorToLocal::feedbackVersionNumberSn, this, _1, _2));
	add_cmd_function(M2LCMD10, boost::bind(&MotorToLocal::feedbackAbnormityEvent, this, _1, _2));
}

MotorToLocal::~MotorToLocal()
{
	del_cmd_function(M2LCMD1);
	del_cmd_function(M2LCMD10);
}

int32_t MotorToLocal::feedbackVersionNumberSn(const common_msgs::msgdata& msg, void* pData)
{
	if(msg.inlen == 0x21)
	{
		MOVE_INFO* pMi = (MOVE_INFO*)pData;
		memcpy(&pMi->ver, &msg.input[0], msg.inlen);
		UDEBUG("[%s]feedback version number and sn complete.", NODENAME);
	}
	else
	{
		UERROR("[%s]feedback version number and sn length fault!", NODENAME);
		return -1;
	}
	return 0;
}

int32_t MotorToLocal::feedbackAbnormityEvent(const common_msgs::msgdata& msg, void* pData)
{
	if(msg.inlen == 0x3)
	{
		MOTOR_ABNOMITYEVENT	abet;
		memcpy(&abet, &msg.input[0], msg.inlen);
		UDEBUG("[%s]feedback abnormity and event complete.", NODENAME);
	}
	else
	{
		UERROR("[%s]feedback abnormity and event length fault!", NODENAME);
		return -1;
	}
	return 0;
}
