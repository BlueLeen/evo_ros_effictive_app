#include "motor2local.h"

MotorToLocal::MotorToLocal()
{
	add_cmd_function(M2LCMD1, boost::bind(&MotorToLocal::feedbackSpecifySpeedDistanceComplete, this, _1, _2));
}

MotorToLocal::~MotorToLocal()
{
	del_cmd_function(M2LCMD1);
}

int32_t MotorToLocal::feedbackSpecifySpeedDistanceComplete(const common_msgs::msgdata& msg, void* pData)
{
	return 0;
}
