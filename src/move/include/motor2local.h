#ifndef __MOTOR_TO_LOCAL_H__
#define __MOTOR_TO_LOCAL_H__

#include "comlib/orrbase.h"
#include "def.h"

class MotorToLocal : public OrrBase
{
public:
	MotorToLocal();
	~MotorToLocal();
private:
	int32_t feedbackSpecifySpeedDistanceComplete(const common_msgs::msgdata& msg, void* pData);
};

#endif
