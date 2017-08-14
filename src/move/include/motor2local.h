#ifndef __MOTOR_TO_LOCAL_H__
#define __MOTOR_TO_LOCAL_H__

#include "comlib/orrbase.h"
#include "def.h"

class MotorToLocal : public OrrBase
{
public:
	MotorToLocal();
	~MotorToLocal();
	virtual bool parseCommand(const common_msgs::msgdata& msg, void* data);
private:
	int32_t feedbackVersionNumberSn(const common_msgs::msgdata& msg, void* pData);
	int32_t feedbackAbnormityEvent(const common_msgs::msgdata& msg, void* pData);
	int32_t feedbackOrderCompleteResponse(const common_msgs::msgdata& msg, void* pData);
	int32_t feedbackOrderResponse(const common_msgs::msgdata& msg, void* pData);
	int32_t feedbackOdometerNumber(const common_msgs::msgdata& msg, void* pData);
	int32_t feedbackDeadReckonig(const common_msgs::msgdata& msg, void* pData);
	bool	orderIndexRight(uint8_t index);
private:
	uint8_t	m_uOrderIndex;
};

#endif
