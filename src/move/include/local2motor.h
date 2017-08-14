#ifndef __LOCAL_TO_MOTOR_H__
#define __LOCAL_TO_MOTOR_H__

#include "comlib/orsbase.h"

class LocalToMotor : public OrsBase
{
public:
	LocalToMotor();
	~LocalToMotor();
private:
	int32_t specifyWheelMove(void* pData);
	int32_t specifyWheelMove2(void* pData);
	int32_t calibrationCorrect(void* pData);
};

#endif
