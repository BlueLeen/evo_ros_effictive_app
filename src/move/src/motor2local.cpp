#include "motor2local.h"
#include <comlib/ulog.h>
#include <comlib/endianconvert.h>

MotorToLocal::MotorToLocal():m_uOrderIndex(0)
{
	add_cmd_function(M2LCMD1, boost::bind(&MotorToLocal::feedbackVersionNumberSn, this, _1, _2));
	add_cmd_function(M2LCMD10, boost::bind(&MotorToLocal::feedbackAbnormityEvent, this, _1, _2));
	add_cmd_function(M2LCMDF0, boost::bind(&MotorToLocal::feedbackOrderCompleteResponse, this, _1, _2));
	add_cmd_function(M2LCMDF1, boost::bind(&MotorToLocal::feedbackOrderResponse, this, _1, _2));
	add_cmd_function(M2LCMD2, boost::bind(&MotorToLocal::feedbackOdometerNumber, this, _1, _2));
	add_cmd_function(M2LCMD3, boost::bind(&MotorToLocal::feedbackDeadReckonig, this, _1, _2));
}

MotorToLocal::~MotorToLocal()
{
	del_cmd_function(M2LCMD1);
	del_cmd_function(M2LCMD10);
	del_cmd_function(M2LCMDF0);
	del_cmd_function(M2LCMDF1);
	del_cmd_function(M2LCMD2);
	del_cmd_function(M2LCMD3);
}

bool MotorToLocal::parseCommand(const common_msgs::msgdata& msg, void* data)
{
	orderIndexRight(msg.reserved[0]);
	return OrrBase::parseCommand(msg, data);
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

int32_t MotorToLocal::feedbackOrderCompleteResponse(const common_msgs::msgdata& msg, void* pData)
{
	if(msg.inlen == 0x3)
	{
		//UDEBUG("[%s]feedback order complete response complete.", NODENAME);
		UINFO("[%s]driver order completed response code:%x,number:%x,result:%s.",
				NODENAME, msg.input[0], msg.input[1], msg.input[2]==1?"success":"fail");
	}
	else
	{
		UERROR("[%s]feedback order complete response length fault!", NODENAME);
		return -1;
	}
	return 0;
}

int32_t MotorToLocal::feedbackOrderResponse(const common_msgs::msgdata& msg, void* pData)
{
	if(msg.inlen == 0x1)
	{
		//UDEBUG("[%s]feedback order response complete.", NODENAME);
		UINFO("[%s]driver order response instruction number:%x.",NODENAME, msg.input[0]);
	}
	else
	{
		UERROR("[%s]feedback order response length fault!", NODENAME);
		return -1;
	}
	return 0;
}

int32_t MotorToLocal::feedbackOdometerNumber(const common_msgs::msgdata& msg, void* pData)
{
	if(msg.inlen == 0x8)
	{
		MOVE_INFO*	pMi = (MOVE_INFO*)pData;
	    litter_big_convert((uint8_t*)&pMi->odo.wheel_left_odometer_number, (const uint8_t*)&msg.input[0], 4);
	    litter_big_convert((uint8_t*)&pMi->odo.wheel_right_odometer_number, (const uint8_t*)&msg.input[4], 4);
		pMi->odo.valid_flag = 1;
		UINFO("odometer number left:%d right:%d \n", pMi->odo.wheel_left_odometer_number
			   , pMi->odo.wheel_right_odometer_number);
	}
	else
	{
		UERROR("[%s]feedback odometer number length fault!", NODENAME);
		return -1;
	}
	return 0;
}

int32_t MotorToLocal::feedbackDeadReckonig(const common_msgs::msgdata& msg, void* pData)
{
	if(msg.inlen == 0xc)
	{
		MOVE_INFO*	pMi = (MOVE_INFO*)pData;
	    litter_big_convert((uint8_t*)&pMi->cal.calibration_x, (uint8_t*)&msg.input[0], 4);
	    litter_big_convert((uint8_t*)&pMi->cal.calibration_y, (uint8_t*)&msg.input[4], 4);
	    litter_big_convert((uint8_t*)&pMi->cal.calibration_th, (uint8_t*)&msg.input[8], 4);
	    pMi->cal.calibration_th = pMi->cal.calibration_th * 180 / PI;
	    pMi->cal.valid_flag = 1;
		static int count = 0;
	    if(30 == ++count)
	    {
	        count = 0;
	        UINFO("move read calibration x:%f y:%f th:%f\n", pMi->cal.calibration_x,
	        		pMi->cal.calibration_y, pMi->cal.calibration_th);
	    }
	}
	else
	{
		UERROR("[%s]feedback dead reckonig length fault!", NODENAME);
		return -1;
	}
	return 0;
}

bool MotorToLocal::orderIndexRight(uint8_t index)
{
	if(m_uOrderIndex+1 == index)
	{
		++m_uOrderIndex;
		return true;
	}
	UERROR("[%s]feedback order index fault, index should be:%x, but receive:%x!",
			NODENAME, m_uOrderIndex+1, index);
	m_uOrderIndex = index;
	return false;
}
