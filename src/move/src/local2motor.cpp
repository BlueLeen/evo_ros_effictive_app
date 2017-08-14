#include "local2motor.h"
#include "def.h"
#include "common_msgs/msgdata.h"
#include "movecenter.h"
#include "comlib/endianconvert.h"

LocalToMotor::LocalToMotor()
{
	add_cmd_function(L2MCMD1, boost::bind(&LocalToMotor::specifyWheelMove, this, _1));
	add_cmd_function(L2MCMD2, boost::bind(&LocalToMotor::specifyWheelMove2, this, _1));
	add_cmd_function(L2MCMD3, boost::bind(&LocalToMotor::calibrationCorrect, this, _1));
}

LocalToMotor::~LocalToMotor()
{
	del_cmd_function(L2MCMD1);
	del_cmd_function(L2MCMD2);
	del_cmd_function(L2MCMD3);
}

int32_t LocalToMotor::specifyWheelMove(void* pData)
{
	WHEEL_PARAM* wheel_param = (WHEEL_PARAM*)pData;
	common_msgs::msgdata msg;
	msg.cmd = L2MCMD1;
	msg.inlen = 0x7;
	msg.input.resize(msg.inlen);
	msg.input[0] = wheel_param->dir_left;
	litter_big_convert(&msg.input[1], (uint8_t*)&wheel_param->speed_left, sizeof(uint16_t));
	msg.input[3] = wheel_param->dir_right;
	litter_big_convert(&msg.input[4], (uint8_t*)&wheel_param->speed_right, sizeof(uint16_t));
	msg.input[6] = wheel_param->is_new;
	return MoveCenter::getInstance()->sendData(msg);
}

int32_t LocalToMotor::specifyWheelMove2(void* pData)
{
	MOVE_PARAM* move_param = (MOVE_PARAM*)pData;
	common_msgs::msgdata msg;
	msg.cmd = L2MCMD2;
	msg.inlen = 0x11;
	msg.input.resize(msg.inlen);
	litter_big_convert(&msg.input[0], (uint8_t*)&move_param->speed, sizeof(float));
	litter_big_convert(&msg.input[4], (uint8_t*)&move_param->rad, sizeof(float));
	litter_big_convert(&msg.input[8], (uint8_t*)&move_param->line_dis, sizeof(float));
	litter_big_convert(&msg.input[12], (uint8_t*)&move_param->angle_dis, sizeof(float));
	msg.input[16] = move_param->is_new;
	return MoveCenter::getInstance()->sendData(msg);
}

int32_t LocalToMotor::calibrationCorrect(void* pData)
{
	CALIBRATION_CORRECT* pMc = (CALIBRATION_CORRECT*)pData;
	common_msgs::msgdata msg;
	msg.cmd = L2MCMD3;
	msg.inlen = 0xc;
	msg.input.resize(msg.inlen);
    litter_big_convert(&msg.input[0], (uint8_t*)&pMc->calibration_x, sizeof(float));
    litter_big_convert(&msg.input[4], (uint8_t*)&pMc->calibration_y, sizeof(float));
    litter_big_convert(&msg.input[8], (uint8_t*)&pMc->calibration_th, sizeof(float));
	return MoveCenter::getInstance()->sendData(msg);
}
