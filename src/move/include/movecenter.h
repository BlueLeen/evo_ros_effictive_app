#ifndef __MOTOR_CENTER_H__
#define __MOTOR_CENTER_H__

#include <ros/ros.h>
#include "def.h"
#include "common_msgs/msgdata.h"
#include "common_msgs/set_dead_reckon_info.h"
#include "common_msgs/get_motion_idsn.h"
#include "common_msgs/set_wheel_differential_motion.h"
#include "common_msgs/set_wheel_fixed_motion.h"
#include "common_msgs/single_status.h"

class MoveCenter
{
public:
    static MoveCenter* getInstance(MOVE_INFO* pmi = NULL);
    static void freeInstance();
    int sendData(common_msgs::msgdata& msg);
    int wheelMove(int cmd, WHEEL_PARAM& wheel_param);
private:
	MoveCenter(MOVE_INFO* pmi);
	~MoveCenter();
	void setServiceServerInit();
	void setPublisherInit();
	void setSubscriberInit();
	void startMoveTimer();
	void shutdownServiceServer();
	void shutdownPublisher();
	void shutdownSubscriber();
	void stopMoveTimer();
	void serialCmdToMove(const common_msgs::msgdata& msg);
	bool setDeadReckonInfo(common_msgs::set_dead_reckon_info::Request& req, common_msgs::set_dead_reckon_info::Response& resp);
	bool getMotionIdSn(common_msgs::get_motion_idsn::Request& req, common_msgs::get_motion_idsn::Response& resp);
    void setWheelDifferentialMove(common_msgs::set_wheel_differential_motion wheel_motion);
    void setWheelFixedMove(common_msgs::set_wheel_fixed_motion wheel_motion);
    void setWheelStop(common_msgs::single_status stop_param);
    void setMotionBarrierEnable(common_msgs::single_status status);
    int  setWheelMove(WHEEL_PARAM* wheel_param);
    bool isMoveStop(WHEEL_PARAM* wheel_param);
    void timerProc(const ros::WallTimerEvent& event);
private:
	static MoveCenter*	m_pMc;
	ros::NodeHandle m_nh;
	MOVE_INFO* 	m_pMi;
	ros::V_ServiceServer m_vServer;
	ros::V_Publisher     m_vPub;
	ros::V_Subscriber    m_vSub;
	ros::WallTimer       m_wtTimer;

    const static uint8_t PUBSIZE = 20;
    const static uint8_t SUBSIZE = 20;
    const static uint8_t SRVSIZE = 20;
};

#endif
