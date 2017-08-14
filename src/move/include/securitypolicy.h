#ifndef __SECURITY_POLICY_H__
#define __SECURITY_POLICY_H__

#include "common_msgs/request_wheels_motion.h"

class SecurityPolicy
{
public:
	static bool requestWheelsMotion(common_msgs::request_wheels_motion::Request& req, common_msgs::request_wheels_motion::Response& resp);
};

#endif
