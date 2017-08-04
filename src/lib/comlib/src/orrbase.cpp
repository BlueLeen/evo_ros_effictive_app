#include "orrbase.h"
#include "ulog.h"
#include <boost/thread.hpp>

OrrBase::~OrrBase()
{

}

int32_t OrrBase::add_cmd_function(int32_t cmd ,const event_func& cmd_function)
{
    event_info info;
    info.func_ = cmd_function;
    {
        bool b = cmdEvent.insert(std::make_pair(cmd, info)).second;
        if (!b){
            UINFO("Tried to add cmd function [%x]", cmd);
            return false;
        }
    }
    return true;
}

bool OrrBase::del_cmd_function(int cmd)
{
    M_event_info::iterator it = cmdEvent.find(cmd);
    if (it != cmdEvent.end())
    {
    	cmdEvent.erase(it);
        return true;
    }
    return false;
}

bool OrrBase::parseCommand(const common_msgs::msgdata& msg, void* data)
{
	M_event_info::iterator it = cmdEvent.find(msg.cmd);
	if(it != cmdEvent.end())
	{
		event_info &info = it->second;
		int32_t result = info.func_(msg, data);
		if(result == 0)
		{
			return true;
		}
	}
	else
	{
		UERROR("Parse Command: cmd  [%x] which does not exist\n", msg.cmd);
	}
	return false;
}

