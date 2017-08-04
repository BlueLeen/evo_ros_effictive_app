#include "orsbase.h"
#include "ulog.h"
#include <boost/thread.hpp>

OrsBase::~OrsBase()
{

}

int32_t OrsBase::add_cmd_function(int32_t cmd ,const event_func& cmd_function)
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

bool OrsBase::del_cmd_function(int cmd)
{
    M_event_info::iterator it = cmdEvent.find(cmd);
    if (it != cmdEvent.end())
    {
    	cmdEvent.erase(it);
        return true;
    }
    return false;
}

bool OrsBase::throwCommand(uint32_t cmd, void* data)
{
	M_event_info::iterator it = cmdEvent.find(cmd);
	if(it != cmdEvent.end())
	{
		event_info &info = it->second;
		int32_t result = info.func_(data);
		if(result == 0)
		{
			return true;
		}
	}
	else
	{
		UERROR("Throw Command: cmd  [%x] which does not exist\n", cmd);
	}
	return false;
}
