#ifndef __MOTION_ORRBASE_H__
#define __MOTION_ORRBASE_H__

#include "common_msgs/msgdata.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <map>

class OrrBase
{
private:
    typedef boost::function<int32_t(const common_msgs::msgdata&, void*)> event_func;
    struct event_info
    {
        event_func func_;
    };
    typedef std::map<int32_t, event_info> M_event_info;
    M_event_info    cmdEvent;
protected:
    virtual int32_t  add_cmd_function(int32_t cmd ,const event_func& cmd_function);
    virtual bool del_cmd_function(int cmd);
public:
    virtual ~OrrBase();
    virtual bool parseCommand(const common_msgs::msgdata& msg, void* data);
};

#endif
