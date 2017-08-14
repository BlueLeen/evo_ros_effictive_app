#include "movecenter.h"
#include "def.h"
#include "local2motor.h"
#include "securitypolicy.h"
#include "comlib/ulog.h"
#include "common_msgs/request_wheels_motion.h"

#define             MOTION_SEND_TIMES_MAX               8 //控制板9900ms内未接到上位机持续指令，则停止发送

static int timer_count = 0;
MoveCenter*	MoveCenter::m_pMc = NULL;

MoveCenter::MoveCenter(MOVE_INFO* pmi):m_pMi(pmi)
{
	setServiceServerInit();
	setPublisherInit();
	setSubscriberInit();
	startMoveTimer();
}

MoveCenter::~MoveCenter()
{
	shutdownServiceServer();
	shutdownPublisher();
	shutdownSubscriber();
	stopMoveTimer();
}


MoveCenter* MoveCenter::getInstance(MOVE_INFO* pmi)
{
    if(m_pMc == NULL)
    {
    	m_pMc = new MoveCenter(pmi);
    }
    return m_pMc;
}

void MoveCenter::freeInstance()
{
    if(m_pMc != NULL)
        delete m_pMc;
    m_pMc = NULL;
}

int MoveCenter::sendData(common_msgs::msgdata& msg)
{
	msg.cmd = MAKECMD(LOCALMODULE, MOTORMODULE, (msg.cmd & 0xff));
	m_pMi->si.write_data(msg);
	return 0;
}

int MoveCenter::wheelMove(int cmd, WHEEL_PARAM& wheel_param)
{
	UINFO("[motion]set wheel move");
	uint8_t dirvier_result = 0;
    uint16_t driver_status = 0;
    {
        //boost::mutex::scoped_lock lock(g_mutex);
//        dirvier_result = m_mD.motion_feedbck.driver_result;
//        driver_status = m_mD.motion_feedbck.driver_status;
    }
    if(0 == wheel_param.is_new && 0 == dirvier_result && 0 == driver_status)
    {
        return 0;
    }
    if(1 == wheel_param.is_timer)
    {
    	UDEBUG("[motion]set wheel move start timer");
    	timer_count = 0;
        memcpy(&m_pMi->wp, &wheel_param, sizeof(WHEEL_PARAM));
        m_wtTimer.start();
    }
    if(0 != setWheelMove(&wheel_param))
    {
        return -1;
    }
    return 0;
}

void MoveCenter::setServiceServerInit()
{
	m_vServer.clear();
	m_vServer.resize(SRVSIZE);
	m_vServer[SET_DEAD_RECKON_INFO] = m_nh.advertiseService("set_dead_reckon_info", &MoveCenter::setDeadReckonInfo, this);
	m_vServer[GET_MOTION_ID_SN] = m_nh.advertiseService("get_motion_id_sn", &MoveCenter::getMotionIdSn, this);
}

void MoveCenter::setPublisherInit()
{
	m_vPub.clear();
	m_vPub.resize(PUBSIZE);
	m_vPub[TOPHONE] = m_nh.advertise<common_msgs::msgdata>("serial_cmd_to_phone", 10);
	m_vPub[GET_DEAD_RECKON_INFO] = m_nh.advertise<common_msgs::dead_reckon_info>("get_dead_reckon_info", 10);
}

void MoveCenter::setSubscriberInit()
{
    m_vSub.clear();
    m_vSub.resize(SUBSIZE);
    m_vSub[FROMPAD] = m_nh.subscribe("serial_cmd_to_move", 10, &MoveCenter::serialCmdToMove, this);
    m_vSub[SET_WHEEL_DIFFERENTIAL_MOVE] = m_nh.subscribe("set_wheel_differential_move", 10, &MoveCenter::setWheelDifferentialMove, this);
    m_vSub[SET_WHEEL_FIXED_MOVE] = m_nh.subscribe("set_wheel_fixed_move", 10, &MoveCenter::setWheelFixedMove, this);
    m_vSub[SET_WHEEL_STOP] = m_nh.subscribe("set_wheel_stop", 10, &MoveCenter::setWheelStop, this);
    m_vSub[SET_MOTION_BARRIER_ENABLE] = m_nh.subscribe("set_motion_barrier_enable", 10, &MoveCenter::setMotionBarrierEnable, this);
}

void MoveCenter::startMoveTimer()
{
	m_wtTimer = m_nh.createWallTimer(ros::WallDuration(0.15/*0.03*/), &MoveCenter::timerProc, this, false, false);
}

void MoveCenter::shutdownServiceServer()
{
    for(int i = 0; i < (int)m_vServer.size(); i++)
    {
    	m_vServer.at(i).shutdown();
    }
    m_vServer.clear();
}

void MoveCenter::shutdownPublisher()
{
    for(int i = 0; i < (int)m_vSub.size(); i++)
    {
    	m_vSub.at(i).shutdown();
    }
    m_vSub.clear();
}

void MoveCenter::shutdownSubscriber()
{
    for(int i = 0; i < (int)m_vPub.size(); i++)
    {
    	m_vPub.at(i).shutdown();
    }
    m_vPub.clear();
}

void MoveCenter::stopMoveTimer()
{
	m_wtTimer.stop();
}

void MoveCenter::serialCmdToMove(const common_msgs::msgdata& msg)
{
	UINFO("[%s]serial command to motion:0x%08x", NODENAME, msg.cmd);
	//msg.cmd = msg.cmd & 0xffff;
	//g_pPtol->parseCommand(msg, &m_mD);
}

bool MoveCenter::setDeadReckonInfo(common_msgs::set_dead_reckon_info::Request& req, common_msgs::set_dead_reckon_info::Response& resp)
{
	CALIBRATION_CORRECT cal_cor;
	cal_cor.calibration_x = req.dead_reckon.calibration_x;
	cal_cor.calibration_y = req.dead_reckon.calibration_y;
	cal_cor.calibration_th = req.dead_reckon.calibration_th;
	UDEBUG("[%s]set dead reckon info, x:%f,y:%f,th:%f",
			cal_cor.calibration_x, cal_cor.calibration_y, cal_cor.calibration_th);
    return ((LocalToMotor*)(m_pMi->ltom))->throwCommand(L2MCMD3, &cal_cor);
}

bool MoveCenter::getMotionIdSn(common_msgs::get_motion_idsn::Request& req, common_msgs::get_motion_idsn::Response& resp)
{
	memcpy(&resp.id_sn, &m_pMi->ver, sizeof(m_pMi->ver));
	if(((uint8_t*)&m_pMi->ver)[0] == 0x0)
		resp.valid = false;
	else
		resp.valid = true;
	UDEBUG("[%s]get motion id_sn, valid:%s", NODENAME, (resp.valid==true)?"true":"false");
	return true;
}

void MoveCenter::setWheelDifferentialMove(common_msgs::set_wheel_differential_motion wheel_motion)
{
}

void MoveCenter::setWheelFixedMove(common_msgs::set_wheel_fixed_motion wheel_motion)
{
}

void MoveCenter::setWheelStop(common_msgs::single_status stop_param)
{
}

void MoveCenter::setMotionBarrierEnable(common_msgs::single_status status)
{
}

int MoveCenter::setWheelMove(WHEEL_PARAM* wheel_param)
{
    //停止运动
    if(isMoveStop(wheel_param))
    {
    	m_wtTimer.stop();
        return ((LocalToMotor*)m_pMi->ltom)->throwCommand(L2MCMD1, wheel_param);
    }
    common_msgs::request_wheels_motion wheels_motion;
    memcpy(&wheels_motion.request.ultra_enable, &m_pMi->sor.ultra_enable, 2);
	wheels_motion.request.is_new = wheel_param->is_new;
    if(SecurityPolicy::requestWheelsMotion(wheels_motion.request, wheels_motion.response))
    {
        if(1 == wheels_motion.response.b_allow)
        {
            if(0 == ((LocalToMotor*)m_pMi->ltom)->throwCommand(L2MCMD1, wheel_param))
            {
                return 0;
            }
        }
//        else if(2 == wheels_motion.response.b_allow)
//        {
//            //wheel_param->is_new = 1;
//			WHEEL_PARAM wheel_param_temp;
//            memcpy(&wheel_param_temp, wheel_param, sizeof(WHEEL_PARAM));
//            wheel_param_temp.is_new = 1;
//            if(0 == ((LocalToMotor*)m_pMi->ltom)->throwCommand(L2MCMD1, wheel_param))
//            {
//                return 0;
//            }
//        }
        else
        {
            WHEEL_PARAM wheel_param_temp;
            wheel_param_temp.speed_left = wheel_param_temp.speed_right = 0;
            wheel_param_temp.dir_left = wheel_param_temp.dir_right = MOTION_DIRECT_STOP;
            wheel_param_temp.is_new = 1;
            setWheelMove(&wheel_param_temp);
            m_wtTimer.stop();
            return 0;
        }
    }
    return -1;
}

bool MoveCenter::isMoveStop(WHEEL_PARAM* wheel_param)
{
    //方向停止
    if(0 == wheel_param->speed_left && 0 == wheel_param->speed_right)
    {
        return true;
    }
    return false;
}

void MoveCenter::timerProc(const ros::WallTimerEvent& event)
{
	UINFO("[motion]set timer proc start.");
    if(timer_count++ >= MOTION_SEND_TIMES_MAX)
    {
        m_wtTimer.stop();
    }
    m_pMi->mp.is_new = 0;
    setWheelMove(&m_pMi->wp);
}
