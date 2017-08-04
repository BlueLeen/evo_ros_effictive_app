#include "serialinit.h"
#include "boostserial.h"
#include "ulog.h"
#include <stdlib.h>

static SerialInit* g_serinfo = NULL;
static BoostSerial* g_pBs = NULL;
static boost::asio::io_service g_ios;
static uint8_t g_ucRecvData[SERIAL_DATA_LEN];
static int32_t g_nSize;

void thread_start()
{
	g_ios.run();
}

void thread_recv(SerialInit* serinfo)
{
    common_msgs::msgdata msg;
    UDEBUG("start to read&parse %c's serial data", serinfo->port_flg);
    while(1)
    {
    	{
			//boost::mutex::scoped_lock lock(g_mtMutexPad);
			if(g_pBs->recvData(g_ucRecvData, g_nSize))
			{
				//evo_msg("read pad's serial data");
				if(0 == serinfo->parse_data(&msg))
				{
				}
			}
			else
			{
				UINFO("read data's fault from %c.", serinfo->port_flg);
			}
    	}
    	//boost::this_thread::interruption_point();
        boost::this_thread::sleep(boost::posix_time::millisec(20));
    }
}

bool serial_init(SerialInit* serinfo)
{
	UDEBUG("serial init.");
	g_pBs = new BoostSerial(serinfo->port_num, g_ios);
	g_serinfo = serinfo;
	boost::thread(boost::bind(thread_start));
	boost::thread(boost::bind(&thread_recv, g_serinfo));
	return true;
}

int serial_sends_data(const common_msgs::msgdata& msg_data)
{
    uint8_t tmp[SERIAL_DATA_LEN] = {0};
    uint8_t frm_reserve = 0x00;
    uint8_t frm_length  = (uint8_t)msg_data.inlen;
    memcpy(&tmp[0], &frm_reserve, 1);
    memcpy(&tmp[1], &frm_length, 1);
    memcpy(&tmp[2], (void*)&msg_data.input[0], msg_data.inlen);
    std::string strText;
    strText.reserve(256);
    char szTmp[100] = { 0 };
    int sndSize = msg_data.inlen+2;
    snprintf(szTmp, sizeof(szTmp), "l2%c scmd:0x%x inlen:%d input", g_serinfo->port_flg, msg_data.cmd<<16|msg_data.input[2], sndSize);
    strText += szTmp;
    if(msg_data.inlen < 128)
    {
        for(int i = 0; i < (int)msg_data.inlen; i++)
        {
        	snprintf(szTmp, sizeof(szTmp), "  0x%02x", msg_data.input[i]);
            strText += szTmp;
        }
    }
    UINFO(strText.c_str());
    g_pBs->sendData(tmp, sndSize);
	return 0;
}

int serial_parse_data(common_msgs::msgdata* msg_data)
{
    if(NULL == msg_data)
    {
        return -1;
    }
    if(g_serinfo->port_flg=='m')
    {
//    	msg_data->cmd = (NODE_MOTION_DOWN << 16) | 0;
    }
    else if(g_serinfo->port_flg=='u')
    {
//    	M_Directive::iterator iter;
//		iter = m_directive.find(g_ucRecvData[2]);
//		if(iter != m_directive.end())
//		{
//			msg_data->cmd = iter->second;
//		}
//		else
//			msg_data->cmd = (NODE_ULTRASONIC_DOWN << 16) | 0;
    }
    msg_data->inlen = g_nSize-2;
    if(msg_data->inlen > 0)
    {
    	msg_data->cmd = msg_data->cmd | g_ucRecvData[4];
        msg_data->input.resize(msg_data->inlen);
        memcpy((void*)&msg_data->input[0], &g_ucRecvData[2], msg_data->inlen);
    }
    else
    {
    	return -1;
    }
    std::string strText;
    strText.reserve(256);
    char szTmp[100] = { 0 };
    snprintf(szTmp, sizeof(szTmp), "%c2l rcmd:0x%x inlen:%d input", g_serinfo->port_flg, msg_data->cmd, msg_data->inlen);
    strText += szTmp;
    if(msg_data->inlen < 128)
    {
        for(unsigned int i = 0; i < msg_data->inlen; i++)
        {
        	snprintf(szTmp, sizeof(szTmp), "  0x%02x", msg_data->input[i]);
            strText += szTmp;
        }
    }
    UINFO(strText.c_str());
    memset(&msg_data->reserved[0], 0, 4);
    memcpy(&msg_data->reserved[0], &g_ucRecvData[0], 1);
    return 0;
}
