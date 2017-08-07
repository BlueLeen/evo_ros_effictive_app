#include "serialinit.h"
#include "boostserial.h"
#include "ulog.h"
#include "orrbase.h"
#include <stdlib.h>

#define         MAKECMD(srcid, dstid, cmd)                        (((srcid) << 24) | ((dstid << 16) | cmd))

static SerialInit* g_serinfo = NULL;
static BoostSerial* g_pBs = NULL;
static boost::asio::io_service g_ios;
static uint8_t g_ucRecvData[SERIAL_DATA_LEN];
static int32_t g_nSize;

static int recv_extract_msg(common_msgs::msgdata* msg_data)
{
    if(NULL == msg_data)
    {
        return -1;
    }
    int nLen = g_nSize-5;
    if(nLen > 0)
    {
    	msg_data->cmd = MAKECMD(g_ucRecvData[1], g_ucRecvData[2], g_ucRecvData[0]);
        memset(&msg_data->reserved[0], 0, 4);
        memcpy(&msg_data->reserved[0], &g_ucRecvData[3], 1);
    	msg_data->inlen = g_ucRecvData[4];
        msg_data->input.resize(msg_data->inlen);
        memcpy((void*)&msg_data->input[0], &g_ucRecvData[5], msg_data->inlen);
    }
    else
    {
    	return -1;
    }
    std::string strText;
    strText.reserve(256);
    char szTmp[100] = { 0 };
    snprintf(szTmp, sizeof(szTmp), "%c2l rcmd:0x%x inlen:%d input", g_serinfo->node_flg[0], msg_data->cmd, msg_data->inlen);
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
    return 0;
}

void thread_start()
{
	g_ios.run();
}

void thread_recv(SerialInit* serinfo)
{
    common_msgs::msgdata msg;
    UDEBUG("start to read&parse %c's serial data", serinfo->node_flg[0]);
    while(1)
    {
    	{
			//boost::mutex::scoped_lock lock(g_mtMutexPad);
			if(g_pBs->recvData(g_ucRecvData, g_nSize))
			{
				if(0 == recv_extract_msg(&msg))
				{
					serinfo->parse_data(&msg);
				}
			}
			else
			{
				UINFO("read data's fault from %c.", serinfo->node_flg[0]);
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
    snprintf(szTmp, sizeof(szTmp), "l2%c scmd:0x%x inlen:%d input", g_serinfo->node_flg[0], msg_data.cmd<<16|msg_data.input[2], sndSize);
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
	UINFO("[%s]serial command to local:0x%08x", g_serinfo->node_flg.c_str(), msg_data->cmd);
	msg_data->cmd = msg_data->cmd & 0xffff;
	OrrBase* orb = (OrrBase*)g_serinfo->orr_par;
	orb->parseCommand(*msg_data, NULL);
    return 0;
}
