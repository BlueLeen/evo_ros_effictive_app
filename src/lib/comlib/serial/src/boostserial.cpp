#include "boostserial.h"
#include "ulog.h"

const int BoostSerial::rate_array[] = {2400, 4800, 9600,115200};
#ifndef E01S
const char* BoostSerial::dev[]={"/dev/ttyO0","/dev/ttyO1","/dev/ttyO2","/dev/ttyO3","/dev/ttyO4","/dev/ttyO5"};
#else
const char* BoostSerial::dev[]={"/dev/ttyO0","/dev/ttyS4","/dev/ttyS3","/dev/ttyO3","/dev/ttyO4","/dev/ttyO5"};
#endif

BoostSerial::BoostSerial(SERIAL_TYPE port, boost::asio::io_service& io):
		port_(dev[port]), baud_rate_(rate_array[3]), shutting_down_(false),serial_(io, port_)
{
//	port_ = dev[port-1];
//	baud_rate_ = rate_array[3];
	UINFO("boost serial name:%s", dev[port]);
	serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	serial_.set_option(boost::asio::serial_port_base::parity());
	serial_.set_option(boost::asio::serial_port::stop_bits());
	serial_.set_option(boost::asio::serial_port::character_size(8));
}

int32_t BoostSerial::sendData(uint8_t* data, uint32_t len)
{
	uint8_t u08_buf_to_serial[SERIAL_DATA_LEN] = { 0 };/*用于存储打包好的数据*/
	uint8_t u08_buf_with_check[SERIAL_DATA_LEN] = { 0 };/*用于存储打包好的数据*/
	uint32_t u32_send_len = 0;
	uint32_t i = 0;
	uint8_t u08_checksum = 0;
	memcpy(u08_buf_with_check,data,len);
	if(SERIAL_DATA_LEN <= len)
	{
		UERROR("boost_serial_write:buflen is invalid!\n");
        return -1;
    }
	for(i=0; i<len; i++)
	{
		u08_checksum+=u08_buf_with_check[i];
	}
	u08_buf_with_check[i] = u08_checksum&0xff;
	u32_send_len =  formatPacket(u08_buf_with_check, u08_buf_to_serial, len+1);
	u08_buf_to_serial[u32_send_len] = 66;
	u08_buf_to_serial[u32_send_len+1] = 88;
#ifdef E01S
	m_mtxWrite.lock();
#endif
	int32_t size = boost::asio::write(serial_, boost::asio::buffer(u08_buf_to_serial, u32_send_len+2));
#ifdef E01S
	m_mtxWrite.unlock();
#endif
	if(size < 0)
	{
		UERROR("boost_serial_write:write failed!\n");
		return -1;
	}
	return 0;
}

bool BoostSerial::recvData(uint8_t* data, int32_t& len)
{
	uint8_t temp_char;
	uint8_t start_flag = 0;
	uint8_t end_flag = 0;
	uint8_t slip_flag = 0;
	uint8_t esc_flag = 0;
	u_int32_t pac_len = 0;
	while(!shutting_down_) {
		boost::asio::read(serial_, boost::asio::buffer(&temp_char,1));
		switch(temp_char)
		{
			case START:
				start_flag = 1;
				end_flag = 0;
				pac_len = 0;
				slip_flag = 0;
				continue;
			case END:
				end_flag = 1;
				break;
			case ESC:
				if(start_flag != 1)
					break;
				esc_flag = 1;
				continue;
			default:
				if(start_flag != 1)
					break;
				if(esc_flag == 1)
				{
					esc_flag = 0;
					switch(temp_char)
					{
						case ESC_START:
							temp_char = START;
							break;
						case ESC_END:
							temp_char = END;
							break;
						case ESC_ESC:
							temp_char = ESC;
							break;
						default:
							slip_flag = 1;
							break;
					}
				}
				data[pac_len++] = temp_char;
				continue;
		}
		if(end_flag)
			break;
	}
	if(slip_flag)
	{
		UERROR("Packet slip packet after ESC...\n ");
		pac_len = 0;
		return false;
	}
	if(start_flag == 0)
	{
		UERROR("Packet has no start flag:0xFD,esmitate:loss data.\n");
		pac_len = 0;
		return false;
	}
	if(end_flag == 0)
	{
		UERROR("Packet has no ended flag:0xF8,esmitate:loss data.\n");
		pac_len = 0;
		return false;
	}
	else
	{
		start_flag = 0;
		end_flag = 0;
		slip_flag = 0;
		if(!checkSum(data, pac_len))
		{
		    std::string strText;
		    strText.reserve(1024);
			char szTmp[100] = { 0 };
			pac_len = (pac_len<256) ? pac_len:256;
			strText += "Packet check sum error,fault packet:";
			for(uint32_t i=0; i<pac_len; i++)
			{
				snprintf(szTmp, sizeof(szTmp), " 0x%02x", data[i]);
				strText += szTmp;
			}
			UERROR(strText.c_str());
			pac_len = 0;
			return false;
		}
		len = pac_len-1;
		return true;
	}
	return false;
}

bool BoostSerial::checkSum(uint8_t* check_data, uint32_t check_num)
{
    if(check_num < 1)
        return false;

	uint32_t j;
	uint32_t nCheck = 0;

	for(j = 0; j < (check_num-1); j++)
	{
		nCheck += check_data[j];
	}
    if((nCheck & 0xff) != check_data[check_num - 1])
		return false;
	else
		return true;
}

int32_t BoostSerial::formatPacket(uint8_t* srcbuf, uint8_t* destbuf, uint32_t srclen)
{
	uint32_t i = 1, j = 0;
	destbuf[0] = START;
	while (srclen--) {
		switch (srcbuf[j]) {
		case START:
			destbuf[i++] = ESC;
			destbuf[i++] = ESC_START;
			break;
		case END:
			destbuf[i++] = ESC;
			destbuf[i++] = ESC_END;
			break;

		case ESC:
			destbuf[i++] = ESC;
			destbuf[i++] = ESC_ESC;
			break;

		default:

			destbuf[i++] = srcbuf[j];
		}
		j++;
	}
	destbuf[i++] = END;
	return i;
}
