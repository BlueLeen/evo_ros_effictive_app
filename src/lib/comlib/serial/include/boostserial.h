#ifndef __BOOST_SERIAL_H__
#define __BOOST_SERIAL_H__

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/asio/io_service.hpp>
#include <map>
#include <string>
#include "serialinit.h"

#define START                   	0xFD        /* indicates start of packet */
#define END                       	0xF8        /* indicates end of packet */
#define ESC                        	0xFE        /* indicates byte stuffing */
#define ESC_START           		0x7D        /* ESC ESC_START means START data byte */
#define ESC_END              		0x78        /* ESC ESC_END means END data byte */
#define ESC_ESC               		0x7E        /* ESC ESC_ESC means ESC data byte */

typedef enum
{
    E_BAUERATE_2400,
    E_BAUERATE_4800,
    E_BAUERATE_9600,
    E_BAUERATE_115200
}SERIAL_BAUDRATE;

typedef enum
{
    E_DATA_BITS_7,
	E_DATA_BITS_8
}DATA_BITS;

typedef enum
{
    E_CHECK_ODD,//奇校验
    E_CHECK_EVEN,//偶数校验
    E_CHECK_NONE //无校验
}CHECK_TYPE;

typedef enum
{
    E_ONE_BIT,
    E_TWO_BIT
}STOP_BITS;

class BoostSerial
{
public:
	BoostSerial(SERIAL_TYPE port, boost::asio::io_service& io);
	int32_t sendData(uint8_t* data, uint32_t len);
	bool recvData(uint8_t* data, int32_t& len);
    void close() { shutting_down_ = true; };
private:
//    int32_t sendRawData(uint8_t* data, uint32_t len);
	bool checkSum(uint8_t* check_data, uint32_t check_num);
	int32_t formatPacket(uint8_t* srcbuf, uint8_t* destbuf, uint32_t srclen);
private:
	std::string port_;
	uint32_t baud_rate_;
	bool shutting_down_;
	boost::asio::serial_port serial_;
	static const int rate_array[];
	static const char* dev[];
	boost::mutex m_mtxWrite;
};

#endif
