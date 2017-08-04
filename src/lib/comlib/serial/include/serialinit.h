#ifndef __SERIAL_INIT_H__
#define __SERIAL_INIT_H__

#include "common_msgs/msgdata.h"

#define SERIAL_DATA_LEN             1024        //串口数据最大大小

typedef enum
{
	E_UNKNOWN = -1,  	//未知串口
    E_MOTORBOARD = 1,   //连接驱动板串口
    E_ULTRASONIC,       //超声波
    E_AIRCHECK,         //激光粉尘
    E_RESERVER,         //预留
    E_PAD        		//上位机平板
}SERIAL_TYPE;

typedef int (*send_data)(const common_msgs::msgdata& msg_data);
typedef int (*recv_data)(common_msgs::msgdata* msg_data);

extern int serial_sends_data(const common_msgs::msgdata& msg_data);
extern int serial_parse_data(common_msgs::msgdata* msg_data);

typedef struct _serial_init
{
	_serial_init(SERIAL_TYPE type, char flag,
				send_data snd=serial_sends_data,
				recv_data rcv=serial_parse_data)
			:port_num(type),port_flg(flag),write_data(snd),parse_data(rcv)
	{
	}
	SERIAL_TYPE port_num;
	char 		port_flg;
	send_data	write_data;
	recv_data	parse_data;
}SerialInit;

extern bool serial_init(SerialInit* serinfo);

#endif
