#ifndef __MOVE_DEF_H__
#define __MOVE_DEF_H__

//#include <stdlib.h>
#include <stdint.h>
#include <boost/thread/mutex.hpp>
#include "comlib/serialinit.h"

#define			NODENAME							"move"
#define 		L2MCMD1							 	0x1
#define 		L2MCMD2							 	0x2
#define 		L2MCMD3							 	0x3
#define 		M2LCMD1							 	0x1
#define 		M2LCMD2							 	0x2
#define 		M2LCMD3							 	0x3
#define 		M2LCMD10						 	0x10
#define 		M2LCMDF0						 	0xF0
#define 		M2LCMDF1						 	0xF1
#define         PI                              	3.141592654
#define         MOTION_DIRECT_FRONT                 0x0             //电机运动正方向
#define         MOTION_DIRECT_BACK                  0x1             //电机运动负方向
#define         MOTION_DIRECT_STOP                  0x2             //电机停止运动

typedef struct _MOTOR_VERSION
{
    char    product_model[4];
    char    model_name[3];
    char    hardware_platform[2];
    char   	soft_version[4];
    char 	time[8];
    char 	sn[12];
}MOTOR_VERSION, *PMOTOR_VERSION;

typedef struct _MOTOR_ABNOMITYEVENT
{
	uint8_t		battery_level_low:1;
	uint8_t		battery_voltage_imbalance:1;
	uint8_t		coreboard_overcurrent:1;
	uint8_t		whole_machine_overcurrent:1;
	uint8_t		charge_voltage_high:1;
	uint8_t		charge_current_high:1;
	uint8_t		battery_not_exist:1;
	uint8_t		control_bus_abnormal:1;
	uint8_t		reserve;
	uint8_t		charge_pile_connect:1;
	uint8_t		charge_pile_supplypower:1;
	uint8_t		charge_pile_disconnect:1;
	uint8_t		emrgcharge_pile_connect:1;
	uint8_t		emrgcharge_pile_supplypower:1;
	uint8_t		emrgcharge_pile_disconnect:1;
	uint8_t		boot_btn_press:1;
	uint8_t		boot_btn_up:1;
}MOTOR_ABNOMITYEVENT, *PMOTOR_ABNOMITYEVENT;

typedef struct _ODOMETERNUMBER
{
	uint8_t	  valid_flag; //数据有效性 0-无效 1-有效
    uint32_t  wheel_left_odometer_number;
    uint32_t  wheel_right_odometer_number;
}ODOMETERNUMBER, *PODOMETERNUMBER;

typedef struct _CALIBRATIONINFO
{
    uint8_t   valid_flag; //数据有效性 0-无效 1-有效
    float  	  calibration_x;
    float     calibration_y;
    float     calibration_th;
}CALIBRATIONINFO, *PCALIBRATIONINFO;

typedef struct _SENSORINFO
{
	uint8_t   ultra_enable[2]; //按位开启超声波探头
}SENSORINFO;

typedef struct _WHEEL_PARAM
{
    uint8_t   dir_left;
    uint8_t   dir_right;
    uint16_t  speed_left;
    uint16_t  speed_right;
    uint8_t   is_timer; //是否启动定时器0-不启动 1-启动
    uint8_t   is_new; //是否新指令 0-心跳指令 1-新指令
}WHEEL_PARAM, *PWHEEL_PARAM;

typedef struct _MOVE_PARAM
{
    float     speed;
    float     rad;
    float  	  line_dis;
    float     angle_dis;
    uint8_t   is_new; //是否新指令 0-心跳指令 1-新指令
}MOVE_PARAM, *PMOVE_PARAM;

typedef struct _MOVE_INFO
{
	MOTOR_VERSION					ver;
//	MOTOR_ABNOMITYEVENT				abet;
	ODOMETERNUMBER					odo;
	CALIBRATIONINFO					cal;
	MOVE_PARAM						mp;
	WHEEL_PARAM						wp;
	SENSORINFO						sor;
	SerialInit&						si;
	void*							mtol;
	void*							ltom;
	boost::mutex 					mtx;
}MOVE_INFO, *PMOVE_INFO;

typedef struct _CALIBRATION_CORRECT
{
    float   calibration_x;
    float   calibration_y;
    float   calibration_th;

}CALIBRATION_CORRECT, *PCALIBRATION_CORRECT;

enum SERVICE
{
	SET_MOTION_NODE_STATUS		= 0x0,
	GET_NODE_MOTION_WORK_STATUS = 0x1,
	SET_DEAD_RECKON_INFO		= 0x2,
	GET_MOTION_ID_SN			= 0x3,
};

enum PUBLISH
{
    TOPHONE                      = 0x0,
	GET_DEAD_RECKON_INFO		 = 0x1,
//    TOMOTORBOAD                  = 0x8,
};

enum SUBCRIB
{
	FROMPAD						 = 0x0,
	SET_WHEEL_DIFFERENTIAL_MOVE	 = 0x1,
	SET_WHEEL_FIXED_MOVE		 = 0x2,
	SET_WHEEL_STOP				 = 0x3,
	SET_MOTION_BARRIER_ENABLE	 = 0x4,
};

#endif
