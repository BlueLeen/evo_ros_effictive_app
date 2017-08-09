#ifndef __MOVE_DEF_H__
#define __MOVE_DEF_H__

//#include <stdlib.h>
#include <stdint.h>

#define			NODENAME							"move"
#define 		L2MCMD1							 	0x1
#define 		L2MCMD2							 	0x2
#define 		M2LCMD1							 	0x1
#define 		M2LCMD10						 	0x10
#define 		M2LCMDF0						 	0xF0
#define 		M2LCMDF1						 	0xF1

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

typedef struct _MOVE_INFO
{
	MOTOR_VERSION					ver;
//	MOTOR_ABNOMITYEVENT				abet;
	void*							mtol;
	void*							ltom;
}MOVE_INFO, *PMOVE_INFO;

#endif
