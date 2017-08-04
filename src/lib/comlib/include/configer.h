#ifndef __CONFIGER_H__
#define __CONFIGER_H__

#include <stdio.h>
#include <stdint.h>

#define         EVO_CONFIG_PATH                 "/app/evo_ros_config.xml"
#define         EVO_VERSION_OLD_PATH            "/app/version.xml"
#define         EVO_VERSION_PATH                "/app/config/version.xml"
#define         DRIVER_CONFIG_PATH              "/app/config/driver_config.xml"
#define         LOCAL_CONFIG_PATH               "/app/config/local_config.xml"
#define         CHARGE_CONFIG_PATH              "/app/config/charge_config.xml"
#define         LOG_CONFIG_PATH              	"/app/config/log_config.xml"
#define         SERIAL_CONFIG_PATH              "/app/config/serial_config.xml"

//双轮补偿
typedef struct
{
    float left_wheel_forward;           //左轮向前补偿
    float left_wheel_backward;          //左轮向后补偿
    float right_wheel_forward;          //右轮向前补偿
    float right_wheel_backward;         //右轮向后补偿

}WHEELS_MODIFY, *PWHEELS_MODIFY;

//码数转距离
typedef struct
{
    float cofficient_modify;            //转换系数
    float driver_servo_interval;        //驱动伺服间隔时间（ms）

}CODES_TO_DISTANCE, *PCODES_TO_DISTANCE_MODIFY;

typedef struct
{
    char			    dest[64];
    WHEELS_MODIFY       control_modify;             //跑直补偿
    WHEELS_MODIFY       dead_reckon_modify;         //航位补偿
    CODES_TO_DISTANCE   codes_to_distance;          //码数转距离
    float               wheels_spaces_modify;       //轮间距补偿

}DRIVER_CONFIG, *PDRIVER_CONFIG;

typedef struct
{
    char	dest[64];
    uint32_t  fan_time;                               //风机工作时间
    char    serial_num[64];                         //串行号
    uint16_t  infrared_init[7];                       //红外数据初始值(7个)
    uint32_t  charge_pile_id;                        //充电桩ID
    uint32_t  charge_type;                           //充电桩绑定状态
    char   charge_pile_ver[128];
    uint8_t charge_channel;

}WORK_CONFIG, *PWORK_CONFIG;

typedef struct
{
    char	dest[64];
    uint32_t  fan_time;                               //风机工作时间
    char    serial_num[64];                         //串行号
    uint16_t  infrared_init[7];                       //红外数据初始值(7个)

}LOCAL_CONFIG, *PLOCAL_CONFIG;

typedef struct
{
    uint32_t  charge_pile_id;                        //充电桩ID
    uint32_t  charge_type;                           //充电桩绑定状态
    char    charge_pile_ver[128];                  //充电桩版本号
    uint32_t  charge_channel;                        //充电桩频段

}CHARGE_CONFIG, *PCHARGE_CONFIG;

typedef struct
{
    char    sys_version[128];                       //下位机版本号
    char    driver_version[128];                    //驱动版本号
    char    ultra_version[128];                     //超声波版本号
    char   charge_version[128];                                //充电桩类型 0-无充电桩  1-带id充电桩 2-不带id充电桩

}VERSION_CONFIG, *PVERSION_CONFIG;

//配置文件参数
typedef struct
{
    DRIVER_CONFIG       driver_config;              //驱动板配置参数
    WORK_CONFIG         work_config;                //运行配置参数
}CONFIG_DATA, *PCONFIG_DATA;

//节点日志信息
typedef struct _node_common_info
{
	int index;
	char node_name[128]; //节点名称
	int log_site; //节点日志输出地点
	int log_level; //节点日志输出级别
	bool modify; //节点日志输出方式是否更改
}NodeCommonInfo;

//串口信息
typedef struct _serial_port_info
{
	char pad_read[128]; //Pad映射串口读文件
	char pad_write[128]; //Pad映射串口写文件
	char ultra_read[128]; //Ultra映射串口读文件
	char ultra_write[128]; //Ultra映射串口写文件
	char motor_read[128]; //Motor映射串口读文件
	char motor_write[128]; //Motor映射串口写文件
}SerialPortInfo;

class CConfiger
{
public:
	~CConfiger();
	static	CConfiger*		GetInstance();

    bool					ReadConfig(const char* strFileName, CONFIG_DATA* pConfigData);   //������(��xml�ļ�ת�����ڲ����ݽṹ)
    bool					SetConfig(const char* strFileName, CONFIG_DATA* pConfigData);    //Set����(��xml�ļ�ת�����ڲ����ݽṹ)

    bool                    CreateLocalConfig(const char* strFileName);
    bool                    CreateDriverConfig(const char* strFileName);
    bool                    CreateChargeConfig(const char* strFileName);
    bool                    CreateVersionConfig(const char* strFileName);

    bool					ReadDriverConfig(const char* strFileName, DRIVER_CONFIG* pConfigData); //������(��xml�ļ�ת�����ڲ����ݽṹ)
    bool					SetDriverConfig(const char* strFileName, DRIVER_CONFIG* pConfigData);    //Set����(��xml�ļ�ת�����ڲ����ݽṹ)

    bool                    ReadLocalConfig(const char* strFileName, LOCAL_CONFIG* pConfigData);
    bool                    SetLocalConfig(const char* strFileName, LOCAL_CONFIG* pConfigData);

    bool                    ReadChargeConfig(const char* strFileName, CHARGE_CONFIG* pConfigData);
    bool                    SetChargeConfig(const char* strFileName, CHARGE_CONFIG* pConfigData);

    bool					ReadWorkConfig(const char* strFileName, WORK_CONFIG* pConfigData);   //������(��xml�ļ�ת�����ڲ����ݽṹ)
    bool                    SetWorkConfig(const char* strFileName, WORK_CONFIG* pConfigData);

    bool                    ReadVersionConfig(const char* strFileName, VERSION_CONFIG* pConfigData);//��ȡ�汾��
    bool                    SetVersionConfig(const char* strFileName, VERSION_CONFIG* pConfigData);

    bool 					ReadLogConfig(const char* strFileName, NodeCommonInfo* pNci);
    bool 					SetLogConfig(const char* strFileName, NodeCommonInfo* pNci);

    bool 					ReadSerialConfig(const char* strFileName, SerialPortInfo* pSpi);

private:
	CConfiger();
	static	CConfiger*		m_Instance;
};
#endif
