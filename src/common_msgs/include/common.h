////////////////////////////////////////////////////////////////////////////////
// 版权所有，2015-2025，北京进化者机器人有限公司。
////////////////////////////////////////////////////////////////////////////////
// 文件名： common.h
// 作者：   张川
// 版本：   2.0
// 日期：   2015/08/05
// 描述：   公共接口，包含指令定义、通信数据结构、通信接口等
// 其他说明：
// 修改历史:
// 日期(YYYY-MM-DD)   版本    修改人      修改内容
////////////////////////////////////////////////////////////////////////////////

#ifndef COMMON_H
#define COMMON_H

#include <string.h>
#include <stdbool.h>
#include "config.h"


typedef unsigned char       uint8;
typedef signed char         sint8;
typedef unsigned short      uint16;
typedef signed short        sint16;
typedef unsigned int        uint32;
typedef signed int          sint32;
typedef unsigned long       ulng32;
typedef signed long         slng32;
typedef signed long long    sint64;
typedef unsigned long long  uint64;

#define                     BITSEL(x, n) (((x)>>(n))&1)                                 // 判断X 的第n位是否为"1"
#define                     BITSET(x, n) ((x)|=(1<<(n)))                                // 设置X 的第n位为"1"
#define                     BITCLR(x, n) ((x)&=~(1<<(n)))                               // 清除X 的第n位为"0"

#define SWAPUINT16(x)       ((uint16)((((uint16)(x) & 0x00ff) << 8) | (((uint16)(x) & (uint16)0xff00) >> 8)))       //2字节数据大小端转换

#define SERIAL_DATA_LEN             1024        //串口数据最大大小

//节点运行状态
#define         NODE_STATUS_STOP                             0          //停止
#define         NODE_STATUS_START                            1          //启动
#define         NODE_STATUS_PAUSE                            2          //暂停
#define         NODE_STATUS_RESUME                           3          //继续

//module
#define 		MODULE_GLOBALCOMM                             0xC00
#define         NODE_IO_DETECTION                             0xC01
#define         NODE_POWER_DETECTION                          0xC02
#define         NODE_ULTRASONIC                               0xC03
#define         NODE_INFRARED                                 0xC04
#define         NODE_MOTION                                   0xC05
#define         NODE_HEADWING                                 0xC15
#define         NODE_UPGRADE                                  0xC06
#define         NODE_NAVIGATION                               0xC07
#define         NODE_MAP_MANAGER                              0xC08
#define         NODE_LOCAL_INFO                               0xC09
#define         NODE_LOCALIZATION                             0xC0A
#define         NODE_MONITOR                                  0xC0B
#define         NODE_LOG                                      0xC0C
#define         NODE_CAMERA                                   0xC0D
#define         NODE_CONFIG                                   0xC0E
#define         NODE_OTHER                                    0xC0F
#define         NODE_ULTRASONIC_DOWN                          0x1C03
#define         NODE_INFRARED_DOWN                            0x1C04
#define         NODE_MOTION_DOWN                              0x1C05
#define         NODE_HEADWING_DOWN                            0x1C15
#define         NODE_OTHER_DOWN                               0x1C0F

//指令类型
#define         CMD_TYPE_NODE                            0x0            //启动节点
#define         CMD_TYPE_MOTION                          0x1            //运动类型
#define         CMD_TYPE_PROJECTOR                       0x2            //投影类型
#define         CMD_TYPE_PURIFIER                        0x3            //净化类型
#define         CMD_TYPE_GET_INFO                        0x4            //获取类型
#define         CMD_TYPE_FEEDBACK                        0x5            //反馈类型
#define         CMD_TYPE_SET                             0x6            //设置类型
#define         CMD_TYPE_UPGRADE                         0x7            //升级类型


//命令定义 module-模块代码，type-指令类型,cmd-具体指令
#define         MAKECMD(module, type, cmd)                        (((module) << 16) | ((type << 8) | cmd))


#endif // COMMON_H
