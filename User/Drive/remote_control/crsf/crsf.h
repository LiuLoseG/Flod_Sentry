#pragma once

#include "bsp_uart.h"
#include "cmsis_gcc.h"
#include "string.h"

#define CRSF_UART huart7
#define CRSF_DATA_SIZE 26         // CRSF数据包最大长度

#define CRSF_REMOTE_CONTROL_MIN 191     // CRSF遥控器数据通道机械最小值
#define CRSF_REMOTE_CONTROL_MID 992     // CRSF遥控器数据通道机械中值
#define CRSF_REMOTE_CONTROL_MAX 1792    // CRSF遥控器数据通道机械最大值
#define CRSF_REMOTE_CONTROL_KEY 997    // CRSF遥控器数据通道机械中值

// 字节序反转：16位，返回uint16_t消除窄化警告
#define __REV16(x) ((uint16_t)__builtin_bswap16(x))
// 字节序反转：24位（3字节），手动交换
#define __REV24(x) ( ((x) & 0xFF) << 16 | ((x) & 0xFF00) | (((x) >> 16) & 0xFF) )
// 字节序反转：32位，返回int32_t匹配GPS坐标类型
#define __REV32(x) ((int32_t)__REV(x))

// 用以支持24位类型，仅用于REV宏
#define int24_t int32_t

// 通用字节序反转宏，支持 16/24/32 位
#define REV(value, type) __REV##type((int##type##_t)value)

// CRSF数据包结构体定义
typedef struct 
{
    uint8_t addr;
    uint8_t length;
    uint8_t type;
    uint8_t payload[60];
    uint8_t crc;
} crsf_boardcast_frame_t;

// CRSF通道数据结构体定义
typedef struct __attribute__((packed))
{
    // 打包状态下，一个通道实际占用11位
    uint16_t ch[16];
    uint32_t link_status; // optional ExpressLRS 4.0
} crsf_channels_t;

// CRSF电池传感器数据结构体定义
typedef struct __attribute__((packed))
{
    signed voltage : 16;      // 25.2V sent as 0x00FC / 252
    signed current : 16;      // 18.9A sent as 0x00BD / 189
    unsigned int used_capacity : 24;     // 2199mAh used sent as 0x0897 / 2199
    uint8_t estimated_remaining_capacity; // 20% battery remaining sent as 0x14 / 20
} crsf_battery_sensor_t;

// CRSF GPS数据结构体定义
typedef struct __attribute__((packed))
{
    int32_t latitude;   // Latitude in degrees * 10^7 ( 28.0805804N sent as 0x10BCC1AC / 280805804)
    int32_t longitude;
    uint16_t groundspeed; // km/h * 10 ( 88 km/h sent as 0x0370 / 880; 15m/s sent as 0x021C / 540)
    uint16_t heading; // degrees * 100 (90 degrees sent as 0x2328 / 9000)
    uint16_t altitude; // meters + 1000 (10m sent as 0x03F2 / 1010; -10m sent as 0x03DE / 990)
    uint16_t satellites; // number of satellites
} crsf_gps_t;

/** CRSF地址定义 */
typedef enum __CRSF_ADDRESS_t{
    CRSF_ADDRESS_BROADCAST                   = 0x00,  // 广播
    CRSF_ADDRESS_USB                         = 0x10,
    CRSF_ADDRESS_BLUETOOTH                   = 0x12,  // 蓝牙
    CRSF_ADDRESS_TBS_CORE_PNP_PRO            = 0x80,
    CRSF_ADDRESS_RESERVED1                   = 0x8A,  // 保留地址1
    CRSF_ADDRESS_CURRENT_SENSOR              = 0xC0,  // 电流传感器
    CRSF_ADDRESS_GPS                         = 0xC2,  // GPS
    CRSF_ADDRESS_TBS_BLACKBOX                = 0xC4,  // TBS黑匣子
    CRSF_ADDRESS_FLIGHT_CONTROLLER           = 0xC8,  // 飞行控制器
    CRSF_ADDRESS_RESERVED2                   = 0xCA,  // 保留地址2
    CRSF_ADDRESS_RACE_TAG                    = 0xCC,  // 比赛标签
    CRSF_ADDRESS_RADIO_TRANSMITTER           = 0xEA,  // 无线电发射器
    CRSF_ADDRESS_CRSF_RECEIVER               = 0xEC,  // Receiver hardware (TBS Nano RX / RadioMaster RP1)
    CRSF_ADDRESS_CRSF_TRANSMITTER            = 0xEE,  // Transmitter module, not handset
    CRSF_ADDRESS_ELRS_LUA                    = 0xEF,  // elrs lua
} CRSF_ADDRESS_t;

/** CRSF Type定义 */
typedef enum __CRSF_FRAMETYPE_t{
    CRSF_FRAMETYPE_GPS                       = 0x02,  // 回传
    CRSF_FRAMETYPE_VARIO                     = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR            = 0x08,  // 回传
    CRSF_FRAMETYPE_BARO_ALTITUDE             = 0x09,  // 回传
    CRSF_FRAMETYPE_HEARTBEAT                 = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS           = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED        = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_RX_ID                = 0x1C,   // 回传
    CRSF_FRAMETYPE_LINK_TX_ID                = 0x1D,   // 回传
    CRSF_FRAMETYPE_ATTITUDE                  = 0x1E,   // 回传
    CRSF_FRAMETYPE_FLIGHT_MODE               = 0x21,   // 回传
} CRSF_FRAMETYPE_t;

// 后面还有拓展类型待补充

/** 发送消息包方法 */

// 发送函数指针，用户需要实现这个函数来发送CRSF数据包
extern void (*send_crsf_packet)(const CRSF_FRAMETYPE_t frame_type, const uint8_t * payload, const uint8_t size_of_payload);

extern crsf_channels_t rc_channels;

void crsf_rx_idle_callback(uint8_t *buf, uint16_t len);

extern crsf_channels_t rc_channels;