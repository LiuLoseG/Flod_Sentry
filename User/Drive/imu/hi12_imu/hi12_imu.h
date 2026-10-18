#pragma once

#include "bsp_uart.h"
#include <string.h>

/*===========================================================================
 * HiPNUC 协议常量
 *===========================================================================*/
#define CHSYNC1              (0x5A)          /* 帧同步码1 */
#define CHSYNC2              (0xA5)          /* 帧同步码2 */
#define CH_HDR_SIZE          (0x06)          /* 帧头: SYNC(2)+LEN(2)+CRC16(2) */

#define HIPNUC_ID_HI91       (0x91)          /* IMU数据包 */
#define HIPNUC_ID_HI81       (0x81)          /* INS数据包 */
#define HIPNUC_ID_HI83       (0x83)          /* 扩展数据包 */

#define HIPNUC_MAX_RAW_SIZE  (256)           /* 最大帧长度 */

#ifndef D2R
#define D2R (0.0174532925199433f)
#endif
#ifndef R2D
#define R2D (57.2957795130823f)
#endif
#ifndef GRAVITY
#define GRAVITY (9.8f)
#endif

/*===========================================================================
 * 数据结构: 协议数据包 (全部 __packed__)
 *===========================================================================*/

/**
 * @brief 0x91 — IMU数据 (浮点, 76字节)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t     tag;                // 标签 0x91
    uint16_t    main_status;        // 主状态
    int8_t      temp;               // 温度 (°C)
    float       air_pressure;       // 气压 (Pa)
    uint32_t    system_time;        // 系统时间戳 (ms)
    float       acc[3];             // 加速度 (m/s²)
    float       gyr[3];             // 角速度 (deg/s)
    float       mag[3];             // 磁力计 (uT)
    float       roll;               // 横滚角 (deg)
    float       pitch;              // 俯仰角 (deg)
    float       yaw;                // 航向角 (deg)
    float       quat[4];            // 四元数 w,x,y,z
} hi91_t;

/**
 * @brief 0x81 — INS组合导航数据 (整型+缩放, 96字节)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t     tag;
    uint16_t    main_status;
    uint8_t     ins_status;
    uint16_t    gpst_wn;            // GPS周数
    uint32_t    gpst_tow;           // GPS周内秒 (ms)
    uint16_t    reserved;
    int16_t     gyr_b[3];           // 陀螺原始值
    int16_t     acc_b[3];           // 加计原始值
    int16_t     mag_b[3];           // 磁力原始值
    int16_t     air_pressure;
    int16_t     reserved1;
    int8_t      temperature;        // 温度 (°C)
    uint8_t     utc_year;           // UTC年 (20xx)
    uint8_t     utc_month;
    uint8_t     utc_day;
    uint8_t     utc_hour;
    uint8_t     utc_min;
    uint16_t    utc_msec;           // UTC毫秒
    int16_t     roll;               // 横滚 (×0.01°)
    int16_t     pitch;              // 俯仰 (×0.01°)
    uint16_t    yaw;                // 航向 (×0.01°)
    int16_t     quat[4];            // 四元数 (×1e-4)
    int32_t     ins_lon;            // INS经度 (×1e-7°)
    int32_t     ins_lat;            // INS纬度 (×1e-7°)
    int32_t     ins_msl;            // INS海拔 (mm)
    uint8_t     pdop;               // 位置DOP (×0.1)
    uint8_t     hdop;               // 水平DOP (×0.1)
    uint8_t     solq_pos;           // 位置质量
    uint8_t     nv_pos;             // 位置卫星数
    uint8_t     solq_heading;       // 航向质量
    uint8_t     nv_heading;         // 航向卫星数
    uint8_t     diff_age;           // 差分龄期 (s)
    int16_t     undulation;         // 高程异常 (×0.01m)
    uint8_t     ant_status;
    int16_t     vel_enu[3];         // ENU速度 (×0.01 m/s)
    int16_t     acc_enu[3];         // ENU加速度
    int32_t     gnss_lon;           // GNSS经度 (×1e-7°)
    int32_t     gnss_lat;           // GNSS纬度 (×1e-7°)
    int32_t     gnss_msl;           // GNSS海拔 (mm)
    uint8_t     reserved2[2];
} hi81_t;

/* HI83 位图掩码 */
#define HI83_BMAP_ACC_B              (1u <<  0)
#define HI83_BMAP_GYR_B              (1u <<  1)
#define HI83_BMAP_MAG_B              (1u <<  2)
#define HI83_BMAP_RPY                (1u <<  3)
#define HI83_BMAP_QUAT               (1u <<  4)
#define HI83_BMAP_SYSTEM_TIME        (1u <<  5)
#define HI83_BMAP_UTC                (1u <<  6)
#define HI83_BMAP_AIR_PRESSURE       (1u <<  7)
#define HI83_BMAP_TEMPERATURE        (1u <<  8)
#define HI83_BMAP_INCLINATION        (1u <<  9)
#define HI83_BMAP_HSS                (1u << 10)
#define HI83_BMAP_HSS_FRQ            (1u << 11)
#define HI83_BMAP_VEL_ENU            (1u << 12)
#define HI83_BMAP_ACC_ENU            (1u << 13)
#define HI83_BMAP_INS_LON_LAT_MSL    (1u << 14)
#define HI83_BMAP_GNSS_QUALITY_NV    (1u << 15)
#define HI83_BMAP_OD_SPEED           (1u << 16)
#define HI83_BMAP_UNDULATION         (1u << 17)
#define HI83_BMAP_DIFF_AGE           (1u << 18)
#define HI83_BMAP_NODE_ID            (1u << 19)
#define HI83_BMAP_GNSS_LON_LAT_MSL   (1u << 30)
#define HI83_BMAP_GNSS_VEL           (1u << 31)

/**
 * @brief 0x83 — 扩展数据 (位图驱动可变长度)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t     tag;
    uint16_t    main_status;
    uint8_t     ins_status;
    uint32_t    data_bitmap;

    float       acc_b[3];
    float       gyr_b[3];
    float       mag_b[3];
    float       rpy[3];
    float       quat[4];
    uint64_t    system_time_us;
    struct __attribute__((__packed__)) {
        uint8_t  year, month, day, hour, min;
        uint16_t sec_ms;
        uint8_t  rev;
    } utc;
    float       air_pressure;
    float       temperature;
    float       inclination[3];
    float       hss[3];
    float       hss_frq[3];
    float       vel_enu[3];
    float       acc_enu[3];
    double      ins_lon_lat_msl[3];
    uint8_t     solq_pos, nv_pos, solq_heading, nv_heading;
    float       od_speed;
    float       undulation;
    float       diff_age;
    struct __attribute__((__packed__)) {
        uint8_t node_id;
        uint8_t reserved[3];
    } node;
    double      gnss_lon_lat_msl[3];
    float       gnss_vel[3];
} hi83_t;

/*===========================================================================
 * HI12 传感器驱动类
 *===========================================================================*/
class hi12_gyro
{
public:
    void init(uart_manger_object *uart_manager) volatile;
    void parse_buffer(uint8_t *buf, uint16_t len) volatile;

    /* ---- HI91 IMU数据 (浮点, 直接读取) ---- */
    inline uint8_t  tag()            volatile { return hi91.tag; }
    inline uint16_t main_status()    volatile { return hi91.main_status; }
    inline int8_t   temp()           volatile { return hi91.temp; }
    inline float    air_pressure()   volatile { return hi91.air_pressure; }
    inline uint32_t system_time()    volatile { return hi91.system_time; }

    /* HI91 加速度原始单位为 g, 此处转换为 m/s² */
    inline float acc_x() volatile { return hi91.acc[0] * GRAVITY; }
    inline float acc_y() volatile { return hi91.acc[1] * GRAVITY; }
    inline float acc_z() volatile { return hi91.acc[2] * GRAVITY; }

    // HI91 单位为 deg/s
    inline float gyr_x() volatile { return hi91.gyr[0]; }   
    inline float gyr_y() volatile { return hi91.gyr[1]; }
    inline float gyr_z() volatile { return hi91.gyr[2]; }

    inline float mag_x() volatile { return hi91.mag[0]; }
    inline float mag_y() volatile { return hi91.mag[1]; }
    inline float mag_z() volatile { return hi91.mag[2]; }

    // HI91 单位为 deg
    inline float roll()  volatile { return hi91.roll; }
    inline float pitch() volatile { return hi91.pitch; }
    inline float yaw()   volatile { return hi91.yaw; }

    inline float quat_w() volatile { return hi91.quat[0]; }
    inline float quat_x() volatile { return hi91.quat[1]; }
    inline float quat_y() volatile { return hi91.quat[2]; }
    inline float quat_z() volatile { return hi91.quat[3]; }

    /* ---- HI81 INS数据 (自动转换为物理量) ---- */
    inline bool valid_hi81() volatile { return hi81.tag == HIPNUC_ID_HI81; }

    inline float ins_roll()  volatile { return hi81.roll  * 0.01f; }
    inline float ins_pitch() volatile { return hi81.pitch * 0.01f; }
    inline float ins_yaw()   volatile { return hi81.yaw   * 0.01f; }

    inline float ins_qw() volatile { return hi81.quat[0] * 0.0001f; }
    inline float ins_qx() volatile { return hi81.quat[1] * 0.0001f; }
    inline float ins_qy() volatile { return hi81.quat[2] * 0.0001f; }
    inline float ins_qz() volatile { return hi81.quat[3] * 0.0001f; }

    inline double ins_lon() volatile { return hi81.ins_lon * 1e-7; }
    inline double ins_lat() volatile { return hi81.ins_lat * 1e-7; }
    inline float  ins_msl() volatile { return hi81.ins_msl * 1e-3f; }

    inline float vel_e() volatile { return hi81.vel_enu[0] * 0.01f; }
    inline float vel_n() volatile { return hi81.vel_enu[1] * 0.01f; }
    inline float vel_u() volatile { return hi81.vel_enu[2] * 0.01f; }

protected:
    /* ---- 状态机 ---- */
    int      nbyte;                            /* 已收字节数 */
    int      payload_len;                      /* 数据载荷长度 */
    uint8_t  raw_buf[HIPNUC_MAX_RAW_SIZE];     /* 帧缓冲区 */

    /* ---- 解码结果 ---- */
    hi91_t   hi91;
    hi81_t   hi81;
    hi83_t   hi83;

    /* ---- UART ---- */
    uart_manger_object *hi12_uart_mgr;

    /* ---- 内部方法 ---- */
    int8_t  feed_byte(uint8_t data) volatile;
    int8_t  decode() volatile;
    void parse_payload() volatile;

    static void crc16_ccitt(uint16_t *crc, const uint8_t *buf, uint32_t len);

    /* 字节序转换 (static inline, 在header实现) */
    static uint16_t read_u16(const uint8_t *p) {
        uint16_t v; memcpy(&v, p, 2); return v;
    }
    static uint32_t read_u32(const uint8_t *p) {
        uint32_t v; memcpy(&v, p, 4); return v;
    }
    static uint64_t read_u64(const uint8_t *p) {
        uint64_t v; memcpy(&v, p, 8); return v;
    }
    static float read_f32(const uint8_t *p) {
        float v; memcpy(&v, p, 4); return v;
    }
    static double read_f64(const uint8_t *p) {
        double v; memcpy(&v, p, 8); return v;
    }
};

extern volatile hi12_gyro hi12_data;
void hi12_gyro_uart_callback(uint8_t *buf, uint16_t len);
void h12_imu_init(uart_manger_object *uart_manager);