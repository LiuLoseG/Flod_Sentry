#pragma once

#include "bsp_uart.h"

#define WIT_UART huart10  //wit传感器使用的UART句柄
#define WIT_DATA_SIZE 11    //wit传感器一帧数据包的长度

typedef enum
{
    WIT_DATA_TYPE_TIME = 0x50,
    WIT_DATA_TYPE_ACC  = 0x51,
    WIT_DATA_TYPE_GYRO = 0x52,
    WIT_DATA_TYPE_ANGLE= 0x53,
    WIT_DATA_TYPE_MAG  = 0x54,
    WIT_DATA_TYPE_PORT = 0x55,
    WIT_DATA_TYPE_QUAT = 0x59 
} wit_data_type_e;  //wit传感器数据类型枚举

typedef struct
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} time_t;                   //时间


typedef struct 
{
    float accx;
    float accy;
    float accz;
    float Temperature;      //温度
} accelerate_t;             //加速度     

typedef struct 
{
    float gyrox;
    float gyroy;
    float gyroz;
    float Voltage;          //电压
} gyro_t;                   //角速度               

typedef struct 
{
    float roll;
    float pitch;
    float yaw;
    int16_t Version;        //固件版本号
} angel_t;                  //欧拉角              

typedef struct 
{
    float magx;
    float magy;
    float magz;
    float Temperature;      //温度
} magnetic_t;               //磁力计          

typedef struct 
{
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_t;            //四元数

class wit_gyro
{
    public:

        void wit_data_analysis(uint8_t *buf, uint16_t len) volatile; //数据解析函数

        inline float get_accx() volatile { return acc.accx; }
        inline float get_accy() volatile { return acc.accy; }
        inline float get_accz() volatile { return acc.accz; }

        inline float get_gyrox() volatile { return gyro.gyrox; }
        inline float get_gyroy() volatile { return gyro.gyroy; }
        inline float get_gyroz() volatile { return gyro.gyroz; }

        inline float get_roll() volatile { return angel.roll; }
        inline float get_pitch() volatile { return angel.pitch; }
        inline float get_yaw() volatile { return angel.yaw; }

        inline float get_quat_q0() volatile { return quat.q0; }
        inline float get_quat_q1() volatile { return quat.q1; }
        inline float get_quat_q2() volatile { return quat.q2; }
        inline float get_quat_q3() volatile { return quat.q3; }

    protected:
        quaternion_t quat; //四元数数据
        angel_t angel;     //欧拉角数据
        gyro_t gyro;       //角速度数据
        accelerate_t acc;  //加速度数据
        magnetic_t mag;    //磁力计数据
        time_t time;       //时间数据

};

extern volatile  wit_gyro wit_data;
void wit_gyro_uart_callback(uint8_t *buf, uint16_t len);
