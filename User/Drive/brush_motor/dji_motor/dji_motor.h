#pragma once

#include "bsp_can.h"
#include <stdbool.h>
#include "user_lib.h"
#include "basic_math.h"

#define DJI_ECD_TO_ANGLE 0.043945f      // (360/8192),将编码器值转化为角度制
#define DJI_ECD_TO_RAD   0.00076699f    // (2*3.1415926/8192),将编码器值转化为弧度制

    /*采用麻神的高阶滤波器进行数据处理，得到优化后的反馈数据*/
// #define RAD_SPEED_FILTER_COEF 0.88f             // 弧度值一阶高通滤波系数
// #define ANGLE_SPEED_FILTER_COEF 0.88f           // 角度值一阶高通滤波系数
// #define CURRENT_FILTER_COEF 0.91f               // 电流值一阶低通滤波系数

#define DJI_6020_VOLTAGE_OUT 25000.0f / 24.0f
#define DJI_6020_CURRENT_OUT 16384.0f / 3.0f
#define DJI_3508_CURRENT_OUT 16384.0f / 20.0f
#define DJI_2006_CURRENT_OUT 10000.0f / 10.0f

#define DJI_6020_3508_CURRENT 16384.0f
#define DJI_6020_VOLTAGE 16384.0f
#define DJI_2006_CURRENT 10000.0f

#define DJI_2006_REDUCTION_RATIO 36.0f    // 2006电机的减速比
#define DJI_3508_REDUCTION_RATIO 19.2f    // 3508电机的减速比
#define DJI_6020_REDUCTION_RATIO 1.0f     // 6020电机的减速比

typedef enum
{
    DJI_L3508_L2006_FB_ID = 0x200,          //GM3508、GM2006(ID=1~4)、TX_ID
    DJI_L6020V_H3508_H2006_ID = 0x1FF,      //GM6020(ID=1~4)电压控制、GM3508(ID=5~8)、GM2006(ID=5~8)
    DJI_H6020V_ID = 0x2FF,                  //GM6020(ID=5~8)电压控制

    DJI_L6020I_ID = 0x1FE,                  //GM6020(ID=1~4)电流控制
    DJI_H6020I_ID = 0x2FE,                  //GM6020(ID=5~8)电流控制
    DJI_6020_FB_ID = 0x204,                 //GM6020(ID=1~8)反馈ID  
}dji_motor_identifier_e;

typedef enum
{
    DJI_2006 = 0,
    DJI_3508,
    DJI_6020,
}dji_motor_type_e;

typedef enum
{
    RPM_FILTER = 0,         // (rpm)为单位的转速滤波
    RAD_FILTER,             // (rad/s)为单位的转速滤波
    ANGLE_FILTER,           // (°/s)值为单位的转速滤波
}dji_motor_filter_type_e;

// 获得的原始电机反馈数据
typedef struct
{
    uint16_t last_ecd;          // 上次的编码器值
    uint16_t encoder;           // 当前的编码器值
    float single_round_angle;   // 单圈角度值 (度)
    float single_round_rad;     // 单圈角度值 (弧度)

    float total_angle;          // 电机总角度值 (度)
    float total_rad;            // 电机总角度值 (弧度)
    int32_t total_round;          // 电机总圈数

    float rotate_speed;         // 电机转速值(rpm,每分钟转多少圈)

    float angle_speed;          // 电机角速度值 (度/s)
    float angular_speed;        // 电机角速度值 (rad/s)

    int16_t electric;           // 电机电流值

    uint8_t temperature;          // 电机温度值
}dji_motor_rx_fdpara_t;

// 滤波后的电机反馈数据
typedef struct 
{
    float filter_rotate_speed;         // 电机转速值(rpm,每分钟转多少圈)
    float filter_angle_speed;          // 电机角速度值 (度/s)
    float filter_angular_speed;        // 电机角速度值 (rad/s)

    int16_t filter_electric;           // 电机电流值
} dji_motor_filter_para_t;

typedef struct 
{
    uint16_t identifier_id;                 // 电机can id
    dji_motor_type_e type;                  // 电机型号
    dji_motor_rx_fdpara_t rx_fdpara;        // 电机反馈数据
    dji_motor_filter_para_t filter_para;    // 电机滤波后的反馈数据
    uint32_t last_fb_time;                  // 上次接收反馈的时间
    bool motor_offline;                     // 电机是否离线
    bool first_fb_flag;                     // 首帧反馈标志，防止上电时误计圈
    float current_to_torque_ratio;            // 电流转扭矩系数
    float motor_control_max;                     // 电机控制最大值
}dji_motor_t;

void dji_motor_init(dji_motor_t *p_motor, uint16_t can_id, dji_motor_type_e motor_type, dji_motor_identifier_e identifier);
void dji_motor_fbdata(dji_motor_t* motor, uint8_t *rx_data);
// void dji_motor_ctrl(FDCAN_HandleTypeDef* hfdcan, dji_motor_identifier_e identifier, int16_t value_1, int16_t value_2, int16_t value_3, int16_t value_4);
void dji_motor_filter_fbdata(dji_motor_t* motor, dji_motor_filter_type_e filter_type, float filter_value);
void dji_motor_ctrl(FDCAN_HandleTypeDef* hfdcan, dji_motor_t *motor, dji_motor_identifier_e identifier, float value_1, float value_2, float value_3, float value_4);