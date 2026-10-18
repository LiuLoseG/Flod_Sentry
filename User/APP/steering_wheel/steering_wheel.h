#pragma once

#include "dm_motor.h"
#include "pid.h"
#include "wheel_system_solution.h"
#include "remote_center.h"
#include "slope_math.h"
#include "frequency_filter.h"

#define STEER_MAX_MOVE_SPEED 1.0f // 底盘最大线速度 m/s
#define STEER_MAX_ROTATE_SPEED 8.0f // 底盘最大角速度 rad/s
#define STEER_ANGULAR_FILTER_ALPHA 0.20f // 舵轮角速度EMA滤波系数, α=0.20→fc≈32Hz@1kHz, τ≈5ms, 相移-26°@16Hz

typedef enum
{
    Steer_Lock_Mode = 0, // 舵轮锁定模式
    Steer_Manual_Mode,   // 舵轮手动模式
}steer_mode_e;


typedef struct
{
    crsf_class *crsf; // 遥控器对象指针

    steer_mode_e steer_mode = Steer_Lock_Mode; // 舵轮模式

    dm_motor_normal_class dm_steer_motor; // 舵轮电机对象
    dm_motor_normal_class dm_drive_motor; // 驱动电机对象

    pid_class steer_angle_pid; // 舵轮角度PID控制器
    pid_class steer_speed_pid; // 舵轮速度PID控制器

    pid_class drive_pid; // 驱动PID控制器

    wheel_motor_status_t ref_status; // 轮电机状态结构体
    wheel_motor_status_t act_status; // 轮电机状态结构体
    float act_steer_angular[4]; // 4个舵轮的角速度值数组,用于舵轮控制

    float wheel_angular[4]; // 4个电机的电流值数组,电机控制值

    float steer_torque[4]; // 舵轮扭矩值,用于舵轮控制
    float steer_angular[4]; // 舵轮角速度值,用于舵轮控制
    class_filter_frequency<10> lowpass_filter[4];  // 4个电机的滤波器对象数组 (20阶→10ms延迟)

    wheel_robot_speed_t ref_speed_robot; // 机器人速度结构体

    slope_class slope_speed[3]; // 速度斜坡函数对象

    slope_class slope_angle[4]; // 角度斜坡函数对象

    float map_vx = 0.0f; // 底盘期望速度X轴方向
    float map_vy = 0.0f; // 底盘期望速度Y轴方向
    float map_vz = 0.0f; // 底盘期望角速度Z轴方向

} steering_wheel_t;