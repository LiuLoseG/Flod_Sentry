/**
 * @file chassis.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 底盘控制头文件
 * @date 2026-09-06
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#pragma once

#include "bsp_can.h"

#include "pid.h"
#include "slope_math.h"
#include "general_filter.h"

#include "dji_motor_new.h"
#include "dm_motor.h"

#include "remote_center.h"
#include "can_distribution.h"
#include "wheel_system_solution.h"
#include "robot_def.h"

#include "robot_decision.h"

#include "power.h"
// #include "chassis_power.h"
#include "chassis_power_rls.h"
#include "safe_daemon_process.h"

#include "sys_phased_freq.h"
#include "mit_controller.h"

/**
 * @brief 底盘控制方式模式枚举
 */
typedef enum
{
    Chassis_Lock_Method = 0, // 底盘无力模式
    Chassis_Manual_Method,   // 底盘手动模式
    Chassis_Auto_Method,     // 底盘自动模式
} chassis_method_e;

/**
 * @brief 底盘控制模式枚举
 */
typedef enum
{
    Chassis_Zero_Force_Mode = 0,    // 底盘无力模式
    Chassis_Follow_Mode,            // 底盘跟随模式
    Chassis_Rotate_Mode,            // 底盘小陀螺
} chassis_mode_e;

typedef struct
{
    float angle; // 轮子转向角度，单位rad
    float speed; // 轮子线速度，单位m/s
} motor_status_t;

/**
 * @brief 底盘控制总结构体
 */
typedef struct
{
    // 遥控器对象指针
    crsf_class *crsf; 
    
    volatile power_data_t *power; // 功率计对象指针

    // 机器人决策对象指针
    robot_decision_class *decision; 

    // 底盘功率控制对象
    chassis_power_rls control_power; 

/*------------------------------------底盘整体目标和状态观测量---------------------------------------------*/

    // 目标底盘速度(设定的目标量)，单位m/s
    wheel_robot_speed_t target_robot_speed; 
    // 当前底盘速度(运动学正解算)，单位m/s
    wheel_robot_speed_t now_robot_speed;    
    // 底盘映射速度(旋转矩阵映射后的速度)，单位m/s
    wheel_robot_speed_t map_robot_speed;
    // 底盘实际的控制速度(斜坡函数输出)，单位m/s
    wheel_robot_speed_t slope_output_speed;

    // 角度偏移量(旋转矩阵)，将云台坐标系下的速度转换为底盘坐标系下的速度，单位rad
    float yaw_angular_offset; 

    // 速度斜坡函数对象
    slope_class slope_speed[3]; // [0]-X轴速度斜坡, [1]-Y轴速度斜坡, [2]-角速度斜坡 

/*------------------------------------电机控制量和测量量---------------------------------------------*/
    // 舵向电机角度环和速度环PID对象数组
    pid_class pid_steer_angle[4]; // 电机角度环PID对象数组
    pid_class pid_steer_omega[4]; // 电机速度环PID对象数组
    
    // 轮向电机速度环PID
    pid_class pid_wheel_omega[4];

    // 舵向
    motor_status_t now_steer_state[4]; // 当前舵向电机状态
    motor_status_t tgt_steer_state[4]; // 目标舵向电机状态

    // 轮向
    motor_status_t now_wheel_state[4]; // 当前轮向电机状态
    motor_status_t tgt_wheel_state[4]; // 目标轮向电机状态
/*------------------------------------电机目标和状态观测量---------------------------------------------*/

    // 目标轮电机速度设置值，单位m/s
    wheel_motor_status_t mtr_stat;
    // 4个电机的滤波器对象数组 (一阶IIR低通，截止频率30Hz)
    low_pass_filter_class lowpass_filter[4];  

    mit_speed_planner_class mit_speed_planner; // 目标转角速度规划器对象

/*------------------------------------电机对象创建---------------------------------------------*/
    dji_c620_motor_class wheel_motor[4]; // 轮电机对象数组
    dji_gm6020_motor_class steer_motor[4]; // 转向电机对象

/*------------------------------------功率对象创建---------------------------------------------*/
    float motor_target_output[4]; // 电机输出值数组, A
    float motor_target_omega[4];  // 电机目标角速度值数组, rad/s
    float motor_rotor_current[4]; // 电机转子电流值数组, A
    float motor_rotor_omega[4];   // 电机转子角速度值数组, rad/s
    bool motor_alive_status[4];   // 电机存活状态数组, 1表示存活, 0表示掉线

/*-------------------------------------底盘控制状态---------------------------------------------*/
    // 当前底盘控制方式
    chassis_method_e now_method;
    // 当前底盘控制模式 
    chassis_mode_e now_mode;

} chassis_t;
