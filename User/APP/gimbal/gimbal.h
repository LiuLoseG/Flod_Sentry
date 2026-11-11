/**
 * @file gimbal.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-11-08
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#pragma once

#include "dm_motor.h"
#include "imu_temp_ctrl.h"
#include "hi12_imu.h"
#include "remote_center.h"

#include "pid.h"
#include "arm_math.h"
#include "slope_math.h"
#include "frequency_filter.h"

#include "robot_def.h"
#include "basic_math.h"

#include "robot_decision.h"
#include "general_filter.h"
#include "wave_generator.h"
#include "gimbal_identification.h"
#include "reference_track_plan.h"

/**
 * @brief 云台模式枚举
 */
typedef enum
{
    Gimbal_Method_Zero_Force = 0,     // 云台无力模式，全部控制量设置为0
    Gimbal_Method_Manual = 1,         // 云台手动模式
    Gimbal_Method_Auto = 2,           // 云台自动模式
} gimbal_method_e;

/**
 * @brief 云台控制模式
 */
typedef enum
{
    Gimbal_Auto_Disable = 0,        // 自瞄关闭
    Gimbal_Auto_Follow = 1,         // 自瞄跟随 
    Gimbal_Auto_Scan = 2,           // 自瞄扫描
} gimbal_mode_e;

/**
 * @brief 大pitch电机状态
 */
typedef enum
{
    Gimbal_Big_Pitch_Down = 0,        // 大pitch电机位于下方
    Gimbal_Big_Pitch_Progress = 1,    // 大pitch电机正在运动
    Gimbal_Big_Pitch_Up = 2,          // 大pitch电机位于上方
} gimbal_big_pitch_status_e;

/**
 * @brief 云台原始控制数据输入和滤波值
 */
typedef struct
{
    float yaw_angle;          // 目标偏航角度
    float pitch_angle;        // 目标俯仰角度

    float yaw_omega;          // 目标偏航角速度
    float pitch_omega;        // 目标俯仰角速度

    float yaw_torque;         // 目标偏航力矩
    float pitch_torque;       // 目标俯仰力矩
} gimbal_input_control_t;

/**
 * @brief 云台角度、角速度结构体
 * @note 云台目标和观测的角度、角速度(反馈来源：陀螺仪)
 */
typedef struct
{
    float gimbal_yaw_angle;          // 云台偏航角度(IMU)

    float small_yaw_angle;           // 小yaw角度
    float small_pitch_angle;         // 小pitch角度
    float big_yaw_angle;             // 大yaw角度
    float big_pitch_angle;           // 大pitch角度

    float small_yaw_omega;           // 小yaw角速度
    float small_pitch_omega;         // 小pitch角速度
    float big_yaw_omega;             // 大yaw角速度
    float big_pitch_omega;           // 大pitch角速度

    gimbal_big_pitch_status_e big_pitch_status;           // 大pitch电机状态

} gimbal_observate_control_t;

/**
 * @brief 云台电机前馈结构体
 * @note 云台电机前馈量，用于补偿电机的惯性和负载变化，提升控制性能
 */
typedef struct
{
    float chassis_angular; // 底盘角速度补偿前馈量

    float small_yaw_omega; // 小yaw电机角速度前馈
    float small_pitch_omega; // 小pitch电机角速度前馈
    float big_yaw_omega;   // 大yaw电机角速度前馈
    float big_pitch_omega; // 大pitch电机角速度前馈

    float small_yaw_torque; // 小yaw电机力矩前馈
    float small_pitch_torque; // 小pitch电机力矩前馈
    float big_yaw_torque;   // 大yaw电机力矩前馈
    float big_pitch_torque; // 大pitch电机力矩前馈

} gimbal_motor_feedforward_t;

/**
 * @brief 云台自瞄低通滤波器结构体
 */
typedef struct
{
    low_pass_filter_class yaw_angle;
    low_pass_filter_class pitch_angle;

    low_pass_filter_class yaw_omega;
    low_pass_filter_class pitch_omega;

    low_pass_filter_class yaw_torque;
    low_pass_filter_class pitch_torque;
} gimbal_low_pass_filter_t;

/**
 * @brief 云台任务两次反馈时间s
 */
typedef struct
{
    float small_yaw_s;     // 小yaw电机反馈时间戳(s)
    float small_pitch_s;   // 小pitch电机反馈时间戳(s)
    float big_yaw_s;       // 大yaw电机反馈时间戳(s)
    float big_pitch_s;     // 大pitch电机反馈时间戳(s)
} gimbal_feedback_timestamp_t;

/**
 * @brief 云台电机反馈时间戳us
 */
typedef struct
{
    uint32_t small_yaw_us;     // 小yaw电机反馈时间戳(us)
    uint32_t small_pitch_us;   // 小pitch电机反馈时间戳(us)
    uint32_t big_yaw_us;       // 大yaw电机反馈时间戳(us)
    uint32_t big_pitch_us;     // 大pitch电机反馈时间戳(us)
} gimbal_timestamp_us_t;

/**
 * @brief 云台PID结构体
 * @note 云台角度和角速度的PID控制器，用于实现精确的姿态控制
 */
typedef struct
{
    pid_class angle; // 云台角度PID控制器
    pid_class speed; // 云台角速度PID控制器
} gimbal_pid_t;

/**
 * @brief 云台结构体
 * @note 云台的主结构体，包含所有云台相关的数据和控制逻辑
 */
typedef struct
{

/*-------------------------------------总观测指针--------------------------------------------*/    
    crsf_class *crsf;                   // 遥控器对象指针
    volatile imu_data_resolve_t *imu_data; // IMU数据对象指针 (volatile: 中断中更新)
    volatile hi12_gyro *hi12_imu;       // 陀螺仪对象指针 (volatile: 中断中更新)  
    gimbal_decision_source_t *source;   // 云台控制用于决策的源数据
    robot_decision_class *decision;     // 机器人决策对象指针

/*------------------------------当期值、目标值、滤波值获取-------------------------------------*/
    // 传感器当前值
    gimbal_observate_control_t now_val;
    // 传感器滤波值          
    gimbal_observate_control_t now_fil_val;    

    // 原始输入目标值
    gimbal_input_control_t input_val;
     // 原始输入目标值滤波
    gimbal_input_control_t input_fil_val;

    // 控制量目标值   
    gimbal_observate_control_t target_val;  

    // 前馈量     
    gimbal_motor_feedforward_t ffd_val;  
    // 前馈滤波量
    gimbal_motor_feedforward_t ffd_fil_val;  

    // 小pitch轴IMU限辐角度
    float small_pitch_imu_limit_angle; 
    // 大pitch电机限辐角度
    bool big_pitch_ready; 

    // 小yaw,小pitch角度扫描方向标志
    bool small_pit_towards_flag;
    bool small_yaw_towards_flag;

    float big_yaw_rho; // 大yaw跟随小yaw的趋近率
    slope_class big_yaw_omega_slope; // 大yaw角速度斜坡规划器

/*------------------------------------控制低通滤波器---------------------------------------------*/
    // 云台自瞄低通滤波器对象
    gimbal_low_pass_filter_t pc_filter;  
    // 云台自瞄轨迹规划器低通滤波器对象
    gimbal_low_pass_filter_t plan_filter;
    // 底盘角速度补偿前馈低通滤波器对象
    low_pass_filter_class chassis_angular_filter;

/*------------------------------------重力/摩擦力+系统辨识参数----------------------------------------*/
    gimbal_identification_t ident_small_yaw;
    gimbal_identification_config_t ident_small_yaw_config;
    gimbal_identification_sample_t ident_small_yaw_sample;
    gimbal_identification_output_t ident_small_yaw_output;

    gimbal_identification_t ident_small_pitch;
    gimbal_identification_config_t ident_small_pitch_config;
    gimbal_identification_sample_t ident_small_pitch_sample;
    gimbal_identification_output_t ident_small_pitch_output;
    
/*--------------------------------------轨迹规划器---------------------------------------------*/
    reference_config_t small_yaw_plan_config; // 小yaw轨迹规划器配置
    reference_track_plan_t small_yaw_plan; // 小yaw轨迹规划器
    reference_value_t small_yaw_plan_output; // 小yaw轨迹规划器输出

    reference_config_t small_pitch_plan_config; // 小pitch轨迹规划器配置
    reference_track_plan_t small_pitch_plan; // 小pitch轨迹规划器
    reference_value_t small_pitch_plan_output; // 小pitch轨迹规划器输出

/*----------------------------------遥控器控制PID---------------------------------------------*/
    gimbal_pid_t rc_small_yaw_pid;
    gimbal_pid_t rc_small_pitch_pid;

    gimbal_pid_t rc_big_yaw_pid;
    gimbal_pid_t rc_big_pitch_pid;

/*-----------------------------------自瞄控制PID----------------------------------------------*/
    gimbal_pid_t pc_small_yaw_pid;
    gimbal_pid_t pc_small_pitch_pid;

    gimbal_pid_t pc_big_yaw_pid;
    gimbal_pid_t pc_big_pitch_pid;

/*------------------------------------电机对象创建---------------------------------------------*/
    dm_motor_normal_class small_yaw_motor;    // 小yaw电机对象
    dm_motor_normal_class small_pitch_motor;  // 小pitch电机对象

    dm_motor_normal_class big_yaw_motor;      // 大yaw电机对象
    dm_motor_normal_class big_pitch_motor;    // 大pitch电机对象

    gimbal_feedback_timestamp_t feedback_timestamp; // 云台电机反馈时间(s)
    gimbal_timestamp_us_t timestamp; // 云台电机反馈时间戳(us)
/*------------------------------------云台控制方式---------------------------------------------*/
    gimbal_method_e now_method; // 云台控制方式
    gimbal_method_e last_method; // 云台上一次控制方式, 用于检测模式切换

    gimbal_mode_e now_mode; // 云台当前控制模式
    gimbal_mode_e last_mode; // 云台上次控制模式

    bool control_ready; // 云台任务自检

} gimbal_t;