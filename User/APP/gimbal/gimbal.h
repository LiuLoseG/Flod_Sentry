#pragma once

#include "dm_motor.h"
#include "dji_motor_new.h"
#include "lk_motor.h"
#include "hi12_imu.h"
#include "remote_center.h"

#include "pid.h"
#include "arm_math.h"
#include "frequency_filter.h"

#include "robot_def.h"
#include "basic_math.h"
#include "controller.h"

#include "robot_decision.h"
#include "general_filter.h"
#include "wave_generator.h"
#include "rls.h"
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
    Gimbal_Auto_Disable = 0,
    Gimbal_Auto_Follow = 1,
    Gimbal_Auto_Scan = 2,
} gimbal_mode_e;

/**
 * @brief 云台角度、角速度结构体
 * @note 云台目标和观测的角度、角速度(反馈来源：陀螺仪)
 */
typedef struct
{
    float gimbal_yaw_angle;          // 云台偏航角度(来自于IMU)
    float small_yaw_angle;          // 小yaw云台偏航角度(来自于IMU)
    float big_yaw_angle;            // 大yaw云台偏航角度(来自于IMU)
    float pitch_angle;              // 云台俯仰角度(来自于IMU)

    float small_yaw_omega;          // 小yaw云台偏航角速度(来自于IMU加速度)
    float big_yaw_omega;            // 大yaw云台偏航角速度(来自于IMU加速度)
    float pitch_omega;              // 云台俯仰角速度(来自于IMU加速度)
} gimbal_observate_control_t;

/**
 * @brief 云台电机前馈结构体
 * @note 云台电机前馈量，用于补偿电机的惯性和负载变化，提升控制性能
 */
typedef struct
{
    float chassis_angular; // 底盘角速度补偿前馈量
    float big_yaw_omega;   // 大yaw电机角速度前馈
    float small_yaw_omega; // 小yaw电机角速度前馈
    float pitch_omega;     // pitch电机角速度前馈

    float big_yaw_torque;   // 大yaw电机力矩前馈
    float small_yaw_torque; // 小yaw电机力矩前馈
    float pitch_torque;     // pitch电机力矩前馈
} gimbal_motor_feedforward_t;

/**
 * @brief 云台PID结构体
 * @note 云台角度和角速度的PID控制器，用于实现精确的姿态控制
 */
typedef struct
{
    pid_class pid_angle; // 云台角度PID控制器
    pid_class pid_omega; // 云台角速度PID控制器
} gimbal_pid_t;

/**
 * @brief pitch轴重力补偿结构体
 * @note 用于补偿pitch轴的重力影响，提升控制精度
 */
typedef struct
{
    // 多项拟合系数(3:三次项系数 2:二次项系数 1:一次项系数 0:常数系数)
    float factor[4]; // pitch轴重力补偿系数
    float amount; // pitch轴重力补偿量
} pitch_gravity_compensation_t;

/**
 * @brief 云台结构体
 * @note 云台的主结构体，包含所有云台相关的数据和控制逻辑
 */
typedef struct
{
    crsf_class *crsf; // 遥控器对象指针
    volatile hi12_gyro *hi12_imu;                   // 陀螺仪对象指针 (volatile: 中断中更新)  
    gimbal_decision_source_t *source; // 云台控制用于决策的源数据
    robot_decision_class *decision; // 机器人决策对象指针

    // Core::Control::Math::RLS<2> rls{1e-5f, 0.99999f};
/*-----------------------------------当期值、目标值、滤波值获取---------------------------------------------*/
    gimbal_observate_control_t now_value;          // 云台当前角度、角速度
    gimbal_observate_control_t filter_value;       // 云台滤波后的角度、角速度
    gimbal_observate_control_t target_value;       // 云台目标角度、角速度

    // 扫频索敌
    float now_s_yaw_angle;

    // yaw,pitch角度扫描方向标志
    uint8_t pit_towards_flag;
    uint8_t yaw_towards_flag;

    // 前馈量相关
    float prev_target_small_yaw_angle;                   // 上一帧目标yaw角度, 用于计算角速度前馈
    float prev_big_yaw_omega;                            // 上一帧大yaw角速度指令(经速率限制后), 平滑Follow/Scan切换阶跃
    uint32_t big_yaw_omega_rl_cnt;                       // 大yaw角速度指令速率限制器的DWT计数缓存
    float prev_small_yaw_ff_omega;                       // 上一帧小yaw角速度前馈(经速率限制后), 用于平滑拐点
    float prev_pitch_ff_torque;                          // 上一帧pitch力矩前馈(经速率限制后), 用于平滑目标抖动
    float yaw_small_ff_dc_mean;                          // DC blocker 均值跟踪, 用于去除前馈目标角度的直流偏置
    float yaw_small_ff_enc_lpf;                          // 前馈参考专用编码器角低通(仅用于前馈通道, 抑制13bit量化被B项微分放大)
    float yaw_small_ff_delta_lpf;                        // 前馈参考专用"视觉跟踪误差"低通(抑制PC零阶保持台阶被B项微分放大)

    float pitch_ff_ref_lpf;                              // pitch前馈参考专用低通(仅前馈通道, 抑制PC零阶保持台阶被B项微分放大) [数据平滑]
    float prev_pitch_ff_omega;                           // 上一帧pitch角速度前馈(经拐点缓冲后), 用于平滑拐点 [拐点缓冲]
    
    gimbal_motor_feedforward_t feedforward_value;       // 云台电机前馈量

    float pitch_limit_angle;
/*------------------------------------底盘角速度补偿前馈---------------------------------------------*/
    // 底盘角速度低通滤波器, 用于抑制底盘角速度的高频噪声对云台控制的影响
    low_pass_filter_class chassis_angular_lpf; 
/*------------------------------------自瞄低通滤波器---------------------------------------------*/
    low_pass_filter_class filter_yaw;
    low_pass_filter_class filter_pitch;
/*------------------------------------重力/摩擦力补偿量---------------------------------------------*/
    pitch_gravity_compensation_t pitch_gravity_comp; // pitch轴重力补偿量
    low_pass_filter_class pitch_gravity_comp_filter; // pitch轴重力补偿量低通滤波器
    // Core::Control::Math::RLS<4> rls{1e-5f, 0.99999f};
    // Matrixf<4, 1> samples;
    // Matrixf<4, 1> params;
/*----------------------------------遥控器控制PID---------------------------------------------*/
    pid_class rc_yaw_big_angle_pid;            // 大yaw电机角度PID控制器
    gimbal_pid_t rc_yaw_small_pid;             // 小yaw电机PID控制器
    gimbal_pid_t rc_pitch_pid;                 // pitch电机PID控制器
/*------------------------------------自瞄控制PID---------------------------------------------*/
    gimbal_pid_t pc_yaw_big_pid;                // 大yaw电机角度PID控制器
    gimbal_pid_t pc_yaw_small_pid;             // 小yaw电机PID控制器
    gimbal_pid_t pc_pitch_pid;                 // pitch电机PID控制器
/*------------------------------------自瞄前馈补偿量---------------------------------------------*/
    Feedforward_t pc_pitch_ffc;                // pitch电机前馈补偿量
    Feedforward_t pc_yaw_small_ffc;            // 小yaw电机前馈补偿量
/*------------------------------------电机对象创建---------------------------------------------*/
    lk_motor_normal_class yaw_big_motor;           // 大yaw电机对象
    dji_gm6020_motor_class yaw_small_motor;        // 小yaw电机对象
    dm_motor_normal_class pitch_motor;             // pitch电机对象
/*------------------------------------云台控制方式---------------------------------------------*/
    gimbal_method_e now_method; // 云台控制方式
    gimbal_method_e last_method; // 云台上一次控制方式, 用于检测模式切换

    gimbal_mode_e now_mode; // 云台当前控制模式
    gimbal_mode_e last_mode; // 云台上次控制模式

    //等待IMU稳定状态
    uint16_t imu_frame_cnt;     // 已接收的有效 HI91 帧计数
    uint8_t  imu_attitude_ready; // 1 = IMU 姿态数据已稳定 (滤波器已收敛)

    // 限位标定状态
    uint8_t  limit_offset_ready; // 1 = 小yaw限位零点偏移已标定并冻结
/*------------------------------------轨迹规划测试---------------------------------------------*/
    reference_track_plan_t yaw_small_track_plan; // 小yaw云台参考轨迹规划器
    reference_value_t yaw_small_plan_output; // 小yaw云台参考轨迹规划器输出

} gimbal_t;

