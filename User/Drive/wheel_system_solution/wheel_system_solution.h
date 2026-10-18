#pragma once

#include "user_lib.h"
#include <math.h>
#include "basic_math.h"
#include "arm_math.h"

#ifndef WHEEL_TYPE
#define WHEEL_TYPE 2 // 0: 全向轮; 1: 麦轮; 2: 舵轮; 3：半舵轮; 4: 差速轮
#endif

#define WHEEL_RADIUS_S 0.0761f // 轮子半径，单位m (全向轮实际半径)

#if WHEEL_TYPE == 0 // 全向轮物理参数
#define OMNI_WHEEL_DISTANCE_R 0.25075f      //全向轮在二维坐标的投影点上轮子到底盘圆心的距离，单位m

#elif WHEEL_TYPE == 1 // 麦轮物理参数
#define MECANUM_WHEEL_PROJECTION_X 0.12f     //在二维坐标的投影点上轮子在X轴方向的坐标，单位m
#define MECANUM_WHEEL_PROJECTION_Y 0.11f     //在二维坐标的投影点上轮子在Y轴方向的坐标，单位m

// 麦轮在水平面的投影距离和 (a+b), 用于角速度/角位移解算
#define MECANUM_WHEEL_SUM_XY (MECANUM_WHEEL_PROJECTION_X + MECANUM_WHEEL_PROJECTION_Y)

#elif WHEEL_TYPE == 2 // 舵轮物理参数
// #define STEER_WHEEL_PROJECTION_X 0.36902f     //在二维坐标的投影点上轮子在X轴方向的坐标，单位m
// #define STEER_WHEEL_PROJECTION_Y 0.36902f     //在二维坐标的投影点上轮子在Y轴方向的坐标，单位m

// 舵轮编码器零位角度 (rad), 用于舵轮绝对角度解算
static const float Enc_Zero_Angle[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 

#ifndef COSINE_OPTIMIZE_ENABLE
#define COSINE_OPTIMIZE_ENABLE 1 // 舵轮转向角余弦优化开关, 1:开启, 0:关闭
#endif

// 舵轮最大线速度，单位rad/s
#define MAX_WHEEL_SPEED 20.0f
#define MIN_LIMIT 0.1f // 舵轮最小线速度，单位rad/s
// 舵轮在二维坐标的投影点上轮子到底盘圆心的距离，单位m (两种优化配置都需要)
#define STEER_WHEEL_DISTANCE_R 0.15902f
#endif

// 轮电机逆运动学解算结构体
typedef struct
{

#if (WHEEL_TYPE == 0 || WHEEL_TYPE == 1) // 全向轮或麦轮运动学参数
    float wheel_speed[4]; // 轮子速度设置值，单位m/s

#elif WHEEL_TYPE == 2 // 舵轮运动学参数
    float now_angle[4];             // 舵向转向角总角度反馈值，单位rad
    float now_speed[4];             // 轮向速度反馈值，单位rad/s

    float solution_angle[4];        // 舵向转向角设置值，单位rad
    float solution_speed[4];        // 轮向速度设置值，单位rad/s

    float cosine_optimize[4];       // 转向角余弦优化值，单位无量纲
    float optimize_speed[4];        // 轮向转向角余弦优化后的速度，单位rad/s
    float optimize_angle[4];        // 舵向转向角余弦优化后的多圈目标角度，单位rad
    float optimize_direction[4];    // 舵向转向角余弦优化后的方向，单位无量纲
    float error_angle[4];           // 舵向转向角误差值，单位rad

#endif

} wheel_motor_status_t;

// 轮电机编码器状态，位置状态
typedef struct
{
    float wheel_angle[4]; // 轮子转向角反馈值，单位rad
} wheel_motor_encoder_t;

// 轮电机正运动学解算结构体
typedef struct
{
    float vx;       // 机器人在X轴方向的速度，单位m/s
    float vy;       // 机器人在Y轴方向的速度，单位m/s
    float vw;       // 机器人绕Z轴的角速度，单位rad/s
} wheel_robot_speed_t;

typedef struct
{
    float x;       // 机器人在X轴方向的位置，单位m
    float y;       // 机器人在Y轴方向的位置，单位m
    float yaw;     // 机器人绕Z轴的角度，单位弧度 (rad)
}wheel_robot_position_t;

#if WHEEL_TYPE == 1 // 麦轮运动学参数
/**
 * @brief 麦轮里程计
 */
typedef struct
{
    float last_angle_rad[4];   // 上一帧累计轮转角 (rad)
    wheel_robot_position_t pose;  // 里程计位姿 (x,y: m, yaw: rad)
    bool    initialized;       // 首次调用标志
} mecanum_encoder_odom_t;

/**
 * @brief 初始化麦轮编码器里程计
 * @param odom 里程计状态结构体指针
 * @note  初始化后首次调用 mecanum_encoder_odom_update 时自动完成编码器基准捕获
 */
void mecanum_encoder_odom_init(mecanum_encoder_odom_t *odom);

/**
 * @brief 麦轮编码器里程计更新 (实时调用, 通常 1kHz)
 * @param odom          里程计状态结构体指针 (含内部状态 & 输出位姿)
 * @param wheel_encoder 四个麦轮的编码器状态 (wheel_motor_encoder_t[4], 单位弧度rad)
 * @param imu_yaw_rad   IMU反馈的绝对航向角 (rad), 用于机体→世界坐标旋转,
 *                      替代轮式解算yaw以避免轮子打滑引起的方向混乱
 * @note  首次调用自动记录基准角度 (位移=0).
 *        轮转角索引: [0]=FL前左, [1]=RL后左, [2]=RR后右, [3]=FR前右.
 *        轮转角正方向需与驱动车体前进的方向一致.
 *        输出位姿写入 odom->pose (x,y: m; yaw: rad).
 */
void mecanum_encoder_odom_update(mecanum_encoder_odom_t *odom, wheel_motor_encoder_t *wheel_encoder, float imu_yaw_rad);

/**
 * @brief 初始化轮式速度里程计位置
 * @param position 位置结构体指针, 初始化为{0,0,0}
 */
void wheel_speed_odometry_init(wheel_robot_position_t *position);

/**
 * @brief 轮式里程计位置更新 (正运动学 + 位移积分)
 * @param position 当前位置结构体指针 (输入/输出), 上电时初始化为{0,0,0}
 * @param robot_speed 机器人当前速度 (vx,vy: body系 m/s, omega: rad/s)
 * @param dt 时间步长 (s)
 * @param imu_yaw_rad IMU反馈的绝对航向角 (rad), 用于机体→世界坐标旋转,
 *                    替代轮式解算yaw以避免轮子打滑引起的方向混乱
 * @note  坐标系: body X=前 Y=左, world 上电位置=(0,0) 上电朝向=前.
 *         yaw CCW为正 (右手则Z上), 单位弧度 (rad).
 */
void wheel_speed_odometry_update(wheel_robot_position_t *position, wheel_robot_speed_t *robot_speed, float dt, float imu_yaw_rad);

#elif WHEEL_TYPE == 2 // 舵轮运动学参数

#endif

#if WHEEL_TYPE == 0 // 全向轮
void omni_wheel_inverse_kinematics(wheel_motor_status_t *wheel_status, float vx, float vy, float vw);
void omni_wheel_speed_soluction(wheel_robot_speed_t *robot_speed, float *wheel_speed);

#elif WHEEL_TYPE == 1 // 麦轮
void mecanum_wheel_inverse_kinematics(wheel_motor_status_t *wheel_status, float vx, float vy, float vw);
void mecanum_wheel_position_soluction(wheel_robot_speed_t *robot_speed, float *wheel_speed);

#elif WHEEL_TYPE == 2 // 舵轮
void steer_wheel_inverse_kinematics(wheel_motor_status_t *wheel_status, float vx, float vy, float vw, uint8_t control_mode);
void steer_wheel_speed_soluction(wheel_robot_speed_t *robot_speed, float *wheel_angle, float *wheel_speed);
#endif
