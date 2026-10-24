#pragma once

#include "user_lib.h"
#include <math.h>
#include <stdio.h>
#include "basic_math.h"
#include "arm_math.h"

#ifndef WHEEL_TYPE
#define WHEEL_TYPE 2 // 0: 全向轮; 1: 麦轮; 2: 舵轮; 3：半舵轮; 4: 差速轮
#endif

#define WHEEL_RADIUS_S 0.0761f // 轮子半径，单位m (全向轮实际半径)
// 舵轮驱动轮实际半径(m)。逆解 / 正解 / 里程计都必须用它, 保证"指令→实际→反馈"三者一致。
// 默认等于 WHEEL_RADIUS_S (0.0761, 全向轮半径), 即当前行为不变; 务必按实车标定后改这里(单点标定)。
#ifndef STEER_WHEEL_RADIUS
#define STEER_WHEEL_RADIUS WHEEL_RADIUS_S
#endif

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

// 舵轮方位角 (rad), 用于舵轮角速度/角位移解算
static const float Wheel_Azimuth[4] = {PI / 4.0f, (3.0f * PI) / 4.0f, (5.0f * PI) / 4.0f, (7.0f * PI) / 4.0f}; // 轮组方位角 (rad)

// 各轮心到底盘中心的距离(旋转力臂半径, m), 用于舵轮角速度/角位移解算
static const float Wheel_To_Core_Distance[4] = {0.239f, 0.239f, 0.239f, 0.239f};
// 如果为正方形舵轮与上面的中心距等效
#define STEER_WHEEL_DISTANCE_R 0.15902f

#ifndef COSINE_OPTIMIZE_ENABLE
#define COSINE_OPTIMIZE_ENABLE 1 // 舵轮转向角余弦优化开关, 1:开启, 0:关闭
#endif

// 舵轮最大线速度，单位rad/s
#define MAX_WHEEL_SPEED 25.0f
#define MIN_LIMIT 0.1f // 舵轮最小线速度，单位rad/s

#endif

typedef enum
{
    IMU_None = 0,       // 无IMU
    IMU_Single_Axis,    // 单轴IMU (仅提供航向角)
    IMU_Multi_Axis,     // 多轴IMU (提供航向角 + 线加速度 + 角速度)
    IMU_Nine_Axis       // 九轴IMU (提供航向角 + 线加速度 + 角速度 + 磁力计)
} IMU_Odometry_TypeDef;

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
    float wheel_angle[4]; // 轮子多圈累计角度，单位rad
    float steer_angle[4]; // 舵向多圈累计角度，单位rad
} wheel_system_encoder_t;

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

// 舵轮编码器里程计
typedef struct
{
    // float last_steer_angle[4];   // 上一帧累计舵向角 (rad)
    // float last_wheel_angle[4];   // 上一帧累计轮向角 (rad)

    IMU_Odometry_TypeDef imu_type; // IMU类型
    wheel_system_encoder_t last_encoder; // 上一帧累计编码器角度 (rad)
    wheel_robot_position_t pose;  // 里程计位姿 (x,y: m, yaw: rad)
    bool    initialized;       // 首次调用标志
} steer_encoder_odom_t;

/**
 * @brief 初始化舵轮编码器里程计状态
 * @param odom     里程计状态结构体指针
 * @param imu_odom IMU 类型 (IMU_None 表示无 IMU, 航向由编码器解算)
 */
void steer_encoder_odom_init(steer_encoder_odom_t *odom, IMU_Odometry_TypeDef imu_odom);

/**
 * @brief 舵轮编码器里程计更新 (实时调用, 通常 1kHz)
 * @param odom              里程计状态结构体指针 (含内部状态 & 输出位姿)
 * @param wheel_encoder_rad 四个舵轮的编码器状态 (wheel_system_encoder_t*, 单位 rad)
 *                          wheel_angle[i] 轮向累计转角, steer_angle[i] 舵向累计转角
 * @param imu_yaw_rad       IMU 绝对航向角 (rad); imu_type == IMU_None 时不使用,
 *                          此时航向由舵轮运动学解算的角位移累加得到
 * @note  首次调用自动记录基准角度并对齐航向 (位移=0).
 *        轮组布局: [0]=FL [1]=RL [2]=RR [3]=FR, 投影 a=STEER_WHEEL_DISTANCE_R*SQRT_2_2.
 *        输出位姿写入 odom->pose (x,y: m; yaw: rad).
 */
void steer_encoder_odom_update(steer_encoder_odom_t *odom, wheel_system_encoder_t *wheel_encoder_rad, float imu_yaw_rad);

/**
 * @brief 输出轮式里程计位姿 (x,y,yaw) 到主机, 便于在线观察/标定
 * @note  仅在主循环中低频调用(会阻塞在 printf), 切勿在 1kHz 控制中断里调用
 */
void odom_debug_printf(const steer_encoder_odom_t *odom);


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
