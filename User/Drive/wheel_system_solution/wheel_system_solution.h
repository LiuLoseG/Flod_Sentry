/**
 * @file wheel_system_solution.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-10-30
 * @version 1.47
 * @copyright HNAU-Robot (c) 2026
 * @note 正方向定义: 机器人坐标系 +X 向前, +Y 向左, +Z 向上; 角速度绕 Z 轴 CCW 为正
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "user_lib.h"
#include "basic_math.h"
#include "arm_math.h"
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 机器人速度 (机体系): 逆解输入 / 正解输出, vx/vy: m/s, vw: rad/s
 */
typedef struct
{
    float vx; // +X 向前, m/s
    float vy; // +Y 向左, m/s
    float vw; // 绕 +Z 逆时针 (CCW) 为正, rad/s
} wheel_robot_speed_t;

/**
 * @brief 机器人加速度 (机体系): 力控与力速混控的目标加速度, ax/ay: m/s^2, aw: rad/s^2
 * @note 由 set_robot_accel_x/y/w() 逐轴写入, 模块内按
 *        F = m·a、M = J·α 换算成等效力/力矩后再分配, 因此质量/惯量是各轮系类的
 */
typedef struct
{
    float ax; // +X 向前, m/s^2
    float ay; // +Y 向左, m/s^2
    float aw; // 绕 +Z 逆时针 (CCW) 为正, rad/s^2
} wheel_robot_accel_t;

/**
 * @brief 机器人位姿 (世界系): x/y: m, yaw: rad
 * @note  yaw 折到 [-π, π); 描述的是**底盘中心**, 其它参考点需上层自行平移.
 */
typedef struct
{
    float x;   // 世界系 X 位置, m
    float y;   // 世界系 Y 位置, m
    float yaw; // 世界系航向角, rad (CCW 为正, [-π, π))
} wheel_robot_position_t;

/**
 * @brief 底盘控制方式 (init 首参选定, 决定上层每拍调哪条流水线)
 */
typedef enum
{
    Wheel_Speed_Control_Method = 0, // 纯速控: 逆解给轮速/舵向目标, 不做力矩前馈
    Wheel_Force_Control_Method,     // 纯力控: 加速度 -> 轮端力矩, 舵轮另给舵向对齐目标
    Wheel_Mixed_Control_Method,     // 力速混控: 逆解给轮速/舵向目标 + 加速度 -> 力矩前馈
} wheel_control_method_e;

/**
 * @brief 轮向电机测量状态: 多圈累计转角(编码器里程计输入) + 轮向角速度(正/逆解输入输出)
 */
typedef struct
{
    float angle[4]; // 四个轮子多圈累计转角, rad ([0]=FL [1]=RL [2]=RR [3]=FR)
    float speed[4]; // 四个轮子轮向角速度, rad/s ([0]=FL [1]=RL [2]=RR [3]=FR)
} wheel_state_t;

/**
 * @brief 舵向电机测量状态: 多圈累计转角 + 舵向角速度, rad / rad/s (以 +X 为 0, CCW 为正)
 */
typedef struct
{
    float angle[4]; // 四个舵向累计转角, rad ([0]=FL [1]=RL [2]=RR [3]=FR)
    float speed[4]; // 四个舵向电机角速度, rad/s ([0]=FL [1]=RL [2]=RR [3]=FR)
} steer_state_t;

/**
 * @brief 舵轮解算状态
 */
typedef struct
{
    // 当前角度和角速度
    float now_angle[4];          // 舵向当前多圈角度反馈, rad
    float now_speed[4];          // 轮向当前角速度反馈, rad/s

    // 速控解算数据(不进行优化的控制参数)
    float solution_angle[4];     // 舵向几何解算角, rad (单圈几何角, 未加零位)
    float solution_speed[4];     // 轮向未优化目标速度, rad/s

    // 优化后的输出参数
    float optimize_angle[4];     // 舵向目标角, rad (多圈, 含零位; 同时是下一拍斜坡基准)
    float optimize_speed[4];     // 轮向目标速度, rad/s (已含余弦削减与方向符号; 纯力控为 0)

    float optimize_direction[4]; // 轮向方向, ±1 (就近转位的翻转状态, 兼作下一拍迟滞记忆; 速度域, 只由逆解写)
    float cosine_optimize[4];    // 余弦削减系数(幅值), 无量纲 [0,1] (1 = 已对齐不削减; 速控与力控共用)
    float error_angle[4];        // 真实未对齐角, rad (相对所选方向分支, 0 表示已对齐; 速控与力控共用)
    float force_direction[4];    // 轮端出力方向, ±1 (+1 = 沿当前舵向 +û(θ) 推底盘; 只由力控写,
                                 //   纯力控 = 就近转位分支, 混控 = 力投影 sign(F·û(θ)), 与轮向方向无关)

    // 力控轮输出力矩
    float wheel_torque[4];       // 四轮轮端目标力矩, N·m ([0]=FL [1]=RL [2]=RR [3]=FR)
} steer_wheel_status_t;

/*-----------------------------------omni_wheel-----------------------------------*/

/**
 * @brief 全向轮运动学解算与编码器里程计
 */
class omni_wheel_class
{
public:

    void init(
              // 控制模式和控制参数
              const wheel_control_method_e &__control_method = Wheel_Speed_Control_Method,
              const float &__wheel_encoder_direction = -1.0f,
              const float &__control_period = 0.001f,
              // 力控死区和最大力矩限制
              const float &__max_wheel_torque = 4.9f,
              const float &__force_deadband = 0.1f);

    // 获取单轮角速度
    inline float get_wheel_speed(const uint8_t &__index) const;

    // 运动学正解(vx/vy/vw: m/s, rad/s)
    inline float get_robot_speed_vx() const;
    inline float get_robot_speed_vy() const;
    inline float get_robot_speed_vw() const;

    // 编码器里程计位姿(编码器累计) (x/y: m, yaw: rad)
    inline float get_position_x() const;
    inline float get_position_y() const;
    inline float get_position_yaw() const;

    // 速度里程计位姿(速度积分) (x/y: m, yaw: rad)
    inline float get_speed_position_x() const;
    inline float get_speed_position_y() const;
    inline float get_speed_position_yaw() const;

    // 轮端目标力矩, N·m
    inline float get_wheel_torque(const uint8_t &__index) const;

    // 轮向电机方向: +1 
    inline void set_wheel_encoder_direction(const float &__direction);
    // 目标加速度, m/s^2 / rad/s^2
    inline void set_robot_accel_x(const float &__accel_x);
    inline void set_robot_accel_y(const float &__accel_y);
    inline void set_robot_accel_w(const float &__accel_w);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);
    // 单轮角速度(正解输入)
    inline void set_wheel_speed(const uint8_t &__index, const float &__speed);

    // 更新底盘目标速度
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed);

    // 更新四轮当前测量状态
    inline void update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad = NAN);


    // 速控: 关断力控输出 + 逆解 -> 轮向目标角速度
    void speed_calculate_callback();
    // 力控: 目标加速度 -> 轮端力矩 (不使用速度目标)
    void force_calculate_callback();
    // 混控: 速度逆解 -> 轮向目标角速度 + 目标加速度 -> 轮端力矩前馈
    void mixed_calculate_callback();

    // 正运动学解算
    void forward_calculate_callback();

    // 编码器里程计(编码器航向参考)
    void encoder_odometry_callback();

    // 编码器里程计(IMU航向参考)
    void imu_odometry_callback();

    // 速度里程计
    void speed_odometry_callback();

    // 编码器里程计位姿清零
    void reset_position();
    // 编码器里程计位姿重定位
    void set_position(const float &__x, const float &__y, const float &__yaw);

    // 速度里程计位姿清零
    void reset_speed_position();
    // 速度里程计位姿重定位
    void set_speed_position(const float &__x, const float &__y, const float &__yaw);

protected:
    // 内部函数

    // 逆运动学解算 (纯几何解, 不碰力控状态): 只由 speed_/mixed_calculate_callback 内部调用
    void inverse_calculate_callback();
    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);
    // 力控: 等效力/力矩 -> 四轮期望轮端力, N; 满足 ΣF_i = F 且 Σ(r_i×F_i) = fw
    void force_wheel_force_vector(const float &__force_x, const float &__force_y, const float &__torque_z,
                                  float (&__wheel_force)[4]) const;

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0761f;
    // 轮心到底盘圆心的距离，m
    static constexpr float Wheel_Distance_To_Core = 0.25075f;
    // 底盘等效质量, kg (> 0); 换配重只改这一项
    static constexpr float Chassis_Mass = 10.0f;
    // 底盘绕 Z 轴等效转动惯量, kg·m^2 (> 0)
    static constexpr float Chassis_Inertia = 0.35f;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 轮向电机方向, ±1
    float wheel_encoder_direction = -1.0f;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;
    // 控制方式
    wheel_control_method_e control_method = Wheel_Speed_Control_Method;

    // 单轮轮端力矩上限, N·m (> 0); 
    float max_wheel_torque = 4.9f;
    // 单轮力死区(扣除式), N (0 = 关闭)
    float force_deadband = 0.1f;

    // 运动学: 底盘目标速度
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    // 力控: 目标加速度
    wheel_robot_accel_t target_accel = {0.0f, 0.0f, 0.0f};
    // 里程计: IMU 绝对航向角, rad
    float imu_yaw = 0.0f;
     // 内部变量

    // 上一帧累计轮转角, rad
    float last_angle_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 首帧标志, 上电首次调用建立里程计基准
    bool first_fb_flag = true;
    // 编码器里程计零帧抑制累积器: 未达死区的机体系位移暂存于此, 达阈值后一次积分
    float odom_residual_dx = 0.0f;
    float odom_residual_dy = 0.0f;
    float odom_residual_dyaw = 0.0f;
    // 读变量

    // angle 为里程计用轮向多圈转角, speed 为正/逆解共用
    wheel_state_t status = {};
    // 四轮轮端目标力矩
    float wheel_torque[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 正运动学结果 (机器人速度, vx/vy: m/s, vw: rad/s)
    wheel_robot_speed_t robot_speed = {0.0f, 0.0f, 0.0f};
    // 编码器里程计世界系位姿 (x,y: m; yaw: rad)
    wheel_robot_position_t encoder_pose = {0.0f, 0.0f, 0.0f};
    // 速度里程计世界系位姿 (x,y: m; yaw: rad), 与 encoder_pose 相互独立
    wheel_robot_position_t speed_pose = {0.0f, 0.0f, 0.0f};
};

/*-----------------------------------mecanum_wheel-----------------------------------*/

/**
 * @brief 麦轮运动学解算与编码器里程计
 */
class mecanum_wheel_class
{
public:

    void init(
              // 控制模式和控制参数
              const wheel_control_method_e &__control_method = Wheel_Speed_Control_Method,
              const float &__wheel_encoder_direction = 1.0f,
              const float &__control_period = 0.001f,
              // 力控死区和最大力矩限制
              const float &__max_wheel_torque = 4.9f,
              const float &__force_deadband = 0.1f);

    // 获取单轮角速度
    inline float get_wheel_speed(const uint8_t &__index) const;

    // 运动学正解(vx/vy/vw: m/s, rad/s)
    inline float get_robot_speed_vx() const;
    inline float get_robot_speed_vy() const;
    inline float get_robot_speed_vw() const;

    // 编码器里程计位姿(编码器累计) (x/y: m, yaw: rad)
    inline float get_position_x() const;
    inline float get_position_y() const;
    inline float get_position_yaw() const;

    // 速度里程计位姿(速度积分) (x/y: m, yaw: rad)
    inline float get_speed_position_x() const;
    inline float get_speed_position_y() const;
    inline float get_speed_position_yaw() const;

    // 轮端目标力矩, N·m
    inline float get_wheel_torque(const uint8_t &__index) const;

    // 轮向电机方向 +1 
    inline void set_wheel_encoder_direction(const float &__direction);
    // 目标加速度, m/s^2 / rad/s^2 (零 = 不出力; 速控流水线入口会整体清零)
    inline void set_robot_accel_x(const float &__accel_x);
    inline void set_robot_accel_y(const float &__accel_y);
    inline void set_robot_accel_w(const float &__accel_w);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);
    // 单轮角速度(正解输入)
    inline void set_wheel_speed(const uint8_t &__index, const float &__speed);

    // 更新底盘目标速度
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed);

    // 四轮当前测量状态
    inline void update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad = NAN);


    // 速控: 关断力控输出 + 逆解 -> 轮向目标角速度
    void speed_calculate_callback();
    // 力控: 目标加速度 -> 轮端力矩 (不使用速度目标)
    void force_calculate_callback();
    // 混控: 速度逆解 -> 轮向目标角速度 + 目标加速度 -> 轮端力矩前馈
    void mixed_calculate_callback();

    // 正运动学解算
    void forward_calculate_callback();

    // 编码器里程计(编码器航向参考)
    void encoder_odometry_callback();

    // 编码器里程计(IMU航向参考)
    void imu_odometry_callback();

    // 速度里程计
    void speed_odometry_callback();

    // 编码器里程计位姿清零
    void reset_position();
    // 编码器里程计位姿重定位
    void set_position(const float &__x, const float &__y, const float &__yaw);

    // 速度里程计位姿清零
    void reset_speed_position();
    // 速度里程计位姿重定位
    void set_speed_position(const float &__x, const float &__y, const float &__yaw);

protected:
    // 内部函数

    // 逆运动学解算 (纯几何解, 不碰力控状态)
    void inverse_calculate_callback();
    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);
    // 力控: 等效力/力矩 -> 四轮期望轮端力, N; 满足 ΣF_i = F 且 Σ(r_i×F_i) = fw
    void force_wheel_force_vector(const float &__force_x, const float &__force_y, const float &__torque_z, float (&__wheel_force)[4]) const;

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0485f;
    // 轮心在机体系 X/Y 方向的投影，m (半轴距 a / 半轮距 b)
    static constexpr float Wheel_Projection_X = 0.12f;
    static constexpr float Wheel_Projection_Y = 0.11f;
    // 投影距离和 (a+b), m
    static constexpr float Wheel_Sum_XY = Wheel_Projection_X + Wheel_Projection_Y;
    // 底盘等效质量, kg (> 0); 换配重只改这一项
    static constexpr float Chassis_Mass = 10.0f;
    // 底盘绕 Z 轴等效转动惯量, kg·m^2 (> 0)
    static constexpr float Chassis_Inertia = 0.35f;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 轮向电机方向 ±1
    float wheel_encoder_direction = 1.0f;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;
    // 控制方式
    wheel_control_method_e control_method = Wheel_Speed_Control_Method;

    // 单轮轮端力矩上限, N·m (> 0); 力控直接对电机出力, 输出端必须留住这一层防过流
    float max_wheel_torque = 4.9f;
    // 单轮力死区(扣除式), N (0 = 关闭)
    float force_deadband = 0.1f;

    // 运动学: 底盘目标速度
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    // 力控: 目标加速度
    wheel_robot_accel_t target_accel = {0.0f, 0.0f, 0.0f};
    // 里程计: IMU 绝对航向角, rad
    float imu_yaw = 0.0f;


    // 上一帧累计轮转角, rad
    float last_angle_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 首帧标志, 上电首次调用建立里程计基准
    bool first_fb_flag = true;
    // 编码器里程计零帧抑制累积器: 未达死区的机体系位移暂存于此, 达阈值后一次积分
    float odom_residual_dx = 0.0f;
    float odom_residual_dy = 0.0f;
    float odom_residual_dyaw = 0.0f;

    // 运动学: angle 为里程计用轮向多圈转角, speed 为正/逆解共用
    wheel_state_t status = {};
    // 力控: 四轮轮端目标力矩, N·m ([0]=FL [1]=RL [2]=RR [3]=FR)
    float wheel_torque[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 正运动学结果 (机器人速度, vx/vy: m/s, vw: rad/s)
    wheel_robot_speed_t robot_speed = {0.0f, 0.0f, 0.0f};
    // 编码器里程计世界系位姿 (x,y: m; yaw: rad)
    wheel_robot_position_t encoder_pose = {0.0f, 0.0f, 0.0f};
    // 速度里程计世界系位姿 (x,y: m; yaw: rad), 与 encoder_pose 相互独立
    wheel_robot_position_t speed_pose = {0.0f, 0.0f, 0.0f};
};

/*-----------------------------------steer_wheel-----------------------------------*/

/**
 * @brief 舵轮运动学解算、余弦优化与编码器里程计
 */
class steer_wheel_class
{
public:

    void init(
              // 控制模式和控制参数
              const wheel_control_method_e &__control_method = Wheel_Speed_Control_Method,
              const float &__steer_encoder_direction = 1.0f,
              const float &__control_period = 0.001f,
              // 速控参数
              const float &__target_max_step = 0.12f,
              const float &__max_wheel_speed = 25.0f,
              const float &__min_limit = 0.1f,
              const float &__steer_angle_deadband = 0.003f,
              const float &__flip_hysteresis = 0.1f,
              // 力控死区和最大力矩限制
              const float &__max_wheel_torque = 4.9f,
              const float &__force_deadband = 0.1f,
              // 优化开关
              const bool &__cosine_optimize_enable = true,
              const bool &__nearest_optimize_enable = true,
              const bool &__target_step_limit_enable = true);

    // 获取逆解输出量(运动学)
    inline float get_solution_angle(const uint8_t &__index) const;
    inline float get_solution_speed(const uint8_t &__index) const;

    inline float get_optimize_angle(const uint8_t &__index) const;
    inline float get_optimize_speed(const uint8_t &__index) const;

    // 轮端目标力矩(动力学), N·m
    inline float get_wheel_torque(const uint8_t &__index) const;

    inline float get_optimize_direction(const uint8_t &__index) const;
    inline float get_error_angle(const uint8_t &__index) const;
    inline float get_cosine_optimize(const uint8_t &__index) const;
    inline float get_force_direction(const uint8_t &__index) const;

    // 运动学正解(vx/vy/vw: m/s, rad/s)
    inline float get_robot_speed_vx() const;
    inline float get_robot_speed_vy() const;
    inline float get_robot_speed_vw() const;

    // 编码器里程计位姿(编码器累计) (x/y: m, yaw: rad)
    inline float get_position_x() const;
    inline float get_position_y() const;
    inline float get_position_yaw() const;

    // 速度里程计位姿(速度积分) (x/y: m, yaw: rad)
    inline float get_speed_position_x() const;
    inline float get_speed_position_y() const;
    inline float get_speed_position_yaw() const;

    // 舵向目标单周期最大变化量, rad
    inline void set_target_max_step(const float &__target_max_step);
    // 目标加速度, m/s^2 / rad/s^2 
    inline void set_robot_accel_x(const float &__accel_x);
    inline void set_robot_accel_y(const float &__accel_y);
    inline void set_robot_accel_w(const float &__accel_w);
    // 轮向电机最大角速度, rad/s
    inline void set_max_wheel_speed(const float &__max_wheel_speed);
    // 静止判定阈值, m/s 与 rad/s (三轴均低于此值视为零指令)
    inline void set_min_limit(const float &__min_limit);
    // 非小陀螺模式舵向误差死区, rad (小于该值不再修舵)
    inline void set_steer_angle_deadband(const float &__steer_angle_deadband);
    // 就近转位翻转迟滞, rad (须在 [0, π/2) 内, 0 表示无迟滞)
    inline void set_flip_hysteresis(const float &__flip_hysteresis);
    // 余弦优化开关 (速控与力控共用: 关闭时两条流水线同时不再按 cos³e 削减出力)
    inline void set_cosine_optimize_enable(const bool &__enable);
    // 就近转位优化开关 (速控与力控共用: 关闭时两条流水线都只转到解算/力方向本体, dir 恒 +1)
    inline void set_nearest_optimize_enable(const bool &__enable);
    // 舵向目标斜坡限幅开关 (速控与力控共用)
    inline void set_target_step_limit_enable(const bool &__enable);
    // 舵向编码器方向 +1 
    inline void set_steer_encoder_direction(const float &__direction);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);

    // 更新底盘目标速度和控制模式
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed, const uint8_t &__control_mode);
    // 各轮状态更新
    inline void update_now_state(const steer_state_t &__now_steer_state, const wheel_state_t &__now_wheel_state, const float &__imu_yaw_rad = NAN);


    // 速控: 关断力控输出 + 逆解 -> 舵向/轮向目标
    void speed_calculate_callback();
    // 力控: 目标加速度 -> 舵向对齐目标 + 轮端力矩 (不使用速度目标)
    void force_calculate_callback();
    // 混控: 速度逆解 -> 舵向/轮向目标, 再按当前实际轮向叠加力矩前馈
    void mixed_calculate_callback();

    // 正运动学解算
    void forward_calculate_callback();

    // 编码器里程计(编码器航向参考)
    void encoder_odometry_callback();

    // 编码器里程计(IMU航向参考)
    void imu_odometry_callback();

    // 速度里程计
    void speed_odometry_callback();

    // 编码器里程计位姿清零
    void reset_position();
    // 编码器里程计位姿重定位
    void set_position(const float &__x, const float &__y, const float &__yaw);

    // 速度里程计位姿清零
    void reset_speed_position();
    // 速度里程计位姿重定位
    void set_speed_position(const float &__x, const float &__y, const float &__yaw);

protected:
    // 内部函数

    // 逆运动学解算 (纯几何解, 不碰力控状态)
    void inverse_calculate_callback();
    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);
    // 由各轮线位移与舵向角解算机体系位移; 任一通道非法时输出清零并返回 false
    bool calc_body_displacement(const float *__ds, float &__dx, float &__dy, float &__dyaw) const;
    // 力控核心
    void force_calculate_core();

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0485f;

    // ---- 轮心几何: 正方形 / 长方形布局通用, 换构型只改下面两项 ----
    // 轮心在机体系 X/Y 方向的投影, m: 前后半轴距 Lx / 左右半轮距 Ly
    // 四轮对称布置: [0]=FL(+Lx,+Ly) [1]=RL(-Lx,+Ly) [2]=RR(-Lx,-Ly) [3]=FR(+Lx,-Ly)
    // 注意是**半**轴距 / 半轮距 (轮心到中心线的距离), 不是整轴距 / 整轮距
    static constexpr float Wheel_Projection_X = 0.12f;
    static constexpr float Wheel_Projection_Y = 0.12f;
    // 轮心坐标符号 (乘上 Lx / Ly 即得轮心 (x_i, y_i)), 供逆解/正解/位移解算的循环使用
    static constexpr float Wheel_Sign_X[4] = { 1.0f, -1.0f, -1.0f,  1.0f};
    static constexpr float Wheel_Sign_Y[4] = { 1.0f,  1.0f, -1.0f, -1.0f};
    // 轮心到底盘中心的径向距离平方 d² = Lx² + Ly², m²; 四轮相同 (对角轮心落在同一圆周上)
    // 角速度相关公式只用到 d², 因此全程不必开方: 逆解/力分配用轮心坐标 (x_i, y_i), 正解用 (x_i, y_i)/d²
    static constexpr float Wheel_D2 = Wheel_Projection_X * Wheel_Projection_X
                                    + Wheel_Projection_Y * Wheel_Projection_Y;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 编码器零位角, rad: 编码器读数为 0 时轮子实际指向相对 +X 的偏差
    static constexpr float Enc_Zero_Angle[4] = {-1.07455f, -0.02924f, 1.01923f, -3.11878f};
    // 底盘等效质量, kg (> 0);
    static constexpr float Chassis_Mass = 10.0f;
    // 底盘绕 Z 轴等效转动惯量, kg·m^2 (> 0)
    static constexpr float Chassis_Inertia = 0.35f;

    // 舵向目标单周期最大变化量(斜坡斜率), rad/拍; 仅在 target_step_limit_enable 为 true 时生效
    float target_max_step = 0.12f;
    // 轮向电机最大角速度, rad/s (超限时四轮等比例缩放)
    float max_wheel_speed = 25.0f;
    // 静止判定阈值, m/s 与 rad/s (三轴指令均不超过该值视为零指令)
    float min_limit = 0.1f;
    // 非小陀螺模式舵向误差死区, rad (小于该值不再修舵, 防抖)
    float steer_angle_deadband = 0.003f;
    // 就近转位翻转迟滞, rad ∈ [0, π/2): 已反转时需 |误差| 小于 90°-迟滞 才转回, 正向时需大于 90°+迟滞 才反转
    float flip_hysteresis = 0.1f;
    // 舵向电机方向(与解算正方向一致为正±1)
    float steer_encoder_direction = 1.0f;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;
    // 优化开关: 余弦削减 / 就近转位 / 舵向斜坡限幅
    bool cosine_optimize_enable = true;
    bool nearest_optimize_enable = true;
    bool target_step_limit_enable = true;
    // 控制方式
    wheel_control_method_e control_method = Wheel_Speed_Control_Method;

    // 单轮轮端力矩上限, N·m (> 0); 力控直接对电机出力, 输出端必须留住这一层防过流
    float max_wheel_torque = 4.9f;
    // 力矩合力死区
    float force_deadband = 0.1f;

    // 运动学: 底盘目标速度 + 底盘行为模式
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    uint8_t control_mode = 0;
    // 力控: 目标加速度
    wheel_robot_accel_t target_accel = {0.0f, 0.0f, 0.0f};
    // 里程计: 舵向累计转角 + 轮向累计转角 + IMU 航向
    float wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float steer_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float imu_yaw = 0.0f;

    // 上一帧累计轮转角, rad
    float last_wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 首帧标志, 上电首次调用建立里程计基准
    bool first_fb_flag = true;
    // 舵向斜坡基准有效性: init()/zero_force() 后为 false, 首拍直接从当前反馈接管, 避免从 0 跳变
    // 速控与力控共用: 两条路径写同一份 optimize_angle, 基准天然连续
    bool optimize_angle_valid[4] = {false, false, false, false};
    // 编码器里程计零帧抑制累积器: 未达死区的机体系位移暂存于此, 达阈值后一次积分
    float odom_residual_dx = 0.0f;
    float odom_residual_dy = 0.0f;
    float odom_residual_dyaw = 0.0f;


    steer_wheel_status_t status = {};

    // 正运动学 (机器人速度, vx/vy: m/s, vw: rad/s)
    wheel_robot_speed_t robot_speed = {0.0f, 0.0f, 0.0f};
    // 编码器里程计世界系位姿 (x,y: m; yaw: rad)
    wheel_robot_position_t encoder_pose = {0.0f, 0.0f, 0.0f};
    // 速度里程计世界系位姿 (x,y: m; yaw: rad), 与 encoder_pose 相互独立
    wheel_robot_position_t speed_pose = {0.0f, 0.0f, 0.0f};
};

/*-----------------------------------omni_wheel inline-----------------------------------*/

inline float omni_wheel_class::get_wheel_torque(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return (0.0f);
    }
    return (wheel_torque[__index]);
}

inline void omni_wheel_class::set_robot_accel_x(const float &__accel_x)
{
    // 单轴校验: 非法值跳过该轴, 不影响其他两轴
    if (!isfinite(__accel_x))
    {
        return;
    }

    target_accel.ax = __accel_x;
}

inline void omni_wheel_class::set_robot_accel_y(const float &__accel_y)
{
    if (!isfinite(__accel_y))
    {
        return;
    }

    target_accel.ay = __accel_y;
}

inline void omni_wheel_class::set_robot_accel_w(const float &__accel_w)
{
    if (!isfinite(__accel_w))
    {
        return;
    }

    target_accel.aw = __accel_w;
}

inline void omni_wheel_class::set_wheel_encoder_direction(const float &__direction)
{
    // 只接受 ±1: 其它值既不是"同向"也不是"反向", 直接忽略避免引入莫名的比例缩放
    if (!isfinite(__direction) || __direction == 0.0f)
    {
        return;
    }

    wheel_encoder_direction = (__direction > 0.0f) ? 1.0f : -1.0f;
}

inline void omni_wheel_class::set_control_period(const float &__control_period)
{
    // 速度里程计以该周期做积分, 必须是有限正值
    if (!isfinite(__control_period) || __control_period <= 0.0f)
    {
        return;
    }
    control_period = __control_period;
}

inline void omni_wheel_class::update_robot_state(const wheel_robot_speed_t &__target_speed)
{
    if (!isfinite(__target_speed.vx) || !isfinite(__target_speed.vy) || !isfinite(__target_speed.vw))
    {
        return;
    }
    target_speed = __target_speed;
}

inline void omni_wheel_class::update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad)
{
    // 轮速逐轮独立校验: 单个通道异常只跳过该通道
    for (uint8_t i = 0; i < 4; i++)
    {
        if (isfinite(__now_state.speed[i]))
        {
            status.speed[i] = __now_state.speed[i];
        }
    }

    // 四轮累计角必须同帧有效才提交: 里程计把四轮位移合成三轴, 只替换坏通道等于用旧值冒充"该轮没动",
    // 会算出虚假平移; 整帧丢弃且不推进基准, 坏通道恢复后位移一次补齐, 一条都不丢
    bool frame_valid = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(__now_state.angle[i]))
        {
            frame_valid = false;
        }
    }

    if (frame_valid)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            status.angle[i] = __now_state.angle[i];
        }
    }

    // IMU 航向与轮角相互独立: 任一通道异常不影响另一路
    if (isfinite(__imu_yaw_rad))
    {
        imu_yaw = __imu_yaw_rad;
    }
}

inline float omni_wheel_class::get_wheel_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.speed[__index]);
}

inline void omni_wheel_class::set_wheel_speed(const uint8_t &__index, const float &__speed)
{
    if (__index >= 4 || !isfinite(__speed))
    {
        return;
    }
    status.speed[__index] = __speed;
}

inline float omni_wheel_class::get_robot_speed_vx() const
{
    return (robot_speed.vx);
}

inline float omni_wheel_class::get_robot_speed_vy() const
{
    return (robot_speed.vy);
}

inline float omni_wheel_class::get_robot_speed_vw() const
{
    return (robot_speed.vw);
}

inline float omni_wheel_class::get_position_x() const
{
    return (encoder_pose.x);
}

inline float omni_wheel_class::get_position_y() const
{
    return (encoder_pose.y);
}

inline float omni_wheel_class::get_position_yaw() const
{
    return (encoder_pose.yaw);
}

inline float omni_wheel_class::get_speed_position_x() const
{
    return (speed_pose.x);
}

inline float omni_wheel_class::get_speed_position_y() const
{
    return (speed_pose.y);
}

inline float omni_wheel_class::get_speed_position_yaw() const
{
    return (speed_pose.yaw);
}

/*-----------------------------------mecanum_wheel inline-----------------------------------*/

inline float mecanum_wheel_class::get_wheel_torque(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return (0.0f);
    }
    return (wheel_torque[__index]);
}

inline void mecanum_wheel_class::set_robot_accel_x(const float &__accel_x)
{
    // 单轴校验: 非法值跳过该轴, 不影响其他两轴
    if (!isfinite(__accel_x))
    {
        return;
    }

    target_accel.ax = __accel_x;
}

inline void mecanum_wheel_class::set_robot_accel_y(const float &__accel_y)
{
    if (!isfinite(__accel_y))
    {
        return;
    }

    target_accel.ay = __accel_y;
}

inline void mecanum_wheel_class::set_robot_accel_w(const float &__accel_w)
{
    if (!isfinite(__accel_w))
    {
        return;
    }

    target_accel.aw = __accel_w;
}

inline void mecanum_wheel_class::set_wheel_encoder_direction(const float &__direction)
{
    // 只接受 ±1: 其它值既不是"同向"也不是"反向", 直接忽略避免引入莫名的比例缩放
    if (!isfinite(__direction) || __direction == 0.0f)
    {
        return;
    }

    wheel_encoder_direction = (__direction > 0.0f) ? 1.0f : -1.0f;
}

inline void mecanum_wheel_class::set_control_period(const float &__control_period)
{
    // 速度里程计以该周期做积分, 必须是有限正值
    if (!isfinite(__control_period) || __control_period <= 0.0f)
    {
        return;
    }
    control_period = __control_period;
}

inline void mecanum_wheel_class::update_robot_state(const wheel_robot_speed_t &__target_speed)
{
    if (!isfinite(__target_speed.vx) || !isfinite(__target_speed.vy) || !isfinite(__target_speed.vw))
    {
        return;
    }
    target_speed = __target_speed;
}

inline void mecanum_wheel_class::update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad)
{
    // 轮速逐轮独立校验: 单个通道异常只跳过该通道
    for (uint8_t i = 0; i < 4; i++)
    {
        if (isfinite(__now_state.speed[i]))
        {
            status.speed[i] = __now_state.speed[i];
        }
    }

    // 四轮累计角必须同帧有效才提交: 里程计把四轮位移合成三轴, 只替换坏通道等于用旧值冒充"该轮没动",
    // 会算出虚假平移; 整帧丢弃且不推进基准, 坏通道恢复后位移一次补齐, 一条都不丢
    bool frame_valid = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(__now_state.angle[i]))
        {
            frame_valid = false;
        }
    }

    if (frame_valid)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            status.angle[i] = __now_state.angle[i];
        }
    }

    // IMU 航向与轮角相互独立: 任一通道异常不影响另一路
    if (isfinite(__imu_yaw_rad))
    {
        imu_yaw = __imu_yaw_rad;
    }
}

inline float mecanum_wheel_class::get_wheel_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.speed[__index]);
}

inline void mecanum_wheel_class::set_wheel_speed(const uint8_t &__index, const float &__speed)
{
    if (__index >= 4 || !isfinite(__speed))
    {
        return;
    }
    status.speed[__index] = __speed;
}

inline float mecanum_wheel_class::get_robot_speed_vx() const
{
    return (robot_speed.vx);
}

inline float mecanum_wheel_class::get_robot_speed_vy() const
{
    return (robot_speed.vy);
}

inline float mecanum_wheel_class::get_robot_speed_vw() const
{
    return (robot_speed.vw);
}

inline float mecanum_wheel_class::get_position_x() const
{
    return (encoder_pose.x);
}

inline float mecanum_wheel_class::get_position_y() const
{
    return (encoder_pose.y);
}

inline float mecanum_wheel_class::get_position_yaw() const
{
    return (encoder_pose.yaw);
}

inline float mecanum_wheel_class::get_speed_position_x() const
{
    return (speed_pose.x);
}

inline float mecanum_wheel_class::get_speed_position_y() const
{
    return (speed_pose.y);
}

inline float mecanum_wheel_class::get_speed_position_yaw() const
{
    return (speed_pose.yaw);
}

/*-----------------------------------steer_wheel inline-----------------------------------*/

inline float steer_wheel_class::get_wheel_torque(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.wheel_torque[__index]);
}

inline void steer_wheel_class::set_robot_accel_x(const float &__accel_x)
{
    // 单轴校验: 非法值跳过该轴, 不影响其他两轴
    if (!isfinite(__accel_x))
    {
        return;
    }

    target_accel.ax = __accel_x;
}

inline void steer_wheel_class::set_robot_accel_y(const float &__accel_y)
{
    if (!isfinite(__accel_y))
    {
        return;
    }

    target_accel.ay = __accel_y;
}

inline void steer_wheel_class::set_robot_accel_w(const float &__accel_w)
{
    if (!isfinite(__accel_w))
    {
        return;
    }

    target_accel.aw = __accel_w;
}

inline float steer_wheel_class::get_solution_angle(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.solution_angle[__index]);
}

inline float steer_wheel_class::get_solution_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.solution_speed[__index]);
}

inline float steer_wheel_class::get_optimize_angle(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.optimize_angle[__index]);
}

inline float steer_wheel_class::get_optimize_speed(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.optimize_speed[__index]);
}

inline float steer_wheel_class::get_optimize_direction(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.optimize_direction[__index]);
}

inline float steer_wheel_class::get_error_angle(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.error_angle[__index]);
}

inline float steer_wheel_class::get_cosine_optimize(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.cosine_optimize[__index]);
}

inline float steer_wheel_class::get_force_direction(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return 0.0f;
    }
    return (status.force_direction[__index]);
}

inline float steer_wheel_class::get_robot_speed_vx() const
{
    return (robot_speed.vx);
}

inline float steer_wheel_class::get_robot_speed_vy() const
{
    return (robot_speed.vy);
}

inline float steer_wheel_class::get_robot_speed_vw() const
{
    return (robot_speed.vw);
}

inline float steer_wheel_class::get_position_x() const
{
    return (encoder_pose.x);
}

inline float steer_wheel_class::get_position_y() const
{
    return (encoder_pose.y);
}

inline float steer_wheel_class::get_position_yaw() const
{
    return (encoder_pose.yaw);
}

inline float steer_wheel_class::get_speed_position_x() const
{
    return (speed_pose.x);
}

inline float steer_wheel_class::get_speed_position_y() const
{
    return (speed_pose.y);
}

inline float steer_wheel_class::get_speed_position_yaw() const
{
    return (speed_pose.yaw);
}

inline void steer_wheel_class::set_target_max_step(const float &__target_max_step)
{
    // 限幅斜率必须为正, 否则目标角无法推进
    if (!isfinite(__target_max_step) || __target_max_step <= 0.0f)
    {
        return;
    }
    target_max_step = __target_max_step;
}

inline void steer_wheel_class::set_max_wheel_speed(const float &__max_wheel_speed)
{
    if (!isfinite(__max_wheel_speed) || __max_wheel_speed <= 0.0f)
    {
        return;
    }
    max_wheel_speed = __max_wheel_speed;
}

inline void steer_wheel_class::set_min_limit(const float &__min_limit)
{
    if (!isfinite(__min_limit) || __min_limit < 0.0f)
    {
        return;
    }
    min_limit = __min_limit;
}

inline void steer_wheel_class::set_steer_angle_deadband(const float &__steer_angle_deadband)
{
    // 死区可为 0 (不防抖), 但不能为负
    if (!isfinite(__steer_angle_deadband) || __steer_angle_deadband < 0.0f)
    {
        return;
    }
    steer_angle_deadband = __steer_angle_deadband;
}

inline void steer_wheel_class::set_flip_hysteresis(const float &__flip_hysteresis)
{
    // 迟滞必须小于 90 度: 否则正向阈值 >= 180 度(永不反转)、反向阈值 <= 0 度(永不转回),
    // 方向会被永久锁死在初始分支, 就近转位退化为"只认一侧"
    if (!isfinite(__flip_hysteresis) || __flip_hysteresis < 0.0f || __flip_hysteresis >= PI * 0.5f)
    {
        return;
    }
    flip_hysteresis = __flip_hysteresis;
}

inline void steer_wheel_class::set_cosine_optimize_enable(const bool &__enable)
{
    cosine_optimize_enable = __enable;
}

inline void steer_wheel_class::set_nearest_optimize_enable(const bool &__enable)
{
    nearest_optimize_enable = __enable;
}

inline void steer_wheel_class::set_target_step_limit_enable(const bool &__enable)
{
    target_step_limit_enable = __enable;
}

inline void steer_wheel_class::set_steer_encoder_direction(const float &__direction)
{
    // 只接受 ±1: 其它值既不是"同向"也不是"反向", 直接忽略避免引入莫名的比例缩放
    if (!isfinite(__direction) || __direction == 0.0f)
    {
        return;
    }

    steer_encoder_direction = (__direction > 0.0f) ? 1.0f : -1.0f;
}

inline void steer_wheel_class::set_control_period(const float &__control_period)
{
    // 速度里程计以该周期做积分, 必须是有限正值
    if (!isfinite(__control_period) || __control_period <= 0.0f)
    {
        return;
    }
    control_period = __control_period;
}

inline void steer_wheel_class::update_robot_state(const wheel_robot_speed_t &__target_speed, const uint8_t &__control_mode)
{
    // 控制模式与速度相互独立: 速度非法时仍更新模式, 保持原 update_control_mode() 的语义
    control_mode = __control_mode;

    if (!isfinite(__target_speed.vx) || !isfinite(__target_speed.vy) || !isfinite(__target_speed.vw))
    {
        return;
    }
    target_speed = __target_speed;
}

inline void steer_wheel_class::update_now_state(const steer_state_t &__now_steer_state, const wheel_state_t &__now_wheel_state, const float &__imu_yaw_rad)
{
    // 逐轮独立校验: 单个通道异常只跳过该通道, 不丢弃另一组有效反馈
    for (uint8_t i = 0; i < 4; i++)
    {
        if (isfinite(__now_steer_state.angle[i]))
        {
            status.now_angle[i] = __now_steer_state.angle[i];
        }

        if (isfinite(__now_wheel_state.speed[i]))
        {
            status.now_speed[i] = __now_wheel_state.speed[i];
        }
    }

    // 轮向角与舵向角必须同帧有效才提交: 位移解算用"轮位移 × 舵向角", 混用新旧快照会算出虚假平移/航向;
    // 整帧丢弃且不推进基准, 恢复后位移一次补齐, 一条都不丢
    bool frame_valid = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(__now_wheel_state.angle[i]) || !isfinite(__now_steer_state.angle[i]))
        {
            frame_valid = false;
        }
    }

    if (frame_valid)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_angle[i] = __now_wheel_state.angle[i];
            steer_angle[i] = __now_steer_state.angle[i];
        }
    }

    // IMU 航向与轮/舵向角相互独立: 任一通道异常不影响另一路
    if (isfinite(__imu_yaw_rad))
    {
        imu_yaw = __imu_yaw_rad;
    }
}
