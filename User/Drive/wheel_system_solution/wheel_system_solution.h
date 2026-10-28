/**
 * @file wheel_system_solution.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-10-25
 * @version 1.2
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
 * @brief 机器人速度 (正/逆运动学通用), vx/vy: m/s, vw: rad/s (CCW 为正)
 */
typedef struct
{
    float vx; // 机器人 X 轴方向速度, m/s
    float vy; // 机器人 Y 轴方向速度, m/s
    float vw; // 机器人绕 Z 轴角速度, rad/s
} wheel_robot_speed_t;

/**
 * @brief 机器人位姿 (世界系)
 */
typedef struct
{
    float x;   // 世界系 X 方向位置, m
    float y;   // 世界系 Y 方向位置, m
    float yaw; // 世界系航向角, rad (CCW 为正)
} wheel_robot_position_t;

/**
 * @brief 轮向电机测量状态: 多圈累计转角(编码器里程计输入) + 轮向角速度(正/逆解输入输出)
 * @note  angle 只喂编码器里程计, speed 只喂运动学; 轮子线速度 = speed × Wheel_Radius
 */
typedef struct
{
    float angle[4]; // 四个轮子多圈累计转角, rad ([0]=FL [1]=RL [2]=RR [3]=FR)
    float speed[4]; // 四个轮子轮向角速度, rad/s ([0]=FL [1]=RL [2]=RR [3]=FR)
} wheel_state_t;

/**
 * @brief 舵向电机测量状态: 多圈累计转角 + 舵向角速度, rad / rad/s (以 +X 为 0, CCW 为正)
 * @note  与 wheel_state_t 布局相同但语义不同, 刻意分型以防混用 (轮向角/舵向角);
 *        angle 同时喂正/逆解与编码器里程计, speed 供上层舵向速度环使用
 */
typedef struct
{
    float angle[4]; // 四个舵向累计转角, rad ([0]=FL [1]=RL [2]=RR [3]=FR)
    float speed[4]; // 四个舵向电机角速度, rad/s ([0]=FL [1]=RL [2]=RR [3]=FR)
} steer_state_t;

/**
 * @brief 舵轮对外接口状态 (含输入反馈与逆解输出)
 * @note  now_xxx 为输入; solution/optimize/cosine/error 为逆解输出,
 *        optimize_angle 同时作为上一周期目标参与限速规划.
 */
typedef struct
{
    float now_angle[4];          // 舵向当前多圈角度反馈, rad (输入)
    float now_speed[4];          // 轮向当前角速度反馈, rad/s (输入)
    float solution_angle[4];     // 舵向目标角, rad
    float solution_speed[4];     // 轮向目标速度(未优化), rad/s
    float cosine_optimize[4];    // 转向角余弦优化系数, 无量纲
    float optimize_speed[4];     // 轮向实际目标速度, rad/s
    float optimize_angle[4];     // 舵向多圈目标角, rad (输出/上周期输入)
    float optimize_direction[4]; // 轮向方向, ±1
    float error_angle[4];        // 舵向真实未对齐角, rad (相对所选方向分支, 0 表示已对齐)
} steer_wheel_status_t;

/*-----------------------------------omni_wheel-----------------------------------*/

/**
 * @brief 全向轮运动学解算与编码器里程计
 * @note  status.speed 同时是正解输入与逆解输出, status.angle 为里程计用轮向多圈转角;
 *        单周期内应按 update_now_state() → forward_calculate_callback() → inverse_calculate_callback() 顺序调用;
 *        若先逆解后正解, 正解会把指令值当反馈, 实测速度恒等于目标速度.
 */
class omni_wheel_class
{
public:

    void init(const float &__distance_to_core = Wheel_Distance_To_Core, const float &__control_period = 0.001f);

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

    // 轮心到底盘圆心的投影距离, m
    inline void set_distance_to_core(const float &__distance_to_core);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);
    // 单轮角速度(正解输入)
    inline void set_wheel_speed(const uint8_t &__index, const float &__speed);

    // 更新底盘目标速度
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed);

    // 四轮当前测量状态 + IMU 航向: speed 喂正/逆解, angle 喂编码器里程计
    // IMU 默认取 NAN = 不更新: 未接 IMU 的调用点不能把航向悄悄钉成 0
    inline void update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad = NAN);

    // 逆运动学解算
    void inverse_calculate_callback();

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

    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0761f;
    // 轮心到底盘圆心的距离，m
    static constexpr float Wheel_Distance_To_Core = 0.25075f;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 轮心到底盘圆心的距离, m
    float distance_to_core = Wheel_Distance_To_Core;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;

    // 写变量(输入)

    // 逆解输入: 底盘目标速度
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    // 里程计输入: IMU 绝对航向角, rad
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

    // 运动学对外接口信息: angle 为里程计用轮向多圈转角, speed 为正/逆解共用
    wheel_state_t status = {};

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
 * @note  外八辊子布局, 辊子方向 FL\ RL/ RR\ FR/ (小口→车内, 大口→车外)
 * @note  status.speed 同时是正解输入与逆解输出, status.angle 为里程计用轮向多圈转角;
 *        单周期内应按 update_now_state() → forward_calculate_callback() → inverse_calculate_callback() 顺序调用;
 *        若先逆解后正解, 正解会把指令值当反馈, 实测速度恒等于目标速度.
 */
class mecanum_wheel_class
{
public:

    void init(const float &__projection_x = Wheel_Projection_X, const float &__projection_y = Wheel_Projection_Y,
              const float &__control_period = 0.001f);

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

    // 轮心在机体系 X 方向的投影, m
    inline void set_projection_x(const float &__projection_x);
    // 轮心在机体系 Y 方向的投影, m
    inline void set_projection_y(const float &__projection_y);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);
    // 单轮角速度(正解输入)
    inline void set_wheel_speed(const uint8_t &__index, const float &__speed);

    // 更新底盘目标速度
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed);

    // 更新当前底盘状态
    // IMU 默认取 NAN = 不更新: 未接 IMU 的调用点不能把航向悄悄钉成 0
    inline void update_now_state(const wheel_state_t &__now_state, const float &__imu_yaw_rad = NAN);

    // 逆运动学解算
    void inverse_calculate_callback();

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

    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0485f;
    // 轮心在机体系 X/Y 方向的投影，m
    static constexpr float Wheel_Projection_X = 0.12f;
    static constexpr float Wheel_Projection_Y = 0.11f;

    // 编码器里程计位移累积死区, m: 单帧位移不足该值时只累积不积分, 累积到该值后一次吐出
    static constexpr float Odom_Linear_Deadband = 0.0001f;
    // 编码器里程计角度累积死区, rad: 同线位移, 抖动正负互抵不会触发
    static constexpr float Odom_Angle_Deadband = 0.001f;
    // 速度里程计线速度死区, m/s
    static constexpr float Speed_Odom_Linear_Deadband = 0.01f;
    // 速度里程计角速度死区, rad/s
    static constexpr float Speed_Odom_Angle_Deadband = 0.05f;

    // 轮心在机体系 X/Y 方向的投影, m
    float projection_x = Wheel_Projection_X;
    float projection_y = Wheel_Projection_Y;
    // 投影距离和 (a+b), m
    float sum_xy = Wheel_Projection_X + Wheel_Projection_Y;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;

    // 写变量(输入)

    // 逆解输入: 底盘目标速度
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    // 里程计输入: IMU 绝对航向角, rad
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

    // 运动学对外接口信息: angle 为里程计用轮向多圈转角, speed 为正/逆解共用
    wheel_state_t status = {};

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
              const float &__steer_encoder_direction = 1.0f,
              const float &__target_max_step = 0.12f,
              const float &__max_wheel_speed = 25.0f,
              const float &__min_limit = 0.1f,
              const float &__steer_angle_deadband = 0.003f,
              const float &__flip_hysteresis = 0.1f,
              const float &__control_period = 0.001f,
              const bool &__cosine_optimize_enable = true,
              const bool &__nearest_optimize_enable = true,
              const bool &__target_step_limit_enable = true);

    // 获取逆解输出量
    inline float get_solution_angle(const uint8_t &__index) const;
    inline float get_solution_speed(const uint8_t &__index) const;

    inline float get_optimize_angle(const uint8_t &__index) const;
    inline float get_optimize_speed(const uint8_t &__index) const;

    inline float get_optimize_direction(const uint8_t &__index) const;
    inline float get_error_angle(const uint8_t &__index) const;
    inline float get_cosine_optimize(const uint8_t &__index) const;

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
    // 轮向电机最大角速度, rad/s
    inline void set_max_wheel_speed(const float &__max_wheel_speed);
    // 静止判定阈值, m/s 与 rad/s (三轴均低于此值视为零指令)
    inline void set_min_limit(const float &__min_limit);
    // 非小陀螺模式舵向误差死区, rad (小于该值不再修舵)
    inline void set_steer_angle_deadband(const float &__steer_angle_deadband);
    // 就近转位翻转迟滞, rad (须在 [0, π/2) 内, 0 表示无迟滞)
    inline void set_flip_hysteresis(const float &__flip_hysteresis);
    // 轮心到底盘中心的距离(旋转力臂), m
    inline void set_distance_to_core(const uint8_t &__index, const float &__distance);
    // 余弦优化开关
    inline void set_cosine_optimize_enable(const bool &__enable);
    // 就近转位优化开关
    inline void set_nearest_optimize_enable(const bool &__enable);
    // 舵向目标单周期最大变化量限制开关
    inline void set_target_step_limit_enable(const bool &__enable);
    // 舵向编码器方向: +1 电机读数正向与几何 CCW 同向, -1 反向(读数增大时轮子顺时针转).
    // 只改这一处, 逆解/正解/里程计/舵向目标会自动保持一致; 传入 0 或非有限值则忽略
    inline void set_steer_encoder_direction(const float &__direction);
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    inline void set_control_period(const float &__control_period);

    // 更新底盘目标速度和控制模式
    inline void update_robot_state(const wheel_robot_speed_t &__target_speed, const uint8_t &__control_mode);
    // 各轮状态更新
    // IMU 默认取 NAN = 不更新: 未接 IMU 的调用点不能把航向悄悄钉成 0
    inline void update_now_state(const steer_state_t &__now_steer_state, const wheel_state_t &__now_wheel_state, const float &__imu_yaw_rad = NAN);

    // 逆运动学解算
    void inverse_calculate_callback();

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

    // 编码器里程计单步: 位移恒定取编码器差分, 航向按 __use_imu_yaw 取 IMU 或自持积分
    bool encoder_odometry_step(const bool &__use_imu_yaw);
    // 由各轮线位移与舵向角解算机体系位移; 任一通道非法时输出清零并返回 false
    bool calc_body_displacement(const float *__ds, float &__dx, float &__dy, float &__dyaw) const;

    // 常量

    // 驱动轮半径，m
    static constexpr float Wheel_Radius = 0.0485f;
    // 轮心到底盘中心的距离，m
    static constexpr float Wheel_Distance_To_Core = 0.12f;

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
    // 轮心单位方向向量 e_i = (cos φ_i, sin φ_i), φ_i = {45°, 135°, 225°, 315°} 为几何 CCW 方位角.
    static constexpr float Wheel_Unit_Cos[4] = {SQRT_2_2, -SQRT_2_2, -SQRT_2_2, SQRT_2_2};
    static constexpr float Wheel_Unit_Sin[4] = {SQRT_2_2, SQRT_2_2, -SQRT_2_2, -SQRT_2_2};

    // 各轮心到底盘中心的距离(旋转力臂半径), m
    float distance_to_core[4] = {Wheel_Distance_To_Core, Wheel_Distance_To_Core,
                                 Wheel_Distance_To_Core, Wheel_Distance_To_Core};

    // 舵向目标单周期最大变化量(斜坡斜率), rad/拍; 仅在 target_step_limit_enable 为 true 时生效
    float target_max_step = 0.20f;
    // 轮向电机最大角速度, rad/s (超限时四轮等比例缩放)
    float max_wheel_speed = 25.0f;
    // 静止判定阈值, m/s 与 rad/s (三轴指令均不超过该值视为零指令)
    float min_limit = 0.1f;
    // 非小陀螺模式舵向误差死区, rad (小于该值不再修舵, 防抖)
    float steer_angle_deadband = 0.05f;
    // 就近转位翻转迟滞, rad ∈ [0, π/2): 已反转时需 |误差| 小于 90°-迟滞 才转回, 正向时需大于 90°+迟滞 才反转
    float flip_hysteresis = 0.1f;
    // 控制周期(里程计回调调用间隔), s; 仅速度里程计积分使用
    float control_period = 0.001f;

    // 余弦优化开关, true: 按 cos^3 削减轮速; false: 仅就近转位
    bool cosine_optimize_enable = true;
    // 就近转位开关, true: 允许加减 pi 取等效角并反转轮向; false: 舵向只转向解算角本体
    bool nearest_optimize_enable = true;
    // 舵向目标斜坡限幅开关, true: 目标角按 target_max_step 逐拍逼近; false: 直接下发解算角
    bool target_step_limit_enable = true;

    // 舵向电机方向(与解算正方向一致为正±1)
    float steer_encoder_direction = 1.0f;

    // 写变量(输入)

    // 底盘目标速度和控制模式
    wheel_robot_speed_t target_speed = {0.0f, 0.0f, 0.0f};
    uint8_t control_mode = 0;

    // 编码器里程计， 舵向累计转角 + 轮向累计转角 + IMU航向
    float wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float steer_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float imu_yaw = 0.0f;

    // 内部变量

    // 上一帧累计轮转角, rad
    float last_wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // 首帧标志, 上电首次调用建立里程计基准
    bool first_fb_flag = true;

    // 舵向斜坡基准有效性: init() 后为 false, 首拍直接从当前反馈接管, 避免从 0 跳变
    bool optimize_angle_valid[4] = {false, false, false, false};

    // 编码器里程计零帧抑制累积器: 未达死区的机体系位移暂存于此, 达阈值后一次积分
    float odom_residual_dx = 0.0f;
    float odom_residual_dy = 0.0f;
    float odom_residual_dyaw = 0.0f;

    // 读变量

    // 解算控制量
    steer_wheel_status_t status = {};

    // 正运动学 (机器人速度, vx/vy: m/s, vw: rad/s)
    wheel_robot_speed_t robot_speed = {0.0f, 0.0f, 0.0f};
    // 编码器里程计世界系位姿 (x,y: m; yaw: rad)
    wheel_robot_position_t encoder_pose = {0.0f, 0.0f, 0.0f};
    // 速度里程计世界系位姿 (x,y: m; yaw: rad), 与 encoder_pose 相互独立
    wheel_robot_position_t speed_pose = {0.0f, 0.0f, 0.0f};
};

/*-----------------------------------omni_wheel inline-----------------------------------*/

inline void omni_wheel_class::set_distance_to_core(const float &__distance_to_core)
{
    // 逆解/正解与里程计以该距离为除数, 必须严格大于 0
    if (!isfinite(__distance_to_core) || __distance_to_core <= 0.0f)
    {
        return;
    }
    distance_to_core = __distance_to_core;
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

inline void mecanum_wheel_class::set_projection_x(const float &__projection_x)
{
    // 单轴为 0 无物理意义, 且投影距离和为 0 会让逆/正解退化, 一并拒绝
    if (!isfinite(__projection_x) || __projection_x < 0.0f || (__projection_x + projection_y) <= 0.0f)
    {
        return;
    }
    projection_x = __projection_x;
    sum_xy = projection_x + projection_y; // 逆/正解用缓存的和, 参数一改必须同步
}

inline void mecanum_wheel_class::set_projection_y(const float &__projection_y)
{
    if (!isfinite(__projection_y) || __projection_y < 0.0f || (projection_x + __projection_y) <= 0.0f)
    {
        return;
    }
    projection_y = __projection_y;
    sum_xy = projection_x + projection_y;
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

inline void steer_wheel_class::set_distance_to_core(const uint8_t &__index, const float &__distance)
{
    // 角速度解算以该力臂为除数, 必须严格大于 0
    if (__index >= 4 || !isfinite(__distance) || __distance <= 0.0f)
    {
        return;
    }
    distance_to_core[__index] = __distance;
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
