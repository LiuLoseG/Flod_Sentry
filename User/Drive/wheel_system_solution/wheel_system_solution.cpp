/**
 * @file wheel_system_solution.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 轮系运动学解算模块 (C++ 版) 实现
 * @note  三种轮系的里程计末段(轮位移差分 → 机体系位移 → 世界系累加)完全一致,
 *        抽为文件内静态函数, 各 class 的 protected 方法只做一行转发; 坐标系见头文件.
 *        编码器里程计的零帧抑制为"累积-触发"(wheel_odom_gate_step), 与调用频率解耦.
 * @date 2026-10-25
 * @version 2.1
 * @copyright HNAU-Robot (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "wheel_system_solution.h"
#include <cmath>
#include <cstdint>
#include "basic_math.h"

/*-----------------------------------编译期常量的类外定义-----------------------------------*/

/** @brief 计算本帧各轮线位移 ds 并更新基准; 首帧只建基准并返回 false
 *  @param __wheel_angle 当前轮转角
 *  @param __last_angle 上一帧轮转角
 *  @param __ds 本帧各轮线位移
 *  @param __radius 轮子半径
 *  @param __first_flag 首帧标志
 *  @retval true: 成功计算并更新了位移
 *  @retval false: 计算失败或首帧
 */
static bool wheel_odom_calc_ds(const float *__wheel_angle, float *__last_angle, float *__ds, const float &__radius, bool &__first_flag)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(__wheel_angle[i]))
        {
            return false;
        }
    }

    if (__first_flag)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            __last_angle[i] = __wheel_angle[i];
        }
        __first_flag = false;
        return false;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        __ds[i] = (__wheel_angle[i] - __last_angle[i]) * __radius;
        __last_angle[i] = __wheel_angle[i];
    }
    return true;
}

/**
 * @brief 里程计零帧抑制(累积-触发): 不足死区的位移先累积, 达阈值后连同历史位移一次吐出
 * @param __dx/__dy/__dyaw      本帧机体系位移 (m, m, rad)
 * @param __res_dx/__res_dy/__res_dyaw 跨帧累积器
 * @param __out_dx/__out_dy/__out_dyaw 触发时输出的机体系位移
 * @param __linear_eps   位移死区, m
 * @param __angular_eps  角度死区, rad
 * @retval true: 已触发, 输出有效且累积器清零
 * @retval false: 仍在死区内, 本帧不积分
 */
static bool wheel_odom_gate_step(const float &__dx, const float &__dy, const float &__dyaw,
                                 float &__res_dx, float &__res_dy, float &__res_dyaw,
                                 float &__out_dx, float &__out_dy, float &__out_dyaw,
                                 const float &__linear_eps, const float &__angular_eps)
{
    if (!isfinite(__dx) || !isfinite(__dy) || !isfinite(__dyaw))
    {
        return false;
    }

    __res_dx += __dx;
    __res_dy += __dy;
    __res_dyaw += __dyaw;

    if (fabsf(__res_dx) < __linear_eps && fabsf(__res_dy) < __linear_eps && fabsf(__res_dyaw) < __angular_eps)
    {
        return false;
    }

    __out_dx = __res_dx;
    __out_dy = __res_dy;
    __out_dyaw = __res_dyaw;

    __res_dx = 0.0f;
    __res_dy = 0.0f;
    __res_dyaw = 0.0f;

    return true;
}

/**
 * @brief 机体系位移 → 世界系, 累加到位姿
 */
static void wheel_odom_apply_body_step(wheel_robot_position_t &__pose, const float &__body_dx, const float &__body_dy)
{
    if (!isfinite(__body_dx) || !isfinite(__body_dy) || !isfinite(__pose.yaw))
    {
        return;
    }

    const float cos_yaw = arm_cos_f32(__pose.yaw);
    const float sin_yaw = arm_sin_f32(__pose.yaw);

    const float dx_world = __body_dx * cos_yaw - __body_dy * sin_yaw;
    const float dy_world = __body_dx * sin_yaw + __body_dy * cos_yaw;

    if (isfinite(dx_world) && isfinite(dy_world))
    {
        __pose.x += dx_world;
        __pose.y += dy_world;
    }
}

/**
 * @brief 速度里程计单帧积分: 正解速度 × 控制周期 → 位姿
 * @param __pose         待积分位姿
 * @param __speed        正运动学速度
 * @param __period       控制周期, s
 * @param __linear_eps   线速度死区, m/s
 * @param __angular_eps  角速度死区, rad/s
 * @note  与编码器里程计相互独立, 各自维护一份 encoder_pose; 死区按速度判定, 与调用频率无关.
 */
static void wheel_odom_integrate_speed(wheel_robot_position_t &__pose, const wheel_robot_speed_t &__speed, const float &__period,
                                       const float &__linear_eps, const float &__angular_eps)
{
    if (!isfinite(__speed.vx) || !isfinite(__speed.vy) || !isfinite(__speed.vw) || !isfinite(__period) || __period <= 0.0f)
    {
        return;
    }

    const bool linear_zero = (fabsf(__speed.vx) < __linear_eps) && (fabsf(__speed.vy) < __linear_eps);
    const bool angular_zero = fabsf(__speed.vw) < __angular_eps;

    // 静止/噪声帧不积分, 避免零速抖动累积成漂移
    if (linear_zero && angular_zero)
    {
        return;
    }

    __pose.yaw = basic_math_modulus_normalization(__pose.yaw + __speed.vw * __period, 2.0f * PI);
    wheel_odom_apply_body_step(__pose, __speed.vx * __period, __speed.vy * __period);
}

/*-----------------------------------omni_wheel-----------------------------------*/

/**
 * @brief 全向轮初始化
 * @param __distance_to_core 轮心到底盘圆心的投影距离, m
 * @param __control_period   控制周期(里程计回调调用间隔), s; 非法值(非有限/<=0)按 0.001 s 处理
 * @note  驱动轮半径固定为编译期常量 Wheel_Radius, 不在 init 内设置.
 */
void omni_wheel_class::init(const float &__distance_to_core, const float &__control_period)
{
    distance_to_core = (isfinite(__distance_to_core) && __distance_to_core > 0.0f) ? __distance_to_core : Wheel_Distance_To_Core;
    control_period = (isfinite(__control_period) && __control_period > 0.0f) ? __control_period : 0.001f;

    for (uint8_t i = 0; i < 4; i++)
    {
        status.wheel_speed[i] = 0.0f;
        wheel_angle[i] = 0.0f;
        last_angle_rad[i] = 0.0f;
    }
    target_speed = {0.0f, 0.0f, 0.0f};
    imu_yaw = 0.0f;
    robot_speed = {0.0f, 0.0f, 0.0f};
    encoder_pose = {0.0f, 0.0f, 0.0f};
    speed_pose = {0.0f, 0.0f, 0.0f};
    first_fb_flag = true;
    odom_residual_dx = 0.0f;
    odom_residual_dy = 0.0f;
    odom_residual_dyaw = 0.0f;
    ds_cache[0] = ds_cache[1] = ds_cache[2] = ds_cache[3] = 0.0f;
    body_dx = 0.0f;
    body_dy = 0.0f;
    body_dyaw = 0.0f;
}

/**
 * @brief 全向轮逆运动学解算
 */
void omni_wheel_class::inverse_calculate_callback()
{
    if (!isfinite(target_speed.vx) || !isfinite(target_speed.vy) || !isfinite(target_speed.vw))
    {
        return;
    }

    status.wheel_speed[0] = -(SQRT_2_2 * (-target_speed.vx + target_speed.vy) + target_speed.vw * distance_to_core) / Wheel_Radius;
    status.wheel_speed[1] = -(SQRT_2_2 * (-target_speed.vx - target_speed.vy) + target_speed.vw * distance_to_core) / Wheel_Radius;
    status.wheel_speed[2] = -(SQRT_2_2 * (target_speed.vx - target_speed.vy)  + target_speed.vw * distance_to_core) / Wheel_Radius;
    status.wheel_speed[3] = -(SQRT_2_2 * (target_speed.vx + target_speed.vy)  + target_speed.vw * distance_to_core) / Wheel_Radius;
}

/**
 * @brief 全向轮正运动学解算
 */
void omni_wheel_class::forward_calculate_callback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(status.wheel_speed[i]))
        {
            return;
        }
    }

    const float ws0 = -status.wheel_speed[0];
    const float ws1 = -status.wheel_speed[1];
    const float ws2 = -status.wheel_speed[2];
    const float ws3 = -status.wheel_speed[3];

    robot_speed.vx = (SQRT_2_4 * (-ws0 - ws1 + ws2 + ws3)) * Wheel_Radius;         // 底盘速度, m/s
    robot_speed.vy = (SQRT_2_4 * (ws0 - ws1 - ws2 + ws3)) * Wheel_Radius;
    robot_speed.vw = (ws0 + ws1 + ws2 + ws3) / (4.0f * distance_to_core) * Wheel_Radius; // 绕 Z 角速度, rad/s
}

/**
 * @brief 计算本帧各轮线位移 ds; 首帧只建基准并返回 false
 */
bool omni_wheel_class::calc_wheel_ds()
{
    return wheel_odom_calc_ds(wheel_angle, last_angle_rad, ds_cache, Wheel_Radius, first_fb_flag);
}

/**
 * @brief 机体系位移 → 世界系, 累加到 encoder_pose
 */
void omni_wheel_class::apply_body_step()
{
    wheel_odom_apply_body_step(encoder_pose, body_dx, body_dy);
}

/**
 * @brief 全向轮编码器里程计(编码器航向参考)
 */
void omni_wheel_class::encoder_odometry_callback()
{
    if (!calc_wheel_ds())
    {
        return;
    }

    const float frame_dx = SQRT_2_4 * ( ds_cache[0] + ds_cache[1] - ds_cache[2] - ds_cache[3]);
    const float frame_dy = SQRT_2_4 * (-ds_cache[0] + ds_cache[1] + ds_cache[2] - ds_cache[3]);
    const float frame_dyaw = -(ds_cache[0] + ds_cache[1] + ds_cache[2] + ds_cache[3]) / (4.0f * distance_to_core);

    // 累积-触发死区: 死区内的位移留在累积器里, 达阈值后连同历史位移一次积分
    if (!wheel_odom_gate_step(frame_dx, frame_dy, frame_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              body_dx, body_dy, body_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return;
    }

    encoder_pose.yaw = basic_math_modulus_normalization(encoder_pose.yaw + body_dyaw, 2.0f * PI);
    apply_body_step();
}

/**
 * @brief 全向轮编码器里程计(IMU航向参考)
 */
void omni_wheel_class::imu_odometry_callback()
{
    if (!isfinite(imu_yaw))
    {
        return;
    }

    // 航向直接取 IMU (首帧即以绝对航向对齐), 位移仍由编码器解算;
    // 折到 [-π, π) 与编码器航向路线一致, 否则同一 pose.yaw 会出现两种取值范围
    encoder_pose.yaw = basic_math_modulus_normalization(imu_yaw, 2.0f * PI);

    if (!calc_wheel_ds())
    {
        return;
    }

    const float frame_dx = SQRT_2_4 * ( ds_cache[0] + ds_cache[1] - ds_cache[2] - ds_cache[3]);
    const float frame_dy = SQRT_2_4 * (-ds_cache[0] + ds_cache[1] + ds_cache[2] - ds_cache[3]);
    const float frame_dyaw = -(ds_cache[0] + ds_cache[1] + ds_cache[2] + ds_cache[3]) / (4.0f * distance_to_core);

    if (!wheel_odom_gate_step(frame_dx, frame_dy, frame_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              body_dx, body_dy, body_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return;
    }

    apply_body_step();
}

/**
 * @brief 全向轮速度里程计
 */
void omni_wheel_class::speed_odometry_callback()
{
    wheel_odom_integrate_speed(speed_pose, robot_speed, control_period, Speed_Odom_Linear_Deadband, Speed_Odom_Angle_Deadband);
}

/**
 * @brief 编码器里程计位姿清零
 */
void omni_wheel_class::reset_position()
{
    encoder_pose = {0.0f, 0.0f, 0.0f};
    first_fb_flag = true; // 下一帧重建基准, 避免跳变
    odom_residual_dx = 0.0f;
    odom_residual_dy = 0.0f;
    odom_residual_dyaw = 0.0f; // 丢弃清零前的未积分位移
}

/**
 * @brief 编码器里程计位姿重定位
 */
void omni_wheel_class::set_position(const float &__x, const float &__y, const float &__yaw)
{
    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__yaw))
    {
        return;
    }
    encoder_pose.x = __x;
    encoder_pose.y = __y;
    encoder_pose.yaw = __yaw;
}

/**
 * @brief 速度里程计位姿清零
 */
void omni_wheel_class::reset_speed_position()
{
    speed_pose = {0.0f, 0.0f, 0.0f};
}

/**
 * @brief 速度里程计位姿重定位
 */
void omni_wheel_class::set_speed_position(const float &__x, const float &__y, const float &__yaw)
{
    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__yaw))
    {
        return;
    }
    speed_pose.x = __x;
    speed_pose.y = __y;
    speed_pose.yaw = __yaw;
}

/*-----------------------------------mecanum_wheel-----------------------------------*/

/**
 * @brief 麦轮初始化
 * @param __projection_x   轮子在 X 方向投影坐标, m
 * @param __projection_y   轮子在 Y 方向投影坐标, m
 * @param __control_period 控制周期(里程计回调调用间隔), s; 非法值(非有限/<=0)按 0.001 s 处理
 * @note  轮子半径固定为编译期常量 Wheel_Radius, 不在 init 内设置.
 */
void mecanum_wheel_class::init(const float &__projection_x, const float &__projection_y, const float &__control_period)
{
    projection_x = (isfinite(__projection_x) && __projection_x >= 0.0f) ? __projection_x : Wheel_Projection_X;
    projection_y = (isfinite(__projection_y) && __projection_y >= 0.0f) ? __projection_y : Wheel_Projection_Y;
    sum_xy = projection_x + projection_y;
    if (sum_xy <= 0.0f)
    {
        projection_x = Wheel_Projection_X;
        projection_y = Wheel_Projection_Y;
        sum_xy = projection_x + projection_y;
    }
    control_period = (isfinite(__control_period) && __control_period > 0.0f) ? __control_period : 0.001f;

    for (uint8_t i = 0; i < 4; i++)
    {
        status.wheel_speed[i] = 0.0f;
        wheel_angle[i] = 0.0f;
        last_angle_rad[i] = 0.0f;
    }
    target_speed = {0.0f, 0.0f, 0.0f};
    imu_yaw = 0.0f;
    robot_speed = {0.0f, 0.0f, 0.0f};
    encoder_pose = {0.0f, 0.0f, 0.0f};
    speed_pose = {0.0f, 0.0f, 0.0f};
    first_fb_flag = true;
    odom_residual_dx = 0.0f;
    odom_residual_dy = 0.0f;
    odom_residual_dyaw = 0.0f;
    ds_cache[0] = ds_cache[1] = ds_cache[2] = ds_cache[3] = 0.0f;
    body_dx = 0.0f;
    body_dy = 0.0f;
    body_dyaw = 0.0f;
}

/**
 * @brief 麦轮逆运动学解算
 */
void mecanum_wheel_class::inverse_calculate_callback()
{
    if (!isfinite(target_speed.vx) || !isfinite(target_speed.vy) || !isfinite(target_speed.vw))
    {
        return;
    }

    status.wheel_speed[0] = ( target_speed.vx - target_speed.vy - target_speed.vw * sum_xy) / Wheel_Radius; // FL
    status.wheel_speed[1] = ( target_speed.vx + target_speed.vy - target_speed.vw * sum_xy) / Wheel_Radius; // RL
    status.wheel_speed[2] = (-target_speed.vx + target_speed.vy - target_speed.vw * sum_xy) / Wheel_Radius; // RR
    status.wheel_speed[3] = (-target_speed.vx - target_speed.vy - target_speed.vw * sum_xy) / Wheel_Radius; // FR
}

/**
 * @brief 麦轮正运动学解算
 */
void mecanum_wheel_class::forward_calculate_callback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(status.wheel_speed[i]))
        {
            return;
        }
    }

    robot_speed.vx = ( status.wheel_speed[0] + status.wheel_speed[1] - status.wheel_speed[2] - status.wheel_speed[3]) * Wheel_Radius / 4.0f;
    robot_speed.vy = (-status.wheel_speed[0] + status.wheel_speed[1] + status.wheel_speed[2] - status.wheel_speed[3]) * Wheel_Radius / 4.0f;
    robot_speed.vw = -(status.wheel_speed[0] + status.wheel_speed[1] + status.wheel_speed[2] + status.wheel_speed[3]) / sum_xy * Wheel_Radius / 4.0f;
}

/**
 * @brief 计算本帧各轮线位移 ds; 首帧只建基准并返回 false
 */
bool mecanum_wheel_class::calc_wheel_ds()
{
    return wheel_odom_calc_ds(wheel_angle, last_angle_rad, ds_cache, Wheel_Radius, first_fb_flag);
}

/**
 * @brief 机体系位移 → 世界系, 累加到 encoder_pose
 */
void mecanum_wheel_class::apply_body_step()
{
    wheel_odom_apply_body_step(encoder_pose, body_dx, body_dy);
}

/**
 * @brief 麦轮编码器里程计(编码器航向参考)
 */
void mecanum_wheel_class::encoder_odometry_callback()
{
    if (!calc_wheel_ds())
    {
        return;
    }

    const float frame_dx = ( ds_cache[0] + ds_cache[1] - ds_cache[2] - ds_cache[3]) / 4.0f;
    const float frame_dy = (-ds_cache[0] + ds_cache[1] + ds_cache[2] - ds_cache[3]) / 4.0f;
    const float frame_dyaw = -(ds_cache[0] + ds_cache[1] + ds_cache[2] + ds_cache[3]) / (4.0f * sum_xy);

    // 累积-触发死区: 死区内的位移留在累积器里, 达阈值后连同历史位移一次积分
    if (!wheel_odom_gate_step(frame_dx, frame_dy, frame_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              body_dx, body_dy, body_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return;
    }

    encoder_pose.yaw = basic_math_modulus_normalization(encoder_pose.yaw + body_dyaw, 2.0f * PI);
    apply_body_step();
}

/**
 * @brief 麦轮编码器里程计(IMU航向参考)
 */
void mecanum_wheel_class::imu_odometry_callback()
{
    if (!isfinite(imu_yaw))
    {
        return;
    }

    // 航向直接取 IMU (首帧即以绝对航向对齐), 位移仍由编码器解算;
    // 折到 [-π, π) 与编码器航向路线一致, 否则同一 pose.yaw 会出现两种取值范围
    encoder_pose.yaw = basic_math_modulus_normalization(imu_yaw, 2.0f * PI);

    if (!calc_wheel_ds())
    {
        return;
    }

    const float frame_dx = ( ds_cache[0] + ds_cache[1] - ds_cache[2] - ds_cache[3]) / 4.0f;
    const float frame_dy = (-ds_cache[0] + ds_cache[1] + ds_cache[2] - ds_cache[3]) / 4.0f;
    const float frame_dyaw = -(ds_cache[0] + ds_cache[1] + ds_cache[2] + ds_cache[3]) / (4.0f * sum_xy);

    if (!wheel_odom_gate_step(frame_dx, frame_dy, frame_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              body_dx, body_dy, body_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return;
    }

    apply_body_step();
}

/**
 * @brief 麦轮速度里程计
 */
void mecanum_wheel_class::speed_odometry_callback()
{
    wheel_odom_integrate_speed(speed_pose, robot_speed, control_period, Speed_Odom_Linear_Deadband, Speed_Odom_Angle_Deadband);
}

/**
 * @brief 编码器里程计位姿清零
 */
void mecanum_wheel_class::reset_position()
{
    encoder_pose = {0.0f, 0.0f, 0.0f};
    first_fb_flag = true;
    odom_residual_dx = 0.0f;
    odom_residual_dy = 0.0f;
    odom_residual_dyaw = 0.0f; // 丢弃清零前的未积分位移
}

/**
 * @brief 编码器里程计位姿重定位
 */
void mecanum_wheel_class::set_position(const float &__x, const float &__y, const float &__yaw)
{
    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__yaw))
    {
        return;
    }
    encoder_pose.x = __x;
    encoder_pose.y = __y;
    encoder_pose.yaw = __yaw;
}

/**
 * @brief 速度里程计位姿清零
 */
void mecanum_wheel_class::reset_speed_position()
{
    speed_pose = {0.0f, 0.0f, 0.0f};
}

/**
 * @brief 速度里程计位姿重定位
 */
void mecanum_wheel_class::set_speed_position(const float &__x, const float &__y, const float &__yaw)
{
    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__yaw))
    {
        return;
    }
    speed_pose.x = __x;
    speed_pose.y = __y;
    speed_pose.yaw = __yaw;
}

/*-----------------------------------steer_wheel-----------------------------------*/

/**
 * @brief 选择舵向电机最近的等效目标, 并决定轮向电机方向
 * @param __angle          解算出的舵向目标(含零位), rad
 * @param __reference      当前舵向角(编码器多圈), rad
 * @param __last_direction 上一周期的轮向方向(±1), 用于翻转迟滞
 * @param __hysteresis     翻转迟滞, rad: 已反转时需 |e| < 90°-h 才转回, 正向时需 |e| > 90°+h 才反转
 * @param __direction      本周期方向输出, ±1
 * @retval 就近的等效舵向目标(多圈, 与 __reference 同框)
 * @note  迟滞消除 90° 边界处的逐拍翻转: 无迟滞时角度噪声会让方向在边界来回切换,
 *        轮速符号随之抖动, 舵向也会来回小角度摆动.
 */
static float steer_select_nearest_target(const float &__angle, const float &__reference, const float &__last_direction,
                                        const float &__hysteresis, float *__direction)
{
    const float angle_error = basic_math_modulus_normalization(__angle - __reference, 2.0f * PI);

    // 已反转(上一拍 dir = -1)时要求误差明显小于 90 度才允许转回, 避免边界抖动
    const float flip_threshold = PI * 0.5f + ((__last_direction < 0.0f) ? -__hysteresis : __hysteresis);

    // 轮子转向超过 90 度时, 使用反向轮速的等效舵向角
    if (fabsf(angle_error) > flip_threshold)
    {
        *__direction = -1.0f;
        return __reference + basic_math_modulus_normalization(angle_error + PI, 2.0f * PI);
    }

    *__direction = 1.0f;
    return __reference + angle_error;
}

/**
 * @brief 限制多圈舵向目标单周期变化量
 * @param __max_step        单周期最大变化量, rad
 * @param __previous_valid  上一周期目标是否有效; init 后为 false, 首拍直接从当前反馈接管
 */
static float steer_limit_target_step(const float &__target, const float &__current, const float &__previous,
                                     const float &__max_step, bool &__previous_valid)
{
    float previous = __previous;

    // 目标未初始化(init 后的首拍) / NaN / 失步(与当前角相差过大)时, 从当前反馈重新接管
    if (!__previous_valid || !isfinite(previous) || (fabsf(previous - __current) > PI))
    {
        previous = __current;
    }

    __previous_valid = true;

    const float target_step = __target - previous;

    if (target_step > __max_step)
    {
        return previous + __max_step;
    }

    if (target_step < -__max_step)
    {
        return previous - __max_step;
    }

    return __target;
}

/**
 * @brief 舵轮初始化
 * @param __target_max_step       舵向目标单周期最大变化量(斜坡斜率), rad
 * @param __max_wheel_speed       轮向电机最大角速度, rad/s
 * @param __min_limit             静止判定阈值, m/s 与 rad/s
 * @param __steer_angle_deadband  非小陀螺模式舵向误差死区, rad
 * @param __flip_hysteresis       就近转位翻转迟滞, rad; 须在 [0, π/2), 非法值(含 >= π/2)按 0.1 rad 处理
 * @param __control_period        控制周期(里程计回调调用间隔), s; 非法值(非有限/<=0)按 0.001 s 处理
 * @param __cosine_optimize_enable    转向角余弦优化开关
 * @param __nearest_optimize_enable   就近转位开关
 * @param __target_step_limit_enable  舵向目标斜坡限幅开关
 * @note  驱动轮半径固定为编译期常量 Wheel_Radius, 不在 init 内设置.
 */
void steer_wheel_class::init(const float &__target_max_step,
                             const float &__max_wheel_speed, const float &__min_limit,
                             const float &__steer_angle_deadband, const float &__flip_hysteresis,
                             const float &__control_period,
                             const bool &__cosine_optimize_enable, const bool &__nearest_optimize_enable,
                             const bool &__target_step_limit_enable)
{
    target_max_step = (isfinite(__target_max_step) && __target_max_step > 0.0f) ? __target_max_step : 0.20f;
    max_wheel_speed = (isfinite(__max_wheel_speed) && __max_wheel_speed > 0.0f) ? __max_wheel_speed : 25.0f;
    min_limit = (isfinite(__min_limit) && __min_limit >= 0.0f) ? __min_limit : 0.1f;
    steer_angle_deadband = (isfinite(__steer_angle_deadband) && __steer_angle_deadband >= 0.0f) ? __steer_angle_deadband : 0.05f;
    // 迟滞必须小于 90 度: 否则正向阈值 >= 180 度(永不反转)、反向阈值 <= 0 度(永不转回), 方向被永久锁定在一侧
    flip_hysteresis = (isfinite(__flip_hysteresis) && __flip_hysteresis >= 0.0f && __flip_hysteresis < PI * 0.5f) ? __flip_hysteresis : 0.1f;
    control_period = (isfinite(__control_period) && __control_period > 0.0f) ? __control_period : 0.001f;
    cosine_optimize_enable = __cosine_optimize_enable;
    nearest_optimize_enable = __nearest_optimize_enable;
    target_step_limit_enable = __target_step_limit_enable;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(distance_to_core[i]) || distance_to_core[i] <= 0.0f)
        {
            distance_to_core[i] = Wheel_Distance_To_Core;
        }

        // 零位为编译期常量, 无需在此校验
        status.now_angle[i] = 0.0f;
        status.now_speed[i] = 0.0f;

        status.solution_angle[i] = 0.0f;
        status.solution_speed[i] = 0.0f;

        status.cosine_optimize[i] = 1.0f;

        status.optimize_speed[i] = 0.0f;
        status.optimize_angle[i] = 0.0f;
        status.optimize_direction[i] = 1.0f;

        status.error_angle[i] = 0.0f;

        wheel_angle[i] = 0.0f;
        steer_angle[i] = 0.0f;

        last_wheel_angle[i] = 0.0f;

        // 斜坡基准无效: 下一拍逆解从当前反馈接管, 不从 0 跳变
        optimize_angle_valid[i] = false;
    }

    robot_speed = {0.0f, 0.0f, 0.0f};
    target_speed = {0.0f, 0.0f, 0.0f};

    control_mode = 0;

    imu_yaw = 0.0f;
    encoder_pose = {0.0f, 0.0f, 0.0f};
    speed_pose = {0.0f, 0.0f, 0.0f}; // 控制周期为配置量, 不随 init 复位
    first_fb_flag = true;

    odom_residual_dx = 0.0f;
    odom_residual_dy = 0.0f;
    odom_residual_dyaw = 0.0f;

    ds_cache[0] = ds_cache[1] = ds_cache[2] = ds_cache[3] = 0.0f;

    body_dx = 0.0f;
    body_dy = 0.0f;
    body_dyaw = 0.0f;
}

/**
 * @brief 舵轮逆运动学解算 + 余弦优化
 * @note  就近转位 / 斜坡限幅 / 余弦削减分别由对应开关控制; 舵向死区依附余弦开关
 */
void steer_wheel_class::inverse_calculate_callback()
{
    if (!isfinite(target_speed.vx) || !isfinite(target_speed.vy) || !isfinite(target_speed.vw))
    {
        return;
    }

    const bool cmd_zero = (fabsf(target_speed.vx) <= min_limit) && (fabsf(target_speed.vy) <= min_limit) && (fabsf(target_speed.vw) <= min_limit);

    // 逐轮逆解
    for (uint8_t i = 0; i < 4; i++)
    {
        const float vel_x = target_speed.vx - target_speed.vw * Wheel_Azimuth_Sin[i] * distance_to_core[i];
        const float vel_y = target_speed.vy + target_speed.vw * Wheel_Azimuth_Cos[i] * distance_to_core[i];

        float vel_modulus = 0.0f;
        arm_sqrt_f32(vel_x * vel_x + vel_y * vel_y, &vel_modulus);

        status.solution_speed[i] = vel_modulus / Wheel_Radius; // 轮向角速度, rad/s
        // 无速度分量时 atan2f(0,0) 无意义, 保持当前舵向角;
        // 这里要折成几何角(单圈、不含零位), 否则与其它分支的坐标系不一致
        status.solution_angle[i] = (vel_modulus > 0.0f)
                                   ? atan2f(vel_y, vel_x)
                                   : basic_math_modulus_normalization(status.now_angle[i] - Enc_Zero_Angle[i], 2.0f * PI);
    }

    // 舵向目标规划: 就近转位 + 斜坡限幅 + 死区 + 余弦削减
    for (uint8_t i = 0; i < 4; i++)
    {
        // 静止指令或无轮速时锁定当前角度, 防止舵轮无意义摆动
        if (cmd_zero || status.solution_speed[i] <= 0.0f)
        {
            status.optimize_angle[i]     = status.now_angle[i];
            status.optimize_direction[i] = 1.0f;
            status.error_angle[i]        = 0.0f;
            status.cosine_optimize[i]    = 1.0f;

            optimize_angle_valid[i] = true; // 本拍输出可作为下一拍的斜坡基准
            continue;
        }

        float optimize_direction = 1.0f;
        const float solution_target = status.solution_angle[i] + Enc_Zero_Angle[i];
        float target;

        if (nearest_optimize_enable)
        {
            // 超过 90 度时改用反向轮速的等效舵向角; 迟滞取本拍之前的方向, 防止边界逐拍翻转
            target = steer_select_nearest_target(solution_target, status.now_angle[i],
                                                 status.optimize_direction[i], flip_hysteresis, &optimize_direction);
        }

        else
        {
            // 关闭就近转位: 舵向转向解算角本体, 只折到当前多圈角度附近, 轮向方向恒为 +1
            target = status.now_angle[i] + basic_math_modulus_normalization(solution_target - status.now_angle[i], 2.0f * PI);
        }

        status.optimize_direction[i] = optimize_direction;

        // 真实未对齐角: 解算目标与当前舵向之差, 折到所选方向分支上 (0 表示已对齐)
        const float align_error = basic_math_modulus_normalization(solution_target - status.now_angle[i], 2.0f * PI);
        const float effective_error = (optimize_direction < 0.0f) ? basic_math_modulus_normalization(align_error + PI, 2.0f * PI) : align_error;

        status.error_angle[i] = effective_error;

        // 死区以真实未对齐角判定: 斜坡中的目标始终贴着当前角, 不能作为"是否已对齐"的依据
        if (cosine_optimize_enable && fabsf(effective_error) < steer_angle_deadband && control_mode != 2)
        {
            status.optimize_angle[i]  = status.now_angle[i];
            status.error_angle[i]     = 0.0f;
            status.cosine_optimize[i] = 1.0f; // 无需修舵, 轮速不削减
            optimize_angle_valid[i] = true;
            continue;
        }

        // 目标角斜坡限幅: 限制单周期变化量, 避免阶跃目标导致舵向电流尖峰与合力突变
        status.optimize_angle[i] = target_step_limit_enable
                                   ? steer_limit_target_step(target, status.now_angle[i], status.optimize_angle[i], target_max_step, optimize_angle_valid[i])
                                   : target;
        optimize_angle_valid[i] = true;

        if (cosine_optimize_enable)
        {
            // 余弦削减必须用真实未对齐角: 斜坡每拍只放行一小步, 若用斜坡后的误差, 削减项恒接近 1 而形同虚设;
            // 就近转位后 |effective_error| <= 90 度(+迟滞), 取绝对值避免与方向标志重复翻转
            const float cos_angle_raw = arm_cos_f32(effective_error);
            const float cos_angle     = nearest_optimize_enable ? fabsf(cos_angle_raw) : cos_angle_raw;
            status.cosine_optimize[i] = basic_math_constrain(cos_angle * cos_angle * cos_angle, 0.0f, 1.0f);
        }

        else
        {
            status.cosine_optimize[i] = 1.0f;
        }
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        status.optimize_speed[i] = status.solution_speed[i] * status.cosine_optimize[i] * status.optimize_direction[i];
    }

    // 静止死区: 舵轮锁定当前角的同时, 轮速一并清零, 避免小幅爬行
    if (cmd_zero)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            status.solution_speed[i] = 0.0f;
            status.optimize_speed[i] = 0.0f;
        }
    }

    float max_vector = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabsf(status.optimize_speed[i]) > max_vector)
        {
            max_vector = fabsf(status.optimize_speed[i]);
        }
    }

    if (max_vector > max_wheel_speed)
    {
        // 缩放比例不改变运动形态
        const float vector_rate = basic_math_constrain(max_wheel_speed / max_vector, 0.0f, 1.0f);

        for (uint8_t i = 0; i < 4; i++)
        {
            status.solution_speed[i] *= vector_rate; // 同步缩放, 避免下游改用 solution_speed 时限速失效
            status.optimize_speed[i] *= vector_rate;
        }
    }
}

/**
 * @brief 舵轮正运动学解算
 * @note  角速度分量直接取 sin(θ−φ), 与 θ 同源、形式直观; 代价是每轮多一次 arm_sin_f32 查表,
 *        sin/cos 已为 vx/vy 算过, 该调用相对整周期解算可忽略.
 */
void steer_wheel_class::forward_calculate_callback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(status.now_angle[i]) || !isfinite(status.now_speed[i]))
        {
            return;
        }
    }

    float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        const float wheel_linear_speed = status.now_speed[i] * Wheel_Radius; // 轮向电机角速度 → 轮子线速度, m/s

        // 反向扣除零位, 同时归一化到 [-pi,pi) 防止多圈精度退化
        const float steer = basic_math_modulus_normalization(status.now_angle[i] - Enc_Zero_Angle[i], 2.0f * PI);

        const float sin_steer = arm_sin_f32(steer);
        const float cos_steer = arm_cos_f32(steer);

        vx_sum += wheel_linear_speed * cos_steer;
        vy_sum += wheel_linear_speed * sin_steer;
        // 角速度分量: v_i·sin(θ_i−φ_i)/R_i = ω, 四轮求和再平均即底盘角速度
        vw_sum += wheel_linear_speed * arm_sin_f32(steer - Wheel_Azimuth[i]) / distance_to_core[i];
    }

    robot_speed.vx = vx_sum * 0.25f;
    robot_speed.vy = vy_sum * 0.25f;
    robot_speed.vw = vw_sum * 0.25f;
}

/**
 * @brief 计算本帧各轮线位移 ds; 首帧只建基准并返回 false
 */
bool steer_wheel_class::calc_wheel_ds()
{
    return wheel_odom_calc_ds(wheel_angle, last_wheel_angle, ds_cache, Wheel_Radius, first_fb_flag);
}

/**
 * @brief 由 ds 与舵向角解算底盘位移 (与正解投影一致)
 */
void steer_wheel_class::calc_body_displacement()
{
    body_dx = 0.0f;
    body_dy = 0.0f;
    body_dyaw = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(steer_angle[i]) || !isfinite(ds_cache[i]))
        {
            return;
        }
    }

    float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        // 扣除零位; sin/cos 对整圈不敏感
        const float steer = basic_math_modulus_normalization(steer_angle[i] - Enc_Zero_Angle[i], 2.0f * PI);
        const float sin_steer = arm_sin_f32(steer);
        const float cos_steer = arm_cos_f32(steer);

        vx_sum += ds_cache[i] * cos_steer;
        vy_sum += ds_cache[i] * sin_steer;
        vw_sum += ds_cache[i] * arm_sin_f32(steer - Wheel_Azimuth[i]) / distance_to_core[i]; // Σ = 4·dyaw
    }

    body_dx = vx_sum * 0.25f;
    body_dy = vy_sum * 0.25f;
    body_dyaw = vw_sum * 0.25f;
}

/**
 * @brief 机体系位移 → 世界系, 累加到 encoder_pose
 */
void steer_wheel_class::apply_body_step()
{
    wheel_odom_apply_body_step(encoder_pose, body_dx, body_dy);
}

/**
 * @brief 舵轮编码器里程计(编码器航向参考)
 */
void steer_wheel_class::encoder_odometry_callback()
{
    if (!calc_wheel_ds())
    {
        return;
    }

    calc_body_displacement();

    float step_dx = 0.0f, step_dy = 0.0f, step_dyaw = 0.0f;

    // 累积-触发死区: 死区内的位移留在累积器里, 达阈值后连同历史位移一次积分
    if (!wheel_odom_gate_step(body_dx, body_dy, body_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              step_dx, step_dy, step_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return;
    }

    body_dx = step_dx;
    body_dy = step_dy;
    body_dyaw = step_dyaw;

    encoder_pose.yaw = basic_math_modulus_normalization(encoder_pose.yaw + body_dyaw, 2.0f * PI);
    apply_body_step();
}

/**
 * @brief 舵轮编码器里程计(IMU航向参考)
 */
void steer_wheel_class::imu_odometry_callback()
{
    if (!isfinite(imu_yaw))
    {
        return;
    }

    // 航向直接取 IMU (首帧即以绝对航向对齐), 位移仍由编码器解算;
    // 折到 [-π, π) 与编码器航向路线一致, 否则同一 pose.yaw 会出现两种取值范围
    encoder_pose.yaw = basic_math_modulus_normalization(imu_yaw, 2.0f * PI);

    if (!calc_wheel_ds())
    {
        return;
    }

    calc_body_displacement();

    float step_dx = 0.0f, step_dy = 0.0f, step_dyaw = 0.0f;

    if (!wheel_odom_gate_step(body_dx, body_dy, body_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              step_dx, step_dy, step_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return;
    }

    body_dx = step_dx;
    body_dy = step_dy;
    body_dyaw = step_dyaw;

    apply_body_step();
}

/**
 * @brief 舵轮速度里程计
 */
void steer_wheel_class::speed_odometry_callback()
{
    wheel_odom_integrate_speed(speed_pose, robot_speed, control_period, Speed_Odom_Linear_Deadband, Speed_Odom_Angle_Deadband);
}

/**
 * @brief 编码器里程计位姿清零
 */
void steer_wheel_class::reset_position()
{
    encoder_pose = {0.0f, 0.0f, 0.0f};
    first_fb_flag = true; // 下一帧重建基准, 避免跳变
    odom_residual_dx = 0.0f;
    odom_residual_dy = 0.0f;
    odom_residual_dyaw = 0.0f; // 丢弃清零前的未积分位移
}

/**
 * @brief 编码器里程计位姿重定位
 */
void steer_wheel_class::set_position(const float &__x, const float &__y, const float &__yaw)
{
    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__yaw))
    {
        return;
    }
    encoder_pose.x = __x;
    encoder_pose.y = __y;
    encoder_pose.yaw = __yaw;
}

/**
 * @brief 速度里程计位姿清零
 */
void steer_wheel_class::reset_speed_position()
{
    speed_pose = {0.0f, 0.0f, 0.0f};
}

/**
 * @brief 速度里程计位姿重定位
 */
void steer_wheel_class::set_speed_position(const float &__x, const float &__y, const float &__yaw)
{
    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__yaw))
    {
        return;
    }
    speed_pose.x = __x;
    speed_pose.y = __y;
    speed_pose.yaw = __yaw;
}

/************************ COPYRIGHT(C) HNAU-Robot **************************/
