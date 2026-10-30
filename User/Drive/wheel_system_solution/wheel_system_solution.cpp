/**
 * @file wheel_system_solution.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-10-25
 * @version 1.2
 * @copyright HNAU-Robot (c) 2026
 * @note 正方向定义: 机器人坐标系 +X 向前, +Y 向左, +Z 向上; 角速度绕 Z 轴 CCW 为正
 */

/* Includes ------------------------------------------------------------------*/

#include "wheel_system_solution.h"
#include <cmath>
#include <cstdint>
#include "basic_math.h"

/*-----------------------------------文件内静态辅助函数-----------------------------------*/

// 编码器里程计单帧允许的最大轮转角变化量, rad.
// 半圈在 100 Hz 下已相当于 314 rad/s 以上, 远超任何实际轮速, 出现即判定为编码器丢帧/复位/多圈计数跳变.
static constexpr float Odom_Max_Frame_Angle = PI;

/**
 * @brief 力控配置整份校验
 * @param __max_wheel_torque 单轮轮端力矩上限, N·m (须 > 0)
 * @param __force_deadband  单轮力死区, N (允许 0 = 关闭)
 * @retval true: 两项都在允许域内; false: 至少一项非法
 * @note  力矩上限与死区是互相配合的一整套值, 因此这里整份判定而不做逐项合并:
 *        任一非法就整份拒绝, 原来的两项一起保留.
 */
static bool wheel_force_config_valid(const float &__max_wheel_torque, const float &__force_deadband)
{
    return isfinite(__max_wheel_torque) && __max_wheel_torque > 0.0f &&
           isfinite(__force_deadband) && __force_deadband >= 0.0f;
}

/**
 * @brief 目标加速度 -> 等效力与力矩 (F = m·a, M = J·α)
 * @param __target_accel     上层下发的目标加速度 (m/s^2, m/s^2, rad/s^2)
 * @param __chassis_mass     底盘等效质量, kg
 * @param __chassis_inertia  绕 Z 轴等效转动惯量, kg·m^2
 * @param __force_x/__force_y/__torque_z 输出等效力与力矩, N / N·m
 * @retval true: 输出有效; false: 输入非法, 所有输出已清零
 */
static bool wheel_force_wrench_from_accel(const wheel_robot_accel_t &__target_accel,
                                          const float &__chassis_mass, const float &__chassis_inertia,
                                          float &__force_x, float &__force_y, float &__torque_z)
{
    __force_x = 0.0f;
    __force_y = 0.0f;
    __torque_z = 0.0f;

    if (!isfinite(__target_accel.ax) || !isfinite(__target_accel.ay) || !isfinite(__target_accel.aw) || !isfinite(__chassis_mass) || __chassis_mass <= 0.0f || !isfinite(__chassis_inertia) || __chassis_inertia <= 0.0f)
    {
        return false;
    }

    __force_x = __chassis_mass * __target_accel.ax;
    __force_y = __chassis_mass * __target_accel.ay;
    __torque_z = __chassis_inertia * __target_accel.aw;

    return isfinite(__force_x) && isfinite(__force_y) && isfinite(__torque_z);
}

/**
 * @brief 四轮轮端力矩清零
 * @param __torque 四轮轮端力矩, N·m ([0]=FL [1]=RL [2]=RR [3]=FR).
 */
static void wheel_torque_clear(float *__torque)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        __torque[i] = 0.0f;
    }
}

/**
 * @brief 单轮期望轮端力 -> 轮端力矩 (死区 + 力矩上限)
 * @param __wheel_force 四轮期望轮端力, N ([0]=FL [1]=RL [2]=RR [3]=FR)
 * @param __torque      输出四轮轮端力矩, N·m
 * @param __radius      驱动轮半径, m
 * @param __max_wheel_torque  单轮轮端力矩上限, N·m
 * @param __deadband    单轮力死区, N
 */
static void wheel_force_to_torque(const float *__wheel_force, float *__torque, const float &__radius, const float &__max_wheel_torque, const float &__deadband)
{
    const float deadband = (isfinite(__deadband) && __deadband > 0.0f) ? __deadband : 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        const float force = __wheel_force[i];

        if (!isfinite(force))
        {
            __torque[i] = 0.0f;
            continue;
        }

        const float force_modulus = fabsf(force);

        // 死区以内不出力; 死区以外扣掉死区, 保证输出随输入连续
        const float shaped = (force_modulus <= deadband) ? 0.0f : ((force > 0.0f) ? (force_modulus - deadband) : (deadband - force_modulus));

        __torque[i] = basic_math_constrain(shaped * __radius, -__max_wheel_torque, __max_wheel_torque);
    }
}

/** @brief 计算本帧各轮线位移 ds 并更新基准; 首帧只建基准并返回 false
 *  @param __wheel_angle 当前轮转角
 *  @param __last_angle 上一帧轮转角
 *  @param __ds 本帧各轮线位移
 *  @param __radius 轮子半径
 *  @param __first_flag 首帧标志
 *  @retval true: 成功计算并更新了位移
 *  @retval false: 计算失败、首帧, 或检测到编码器跳变(已重建基准)
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

    // 跳变检测必须整帧判定
    for (uint8_t i = 0; i < 4; i++)
    {
        if (fabsf(__wheel_angle[i] - __last_angle[i]) > Odom_Max_Frame_Angle)
        {
            for (uint8_t j = 0; j < 4; j++)
            {
                __last_angle[j] = __wheel_angle[j];
            }
            return false;
        }
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
 * @note  与编码器里程计相互独立, 各自维护一份位姿; 死区按速度判定, 与调用频率无关.
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

/**
 * @brief 编码器里程计位姿清零: 重建位移基准并丢弃清零前未积分的残余位移
 */
static void wheel_odom_reset_pose(wheel_robot_position_t &__pose, bool &__first_flag,
                                  float &__res_dx, float &__res_dy, float &__res_dyaw)
{
    __pose = {0.0f, 0.0f, 0.0f};
    __first_flag = true; // 下一帧重建基准, 避免跳变
    __res_dx = 0.0f;
    __res_dy = 0.0f;
    __res_dyaw = 0.0f;
}

/**
 * @brief 里程计位姿重定位; 任一参数非有限时保持原位姿
 */
static void wheel_odom_set_pose(wheel_robot_position_t &__pose, const float &__x, const float &__y, const float &__yaw)
{
    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__yaw))
    {
        return;
    }

    __pose.x = __x;
    __pose.y = __y;
    __pose.yaw = __yaw;
}

/*-----------------------------------omni_wheel-----------------------------------*/

/**
 * @brief 全向轮初始化
 * @param __control_method   控制方式 (速控/力控/力速混控), 记录到 control_method 供上层对照
 * @param __wheel_encoder_direction 轮向电机方向, ±1: 读数正向与几何正方向同向取 +1, 反向取 -1; 其它值按 +1
 * @param __control_period   控制周期(里程计回调调用间隔), s; 非法值(非有限/<=0)按 0.001 s 处理
 * @param __max_wheel_torque 单轮轮端力矩上限, N·m (> 0)
 * @param __force_deadband   单轮力死区, N (0 = 关闭)
 * @note  力控两项互相配合, 整份校验: 任一项非法则两项全不改, 保持字段默认值.
 * @note  轮径/轮心距/底盘质量/转动惯量都是编译期常量, 本模块不提供运行期修改入口.
 */
void omni_wheel_class::init(const wheel_control_method_e &__control_method,
                            const float &__wheel_encoder_direction, const float &__control_period,
                            const float &__max_wheel_torque, const float &__force_deadband)
{
    control_method = __control_method;

    wheel_encoder_direction = (isfinite(__wheel_encoder_direction) && __wheel_encoder_direction < 0.0f) ? -1.0f : 1.0f;

    control_period = (isfinite(__control_period) && __control_period > 0.0f) ? __control_period : 0.001f;

    // 力控两项: 整份校验后写入, 任一项非法则两项全不改
    if (wheel_force_config_valid(__max_wheel_torque, __force_deadband))
    {
        max_wheel_torque = __max_wheel_torque;
        force_deadband = __force_deadband;
    }

    // status 同时承载正/逆解轮速与里程计用多圈转角, 一并清零
    status = {};

    for (uint8_t i = 0; i < 4; i++)
    {
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

    // 力控状态复位: 参数 (力矩上限/死区) 不改, 只清输入与输出
    target_accel = {0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_torque[i] = 0.0f;
    }
}

/**
 * @brief 全向轮逆运动学解算
 * @note  status.speed 是**电机域**轮速 (可直接下发 set_target_speed()): 先按几何约定
 *        (驱动方向 t_i = (−sinφ_i, cosφ_i), 即轮心切向) 算出切向轮速, 再乘轮向方向系数转回电机域.
 *        公式本身不含硬件极性: 电机正方向与切向同向取 dir=+1, 反向取 dir=-1.
 */
void omni_wheel_class::inverse_calculate_callback()
{
    if (!isfinite(target_speed.vx) || !isfinite(target_speed.vy) || !isfinite(target_speed.vw))
    {
        return;
    }

    status.speed[0] = wheel_encoder_direction * (SQRT_2_2 * (-target_speed.vx + target_speed.vy) + target_speed.vw * Wheel_Distance_To_Core) / Wheel_Radius; // FL, φ=45°
    status.speed[1] = wheel_encoder_direction * (SQRT_2_2 * (-target_speed.vx - target_speed.vy) + target_speed.vw * Wheel_Distance_To_Core) / Wheel_Radius; // RL, φ=135°
    status.speed[2] = wheel_encoder_direction * (SQRT_2_2 * ( target_speed.vx - target_speed.vy) + target_speed.vw * Wheel_Distance_To_Core) / Wheel_Radius; // RR, φ=225°
    status.speed[3] = wheel_encoder_direction * (SQRT_2_2 * ( target_speed.vx + target_speed.vy) + target_speed.vw * Wheel_Distance_To_Core) / Wheel_Radius; // FR, φ=315°
}

/**
 * @brief 全向轮正运动学解算
 * @note  输入是电机域轮速: 先乘方向系数还原成切向轮速 ws_i = dir·ω_i·R, 再按逆解的转置加权合成.
 */
void omni_wheel_class::forward_calculate_callback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(status.speed[i]))
        {
            return;
        }
    }

    const float ws0 = wheel_encoder_direction * status.speed[0];
    const float ws1 = wheel_encoder_direction * status.speed[1];
    const float ws2 = wheel_encoder_direction * status.speed[2];
    const float ws3 = wheel_encoder_direction * status.speed[3];

    robot_speed.vx = (SQRT_2_4 * (-ws0 - ws1 + ws2 + ws3)) * Wheel_Radius;      // 底盘速度, m/s
    robot_speed.vy = (SQRT_2_4 * ( ws0 - ws1 - ws2 + ws3)) * Wheel_Radius;
    robot_speed.vw = (ws0 + ws1 + ws2 + ws3) / (4.0f * Wheel_Distance_To_Core) * Wheel_Radius; // 绕 Z 角速度, rad/s
}

/**
 * @brief 全向轮编码器里程计单步
 * @param __use_imu_yaw true: 航向取 IMU; false: 航向由 dyaw 自持积分
 * @retval true: 本帧已积分; false: 数据非法 / 首帧建基准 / 仍在死区内
 */
bool omni_wheel_class::encoder_odometry_step(const bool &__use_imu_yaw)
{
    if (__use_imu_yaw)
    {
        if (!isfinite(imu_yaw))
        {
            return false;
        }

        // 航向直接取 IMU (首帧即以绝对航向对齐), 位移仍由编码器解算;
        // 折到 [-π, π) 与编码器航向路线一致, 否则同一 pose.yaw 会出现两种取值范围
        encoder_pose.yaw = basic_math_modulus_normalization(imu_yaw, 2.0f * PI);
    }

    float ds[4];
    if (!wheel_odom_calc_ds(status.angle, last_angle_rad, ds, Wheel_Radius, first_fb_flag))
    {
        return false;
    }

    // ds 是电机域线位移, 乘方向系数还原成切向位移后再合成机体系位移
    for (uint8_t i = 0; i < 4; i++)
    {
        ds[i] *= wheel_encoder_direction;
    }

    const float frame_dx = SQRT_2_4 * (-ds[0] - ds[1] + ds[2] + ds[3]);
    const float frame_dy = SQRT_2_4 * ( ds[0] - ds[1] - ds[2] + ds[3]);
    const float frame_dyaw = (ds[0] + ds[1] + ds[2] + ds[3]) / (4.0f * Wheel_Distance_To_Core);

    float step_dx = 0.0f, step_dy = 0.0f, step_dyaw = 0.0f;

    // 累积-触发死区: 死区内的位移留在累积器里, 达阈值后连同历史位移一次积分
    if (!wheel_odom_gate_step(frame_dx, frame_dy, frame_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              step_dx, step_dy, step_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return false;
    }

    if (!__use_imu_yaw)
    {
        encoder_pose.yaw = basic_math_modulus_normalization(encoder_pose.yaw + step_dyaw, 2.0f * PI);
    }

    wheel_odom_apply_body_step(encoder_pose, step_dx, step_dy);
    return true;
}

/**
 * @brief 全向轮编码器里程计(编码器航向参考)
 */
void omni_wheel_class::encoder_odometry_callback()
{
    encoder_odometry_step(false);
}

/**
 * @brief 全向轮编码器里程计(IMU航向参考)
 */
void omni_wheel_class::imu_odometry_callback()
{
    encoder_odometry_step(true);
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
    wheel_odom_reset_pose(encoder_pose, first_fb_flag, odom_residual_dx, odom_residual_dy, odom_residual_dyaw);
}

/**
 * @brief 编码器里程计位姿重定位
 */
void omni_wheel_class::set_position(const float &__x, const float &__y, const float &__yaw)
{
    wheel_odom_set_pose(encoder_pose, __x, __y, __yaw);
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
    wheel_odom_set_pose(speed_pose, __x, __y, __yaw);
}

/**
 * @brief 全向轮速控流水线: 关断力控输出 + 逆解 -> 轮向目标角速度
 * @note  入口先关断力控: 把目标加速度清成零即等价于不出力; 力矩另需显式清,
 *        因为逆解不写 wheel_torque, 不清会漏出上一条流水线的残留力矩.
 */
void omni_wheel_class::speed_calculate_callback()
{
    target_accel = {0.0f, 0.0f, 0.0f};
    wheel_torque_clear(wheel_torque);

    inverse_calculate_callback();
}

/**
 * @brief 等效力/力矩 -> 四轮期望轮端力
 * @note  力分配是逆解矩阵的转置(功率对偶): 驱动方向 t_i = (−sinφ_i, cosφ_i) 为单位向量,
 *        故平动项系数为 √2/4, 旋转项每轮 +fw/(4d). 逆解结果喂回本式可还原 (fx, fy, fw).
 * @note  输出已乘轮向方向系数, 即**电机域**轮端力 (与 status.speed 同域), 可直接乘轮径成力矩下发.
 */
void omni_wheel_class::force_wheel_force_vector(const float &__force_x, const float &__force_y, const float &__torque_z,
                                                float (&__wheel_force)[4]) const
{
    const float yaw_force = __torque_z / (4.0f * Wheel_Distance_To_Core);

    __wheel_force[0] = wheel_encoder_direction * (SQRT_2_4 * (-__force_x + __force_y) + yaw_force); // FL
    __wheel_force[1] = wheel_encoder_direction * (SQRT_2_4 * (-__force_x - __force_y) + yaw_force); // RL
    __wheel_force[2] = wheel_encoder_direction * (SQRT_2_4 * ( __force_x - __force_y) + yaw_force); // RR
    __wheel_force[3] = wheel_encoder_direction * (SQRT_2_4 * ( __force_x + __force_y) + yaw_force); // FR
}

/**
 * @brief 全向轮纯力控: 目标加速度 -> 轮端力矩
 */
void omni_wheel_class::force_calculate_callback()
{
    float force_x = 0.0f, force_y = 0.0f, torque_z = 0.0f;

    // 目标加速度为零 (未喂 / 已切速控) 时解出的力也是零, 无需另设使能位
    if (!wheel_force_wrench_from_accel(target_accel, Chassis_Mass, Chassis_Inertia,
                                       force_x, force_y, torque_z))
    {
        wheel_torque_clear(wheel_torque);
        return;
    }

    float wheel_force[4];
    force_wheel_force_vector(force_x, force_y, torque_z, wheel_force);
    wheel_force_to_torque(wheel_force, wheel_torque, Wheel_Radius, max_wheel_torque, force_deadband);
}

/**
 * @brief 全向轮力速混控: 速度逆解 -> 轮向目标角速度, 目标加速度 -> 轮端力矩前馈
 * @note  速度项与力矩项解耦: 速度由逆解给目标角速度(下层速度环闭环), 力矩只做加速度前馈.
 *        逆解不写力矩也不清力矩, 所以力矩必须在本函数里自己算; force_calculate_callback()
 *        正好就是"加速度 -> 轮端力矩", 两者输出同域, 直接复用它. 逆解与力矩项写不同字段,
 *        顺序无关.
 */
void omni_wheel_class::mixed_calculate_callback()
{
    inverse_calculate_callback();
    force_calculate_callback();
}

/*-----------------------------------mecanum_wheel-----------------------------------*/

/**
 * @brief 麦轮初始化
 * @param __control_method 控制方式 (速控/力控/力速混控), 记录到 control_method 供上层对照
 * @param __wheel_encoder_direction 轮向电机方向, ±1: 读数正向与几何正方向同向取 +1, 反向取 -1; 其它值按 +1
 * @param __control_period 控制周期(里程计回调调用间隔), s; 非法值(非有限/<=0)按 0.001 s 处理
 * @param __max_wheel_torque 单轮轮端力矩上限, N·m (> 0)
 * @param __force_deadband   单轮力死区, N (0 = 关闭)
 * @note  力控两项互相配合, 整份校验: 任一项非法则两项全不改, 保持字段默认值.
 * @note  轮径/轮心投影/底盘质量/转动惯量都是编译期常量, 本模块不提供运行期修改入口.
 */
void mecanum_wheel_class::init(const wheel_control_method_e &__control_method,
                               const float &__wheel_encoder_direction, const float &__control_period,
                               const float &__max_wheel_torque, const float &__force_deadband)
{
    control_method = __control_method;

    wheel_encoder_direction = (isfinite(__wheel_encoder_direction) && __wheel_encoder_direction < 0.0f) ? -1.0f : 1.0f;

    control_period = (isfinite(__control_period) && __control_period > 0.0f) ? __control_period : 0.001f;

    // 力控两项: 整份校验后写入, 任一项非法则两项全不改
    if (wheel_force_config_valid(__max_wheel_torque, __force_deadband))
    {
        max_wheel_torque = __max_wheel_torque;
        force_deadband = __force_deadband;
    }

    // status 同时承载正/逆解轮速与里程计用多圈转角, 一并清零
    status = {};

    for (uint8_t i = 0; i < 4; i++)
    {
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

    // 力控状态复位: 参数 (力矩上限/死区) 不改, 只清输入与输出
    target_accel = {0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_torque[i] = 0.0f;
    }
}

/**
 * @brief 麦轮逆运动学解算
 * @note  status.speed 是**电机域**轮速 (可直接下发 set_target_speed()): 几何轮速再乘轮向方向系数.
 */
void mecanum_wheel_class::inverse_calculate_callback()
{
    if (!isfinite(target_speed.vx) || !isfinite(target_speed.vy) || !isfinite(target_speed.vw))
    {
        return;
    }

    status.speed[0] = wheel_encoder_direction * ( target_speed.vx - target_speed.vy - target_speed.vw * Wheel_Sum_XY) / Wheel_Radius; // FL
    status.speed[1] = wheel_encoder_direction * ( target_speed.vx + target_speed.vy - target_speed.vw * Wheel_Sum_XY) / Wheel_Radius; // RL
    status.speed[2] = wheel_encoder_direction * (-target_speed.vx + target_speed.vy - target_speed.vw * Wheel_Sum_XY) / Wheel_Radius; // RR
    status.speed[3] = wheel_encoder_direction * (-target_speed.vx - target_speed.vy - target_speed.vw * Wheel_Sum_XY) / Wheel_Radius; // FR
}

/**
 * @brief 麦轮正运动学解算
 * @note  先把电机域轮速还原成几何轮速, 再参与合成.
 */
void mecanum_wheel_class::forward_calculate_callback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(status.speed[i]))
        {
            return;
        }
    }

    const float ws0 = wheel_encoder_direction * status.speed[0];
    const float ws1 = wheel_encoder_direction * status.speed[1];
    const float ws2 = wheel_encoder_direction * status.speed[2];
    const float ws3 = wheel_encoder_direction * status.speed[3];

    robot_speed.vx = ( ws0 + ws1 - ws2 - ws3) * Wheel_Radius / 4.0f;
    robot_speed.vy = (-ws0 + ws1 + ws2 - ws3) * Wheel_Radius / 4.0f;
    robot_speed.vw = -(ws0 + ws1 + ws2 + ws3) / Wheel_Sum_XY * Wheel_Radius / 4.0f;
}

/**
 * @brief 麦轮编码器里程计单步
 * @param __use_imu_yaw true: 航向取 IMU; false: 航向由 dyaw 自持积分
 * @retval true: 本帧已积分; false: 数据非法 / 首帧建基准 / 仍在死区内
 */
bool mecanum_wheel_class::encoder_odometry_step(const bool &__use_imu_yaw)
{
    if (__use_imu_yaw)
    {
        if (!isfinite(imu_yaw))
        {
            return false;
        }

        // 航向直接取 IMU (首帧即以绝对航向对齐), 位移仍由编码器解算;
        // 折到 [-π, π) 与编码器航向路线一致, 否则同一 pose.yaw 会出现两种取值范围
        encoder_pose.yaw = basic_math_modulus_normalization(imu_yaw, 2.0f * PI);
    }

    float ds[4];
    if (!wheel_odom_calc_ds(status.angle, last_angle_rad, ds, Wheel_Radius, first_fb_flag))
    {
        return false;
    }

    // ds 是电机域线位移, 乘方向系数还原成几何驱动方向上的位移后再合成机体系位移
    for (uint8_t i = 0; i < 4; i++)
    {
        ds[i] *= wheel_encoder_direction;
    }

    const float frame_dx = ( ds[0] + ds[1] - ds[2] - ds[3]) / 4.0f;
    const float frame_dy = (-ds[0] + ds[1] + ds[2] - ds[3]) / 4.0f;
    const float frame_dyaw = -(ds[0] + ds[1] + ds[2] + ds[3]) / (4.0f * Wheel_Sum_XY);

    float step_dx = 0.0f, step_dy = 0.0f, step_dyaw = 0.0f;

    // 累积-触发死区: 死区内的位移留在累积器里, 达阈值后连同历史位移一次积分
    if (!wheel_odom_gate_step(frame_dx, frame_dy, frame_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              step_dx, step_dy, step_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return false;
    }

    if (!__use_imu_yaw)
    {
        encoder_pose.yaw = basic_math_modulus_normalization(encoder_pose.yaw + step_dyaw, 2.0f * PI);
    }

    wheel_odom_apply_body_step(encoder_pose, step_dx, step_dy);
    return true;
}

/**
 * @brief 麦轮编码器里程计(编码器航向参考)
 */
void mecanum_wheel_class::encoder_odometry_callback()
{
    encoder_odometry_step(false);
}

/**
 * @brief 麦轮编码器里程计(IMU航向参考)
 */
void mecanum_wheel_class::imu_odometry_callback()
{
    encoder_odometry_step(true);
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
    wheel_odom_reset_pose(encoder_pose, first_fb_flag, odom_residual_dx, odom_residual_dy, odom_residual_dyaw);
}

/**
 * @brief 编码器里程计位姿重定位
 */
void mecanum_wheel_class::set_position(const float &__x, const float &__y, const float &__yaw)
{
    wheel_odom_set_pose(encoder_pose, __x, __y, __yaw);
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
    wheel_odom_set_pose(speed_pose, __x, __y, __yaw);
}

/**
 * @brief 麦轮速控流水线: 关断力控输出 + 逆解 -> 轮向目标角速度
 * @note  入口先关断力控: 把目标加速度清成零即等价于不出力; 力矩另需显式清,
 *        因为逆解不写 wheel_torque, 不清会漏出上一条流水线的残留力矩.
 */
void mecanum_wheel_class::speed_calculate_callback()
{
    target_accel = {0.0f, 0.0f, 0.0f};
    wheel_torque_clear(wheel_torque);

    inverse_calculate_callback();
}

/**
 * @brief 等效力/力矩 -> 四轮期望轮端力
 * @note  力分配是逆解矩阵的转置(功率对偶): 逆解每轮驱动方向 d_i 长度为 √2 且 Σd_i·d_iᵀ = 4I,
 *        故平动项系数为 1/4; 逆解 vw 系数恒为 -(a+b), 对应 Σf_i = -fw/(a+b),
 *        即每轮旋转项 -fw/(4(a+b)). 逆解结果喂回本式可还原 (fx, fy, fw).
 * @note  输出已乘轮向方向系数, 即**电机域**轮端力 (与 status.speed 同域), 可直接乘轮径成力矩下发.
 */
void mecanum_wheel_class::force_wheel_force_vector(const float &__force_x, const float &__force_y, const float &__torque_z,
                                                   float (&__wheel_force)[4]) const
{
    const float yaw_force = -__torque_z / (4.0f * Wheel_Sum_XY);

    __wheel_force[0] = wheel_encoder_direction * (( __force_x - __force_y) * 0.25f + yaw_force); // FL
    __wheel_force[1] = wheel_encoder_direction * (( __force_x + __force_y) * 0.25f + yaw_force); // RL
    __wheel_force[2] = wheel_encoder_direction * ((-__force_x + __force_y) * 0.25f + yaw_force); // RR
    __wheel_force[3] = wheel_encoder_direction * ((-__force_x - __force_y) * 0.25f + yaw_force); // FR
}

/**
 * @brief 麦轮纯力控: 目标加速度 -> 轮端力矩
 */
void mecanum_wheel_class::force_calculate_callback()
{
    float force_x = 0.0f, force_y = 0.0f, torque_z = 0.0f;

    // 目标加速度为零 (未喂 / 已切速控) 时解出的力也是零, 无需另设使能位
    if (!wheel_force_wrench_from_accel(target_accel, Chassis_Mass, Chassis_Inertia, force_x, force_y, torque_z))
    {
        wheel_torque_clear(wheel_torque);
        return;
    }

    float wheel_force[4];
    force_wheel_force_vector(force_x, force_y, torque_z, wheel_force);
    wheel_force_to_torque(wheel_force, wheel_torque, Wheel_Radius, max_wheel_torque, force_deadband);
}

/**
 * @brief 麦轮力速混控: 速度逆解 -> 轮向目标角速度, 目标加速度 -> 轮端力矩前馈
 * @note  逆解不写力矩也不清力矩, 力矩由 force_calculate_callback() 自己算; 两者写不同字段,
 *        顺序无关.
 */
void mecanum_wheel_class::mixed_calculate_callback()
{
    inverse_calculate_callback();
    force_calculate_callback();
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
 * @brief 舵轮初始化: 一次性写入全部可配参数
 * @param __control_method        控制方式 (速控/力控/力速混控), 记录到 control_method 供上层对照
 * @param __steer_encoder_direction 舵向编码器方向, ±1: 读数正向与几何 CCW 同向取 +1, 反向取 -1; 其它值按 +1
 * @param __control_period        控制周期(里程计回调调用间隔), s; 非法值(非有限/<=0)按 0.001 s 处理
 * @param __target_max_step       舵向目标单周期最大变化量(斜坡斜率), rad; 须 > 0
 * @param __max_wheel_speed       轮向电机最大角速度, rad/s; 须 > 0
 * @param __min_limit             静止判定阈值, m/s 与 rad/s; 允许 0
 * @param __steer_angle_deadband  非小陀螺模式舵向误差死区, rad; 允许 0
 * @param __flip_hysteresis       就近转位翻转迟滞, rad; 须在 [0, π/2), 否则跳过
 * @param __max_wheel_torque      单轮轮端力矩上限, N·m; 须 > 0
 * @param __force_deadband        单轮力死区(扣除式), N; 允许 0
 * @param __cosine_optimize_enable    余弦削减开关 (速控与力控共用)
 * @param __nearest_optimize_enable   就近转位开关 (速控与力控共用)
 * @param __target_step_limit_enable  舵向目标斜坡限幅开关 (速控与力控共用)
 * @note  校验口径分两种, 因为两类参数的性质不同:
 *          速控八项互相独立 (改斜坡不会让限速失效) —— 逐项调 setter, 单项非法只跳过该项;
 *          力控两项互相配合 (力矩上限与死区是一套) —— 整份校验, 任一项非法则两项全不改.
 * @note  驱动轮半径/编码器零位/轮心力臂/底盘质量与转动惯量都是编译期常量, 不在 init 内设置.
 */
void steer_wheel_class::init(const wheel_control_method_e &__control_method,
                             const float &__steer_encoder_direction,
                             const float &__control_period,
                             const float &__target_max_step, const float &__max_wheel_speed,
                             const float &__min_limit, const float &__steer_angle_deadband,
                             const float &__flip_hysteresis,
                             const float &__max_wheel_torque, const float &__force_deadband,
                             const bool &__cosine_optimize_enable, const bool &__nearest_optimize_enable,
                             const bool &__target_step_limit_enable)
{
    control_method = __control_method;

    steer_encoder_direction = (isfinite(__steer_encoder_direction) && __steer_encoder_direction < 0.0f) ? -1.0f : 1.0f;

    control_period = (isfinite(__control_period) && __control_period > 0.0f) ? __control_period : 0.001f;

    // 速控八项: 逐项委托给 setter, 各自校验, 非法项保持字段默认值
    // (后三个优化开关虽归在"速控参数"组, 但速控与力控两条流水线共用同一份配置, 见 setter 注释)
    set_target_max_step(__target_max_step);
    set_max_wheel_speed(__max_wheel_speed);
    set_min_limit(__min_limit);
    set_steer_angle_deadband(__steer_angle_deadband);
    set_flip_hysteresis(__flip_hysteresis);
    set_cosine_optimize_enable(__cosine_optimize_enable);
    set_nearest_optimize_enable(__nearest_optimize_enable);
    set_target_step_limit_enable(__target_step_limit_enable);

    // 力控两项: 整份校验后写入, 任一项非法则两项全不改
    if (wheel_force_config_valid(__max_wheel_torque, __force_deadband))
    {
        max_wheel_torque = __max_wheel_torque;
        force_deadband = __force_deadband;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        // 零位与力臂均为编译期常量, 无需在此校验
        status.now_angle[i] = 0.0f;
        status.now_speed[i] = 0.0f;

        status.solution_angle[i] = 0.0f;
        status.solution_speed[i] = 0.0f;

        status.cosine_optimize[i] = 1.0f;

        status.optimize_speed[i] = 0.0f;
        status.optimize_angle[i] = 0.0f;
        status.optimize_direction[i] = 1.0f;

        status.error_angle[i] = 0.0f;

        status.force_direction[i] = 1.0f;

        status.wheel_torque[i] = 0.0f;

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

    // 力控输入复位: 参数 (力矩上限/死区) 不改; 输出力矩已随上面 status 的清零覆盖, 不重复清
    target_accel = {0.0f, 0.0f, 0.0f};
}

/**
 * @brief 舵轮逆运动学解算 + 余弦优化
 * @note  就近转位 / 斜坡限幅 / 余弦削减分别由对应开关控制; 舵向死区依附余弦开关
 * @note  入口清零力控输出: wheel_torque 只在本拍有力控调用时才非零, 避免上一拍力矩残留
 */
void steer_wheel_class::inverse_calculate_callback()
{
    if (!isfinite(target_speed.vx) || !isfinite(target_speed.vy) || !isfinite(target_speed.vw))
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        status.wheel_torque[i] = 0.0f;
    }

    const bool cmd_zero = (fabsf(target_speed.vx) <= min_limit) && (fabsf(target_speed.vy) <= min_limit) && (fabsf(target_speed.vw) <= min_limit);

    for (uint8_t i = 0; i < 4; i++)
    {
        // 轮心速度 = 底盘平移 + ω×r_i = (vx − ω·y_i, vy + ω·x_i); 正方形/长方形布局同一式
        const float vel_x = target_speed.vx - target_speed.vw * Wheel_Sign_Y[i] * Wheel_Projection_Y;
        const float vel_y = target_speed.vy + target_speed.vw * Wheel_Sign_X[i] * Wheel_Projection_X;

        float vel_modulus = 0.0f;
        arm_sqrt_f32(vel_x * vel_x + vel_y * vel_y, &vel_modulus);

        status.solution_speed[i] = vel_modulus / Wheel_Radius; // 轮向角速度, rad/s
        // 无速度分量时 atan2f(0,0) 无意义, 保持当前舵向角;
        // 这里要折成几何角(单圈、不含零位), 否则与其它分支的坐标系不一致
        status.solution_angle[i] = (vel_modulus > 0.0f)
                                   ? atan2f(vel_y, vel_x)
                                   : steer_encoder_direction * basic_math_modulus_normalization(status.now_angle[i] - Enc_Zero_Angle[i], 2.0f * PI);
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
        const float solution_target = steer_encoder_direction * status.solution_angle[i] + Enc_Zero_Angle[i];
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

        //开启余弦优化，且不为小陀螺模式下，舵向误差在死区内，认为舵向已足够对齐，无需再压轮速

        // 目标角斜坡限幅: 限制单周期变化量, 避免阶跃目标导致舵向电流尖峰与合力突变
        status.optimize_angle[i] = target_step_limit_enable
                                   ? steer_limit_target_step(target, status.now_angle[i], status.optimize_angle[i], target_max_step, optimize_angle_valid[i])
                                   : target;
        optimize_angle_valid[i] = true;

        if (cosine_optimize_enable && (fabsf(effective_error) < steer_angle_deadband) && (control_mode != 2))
        {
            // 余弦削减必须用真实未对齐角: 斜坡每拍只放行一小步, 若用斜坡后的误差, 削减项恒接近 1 而形同虚设;
            // 就近转位后 |effective_error| <= 90 度(+迟滞), 取绝对值避免与方向标志重复翻转
            const float cos_angle_raw = arm_cos_f32(effective_error);
            const float cos_angle     = nearest_optimize_enable ? fabsf(cos_angle_raw) : cos_angle_raw;
            status.cosine_optimize[i] = basic_math_constrain(cos_angle * cos_angle * cos_angle, 0.0f, 1.0f);
        }

        else
        {
            // 不削减: 余弦开关关闭, 或真实未对齐角已在死区内 (已足够对齐, 无需再压轮速)
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
 * @note  角速度分量取轮速在"绕底盘中心的切向" t_i = (−sinφ_i, cosφ_i) 上的投影除以力臂:
 *        u_i·(p_i·t_i)/d_i = u_i·sin(θ_i−φ_i)/d_i. 实现上复用已算好的 sin/cos:
 *        sin(θ−φ) = sinθ·cosφ − cosθ·sinφ, 不额外查表.
 *        此处的 "−φ" 与逆解耦合项 (vx − ω·d·sinφ, vy + ω·d·cosφ) 严格互为逆运算.
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

        // 反向扣除零位并按编码器方向换算成几何角, 同时归一化到 [-pi,pi) 防止多圈精度退化
        const float steer = basic_math_modulus_normalization(steer_encoder_direction * (status.now_angle[i] - Enc_Zero_Angle[i]), 2.0f * PI);

        const float sin_steer = arm_sin_f32(steer);
        const float cos_steer = arm_cos_f32(steer);

        vx_sum += wheel_linear_speed * cos_steer;
        vy_sum += wheel_linear_speed * sin_steer;
        // 角速度分量: u·(sinθ·x_i − cosθ·y_i)/d²
        vw_sum += wheel_linear_speed * (sin_steer * Wheel_Sign_X[i] * Wheel_Projection_X - cos_steer * Wheel_Sign_Y[i] * Wheel_Projection_Y);
    }

    robot_speed.vx = vx_sum * 0.25f;
    robot_speed.vy = vy_sum * 0.25f;
    robot_speed.vw = vw_sum * 0.25f / Wheel_D2;
}

/**
 * @brief 由各轮线位移与舵向角解算机体系位移 (与正解投影一致)
 * @retval true: 输出有效; false: 任一舵向角或轮位移非有限, 输出已清零
 */
bool steer_wheel_class::calc_body_displacement(const float *__ds, float &__dx, float &__dy, float &__dyaw) const
{
    __dx = 0.0f;
    __dy = 0.0f;
    __dyaw = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (!isfinite(steer_angle[i]) || !isfinite(__ds[i]))
        {
            return false;
        }
    }

    float vx_sum = 0.0f, vy_sum = 0.0f, vw_sum = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        // 扣除零位并按编码器方向换算成几何角; sin/cos 对整圈不敏感
        const float steer = basic_math_modulus_normalization(steer_encoder_direction * (steer_angle[i] - Enc_Zero_Angle[i]), 2.0f * PI);
        const float sin_steer = arm_sin_f32(steer);
        const float cos_steer = arm_cos_f32(steer);

        vx_sum += __ds[i] * cos_steer;
        vy_sum += __ds[i] * sin_steer;
        // 角速度分量: 与正解同一式 u·(sinθ·x_i − cosθ·y_i)/d², d² 提到循环外除一次
        vw_sum += __ds[i] * (sin_steer * Wheel_Sign_X[i] * Wheel_Projection_X
                           - cos_steer * Wheel_Sign_Y[i] * Wheel_Projection_Y);
    }

    __dx = vx_sum * 0.25f;
    __dy = vy_sum * 0.25f;
    __dyaw = vw_sum * 0.25f / Wheel_D2;
    return true;
}

/**
 * @brief 舵轮编码器里程计单步
 * @param __use_imu_yaw true: 航向取 IMU; false: 航向由 dyaw 自持积分
 * @retval true: 本帧已积分; false: 数据非法 / 首帧建基准 / 仍在死区内
 */
bool steer_wheel_class::encoder_odometry_step(const bool &__use_imu_yaw)
{
    if (__use_imu_yaw)
    {
        if (!isfinite(imu_yaw))
        {
            return false;
        }

        // 航向直接取 IMU (首帧即以绝对航向对齐), 位移仍由编码器解算;
        // 折到 [-π, π) 与编码器航向路线一致, 否则同一 pose.yaw 会出现两种取值范围
        encoder_pose.yaw = basic_math_modulus_normalization(imu_yaw, 2.0f * PI);
    }

    float ds[4];
    if (!wheel_odom_calc_ds(wheel_angle, last_wheel_angle, ds, Wheel_Radius, first_fb_flag))
    {
        return false;
    }

    float frame_dx = 0.0f, frame_dy = 0.0f, frame_dyaw = 0.0f;
    if (!calc_body_displacement(ds, frame_dx, frame_dy, frame_dyaw))
    {
        return false;
    }

    float step_dx = 0.0f, step_dy = 0.0f, step_dyaw = 0.0f;

    // 累积-触发死区: 死区内的位移留在累积器里, 达阈值后连同历史位移一次积分
    if (!wheel_odom_gate_step(frame_dx, frame_dy, frame_dyaw,
                              odom_residual_dx, odom_residual_dy, odom_residual_dyaw,
                              step_dx, step_dy, step_dyaw, Odom_Linear_Deadband, Odom_Angle_Deadband))
    {
        return false;
    }

    if (!__use_imu_yaw)
    {
        encoder_pose.yaw = basic_math_modulus_normalization(encoder_pose.yaw + step_dyaw, 2.0f * PI);
    }

    wheel_odom_apply_body_step(encoder_pose, step_dx, step_dy);
    return true;
}

/**
 * @brief 舵轮编码器里程计(编码器航向参考)
 */
void steer_wheel_class::encoder_odometry_callback()
{
    encoder_odometry_step(false);
}

/**
 * @brief 舵轮编码器里程计(IMU航向参考)
 */
void steer_wheel_class::imu_odometry_callback()
{
    encoder_odometry_step(true);
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
    wheel_odom_reset_pose(encoder_pose, first_fb_flag, odom_residual_dx, odom_residual_dy, odom_residual_dyaw);
}

/**
 * @brief 编码器里程计位姿重定位
 */
void steer_wheel_class::set_position(const float &__x, const float &__y, const float &__yaw)
{
    wheel_odom_set_pose(encoder_pose, __x, __y, __yaw);
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
    wheel_odom_set_pose(speed_pose, __x, __y, __yaw);
}

/**
 * @brief 舵轮速控流水线: 关断力控输出 + 逆解 -> 舵向/轮向目标
 * @note  关断这一步不能省: 逆解本身不清目标加速度, 从力控/混控切回速控后若不清,
 *        上一拍加速度会继续参与诊断与后续力矩解算.
 *        力矩不必在此清: inverse_calculate_callback() 入口已清零 status.wheel_torque.
 * @note  与 inverse_calculate_callback() 的分工: 后者是纯逆运动学, 不碰力控状态,
 *        混控要用它(加速度刚写入, 不能被关断); 本函数是速控的完整入口.
 */
void steer_wheel_class::speed_calculate_callback()
{
    target_accel = {0.0f, 0.0f, 0.0f};

    inverse_calculate_callback();
}

/**
 * @brief 力控核心
 * @note  方向 / 余弦削减系数 / 未对齐角 直接读写 status 的同一组字段 (与 inverse_calculate_callback()
 *        共用), 本函数不自备 cosine / optimize_direction 局部副本:
 *        纯力控由力方向自己把这三位算出来, 混控复用逆解算出的 optimize_direction / error_angle,
 *        只按"当前实际舵向 vs 力方向"的夹角覆盖 cosine_optimize.
 * @note  出力只有一处: wheel_force[i] = force_modulus * cosine_optimize * force_direction.
 *        出力方向用**力域**的 force_direction, 不用逆解的轮向方向: 电机域正力矩恒把底盘推向
 *        +û(θ), 与轮子正/反转无关, 而混控减速时力与轮向恰好相反 —— 若用轮向方向定符号会越减越快.
 */
void steer_wheel_class::force_calculate_core()
{
    float robot_force_x = 0.0f, robot_force_y = 0.0f, robot_torque_w = 0.0f;

    // 加速度非法直接退出
    if (!wheel_force_wrench_from_accel(target_accel, Chassis_Mass, Chassis_Inertia, robot_force_x, robot_force_y, robot_torque_w))
    {
        wheel_torque_clear(status.wheel_torque);

        if (control_method != Wheel_Mixed_Control_Method)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                status.optimize_speed[i]     = 0.0f;
                status.optimize_angle[i]     = status.now_angle[i];
                status.optimize_direction[i] = 1.0f;
                status.cosine_optimize[i]    = 0.0f;
                status.error_angle[i]        = 0.0f;
                status.force_direction[i]    = 1.0f;
                optimize_angle_valid[i]      = false;
            }
        }
        return;
    }

    // 偏航力增益 Y = mz/Σd², N/m
    const float yaw_force = robot_torque_w / (4.0f * Wheel_D2);
    // |e| < db ⇔ |cos e| > cos(db): 混控拿幅值比它定"是否削减"
    const float cos_deadband = arm_cos_f32(steer_angle_deadband);

    float wheel_force[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0; i < 4; i++)
    {
        // 期望轮端力矢量
        const float force_x = robot_force_x * 0.25f - yaw_force * Wheel_Sign_Y[i] * Wheel_Projection_Y;
        const float force_y = robot_force_y * 0.25f + yaw_force * Wheel_Sign_X[i] * Wheel_Projection_X;

        // 合力矩
        float force_modulus = 0.0f;
        arm_sqrt_f32(force_x * force_x + force_y * force_y, &force_modulus);

        // 死区内的力不作舵向规划, 直接锁定当前角度, 防止舵轮无意义摆动
        if (force_modulus <= force_deadband)
        {
            if (control_method != Wheel_Mixed_Control_Method)
            {
                // 纯力控: 舵向锁定当前角, 不做无意义摆动
                status.optimize_speed[i]     = 0.0f;
                status.optimize_angle[i]     = status.now_angle[i];
                status.optimize_direction[i] = 1.0f;
                status.cosine_optimize[i]    = 0.0f;
                status.error_angle[i]        = 0.0f;
                status.force_direction[i]    = 1.0f;
                optimize_angle_valid[i]      = true;
            }
            continue;
        }

        // 出力系数直接落在共用状态上: 余弦幅值缺省不削减 (1 = 已对齐), 出力方向由各分支各自定
        status.cosine_optimize[i] = 1.0f;

        if (control_method != Wheel_Mixed_Control_Method)
        {
            // 纯力控不产生轮速目标, 防止下游速度环误用
            status.optimize_speed[i] = 0.0f;

            const float force_target = steer_encoder_direction * atan2f(force_y, force_x) + Enc_Zero_Angle[i];
            float target;

            if (nearest_optimize_enable)
            {
                // 超过 90 度时改用反向出力的等效舵向角; 迟滞取本拍之前的方向, 防止边界逐拍翻转
                target = steer_select_nearest_target(force_target, status.now_angle[i], status.optimize_direction[i], flip_hysteresis, &status.optimize_direction[i]);
            }

            else// 关闭就近转位
            {
                // 舵向转向力方向本体, 轮向方向恒为 +1
                status.optimize_direction[i] = 1.0f;
                target = status.now_angle[i] + basic_math_modulus_normalization(force_target - status.now_angle[i], 2.0f * PI);
            }

            const float align_error = basic_math_modulus_normalization(force_target - status.now_angle[i], 2.0f * PI);
            const float effective_error = (status.optimize_direction[i] < 0.0f) ? basic_math_modulus_normalization(align_error + PI, 2.0f * PI) : align_error;

            // 纯力控的舵向就由力方向决定, 出力方向即就近转位选定的分支
            status.force_direction[i] = status.optimize_direction[i];

            status.error_angle[i] = effective_error;

            // 余弦削减
            if (cosine_optimize_enable && (fabsf(effective_error) >= steer_angle_deadband))
            {
                const float cos_angle_raw = arm_cos_f32(effective_error);
                const float cos_angle     = nearest_optimize_enable ? fabsf(cos_angle_raw) : cos_angle_raw;
                status.cosine_optimize[i] = basic_math_constrain(cos_angle * cos_angle * cos_angle, 0.0f, 1.0f);
            }

            // 目标角斜坡限幅: 限制单周期变化量, 避免阶跃目标导致舵向电流尖峰与合力突变
            status.optimize_angle[i] = target_step_limit_enable ? steer_limit_target_step(target, status.now_angle[i], status.optimize_angle[i], target_max_step, optimize_angle_valid[i]) : target;
            optimize_angle_valid[i] = true;
        }

        else
        {
            // 混控，舵向与轮速目标已由逆解算出
            const float steer = basic_math_modulus_normalization(steer_encoder_direction * (status.now_angle[i] - Enc_Zero_Angle[i]), 2.0f * PI);
            const float cos_angle = (force_x * arm_cos_f32(steer) + force_y * arm_sin_f32(steer)) / force_modulus;

            status.force_direction[i] = (cos_angle >= 0.0f) ? 1.0f : -1.0f;

            // 余弦削减
            if (cosine_optimize_enable && (fabsf(cos_angle) <= cos_deadband))
            {
                const float cos_angle_mag = basic_math_constrain(fabsf(cos_angle), 0.0f, 1.0f);
                status.cosine_optimize[i] = cos_angle_mag * cos_angle_mag * cos_angle_mag;
            }
        }

        // 轮端力矩 = 轮端力 * 余弦削减 * 出力方向
        wheel_force[i] = force_modulus * status.cosine_optimize[i] * status.force_direction[i];
    }

    wheel_force_to_torque(wheel_force, status.wheel_torque, Wheel_Radius, max_wheel_torque, force_deadband);
}

/**
 * @brief 纯力控
 */
void steer_wheel_class::force_calculate_callback()
{
    force_calculate_core();
}

/**
 * @brief 力速混控
 * @note 运动学逆解计算出舵向/轮向目标，再按力控计算出轮端力矩，舵向目标不受力控影响
 */
void steer_wheel_class::mixed_calculate_callback()
{
    inverse_calculate_callback();

    force_calculate_core();
}

/************************ COPYRIGHT(C) HNAU-Robot **************************/
