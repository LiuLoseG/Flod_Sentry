/**
 * @file gimbal.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-11-08
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "gimbal.h"

// 统一CAN分发层
#include "can_distribution.h"

// 安全守护进程(电机离线检测)
#include "dm_motor.h"
#include "dsp/basic_math_functions.h"
#include "gimbal_identification.h"
#include "pid.h"
#include "reference_track_plan.h"
#include "robot_def.h"
#include "safe_daemon_process.h"
#include "wave_generator.h"
#include "bsp_dwt.h"

// 统一数学库
#include "arm_math.h"
#include "basic_math.h"
#include <cmath>
#include <cstring>

// FreeRTOS
#include "freertos.h"
#include "cmsis_os2.h"

// 云台总结构体
gimbal_t gimbal;

// 调试变量

wave_generator_t test_wave; // 波形发生器
float small_pitch_c_angle = 0.00065f;
float small_yaw_c_angle = 0.00112f;

// 调试目标值
float debug_data = 0.6f;
float small_pitch_gravity = 0.0f;
float gravity_torque = 0.0f; // 小pitch轴重力前馈量(同时供 Ozone 观察)

/**
 * @brief 小pitch轴前馈参数
 * @note 故意不加 const: 便于在 Ozone 中直接改写做参数迭代, 重新上电恢复下表的初值
 *       与原辨识模型同构: tau = J*acc + B*rate + Fc*tanh(rate/eps) + A*sin(q) + C*cos(q)
 */
typedef struct
{
    float j;           // 等效惯量, kg·m^2
    float b;           // 粘滞阻尼, N·m·s/rad
    float fc;          // 库仑摩擦幅值, N·m
    float eps;         // 库仑摩擦 tanh 速度尺度, rad/s
    float grav_sin;    // 重力项 sin 系数, N·m
    float grav_cos;    // 重力项 cos 系数, N·m
    float dynamic_max; // 动态项限幅, N·m
} pitch_feedforward_param_t;

pitch_feedforward_param_t pitch_ff = {
    0.000226f,      // j: RLS 在线辨识值 (原值 0.0109 是辨识初值, 比在线估计大 48 倍)
    0.0146f,        // b: RLS 在线辨识值, 协方差偏大, 需高加速度激励后复核
    0.13f,          // fc: 实测滞回半宽 ≈0.13 N·m; 原值 0.553 会让摩擦前馈过补偿约 4 倍
    0.08f,          // eps: 与辨识配置 friction_velocity_rad_s 保持一致
    -0.134125665f,  // grav_sin: 静态实测拟合 A≈-0.03~-0.08, 原值保留待复核
    1.00039673f,    // grav_cos: 静态实测拟合 C≈1.00~1.05
    0.5f            // dynamic_max: ≈J*max_accel + B*max_rate + Fc, 须远小于 SMALL_PITCH_TORQUE_LIMIT
};

/*------------------------------------云台电机can接收对象(注册到统一分发层）-------------------------------------------*/
static void small_yaw_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.small_yaw_motor.can_rx_callback();     // CAN1 0x11
    gimbal.timestamp.small_yaw_us = (uint32_t)bsp_dwt_get_timeline_us(); // 更新时间戳
}

static void small_pitch_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.small_pitch_motor.can_rx_callback();   // CAN1 0x12
    gimbal.timestamp.small_pitch_us = (uint32_t)bsp_dwt_get_timeline_us(); // 更新时间戳
}

static void big_pitch_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.big_pitch_motor.can_rx_callback();     // CAN1 0x13
    gimbal.timestamp.big_pitch_us = (uint32_t)bsp_dwt_get_timeline_us(); // 更新时间戳
}

static void big_yaw_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.big_yaw_motor.can_rx_callback();       // CAN2 0x14
    gimbal.timestamp.big_yaw_us = (uint32_t)bsp_dwt_get_timeline_us(); // 更新时间戳
}

/*------------------------------------云台电机系统辨识函数-------------------------------------------*/

/**
 * @brief 云台小yaw轴系统辨识初始化函数
 */
static void small_yaw_identification_config_init(gimbal_identification_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->axis = GIMBAL_IDENTIFICATION_YAW;
    config->configuration_id = GIMBAL_IDENTIFICATION_YAW;   // 必须与样本里的 configuration_id 一致，否则每帧都会被判为配置不匹配
    config->window_s = 0.05f;                 // 50 ms：覆盖约 50 个 1 kHz 控制周期，降低单帧噪声
    config->max_gap_s = 0.02f;                // 20 ms：允许少量任务抖动，但不跨越明显丢帧
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区

    rls_last_default_config(&config->rls, 3u);

    config->rls.initial[0] = 0.01f;          // J 起步量：仅用于 RLS 初值，不是实测标定值
    config->rls.initial[1] = 0.005f;         // B 起步量：保持与小型云台输出轴量级同阶
    config->rls.initial[2] = 0.02f;          // Fc 起步量：给低速摩擦项一个非零可学习初值

    config->rls.minimum[0] = 0.000001f;      // J 必须为正，禁止 RLS 退化成非物理负惯量
    config->rls.minimum[1] = 0.0f;
    config->rls.minimum[2] = 0.0f;

    config->rls.maximum[0] = 1.0f;           // 覆盖常见云台等效惯量，超出需按实测机构放宽
    config->rls.maximum[1] = 5.0f;           // B 上界避免噪声把阻尼推向异常大值
    config->rls.maximum[2] = 5.0f;           // Fc 上界与最大反馈扭矩同量级

    config->rls.feature_scale[0] = 10.0f;    // 角加速度通常最大，缩放改善 J 列条件数
    config->rls.dt_min_s = 0.000001f;
    config->rls.dt_max_s = 0.1f;
}

/**
 * @brief 云台小pitch轴系统辨识初始化函数
 */
static void small_pitch_identification_config_init(gimbal_identification_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));

    config->axis = GIMBAL_IDENTIFICATION_PITCH;
    config->configuration_id = GIMBAL_IDENTIFICATION_PITCH;   // 必须与样本里的 configuration_id 一致，否则每帧都会被判为配置不匹配
    config->window_s = 0.05f;                 // 50 ms：覆盖约 50 个 1 kHz 控制周期，降低单帧噪声
    config->max_gap_s = 0.02f;                // 20 ms：允许少量任务抖动，但不跨越明显丢帧
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区
    rls_last_default_config(&config->rls, 5u);

    config->rls.initial[0] = 0.000226f;          // J 起步量：仅用于 RLS 初值，不是实测标定值
    config->rls.initial[1] = 0.0146f;         // B 起步量：保持与小型云台输出轴量级同阶
    config->rls.initial[2] = 0.13f;          // Fc 起步量：给低速摩擦项一个非零可学习初值
    config->rls.initial[3] = -0.134125665f;       // sin 重力项无先验方向，使用零中心
    config->rls.initial[4] = 1.00039673f;       // cos 重力项提供轻微非零起点，避免初期完全失激励

    config->rls.minimum[0] = 0.000001f;      // J 必须为正，禁止 RLS 退化成非物理负惯量
    config->rls.minimum[1] = 0.0f;
    config->rls.minimum[2] = 0.0f;
    config->rls.minimum[3] = -5.0f;
    config->rls.minimum[4] = -5.0f;

    config->rls.maximum[0] = 1.0f;           // 覆盖常见云台等效惯量，超出需按实测机构放宽
    config->rls.maximum[1] = 5.0f;           // B 上界避免噪声把阻尼推向异常大值
    config->rls.maximum[2] = 5.0f;           // Fc 上界与最大反馈扭矩同量级
    config->rls.maximum[3] = 5.0f;
    config->rls.maximum[4] = 5.0f;

    config->rls.feature_scale[0] = 10.0f;    // 角加速度通常最大，缩放改善 J 列条件数

    config->rls.dt_min_s = 0.000001f;
    config->rls.dt_max_s = 0.1f;
}

/*------------------------------------云台控制器轨迹规划函数-------------------------------------------*/

/**
 * @brief 云台小yaw轨迹规划器配置初始化函数
 */
static void small_yaw_track_plan_config_init(reference_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));

    config->max_rate_rad_s = 5.0f;           // 最大角速度，单位：rad/s
    config->max_accel_rad_s2 = 65.0f;        // 最大角加速度，单位：rad/s^2 (不要降低: 它同时决定跟随的加速度能力, 降低会加大跟踪滞后)
    config->max_jerk_rad_s3 = 5000.0f;       // 最大角加加速度，单位：rad/s^3 (≈透明, 仅挡异常尖峰; 太小会削掉正常拐点)
    config->position_gain_per_s = 55.0f;      // 位置比例增益，单位：1/秒 (P滞后≈v/Kp)
    config->rate_servo_gain_per_s = 450.0f;   // 速率环比例增益，单位：1/秒 

    config->dt_min_s = 0.0002f;               // 最小采样时间，单位：秒
    config->dt_max_s = 0.005f;                // 最大采样时间，单位：秒 (真正被长时间抢占的判据)
    config->dt_filter_alpha = 0.03f;          // 采样时间低通系数 (等效时间常数 ≈33ms, 抑制 1kHz 调度抖动)

    config->wrap_angle = true;                // 是否进行角度环绕处理(最短角规划)
    config->accel_sat_guard_enable = true;    // 轨迹规划器饱和守护函数

    config->sat_gain_scale = 0.5f;           // 饱和时位置增益缩放系数, 0 < scale < 1 (建议 0.4~0.6)
    config->sat_release_per_s = 2.0f;        // 饱和解除后增益恢复速率, 单位 1/s 
}

/**
 * @brief 云台小pitch轨迹规划器配置初始化函数
 */
static void small_pitch_track_plan_config_init(reference_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));

    config->max_rate_rad_s = 5.0f;           // 最大角速度，单位：rad/s
    config->max_accel_rad_s2 = 58.0f;        // 最大角加速度，单位：rad/s^2 (不要降低: 它同时决定跟随的加速度能力, 降低会加大跟踪滞后)
    config->max_jerk_rad_s3 = 4500.0f;       // 最大角加加速度，单位：rad/s^3 (≈透明, 仅挡异常尖峰; 太小会削掉正常拐点)
    config->position_gain_per_s = 35.0f;      // 位置比例增益，单位：1/秒 (P滞后≈v/Kp)
    config->rate_servo_gain_per_s = 450.0f;   // 速率环比例增益，单位：1/秒 (wn=13.8Hz, zeta=0.80)

    config->dt_min_s = 0.0002f;               // 最小采样时间，单位：秒
                                             // 只作为"不可能为真"的健全性下界, 见 small_yaw 配置处说明
    config->dt_max_s = 0.005f;                // 最大采样时间，单位：秒
    config->dt_filter_alpha = 0.03f;          // 采样时间低通系数 (等效时间常数 ≈33ms, 抑制 1kHz 调度抖动)

    config->wrap_angle = false;                // 是否进行角度环绕处理(最短角规划)
    config->accel_sat_guard_enable = true;    // 轨迹规划器饱和守护函数

    config->sat_gain_scale = 0.5f;           // 饱和时位置增益缩放系数, 0 < scale < 1 (0.4~0.6)
    config->sat_release_per_s = 2.0f;        // 饱和解除后增益恢复速率, 单位 1/s (1~3)
}

/**
 * @brief 填充云台系统辨识样本
 */
static void gimbal_identification_sample_fill(gimbal_identification_sample_t *sample, uint32_t dt_us, float feedback_s, uint32_t seg_id, uint32_t conf_id, 
                                              float angle_rad, float rate_rad_s, float torque_nm, float gravity_angle_rad, float torque_limit,
                                              bool enabled, bool valid)
{
    if (sample == NULL) return;
    sample->timestamp_us = dt_us;
    sample->feedback_age_s = feedback_s;
    sample->segment_id = seg_id;
    sample->configuration_id = conf_id;

    sample->angle_rad = angle_rad;
    sample->rate_rad_s = rate_rad_s;
    sample->torque_nm = torque_nm;
    sample->gravity_angle_rad = gravity_angle_rad;

    sample->enabled = enabled;
    sample->valid = valid;


    sample->saturated = fabsf(torque_nm) >= 0.98f * torque_limit;
    sample->operating_conditions_valid = enabled && valid;
    sample->torque_calibrated = true;
    sample->torque_time_aligned = valid;
}

/*------------------------------------云台任务-------------------------------------------*/

/**
 * @brief 云台初始化函数
 * @note 初始化云台相关的传感器、控制器和参数
 */

void gimbal_init(void)
{
    // 绑定外部对象指针
    gimbal.imu_data = &imu_frame;
    gimbal.hi12_imu = &hi12_data;
    gimbal.crsf = &crsf;
    gimbal.source = &gimbal_decision_source;  
    gimbal.decision = &robot_decision; 

    // 初始化云台电机对象
    gimbal.small_yaw_motor.init(&hfdcan1, 0x11, 0x01, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);
    gimbal.small_pitch_motor.init(&hfdcan1, 0x12, 0x02, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);
    gimbal.big_pitch_motor.init(&hfdcan1, 0x13, 0x03, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);
    gimbal.big_yaw_motor.init(&hfdcan2, 0x14, 0x04, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);

    // 初始化控制低通滤波器
    gimbal.chassis_angular_filter.init(5.0f, 0.001f);

    gimbal.pc_filter.yaw_angle.init(50.0f, 0.001f);
    gimbal.pc_filter.yaw_omega.init(50.0f, 0.001f);
    gimbal.pc_filter.yaw_torque.init(50.0f, 0.001f);
    
    gimbal.pc_filter.pitch_angle.init(50.0f, 0.001f);
    gimbal.pc_filter.pitch_omega.init(50.0f, 0.001f);
    gimbal.pc_filter.pitch_torque.init(50.0f, 0.001f);

    // 规划器滤波器
    gimbal.plan_filter.yaw_angle.init(100, 0.001f);
    gimbal.plan_filter.yaw_omega.init(15, 0.001f);
    gimbal.plan_filter.yaw_torque.init(15, 0.001f);

    gimbal.plan_filter.pitch_angle.init(100, 0.001f);
    gimbal.plan_filter.pitch_omega.init(10, 0.001f);
    gimbal.plan_filter.pitch_torque.init(10, 0.001f);

    // 初始化云台轨迹规划器
    small_yaw_track_plan_config_init(&gimbal.small_yaw_plan_config);
    reference_track_plan_init(&gimbal.small_yaw_plan, &gimbal.small_yaw_plan_config, 0.0f);

    small_pitch_track_plan_config_init(&gimbal.small_pitch_plan_config);
    reference_track_plan_init(&gimbal.small_pitch_plan, &gimbal.small_pitch_plan_config, 0.0f);

    // 初始化遥控器PID控制器
    gimbal.rc_small_yaw_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.rc_small_yaw_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.rc_small_pitch_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.rc_small_pitch_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.rc_big_yaw_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.rc_big_yaw_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.rc_big_pitch_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.rc_big_pitch_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    // 初始化云台自瞄PID控制器
    gimbal.pc_small_yaw_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_small_yaw_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.pc_small_pitch_pid.angle.init(10.85f, 0.0f, 0.05f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    // gimbal.pc_small_pitch_pid.speed.init(0.65f, 0.23f, 0.0f, 0.0f, 0.65f, 10.0f, 0.001f, 0.0f, 0.45f, 0.82f, 0.86f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_small_pitch_pid.speed.init(0.58f, 0.32f, 0.0f, 0.0f, 0.65f, 10.0f, 0.001f, 0.0f, 0.45f, 0.82f, 0.88f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.pc_big_yaw_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_big_yaw_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.pc_big_pitch_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_big_pitch_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.big_yaw_omega_slope.init(0.001f, 0.001f, Slope_First_REAL);

    // 重力补偿: 历史四次多项式系数, 仅保留作对照,
    // 当前重力前馈已改用 pitch_ff 的 A*sin(q) + C*cos(q) 解析式(与辨识模型同构)
    gimbal.s_pit_gra_comp.factor[0] = 0.8920f;
    gimbal.s_pit_gra_comp.factor[1] = 0.0353f;
    gimbal.s_pit_gra_comp.factor[2] = -0.1295f;
    gimbal.s_pit_gra_comp.factor[3] = 0.1594f;
    gimbal.s_pit_gra_comp.factor[4] = -0.0285f;
    gimbal.s_pit_gra_comp.amount = 0.0f;
    // 控制器前馈量
    

    // 初始化云台系统辨识参数
    small_yaw_identification_config_init(&gimbal.ident_small_yaw_config);
    gimbal_identification_init(&gimbal.ident_small_yaw, &gimbal.ident_small_yaw_config);
    small_pitch_identification_config_init(&gimbal.ident_small_pitch_config);
    gimbal_identification_init(&gimbal.ident_small_pitch, &gimbal.ident_small_pitch_config);

    // 注册CAN接收回调函数到统一分发层
    can_distribution_register(1, 0x11, small_yaw_motor_rx_client);
    can_distribution_register(1, 0x12, small_pitch_motor_rx_client);
    can_distribution_register(1, 0x13, big_pitch_motor_rx_client);
    can_distribution_register(2, 0x14, big_yaw_motor_rx_client);

    // 注册电机离线检测回调函数到安全守护进程
    SAFE_DAEMON_PROCESS_REGISTER("small_yaw_motor", gimbal.small_yaw_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("small_pitch_motor", gimbal.small_pitch_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("big_pitch_motor", gimbal.big_pitch_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("big_yaw_motor", gimbal.big_yaw_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);

    // 初始化云台模式
    gimbal.big_pitch_ready = false;
    gimbal.control_ready = false;
    gimbal.now_method = Gimbal_Method_Zero_Force;
    gimbal.last_method = Gimbal_Method_Zero_Force;
    gimbal.now_mode = Gimbal_Auto_Disable;
    gimbal.last_mode = Gimbal_Auto_Disable;

    // 波形发送器(调试用), 初始化后必须使能, 否则 calc() 直接返回 offset 恒定值
    wave_generator_init(&test_wave, WAVE_SINE, 0.1f, 0.5f, 0.0f);
    wave_generator_enable(&test_wave);
}

/**
 * @brief 更新云台模式
 */
void gimbal_update_mode(void)
{
    switch (gimbal.crsf->get_left_toggle())
    {
        case CRSF_SWITCH_TRIG_MID_UP:               // 中拨→上拨过渡
        case CRSF_SWITCH_UP:                        // 左拨动开关上拨
        {
            gimbal.now_method = Gimbal_Method_Auto;
            break;
        }

        case CRSF_SWITCH_TRIG_UP_MID:               // 上拨→中拨过渡
        case CRSF_SWITCH_TRIG_DOWN_MID:             // 下拨→中拨过渡
        case CRSF_SWITCH_MID:                       // 左拨动开关中拨
        {
            gimbal.now_method = Gimbal_Method_Manual;
            break;
        }

        case CRSF_SWITCH_TRIG_MID_DOWN:             // 中拨→下拨过渡
        case CRSF_SWITCH_DOWN:                      // 左拨动开关下拨
        {
            gimbal.now_method = Gimbal_Method_Zero_Force;
            break;
        }

        default:
        {
            gimbal.now_method = Gimbal_Method_Zero_Force;
            break;
        }
    }

    if(gimbal.crsf->get_left_lock() == CRSF_SWITCH_PRESS)
    {
        gimbal.target_val.big_pitch_status = Gimbal_Big_Pitch_Up;
    }

    else
    {
        gimbal.target_val.big_pitch_status = Gimbal_Big_Pitch_Down;
    }

}

/**
 * @brief 更新云台测量值
 */
void gimbal_update_measure(void)
{
    // 角速度更新

    //旋转矩阵消除z轴偏置导致的角速度分量偏转
    gimbal.now_val.small_yaw_omega = (gimbal.hi12_imu->gyr_z() * BASIC_MATH_DEG_TO_RAD) * arm_cos_f32(gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD) - 
                                     (gimbal.hi12_imu->gyr_x() *  BASIC_MATH_DEG_TO_RAD) * arm_sin_f32(gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD);

    gimbal.now_val.small_pitch_omega = gimbal.hi12_imu->gyr_y() * BASIC_MATH_DEG_TO_RAD;        //  pitch电机角速度来自于IMU陀螺仪

    gimbal.now_val.big_yaw_omega = gimbal.imu_data->gyro[2];
    gimbal.now_val.big_pitch_omega = gimbal.big_pitch_motor.get_now_omega();

    // 角度更新
    gimbal.now_val.gimbal_yaw_angle = gimbal.hi12_imu->yaw() * BASIC_MATH_DEG_TO_RAD;
    gimbal.now_val.small_yaw_angle = gimbal.small_yaw_motor.get_total_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
    gimbal.now_val.small_pitch_angle = gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD;
    gimbal.now_val.big_yaw_angle = gimbal.small_yaw_motor.get_total_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
    gimbal.now_val.big_pitch_angle = gimbal.big_pitch_motor.get_total_angle();    

    gimbal.ffd_val.chassis_angular = gimbal.decision->can_board_communication.get_chassis_angular();

    // 编码器到imu角度限制更新
    gimbal.small_pitch_imu_limit_angle = gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD - (gimbal.small_pitch_motor.get_total_angle() - SMALL_PITCH_MOTOR_MID_ANGLE);

    // pitch电机角度是否到位判断
    if (basic_math_abs(BIG_PITCH_MOTOR_TAG_ANGLE - gimbal.now_val.big_pitch_angle) > BIG_PITCH_MOTOR_FIL_DIF_ANGLE)
    {
        gimbal.big_pitch_ready = false;
    }
    else
    {
        gimbal.big_pitch_ready = true;
    }

    if(basic_math_abs(gimbal.now_val.big_pitch_angle - BIG_PITCH_START_ANGLE) <= BIG_PITCH_MOTOR_FIL_DIF_ANGLE)
    {
        gimbal.now_val.big_pitch_status = Gimbal_Big_Pitch_Down;
    }

    else if(basic_math_abs(gimbal.now_val.big_pitch_angle - BIG_PITCH_MOTOR_TAG_ANGLE) <= BIG_PITCH_MOTOR_FIL_DIF_ANGLE)
    {
        gimbal.now_val.big_pitch_status = Gimbal_Big_Pitch_Up;
    }

    else
    {
        gimbal.now_val.big_pitch_status = Gimbal_Big_Pitch_Progress;
    }

}

/**
 * @brief 更新云台观测值(低通滤波等状态观测量)
 */
void gimbal_update_observate(void)
{
    gimbal.input_val.yaw_angle = gimbal.decision->pc_to_mcu_aim.get_yaw_angle();
    gimbal.input_val.yaw_omega = gimbal.decision->pc_to_mcu_aim.get_ff_yaw_w();
    gimbal.input_val.yaw_torque = gimbal.decision->pc_to_mcu_aim.get_ff_yaw_a();

    gimbal.input_val.pitch_angle = gimbal.decision->pc_to_mcu_aim.get_pitch_angle();
    gimbal.input_val.pitch_omega = gimbal.decision->pc_to_mcu_aim.get_ff_pitch_w();
    gimbal.input_val.pitch_torque = gimbal.decision->pc_to_mcu_aim.get_ff_pitch_a();

    gimbal.pc_filter.yaw_angle.set_now_input(gimbal.input_val.yaw_angle);
    gimbal.pc_filter.yaw_omega.set_now_input(gimbal.input_val.yaw_omega);
    gimbal.pc_filter.yaw_torque.set_now_input(gimbal.input_val.yaw_torque);

    gimbal.pc_filter.pitch_angle.set_now_input(gimbal.input_val.pitch_angle);
    gimbal.pc_filter.pitch_omega.set_now_input(gimbal.input_val.pitch_omega);
    gimbal.pc_filter.pitch_torque.set_now_input(gimbal.input_val.pitch_torque);

    gimbal.pc_filter.yaw_angle.low_calculate_callback();
    gimbal.pc_filter.yaw_omega.low_calculate_callback();
    gimbal.pc_filter.yaw_torque.low_calculate_callback();

    gimbal.pc_filter.pitch_angle.low_calculate_callback();
    gimbal.pc_filter.pitch_omega.low_calculate_callback();
    gimbal.pc_filter.pitch_torque.low_calculate_callback();


    gimbal.input_fil_val.yaw_angle = gimbal.pc_filter.yaw_angle.get_out();
    gimbal.input_fil_val.yaw_omega = gimbal.pc_filter.yaw_omega.get_out();
    gimbal.input_fil_val.yaw_torque = gimbal.pc_filter.yaw_torque.get_out();

    gimbal.input_fil_val.pitch_angle = gimbal.pc_filter.pitch_angle.get_out();
    gimbal.input_fil_val.pitch_omega = gimbal.pc_filter.pitch_omega.get_out();
    gimbal.input_fil_val.pitch_torque = gimbal.pc_filter.pitch_torque.get_out();
}

/**
 * @brief 切换模式时，清除控制器和目标值的状态，防止数据冲击和跳变
 */
static void gimbal_switch_clear_condition(void)
{
    gimbal.target_val.gimbal_yaw_angle = gimbal.now_val.gimbal_yaw_angle;
    gimbal.target_val.small_yaw_angle = gimbal.now_val.small_yaw_angle;
    gimbal.target_val.small_pitch_angle = gimbal.now_val.small_pitch_angle;
    gimbal.target_val.big_yaw_angle = gimbal.now_val.big_yaw_angle;
    gimbal.target_val.big_pitch_angle = gimbal.now_val.big_pitch_angle;

    gimbal.target_val.small_yaw_omega = 0.0f;
    gimbal.target_val.small_pitch_omega = 0.0f;
    gimbal.target_val.big_yaw_omega = 0.0f;
    gimbal.target_val.big_pitch_omega = 0.0f;

    gimbal.rc_small_yaw_pid.angle.pid_clear();
    gimbal.rc_small_yaw_pid.speed.pid_clear();

    gimbal.rc_small_pitch_pid.angle.pid_clear();
    gimbal.rc_small_pitch_pid.speed.pid_clear();

    gimbal.rc_big_yaw_pid.angle.pid_clear();
    gimbal.rc_big_yaw_pid.speed.pid_clear();

    gimbal.rc_big_pitch_pid.angle.pid_clear();
    gimbal.rc_big_pitch_pid.speed.pid_clear();

    gimbal.pc_small_yaw_pid.angle.pid_clear();
    gimbal.pc_small_yaw_pid.speed.pid_clear();

    gimbal.pc_small_pitch_pid.angle.pid_clear();
    gimbal.pc_small_pitch_pid.speed.pid_clear();

    gimbal.pc_big_yaw_pid.angle.pid_clear();
    gimbal.pc_big_yaw_pid.speed.pid_clear();

    gimbal.pc_big_pitch_pid.angle.pid_clear();
    gimbal.pc_big_pitch_pid.speed.pid_clear();
}

/**
 * @brief 云台控制决策函数
 */
void gimbal_control_decision(void)
{
    if(gimbal.now_method == Gimbal_Method_Zero_Force)   // 零力矩模式(目标值等于当前值)
    {
        gimbal_switch_clear_condition();
    }

    else if (gimbal.now_method == Gimbal_Method_Manual)
    {
        if(gimbal.last_method != Gimbal_Method_Manual)
        {
            gimbal_switch_clear_condition();
        }

        else
        {
            if (gimbal.big_pitch_ready)
            {
                float yaw_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_x(), -1.0f, 1.0f, 0.065f) * SMALL_YAW_ANGLE_INCREMENT_RATION;
                float pitch_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_y(), -1.0f, 1.0f, 0.065f) * SMALL_PITCH_ANGLE_INCREMENT_RATION;

                gimbal.target_val.gimbal_yaw_angle = basic_math_loop_constrain(gimbal.target_val.gimbal_yaw_angle + yaw_angle_increment, -PI, PI);

                // 小yaw
                {
                    float delta = basic_math_loop_constrain(gimbal.target_val.gimbal_yaw_angle - gimbal.now_val.gimbal_yaw_angle, -PI, PI);
                    gimbal.target_val.small_yaw_angle = basic_math_constrain(gimbal.now_val.small_yaw_angle + delta, SMALL_YAW_MIN_OFFSET, SMALL_YAW_MAX_OFFSET);
                }

                // 小pitch
                {
                    gimbal.target_val.small_pitch_angle = basic_math_constrain(gimbal.target_val.small_pitch_angle + pitch_angle_increment, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MIN_OFFSET, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MAX_OFFSET);
                }

                // 大yaw
                {
                    float small_yaw_offset = gimbal.small_yaw_motor.get_total_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
                    if(small_yaw_offset <= SMALL_YAW_MIN_OFFSET * 1.1f || small_yaw_offset >= SMALL_YAW_MAX_OFFSET * 1.1f)
                    {
                        // 大小yaw相对角异常
                        gimbal.target_val.big_yaw_angle = gimbal.now_val.big_yaw_angle;
                    }

                    else
                    {
                        gimbal.target_val.big_yaw_angle = 0.0f;
                    }
                }
            }

            else
            {
                gimbal.now_val.big_yaw_angle = gimbal.big_yaw_motor.get_now_angle();
                float yaw_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_x(), -1.0f, 1.0f, 0.065f) * SMALL_YAW_ANGLE_INCREMENT_RATION;

                gimbal.target_val.gimbal_yaw_angle = gimbal.now_val.gimbal_yaw_angle;
                gimbal.target_val.small_yaw_angle = 0.0f;
                gimbal.target_val.small_pitch_angle = 0.0f;
                gimbal.target_val.big_yaw_angle = basic_math_loop_constrain(gimbal.target_val.big_yaw_angle - gimbal.now_val.big_yaw_angle + yaw_angle_increment, -PI, PI) + gimbal.now_val.big_yaw_angle;
            }

            // 大pitch
            if (gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Up)
            {
                gimbal.target_val.big_pitch_angle = BIG_PITCH_MOTOR_TAG_ANGLE;
            }
            else if (gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Down)
            {
                gimbal.target_val.big_pitch_angle = BIG_PITCH_START_ANGLE;
            }
            else
            {
                gimbal.target_val.big_pitch_angle = gimbal.now_val.big_pitch_angle;
            }
        }
    }

    else if (gimbal.now_method == Gimbal_Method_Auto)
    {
        if(gimbal.last_method != Gimbal_Method_Auto)
        {
            gimbal_switch_clear_condition();
        }

        else 
        {
            // 自瞄未识别到目标，进行索敌扫频
            if(gimbal.now_mode == Gimbal_Auto_Scan)
            {
                gimbal.target_val.gimbal_yaw_angle = gimbal.now_val.gimbal_yaw_angle;

                if(gimbal.last_mode != Gimbal_Auto_Scan)
                {
                    gimbal_switch_clear_condition();
                }   
                else
                {
                    if(gimbal.big_pitch_ready)
                    {
                        // 小yaw
                        if(gimbal.small_yaw_towards_flag == false)
                        {
                            gimbal.target_val.small_yaw_angle += small_yaw_c_angle;

                            if (gimbal.target_val.small_yaw_angle >= (SMALL_YAW_MAX_OFFSET - 0.03f))
                            {
                                gimbal.small_yaw_towards_flag = true;
                            }
                        }

                        else
                        {
                            gimbal.target_val.small_yaw_angle -= small_yaw_c_angle;

                            if (gimbal.target_val.small_yaw_angle <= (SMALL_YAW_MIN_OFFSET + 0.03f))
                            {
                                gimbal.small_yaw_towards_flag = false;
                            }
                        }
                        gimbal.target_val.small_yaw_angle = basic_math_constrain(gimbal.target_val.small_yaw_angle, SMALL_YAW_MIN_OFFSET, SMALL_YAW_MAX_OFFSET);

                        // 小pitch
                        if(gimbal.small_pit_towards_flag == false)
                        {
                            gimbal.target_val.small_pitch_angle += small_pitch_c_angle;

                            if (gimbal.target_val.small_pitch_angle >= (gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MAX_OFFSET - 0.06f))
                            {
                                gimbal.small_pit_towards_flag = true;
                            }
                        }

                        else
                        {
                            gimbal.target_val.small_pitch_angle -= small_pitch_c_angle;

                            if (gimbal.target_val.small_pitch_angle <= (gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MIN_OFFSET + 0.06f))
                            {
                                gimbal.small_pit_towards_flag = false;
                            }
                        }
                        gimbal.target_val.small_pitch_angle = basic_math_constrain(gimbal.target_val.small_pitch_angle, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MIN_OFFSET, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MAX_OFFSET);
                    }

                    else
                    {
                        // 小yaw、小pitch保持当前位置
                        gimbal.target_val.small_yaw_angle = 0.0f;
                        gimbal.target_val.small_pitch_angle = 0.0f;
                    }

                    // 大yaw
                    gimbal.target_val.big_yaw_omega = BIG_YAW_SCAN_OMEGA;

                    // 大pitch
                    if (gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Up)
                    {
                        gimbal.target_val.big_pitch_angle = BIG_PITCH_MOTOR_TAG_ANGLE;
                    }
                    else if (gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Down)
                    {
                        gimbal.target_val.big_pitch_angle = BIG_PITCH_START_ANGLE;
                    }
                    else
                    {
                        gimbal.target_val.big_pitch_angle = gimbal.now_val.big_pitch_angle;
                    }
                }
            }

            else if (gimbal.now_mode == Gimbal_Auto_Follow)
            {
                if(gimbal.last_mode != Gimbal_Auto_Follow)
                {
                    gimbal_switch_clear_condition();
                }

                else
                {
                    if(gimbal.big_pitch_ready)
                    {
                        // 小yaw、小pitch跟随目标
                        gimbal.target_val.gimbal_yaw_angle = basic_math_loop_constrain(gimbal.input_fil_val.yaw_angle, -PI, PI);
                        gimbal.target_val.small_pitch_angle = basic_math_constrain(gimbal.input_fil_val.pitch_angle, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MIN_OFFSET, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MAX_OFFSET);

                        float delta = basic_math_loop_constrain(gimbal.target_val.gimbal_yaw_angle - gimbal.now_val.gimbal_yaw_angle, -PI, PI);
                        gimbal.target_val.small_yaw_angle = basic_math_constrain(gimbal.now_val.small_yaw_angle + delta, SMALL_YAW_MIN_OFFSET, SMALL_YAW_MAX_OFFSET);

                        // 大yaw
                        {
                            float small_yaw_offset = gimbal.small_yaw_motor.get_total_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
                            if(small_yaw_offset <= SMALL_YAW_MIN_OFFSET * 1.1f || small_yaw_offset >= SMALL_YAW_MAX_OFFSET * 1.1f)
                            {
                                // 大小yaw相对角异常
                                gimbal.target_val.big_yaw_angle = gimbal.now_val.big_yaw_angle;
                            }

                            else
                            {
                                float n = basic_math_abs(gimbal.now_val.big_yaw_angle) / SMALL_YAW_MAX_OFFSET;
                                gimbal.big_yaw_rho = basic_math_constrain(0.25f + 0.75f * n * n * n, 0.0f, 1.0f);

                                gimbal.target_val.big_yaw_angle = (1.0f - gimbal.big_yaw_rho) * gimbal.now_val.big_yaw_angle;
                            }   
                        }
                    }

                    else 
                    {
                        gimbal.now_val.big_yaw_angle = gimbal.big_yaw_motor.get_now_angle();
                        gimbal.target_val.gimbal_yaw_angle = basic_math_loop_constrain(gimbal.input_fil_val.yaw_angle, -PI, PI);
                        float delta = basic_math_loop_constrain(gimbal.target_val.gimbal_yaw_angle - gimbal.now_val.gimbal_yaw_angle, -PI, PI);
                        // 小yaw
                        gimbal.target_val.small_yaw_angle = 0.0f;
                        // 大yaw
                        gimbal.target_val.big_yaw_angle = gimbal.now_val.big_yaw_angle + delta;
                        // 小pitch
                        gimbal.target_val.small_pitch_angle = 0.0f;
                    }

                    // 大pitch
                    if (gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Up)
                    {
                        gimbal.target_val.big_pitch_angle = BIG_PITCH_MOTOR_TAG_ANGLE;
                    }
                    else if (gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Down)
                    {
                        gimbal.target_val.big_pitch_angle = BIG_PITCH_START_ANGLE;
                    }
                    else
                    {
                        gimbal.target_val.big_pitch_angle = gimbal.now_val.big_pitch_angle;
                    }
                }
            }

            // 自瞄未启动(无控制数据发送)，保持当前角度
            else
            {
                gimbal.target_val.small_yaw_angle = gimbal.now_val.small_yaw_angle;
                gimbal.target_val.small_pitch_angle = gimbal.now_val.small_pitch_angle;
                gimbal.target_val.big_yaw_angle = gimbal.now_val.big_yaw_angle;
                gimbal.target_val.big_pitch_angle = gimbal.now_val.big_pitch_angle;
                gimbal.target_val.big_yaw_omega = 0.0f;
            }
        }
    }
}

/**
 * @brief 更新云台前馈控制量
 * @note 任务周期存在几十~上百微秒级抖动, 逐拍 dt 会被 K*(Kp*e-rate) 放大成可见的加速度噪声
 *       (实测峰值可达 1.7rad/s²), 故实测 dt 由规划器内部低通(dt_filter_alpha)后再推进参考轨迹;
 *       这里只负责判定调度异常(长时间被抢占)并整拍跳过
 */
void gimbal_update_feedforward(void)
{
    /* 重力项只取决于实测姿态, 与规划器是否跳拍无关, 所以每拍都要刷新 */
    gravity_torque = pitch_ff.grav_sin * arm_sin_f32(gimbal.now_val.small_pitch_angle) +
                     pitch_ff.grav_cos * arm_cos_f32(gimbal.now_val.small_pitch_angle);

    // 首次调用规划标记
    const bool first_call = (gimbal.plan_time.small_yaw_us == 0u) || (gimbal.plan_time.small_pitch_us == 0u);

    // 实测周期(秒): 用于调度异常判定, 并经规划器内部低通后推进参考轨迹
    const float meas_yaw_s   = bsp_dwt_get_deltat32(&gimbal.plan_time.small_yaw_us);
    const float meas_pitch_s = bsp_dwt_get_deltat32(&gimbal.plan_time.small_pitch_us);

    // 首次调用时没有上一拍, 先把规划器对齐到当前角度, 下一拍再开始真正规划
    if (first_call)
    {
        reference_track_plan_reset(&gimbal.small_yaw_plan, gimbal.now_val.small_yaw_angle);
        reference_track_plan_reset(&gimbal.small_pitch_plan, gimbal.now_val.small_pitch_angle);
        // 规划输出同步对齐, 否则以规划角作角度环给定时会从 0 跳变
        gimbal.small_yaw_plan_output   = gimbal.small_yaw_plan.value;
        gimbal.small_pitch_plan_output = gimbal.small_pitch_plan.value;
        gimbal.dynamic_comp     = 0.0f;
        gimbal.all_comp         = basic_math_constrain(gravity_torque, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT);
        gimbal.plan_dt_meas_s   = 0.0f;
        gimbal.plan_skip_frames = 0;
        return;
    }

    const float meas_s = 0.5f * (meas_yaw_s + meas_pitch_s);
    gimbal.plan_dt_meas_s = meas_s;

    // 调度异常(被高优先级任务长时间抢占 / 同一毫秒内重入)时整拍跳过, 而不是硬塞一个 1ms:
    // 硬塞会让规划器时间基与真实时间脱节, 反而在恢复的那一拍引入跳变
    if (!isfinite(meas_s) ||
        meas_s <= gimbal.small_pitch_plan_config.dt_min_s ||
        meas_s >= gimbal.small_pitch_plan_config.dt_max_s)
    {
        if (gimbal.plan_skip_frames < 0xFFFFFFFFu) gimbal.plan_skip_frames++;
        // 本拍参考没有推进, 动态项沿用上一拍会给出与实际运动不符的力矩
        gimbal.dynamic_comp = 0.0f;
        gimbal.all_comp     = basic_math_constrain(gravity_torque, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT);
        return;
    }

    // 添加滤波处理算法
    gimbal.plan_filter.yaw_angle.set_now_input(gimbal.target_val.small_yaw_angle);
    gimbal.plan_filter.yaw_angle.low_calculate_callback();
    gimbal.plan_filter.pitch_angle.set_now_input(gimbal.target_val.small_pitch_angle);
    gimbal.plan_filter.pitch_angle.low_calculate_callback();

    gimbal.plan_fil_val.yaw_angle = gimbal.plan_filter.yaw_angle.get_out();
    gimbal.plan_fil_val.pitch_angle = gimbal.plan_filter.pitch_angle.get_out();

    // 更新数据(规划器内部对 dt 再做低通)
    reference_track_plan_update(&gimbal.small_yaw_plan, gimbal.target_val.small_yaw_angle, 0.0f, 0.0f, meas_s, &gimbal.small_yaw_plan_output);
    reference_track_plan_update(&gimbal.small_pitch_plan, gimbal.target_val.small_pitch_angle, 0.0f, 0.0f, meas_s, &gimbal.small_pitch_plan_output);

    // 更新前馈控制量
    gimbal.plan_filter.yaw_omega.set_now_input(gimbal.small_yaw_plan_output.rate_rad_s);
    gimbal.plan_filter.yaw_omega.low_calculate_callback();
    gimbal.plan_filter.yaw_torque.set_now_input(gimbal.small_yaw_plan_output.accel_rad_s2);
    gimbal.plan_filter.yaw_torque.low_calculate_callback();
    gimbal.plan_filter.pitch_omega.set_now_input(gimbal.small_pitch_plan_output.rate_rad_s);
    gimbal.plan_filter.pitch_omega.low_calculate_callback();
    gimbal.plan_filter.pitch_torque.set_now_input(gimbal.small_pitch_plan_output.accel_rad_s2);
    gimbal.plan_filter.pitch_torque.low_calculate_callback();

    gimbal.plan_fil_val.yaw_omega = gimbal.plan_filter.yaw_omega.get_out();
    gimbal.plan_fil_val.yaw_torque = gimbal.plan_filter.yaw_torque.get_out();
    gimbal.plan_fil_val.pitch_omega = gimbal.plan_filter.pitch_omega.get_out();
    gimbal.plan_fil_val.pitch_torque = gimbal.plan_filter.pitch_torque.get_out();

    // 小pitch轴前馈动态项: 惯量 + 阻尼 + 库仑摩擦, 与辨识模型 J*acc + B*rate + Fc*tanh(rate/eps) 同构
    const float rate  = gimbal.plan_fil_val.pitch_omega;     // 规划角速度(已低通)
    const float accel = gimbal.plan_fil_val.pitch_torque;    // 规划角加速度(已低通)

    // tanh 在 0 附近本身就是连续的, 不能再叠死区: 死区会在 ±deadband 处凭空产生 Fc*tanh(deadband/eps) 的力矩台阶
    const float coulomb = pitch_ff.fc * tanhf(rate / pitch_ff.eps);

    /* 4.1 动态项(随参考速率/加速度变化), 单独限幅 */
    gimbal.dynamic_comp = basic_math_constrain(pitch_ff.j * accel + pitch_ff.b * rate + coulomb,
                                               -pitch_ff.dynamic_max, pitch_ff.dynamic_max);

    // 总前馈: 动态项 + 重力项, 再按整轴力矩上限收口(两项各自限幅后相加仍可能超出轴上限)
    gimbal.all_comp = basic_math_constrain(gimbal.dynamic_comp + gravity_torque,
                                           -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT);
}

/**
 * @brief 计算云台电机的控制量
 */
void gimbal_motor_calculate(void)
{
    // 前馈依赖本拍目标值, 且角度环给定量取自规划输出, 必须先算
    gimbal_update_feedforward();

    if (gimbal.big_pitch_ready != gimbal.last_big_pitch_ready)
    {
        gimbal.last_big_pitch_ready = gimbal.big_pitch_ready;

        gimbal.target_val.big_yaw_angle = gimbal.now_val.big_yaw_angle;

        gimbal.rc_big_yaw_pid.angle.pid_clear();
        gimbal.rc_big_yaw_pid.speed.pid_clear();
        gimbal.pc_big_yaw_pid.angle.pid_clear();
        gimbal.pc_big_yaw_pid.speed.pid_clear();
    }

    if(gimbal.now_method == Gimbal_Method_Zero_Force)
    {
        gimbal.small_yaw_motor.set_control_torque(0.0f);
        gimbal.small_pitch_motor.set_control_torque(0.0f);
        gimbal.big_yaw_motor.set_control_torque(0.0f);
        gimbal.big_pitch_motor.set_control_torque(0.0f);
    }

    else if (gimbal.now_method == Gimbal_Method_Manual)
    {
        // 小yaw
        gimbal.rc_small_yaw_pid.angle.set_target(gimbal.target_val.small_yaw_angle);
        gimbal.rc_small_yaw_pid.angle.set_now(gimbal.now_val.small_yaw_angle);
        gimbal.rc_small_yaw_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_yaw_omega = gimbal.rc_small_yaw_pid.angle.get_out();

        gimbal.rc_small_yaw_pid.speed.set_target(gimbal.target_val.small_yaw_omega);
        gimbal.rc_small_yaw_pid.speed.set_now(gimbal.now_val.small_yaw_omega);
        gimbal.rc_small_yaw_pid.speed.pid_calculate_callback();
        gimbal.small_yaw_motor.set_control_torque(gimbal.rc_small_yaw_pid.speed.get_out());

        // 小pitch: 角度环跟踪规划输出(与前馈同源), 力矩 = 速度环 + 总前馈
        gimbal.rc_small_pitch_pid.angle.set_target(gimbal.target_val.small_pitch_angle);
        gimbal.rc_small_pitch_pid.angle.set_now(gimbal.now_val.small_pitch_angle);
        gimbal.rc_small_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_pitch_omega = gimbal.rc_small_pitch_pid.angle.get_out();

        gimbal.rc_small_pitch_pid.speed.set_target(gimbal.target_val.small_pitch_omega);
        gimbal.rc_small_pitch_pid.speed.set_now(gimbal.now_val.small_pitch_omega);
        gimbal.rc_small_pitch_pid.speed.pid_calculate_callback();
        gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.rc_small_pitch_pid.speed.get_out() + gravity_torque, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));

        // 大yaw
        gimbal.rc_big_yaw_pid.angle.set_target(gimbal.target_val.big_yaw_angle);
        gimbal.rc_big_yaw_pid.angle.set_now(gimbal.now_val.big_yaw_angle);
        gimbal.rc_big_yaw_pid.angle.pid_calculate_callback();
        gimbal.target_val.big_yaw_omega = gimbal.rc_big_yaw_pid.angle.get_out();

        gimbal.rc_big_yaw_pid.speed.set_target(gimbal.target_val.big_yaw_omega);
        gimbal.rc_big_yaw_pid.speed.set_now(gimbal.now_val.big_yaw_omega);
        gimbal.rc_big_yaw_pid.speed.pid_calculate_callback();
        gimbal.big_yaw_motor.set_control_torque(gimbal.rc_big_yaw_pid.speed.get_out());

        // 大pitch
        gimbal.rc_big_pitch_pid.angle.set_target(gimbal.target_val.big_pitch_angle);
        gimbal.rc_big_pitch_pid.angle.set_now(gimbal.now_val.big_pitch_angle);
        gimbal.rc_big_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.big_pitch_omega = gimbal.rc_big_pitch_pid.angle.get_out();

        gimbal.rc_big_pitch_pid.speed.set_target(gimbal.target_val.big_pitch_omega);
        gimbal.rc_big_pitch_pid.speed.set_now(gimbal.now_val.big_pitch_omega);
        gimbal.rc_big_pitch_pid.speed.pid_calculate_callback();
        gimbal.big_pitch_motor.set_control_torque(gimbal.rc_big_pitch_pid.speed.get_out());
    }

    else if (gimbal.now_method == Gimbal_Method_Auto)
    {
        // 小yaw
        gimbal.pc_small_yaw_pid.angle.set_target(gimbal.target_val.small_yaw_angle);
        gimbal.pc_small_yaw_pid.angle.set_now(gimbal.now_val.small_yaw_angle);
        gimbal.pc_small_yaw_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_yaw_omega = gimbal.pc_small_yaw_pid.angle.get_out();

        gimbal.pc_small_yaw_pid.speed.set_target(gimbal.target_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.set_now(gimbal.now_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.pid_calculate_callback();
        gimbal.small_yaw_motor.set_control_torque(gimbal.pc_small_yaw_pid.speed.get_out());

        // 小pitch: 角度环跟踪规划输出(与前馈同源), 力矩 = 速度环 + 总前馈
        gimbal.pc_small_pitch_pid.angle.set_target(gimbal.target_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.set_now(gimbal.now_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_pitch_omega = gimbal.pc_small_pitch_pid.angle.get_out();

        gimbal.pc_small_pitch_pid.speed.set_target(gimbal.target_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.set_now(gimbal.now_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.pid_calculate_callback();
        gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_pitch_pid.speed.get_out() + gravity_torque, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));

        // 大pitch
        gimbal.pc_big_pitch_pid.angle.set_target(gimbal.target_val.big_pitch_angle);
        gimbal.pc_big_pitch_pid.angle.set_now(gimbal.now_val.big_pitch_angle);
        gimbal.pc_big_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.big_pitch_omega = gimbal.pc_big_pitch_pid.angle.get_out();

        gimbal.pc_big_pitch_pid.speed.set_target(gimbal.target_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.set_now(gimbal.now_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.pid_calculate_callback();
        gimbal.big_pitch_motor.set_control_torque(gimbal.pc_big_pitch_pid.speed.get_out());

        if (gimbal.now_mode == Gimbal_Auto_Follow)
        {
            // 大yaw
            gimbal.pc_big_yaw_pid.angle.set_target(gimbal.target_val.big_yaw_angle);
            gimbal.pc_big_yaw_pid.angle.set_now(gimbal.now_val.big_yaw_angle);
            gimbal.pc_big_yaw_pid.angle.pid_calculate_callback();
            gimbal.target_val.big_yaw_omega = gimbal.pc_big_yaw_pid.angle.get_out();
        }

        // 大yaw速度指令斜坡
        {
            gimbal.big_yaw_omega_slope.set_now_real(gimbal.now_val.big_yaw_omega);
            gimbal.big_yaw_omega_slope.set_target(gimbal.target_val.big_yaw_omega);
            gimbal.big_yaw_omega_slope.slope_calculate_callback();
            gimbal.target_val.big_yaw_omega = gimbal.big_yaw_omega_slope.get_out();
        }

        gimbal.pc_big_yaw_pid.speed.set_target(gimbal.target_val.big_yaw_omega);
        gimbal.pc_big_yaw_pid.speed.set_now(gimbal.now_val.big_yaw_omega);
        gimbal.pc_big_yaw_pid.speed.pid_calculate_callback();
        gimbal.big_yaw_motor.set_control_torque(gimbal.pc_big_yaw_pid.speed.get_out());
    }

    gimbal.small_yaw_motor.calculate_callback();
    gimbal.small_pitch_motor.calculate_callback();
    gimbal.big_yaw_motor.calculate_callback();
    gimbal.big_pitch_motor.calculate_callback();

    gimbal.last_method = gimbal.now_method;
    gimbal.last_mode = gimbal.now_mode;

    gimbal.last_big_pitch_ready = gimbal.big_pitch_ready;
}

/**
 * @brief 更新云台系统辨识参数
 */
void gimbal_identification_update(void)
{
    const uint32_t time_us = bsp_dwt_get_timeline_us();
    const bool ident_enable = gimbal.now_method != Gimbal_Method_Zero_Force;

    const float pitch_feedback_s = gimbal.timestamp.small_pitch_us == 0u ? 1.0f : (float)(time_us - gimbal.timestamp.small_pitch_us) * 0.000001f;
    const float yaw_feedback_s = gimbal.timestamp.small_yaw_us == 0u ? 1.0f : (float)(time_us - gimbal.timestamp.small_yaw_us) * 0.000001f;

    // control_ready 在本工程中从未被置位，若继续作为门限会让辨识永远无效；有效性只看本轴电机状态与反馈新鲜度
    const bool pitch_valid = (gimbal.small_pitch_motor.get_status() == Motor_DM_Status_Enable) && (pitch_feedback_s < 0.005f);
    const bool yaw_valid = (gimbal.small_yaw_motor.get_status() == Motor_DM_Status_Enable) && (yaw_feedback_s < 0.005f);

    // pitch电机系统辨识
    gimbal_identification_sample_fill(&gimbal.ident_small_pitch_sample, time_us, pitch_feedback_s, 0u, GIMBAL_IDENTIFICATION_PITCH, 
                                      gimbal.small_pitch_motor.get_total_angle() - SMALL_PITCH_MOTOR_MID_ANGLE, gimbal.small_pitch_motor.get_now_omega(), gimbal.small_pitch_motor.get_now_torque(),
                                      gimbal.now_val.small_pitch_angle, SMALL_PITCH_TORQUE_LIMIT, ident_enable, pitch_valid);

    (void)gimbal_identification_update(&gimbal.ident_small_pitch, &gimbal.ident_small_pitch_sample, &gimbal.ident_small_pitch_output);

    // yaw电机系统辨识
    gimbal_identification_sample_fill(&gimbal.ident_small_yaw_sample, time_us, yaw_feedback_s, 0u, GIMBAL_IDENTIFICATION_YAW, 
                                      gimbal.small_yaw_motor.get_total_angle() - SMALL_YAW_MOTOR_MID_ANGLE, gimbal.small_yaw_motor.get_now_omega(), gimbal.small_yaw_motor.get_now_torque(),
                                      gimbal.now_val.small_yaw_angle, SMALL_YAW_TORQUE_LIMIT, ident_enable, yaw_valid);
                                      
    (void)gimbal_identification_update(&gimbal.ident_small_yaw, &gimbal.ident_small_yaw_sample, &gimbal.ident_small_yaw_output);
}

/**
 * @brief 电机和调试测试函数
 */
void gimbal_motor_control_test(void)
{
    wave_generator_calc(&test_wave);
    // 单电机调试

    // 目标值设置
    // const float target_val = basic_math_abs(wave_generator_get_value(&test_wave));
    // const float now_small_yaw = gimbal.small_yaw_motor.get_total_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
    const float now_small_pitch = gimbal.small_pitch_motor.get_total_angle() - SMALL_PITCH_MOTOR_MID_ANGLE;

    // 小yaw目标值设置
    if(now_small_pitch > SMALL_PITCH_MAX_OFFSET - 0.1f)
    {
        // gimbal.target_val.small_pitch_omega = -target_val;
        // gimbal.target_val.small_yaw_omega = -debug_data;
        // gimbal.target_val.small_pitch_omega = -debug_data;
    }

    else if(now_small_pitch < SMALL_PITCH_MIN_OFFSET + 0.08f)
    {
        // gimbal.target_val.small_pitch_omega = target_val;
        // gimbal.target_val.small_yaw_omega = debug_data;
        // gimbal.target_val.small_pitch_omega = debug_data;
    }

    // gimbal.target_val.small_pitch_angle = basic_math_constrain(wave_generator_get_value(&test_wave), gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MIN_OFFSET + 0.1f, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MAX_OFFSET - 0.2f);
    gimbal.target_val.small_pitch_angle = 0.0f;

    // 轨迹规划 + 前馈(重力/动态补偿在函数内部算好)
    gimbal_update_feedforward();

    // 速度环
    if(gimbal.now_method == Gimbal_Method_Zero_Force)
    {
        gimbal.small_yaw_motor.set_control_torque(0.0f);
        gimbal.small_pitch_motor.set_control_torque(0.0f);
        gimbal.big_yaw_motor.set_control_torque(0.0f);
        gimbal.big_pitch_motor.set_control_torque(0.0f);
    }


    else if (gimbal.now_method == Gimbal_Method_Manual) // 直接调试pc模式的pid
    {
        // 小yaw
        // gimbal.pc_small_yaw_pid.angle.set_target(gimbal.target_val.small_yaw_angle);
        // gimbal.pc_small_yaw_pid.angle.set_now(gimbal.now_val.small_yaw_angle);
        // gimbal.pc_small_yaw_pid.angle.pid_calculate_callback();
        // gimbal.target_val.small_yaw_omega = gimbal.pc_small_yaw_pid.angle.get_out();

        gimbal.pc_small_yaw_pid.speed.set_target(gimbal.target_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.set_now(gimbal.now_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.pid_calculate_callback();
        gimbal.small_yaw_motor.set_control_torque(gimbal.pc_small_yaw_pid.speed.get_out());

        // 小pitch: 角度环跟踪规划输出(与前馈同源), 力矩 = 速度环 + 总前馈
        gimbal.pc_small_pitch_pid.angle.set_target(gimbal.target_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.set_now(gimbal.now_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_pitch_omega = gimbal.pc_small_pitch_pid.angle.get_out();

        gimbal.pc_small_pitch_pid.speed.set_target(gimbal.target_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.set_now(gimbal.now_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.pid_calculate_callback();
        // gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_pitch_pid.speed.get_out() + gravity_torque, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));
        gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_pitch_pid.speed.get_out() + gimbal.all_comp, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));

        // 大yaw
        // gimbal.pc_big_yaw_pid.angle.set_target(gimbal.target_val.big_yaw_angle);
        // gimbal.pc_big_yaw_pid.angle.set_now(gimbal.now_val.big_yaw_angle);
        // gimbal.pc_big_yaw_pid.angle.pid_calculate_callback();
        // gimbal.target_val.big_yaw_omega = gimbal.pc_big_yaw_pid.angle.get_out();

        gimbal.pc_big_yaw_pid.speed.set_target(gimbal.target_val.big_yaw_omega);
        gimbal.pc_big_yaw_pid.speed.set_now(gimbal.now_val.big_yaw_omega);
        gimbal.pc_big_yaw_pid.speed.pid_calculate_callback();
        gimbal.big_yaw_motor.set_control_torque(gimbal.pc_big_yaw_pid.speed.get_out());

        // 大pitch
        // gimbal.pc_big_pitch_pid.angle.set_target(gimbal.target_val.big_pitch_angle);
        // gimbal.pc_big_pitch_pid.angle.set_now(gimbal.now_val.big_pitch_angle);
        // gimbal.pc_big_pitch_pid.angle.pid_calculate_callback();
        // gimbal.target_val.big_pitch_omega = gimbal.pc_big_pitch_pid.angle.get_out();

        gimbal.pc_big_pitch_pid.speed.set_target(gimbal.target_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.set_now(gimbal.now_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.pid_calculate_callback();
        gimbal.big_pitch_motor.set_control_torque(gimbal.pc_big_pitch_pid.speed.get_out());
    }


    // else if (gimbal.now_method == Gimbal_Method_Manual) // 直接调试pc模式的pid
    // {
    //     gimbal.small_pitch_motor.set_control_torque(gravity_torque);
    //     // gimbal.small_pitch_motor.set_control_torque(small_pitch_gravity);
    // }

    // gimbal.small_yaw_motor.calculate_callback();
    gimbal.small_pitch_motor.calculate_callback();
    // gimbal.big_yaw_motor.calculate_callback();
    // gimbal.big_pitch_motor.calculate_callback();

    gimbal.last_method = gimbal.now_method;
    gimbal.last_mode = gimbal.now_mode;
    gimbal.last_big_pitch_ready = gimbal.big_pitch_ready;
}

extern "C" void gimbal_task(void *argument)
{
    // 初始化云台
    gimbal_init();
    osDelay(1000);

    for(;;)
    {
        // 更新云台模式
        gimbal_update_mode();

        // 更新测量值
        gimbal_update_measure();

        // 更新滤波器和目标观测值
        gimbal_update_observate();

        // // 更新目标值更新决策数据
        // gimbal_control_decision();

        // // 计算电机控制量
        // gimbal_motor_calculate();

        // 电机辨识
        gimbal_identification_update();

        // 调试程序
        gimbal_motor_control_test();

        // 延时
        osDelay(1);
    }
}
