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
#include "dsp/fast_math_functions.h"
#include "gimbal_identification.h"
#include "pid.h"
#include "robot_def.h"
#include "safe_daemon_process.h"
#include "wave_generator.h"
#include "bsp_dwt.h"
#include "feedforward.h"

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
float debug_data = 0.0f;
float test_torque = 0.0f;

/**
 * @brief 小pitch轴前馈参数
 * @note 故意不加 const: 便于在 Ozone 中直接改写做参数迭代, 重新上电恢复下表的初值
 *       模型与辨识同构: tau = J*acc + B*rate + Fc*tanh(rate/eps) + A*sin(q) + C*cos(q)
 *       计算公式见 feedforward_calculate(), 小pitch 与大pitch 共用同一套实现。
 */
feedforward_param_t pitch_ff = {
    0.0040149251f,      // j: RLS 在线辨识值 (原值 0.0109 是辨识初值, 比在线估计大 48 倍)
    0.0162701607f,        // b: RLS 在线辨识值, 协方差偏大, 需高加速度激励后复核
    0.152315259f,          // fc: 实测滞回半宽 ≈0.13 N·m; 原值 0.553 会让摩擦前馈过补偿约 4 倍
    0.08f,          // eps: 与辨识配置 friction_velocity_rad_s 保持一致
    -0.0614492036f,  // grav_sin: 静态实测拟合 A≈-0.03~-0.08, 原值保留待复核
    0.995969772f,    // grav_cos: 静态实测拟合 C≈1.00~1.05
    0.5f,           // dynamic_max: ≈J*max_accel + B*max_rate + Fc, 须远小于 SMALL_PITCH_TORQUE_LIMIT
    FEEDFORWARD_DEFAULT_EPS_MIN_RAD_S // eps_min_rad_s: tanh 速度尺度下限
};

/**
 * @brief 小yaw轴系统辨识前馈参数
 * @note 小yaw辨识模型不包含重力项；完成辨识后将 J/B/Fc 填入本表。
 */
feedforward_param_t small_yaw_ff = {
    0.001f,                     // j: RLS 在线辨识值
    0.0761054382f,                       // b: RLS 在线辨识值
    0.08f,                         // fc: 低速噪声下的保守摩擦补偿, 避免过零时力矩跳变
    0.20f,                         // eps: 增大低速 tanh 平滑区, 抑制速率过零抖动
    0.0f,                          // grav_sin: 小yaw无重力项
    0.0f,                          // grav_cos: 小yaw无重力项
    0.65f,                          // dynamic_max: 动态前馈限幅，须小于 SMALL_YAW_TORQUE_LIMIT
    FEEDFORWARD_DEFAULT_EPS_MIN_RAD_S // eps_min_rad_s: tanh 速度尺度下限
};

/**
 * @brief 大pitch轴重力前馈参数
 * @note 仅使用重力项, 默认全 0
 *       辨识完成前前馈恒为 0, 不改变任何现有控制行为。
 *       辨识流程: 跑速度扫掠 → 读 gimbal.ident_big_pitch_output.model → 填回本表
 */
feedforward_param_t big_pitch_ff = {
    0.0f,                           // j: 不使用动态前馈
    0.0f,                           // b: 不使用动态前馈
    0.0f,                           // fc: 不使用动态前馈
    0.08f,                          // eps: 与辨识配置 friction_velocity_rad_s 保持一致
    0.0f,                           // grav_sin: 待辨识
    0.0f,                           // grav_cos: 待辨识
    0.0f,                           // dynamic_max: 禁用动态前馈
    FEEDFORWARD_DEFAULT_EPS_MIN_RAD_S // eps_min_rad_s: tanh 速度尺度下限
};

// 大pitch下降时小轴回中目标的单周期最大变化量，避免目标角度突变造成冲击。
#define GIMBAL_SMALL_PITCH_RETURN_STEP_RAD 0.0051f

/**
 * @brief 小yaw前馈参考调理配置
 * @note 本表是初始化参数; Ozone 中可直接改写该表, 重新调用初始化后生效。
 *       运行中调参请改写 small_yaw_ff_tracker.config, 下一拍生效。
 */
feedforward_tracker_config_t small_yaw_ff_tracker_config = {
    3.0f,                                 // max_rate_rad_s
    8.0f,                                 // max_accel_rad_s2
    0.2f,                                 // rate_filter_alpha
    0.08f,                                // accel_filter_alpha
    20.0f,                                // accel_jerk_limit_rad_s3
    FEEDFORWARD_DEFAULT_REF_DT_MAX_S,     // ref_dt_max_s
    FEEDFORWARD_DEFAULT_FILTER_BASE_DT_S  // filter_base_dt_s
};

/**
 * @brief 小pitch前馈参考调理配置
 * @note 本表是初始化参数; Ozone 中可直接改写该表, 重新调用初始化后生效。
 *       运行中调参请改写 gimbal.small_pitch_ff_tracker.config, 下一拍生效。
 */
feedforward_tracker_config_t small_pitch_ff_tracker_config = {
    3.0f,                                 // max_rate_rad_s
    8.0f,                                 // max_accel_rad_s2
    0.2f,                                 // rate_filter_alpha
    0.08f,                                // accel_filter_alpha
    20.0f,                                // accel_jerk_limit_rad_s3
    FEEDFORWARD_DEFAULT_REF_DT_MAX_S,     // ref_dt_max_s
    FEEDFORWARD_DEFAULT_FILTER_BASE_DT_S  // filter_base_dt_s
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

/*------------------------------------云台控制辅助函数-------------------------------------------*/

/**
 * @brief 限制目标角度的变化量
 */
static float gimbal_limit_target_step(float current, float target, float max_step)
{
    const float delta = target - current;
    return current + basic_math_constrain(delta, -max_step, max_step);
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
    config->window_s = 0.05f;                 // 50 ms：仿真验证过 20/30/50/80/120/200 ms，50 ms 精度最优
    config->max_gap_s = 0.05f;                // 50 ms：仿真表明 35 ms 的偶发抖动若超限会反复丢弃未完成窗口
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区

    config->accept_saturated = true;          // 饱和样本仍参与回归：指令限幅时反馈的力矩就是限幅后的真实值，仿真验证饱和数据可用
    config->max_relative_uncertainty = 5.0f;  // 可信门限 = sqrt(P_ii)/|theta_i|。仿真标定: 良好激励 0.7~1.7，不足激励 50+。5.0 可放行 ±0.20 rad 以上的激励，拦住 ±0.10 rad 以下。

    rls_last_default_config(&config->rls, 3u);

    config->rls.initial[0] = 0.0187080242f;          // J 起步量：仅用于 RLS 初值，不是实测标定值
    config->rls.initial[1] = 0.168810293f;         // B 起步量：保持与小型云台输出轴量级同阶
    config->rls.initial[2] = 0.0995386392f;          // Fc 起步量：给低速摩擦项一个非零可学习初值

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
    config->window_s = 0.05f;                 // 50 ms：仿真验证过 20/30/50/80/120/200 ms，50 ms 精度最优
    config->max_gap_s = 0.05f;                // 50 ms：仿真表明 35 ms 的偶发抖动若超限会反复丢弃未完成窗口
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区

    config->accept_saturated = true;          // 饱和样本仍参与回归：指令限幅时反馈的力矩就是限幅后的真实值，仿真验证饱和数据可用
    config->max_relative_uncertainty = 5.0f;  // 可信门限 = sqrt(P_ii)/|theta_i|。仿真标定: 良好激励 0.7~1.7，不足激励 50+。5.0 可放行 ±0.20 rad 以上的激励，拦住 ±0.10 rad 以下。

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

/**
 * @brief 云台大pitch轴系统辨识初始化函数
 * @note 与 small_pitch 同构(绕水平轴, 受重力), 但量级完全不同:
 *       实测惯量 J≈0.007 kg·m^2、重力项幅值≈0.6 N·m、扫掠速度 ±0.6 rad/s。
 *       辨识结果直接填入 big_pitch_ff, 作为速度环的前馈。
 */
static void big_pitch_identification_config_init(gimbal_identification_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));

    config->axis = GIMBAL_IDENTIFICATION_BIG_PITCH;
    config->configuration_id = GIMBAL_IDENTIFICATION_BIG_PITCH;   // 必须与样本里的 configuration_id 一致，否则每帧都会被判为配置不匹配
    config->window_s = 0.05f;                 // 50 ms：与小pitch一致, 兼顾单帧噪声与激励频率
    config->max_gap_s = 0.05f;                // 50 ms：容忍偶发调度抖动, 不跨越明显丢帧
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区
    config->accept_saturated = true;          // 饱和样本仍参与回归：指令限幅时反馈的力矩就是限幅后的真实值，仿真验证饱和数据可用
    config->derive_rate_from_angle = true;    // 大pitch 速度环不可靠, 由角度环微分得到速度, 仅用于辨识
    config->rate_window_samples = 20u;        // 20 个采样点的滑动均值窗口, 约 20~30 ms, 用于平滑微分噪声

    config->max_relative_uncertainty = 5.0f;  // 可信门限 = sqrt(P_ii)/|theta_i|。仿真标定: 良好激励 0.7~1.7，不足激励 50+。5.0 可放行 ±0.20 rad 以上的激励，拦住 ±0.10 rad 以下。

    rls_last_default_config(&config->rls, 5u);

    config->rls.initial[0] = 0.0072f;         // J: 由换向瞬态反解的初值(0.005~0.008), 仅作起点
    config->rls.initial[1] = 0.12f;           // B: 由扫掠稳态力矩反解的初值
    config->rls.initial[2] = 0.10f;           // Fc: 起步给一个非零可学习值
    config->rls.initial[3] = 0.0f;            // sin 重力项无先验方向，使用零中心
    config->rls.initial[4] = 0.0f;            // cos 重力项同上, 由扫掠数据自行学习

    config->rls.minimum[0] = 0.000001f;       // J 必须为正，禁止 RLS 退化成非物理负惯量
    config->rls.minimum[1] = 0.0f;
    config->rls.minimum[2] = 0.0f;
    config->rls.minimum[3] = -5.0f;
    config->rls.minimum[4] = -5.0f;

    config->rls.maximum[0] = 1.0f;            // 大pitch 输出轴等效惯量上界(实测 0.007, 留两个数量级)
    config->rls.maximum[1] = 5.0f;            // B 上界避免噪声把阻尼推向异常大值
    config->rls.maximum[2] = 5.0f;            // Fc 上界与最大反馈扭矩同量级
    config->rls.maximum[3] = 5.0f;
    config->rls.maximum[4] = 5.0f;

    config->rls.feature_scale[0] = 10.0f;     // 角加速度通常最大，缩放改善 J 列条件数

    config->rls.dt_min_s = 0.000001f;
    config->rls.dt_max_s = 0.1f;
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

    gimbal.observate_filter.small_yaw_omega.init(50.0f, 0.001f);
    gimbal.observate_filter.small_pitch_omega.init(50.0f, 0.001f);
    gimbal.observate_filter.big_yaw_omega.init(45.0f, 0.001f);
    gimbal.observate_filter.big_pit_omega.init(45.0f, 0.001f);

    // 大pitch角度目标值低通滤波器
    gimbal.big_pit_ang_fil.init(0.82f, 0.001f);

    // 初始化遥控器PID控制器
    gimbal.rc_small_yaw_pid.angle.init(12.5f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    gimbal.rc_small_yaw_pid.speed.init(0.68f, 0.0f, 0.0f, 0.0f, 0.85f, 3.5f, 0.001f, 0.0101f, 0.12f, 0.35f, 0.45f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    gimbal.rc_small_pitch_pid.angle.init(12.85f, 0.0f, 0.05f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    gimbal.rc_small_pitch_pid.speed.init(0.52f, 0.0f, 0.0f, 0.0f, 0.65f, 10.0f, 0.001f, 0.0f, 0.45f, 0.82f, 0.88f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    gimbal.rc_big_yaw_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    gimbal.rc_big_yaw_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    gimbal.rc_big_pitch_pid.angle.init(11.25f, 1.2f, 0.12f, 0.0f, 1.2f, 10.0f, 0.001f, 0.0f, 0.82f, 1.45f, 1.98f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    gimbal.rc_big_pitch_pid.speed.init(0.65f, 1.65f, 0.0f, 0.0f, 1.2f, 5.0f, 0.001f, 0.0f, 0.58f, 1.52f, 1.8f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    // 初始化云台自瞄PID控制器
    gimbal.pc_small_yaw_pid.angle.init(12.5f, 0.0f, 0.54f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    gimbal.pc_small_yaw_pid.speed.init(0.72f, 0.25f, 0.0f, 0.0f, 0.85f, 3.5f, 0.001f, 0.0101f, 0.12f, 0.35f, 0.45f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    gimbal.pc_small_pitch_pid.angle.init(10.85f, 0.0f, 0.05f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    // gimbal.pc_small_pitch_pid.speed.init(0.65f, 0.23f, 0.0f, 0.0f, 0.65f, 10.0f, 0.001f, 0.0f, 0.45f, 0.82f, 0.86f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_small_pitch_pid.speed.init(0.35f, 0.58f, 0.0f, 0.0f, 0.65f, 10.0f, 0.001f, 0.0f, 0.45f, 0.82f, 0.88f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    gimbal.pc_big_yaw_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    gimbal.pc_big_yaw_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    gimbal.pc_big_pitch_pid.angle.init(11.25f, 1.2f, 0.12f, 0.0f, 1.2f, 10.0f, 0.001f, 0.0f, 0.82f, 1.45f, 1.98f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);
    gimbal.pc_big_pitch_pid.speed.init(0.65f, 1.65f, 0.0f, 0.0f, 1.2f, 5.0f, 0.001f, 0.0f, 0.58f, 1.52f, 1.8f, PID_D_First_ENABLE, PID_Anti_Windup_CLAMP);

    gimbal.big_yaw_omega_slope.init(0.001f, 0.001f, Slope_First_REAL);
    

    // 控制器前馈量
    feedforward_tracker_init(&gimbal.small_yaw_ff_tracker, &small_yaw_ff_tracker_config);
    feedforward_tracker_init(&gimbal.small_pitch_ff_tracker, &small_pitch_ff_tracker_config);
    gimbal.big_pit_gravity_comp = 0.0f;

    // 初始化云台系统辨识参数
    small_yaw_identification_config_init(&gimbal.ident_small_yaw_config);
    gimbal_identification_init(&gimbal.ident_small_yaw, &gimbal.ident_small_yaw_config);
    small_pitch_identification_config_init(&gimbal.ident_small_pitch_config);
    gimbal_identification_init(&gimbal.ident_small_pitch, &gimbal.ident_small_pitch_config);

    big_pitch_identification_config_init(&gimbal.ident_big_pitch_config);
    gimbal_identification_init(&gimbal.ident_big_pitch, &gimbal.ident_big_pitch_config);

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
    wave_generator_init(&test_wave, WAVE_SINE, 0.35f, 0.3f, 0.0f);
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

    if(gimbal.crsf->get_left_lock() == CRSF_SWITCH_FREE)
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

    if(gimbal.now_val.big_pitch_angle - BIG_PITCH_START_ANGLE >= 0.0f)
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

    gimbal.observate_filter.small_yaw_omega.set_now_input(gimbal.now_val.small_yaw_omega);
    gimbal.observate_filter.small_pitch_omega.set_now_input(gimbal.now_val.small_pitch_omega);
    gimbal.observate_filter.big_yaw_omega.set_now_input(gimbal.now_val.big_yaw_omega);
    gimbal.observate_filter.big_pit_omega.set_now_input(gimbal.now_val.big_pitch_omega);

    gimbal.observate_filter.small_yaw_omega.low_calculate_callback();
    gimbal.observate_filter.small_pitch_omega.low_calculate_callback();
    gimbal.observate_filter.big_yaw_omega.low_calculate_callback();
    gimbal.observate_filter.big_pit_omega.low_calculate_callback();


    gimbal.now_fil_val.small_yaw_omega = gimbal.observate_filter.small_yaw_omega.get_out();
    gimbal.now_fil_val.small_pitch_omega = gimbal.observate_filter.small_pitch_omega.get_out();
    gimbal.now_fil_val.big_yaw_omega = gimbal.observate_filter.big_yaw_omega.get_out();
    gimbal.now_fil_val.big_pitch_omega = gimbal.observate_filter.big_pit_omega.get_out();


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

    // 前馈参考历史属于上一模式的目标轨迹, 切换后必须清掉, 下一拍重新对齐
    feedforward_tracker_reset(&gimbal.small_yaw_ff_tracker);
    feedforward_tracker_reset(&gimbal.small_pitch_ff_tracker);
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
                float yaw_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_y(), -1.0f, 1.0f, 0.065f) * SMALL_YAW_ANGLE_INCREMENT_RATION;
                float pitch_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_x(), -1.0f, 1.0f, 0.065f) * SMALL_PITCH_ANGLE_INCREMENT_RATION;

                gimbal.target_val.gimbal_yaw_angle = basic_math_loop_constrain(gimbal.target_val.gimbal_yaw_angle + yaw_angle_increment, -PI, PI);

                // 小yaw
                {
                    float delta = basic_math_constrain(gimbal.target_val.gimbal_yaw_angle - gimbal.now_val.gimbal_yaw_angle, -SMALL_YAW_MAX_INCREASE_ANGLE, SMALL_YAW_MAX_INCREASE_ANGLE);
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

            if(gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Down)
            {
                gimbal.target_val.small_yaw_angle = 0.0f;
                gimbal.target_val.small_pitch_angle = gimbal_limit_target_step(gimbal.target_val.small_pitch_angle, 0.0f, GIMBAL_SMALL_PITCH_RETURN_STEP_RAD);
                gimbal.target_val.gimbal_yaw_angle = gimbal.now_val.gimbal_yaw_angle;
            }

            // 大pitch
            if ((gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Down) && (basic_math_abs(gimbal.now_val.small_yaw_angle) < SMALL_YAW_DOWN_MAX_ANGLE) && (basic_math_abs(gimbal.now_val.small_pitch_angle) < 0.045f))
            {
                gimbal.target_val.big_pitch_angle = BIG_PITCH_START_ANGLE;
            }

            else if (gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Up)
            {
                gimbal.target_val.big_pitch_angle = BIG_PITCH_MOTOR_TAG_ANGLE;
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

    gimbal.big_pit_ang_fil.set_now_input(gimbal.target_val.big_pitch_angle);
    gimbal.big_pit_ang_fil.low_calculate_callback();
}

/**
 * @brief 更新云台前馈控制量
 * @note 目标角速度/角加速度由外部写入 gimbal.ffd_val.small_yaw_omega/small_pitch_omega
 *       和 small_yaw_accel/small_pitch_accel,
 *       跟踪器只做限幅、低通与 jerk 限制，不再对目标角差分。
 */
void gimbal_update_feedforward(void)
{
    // 未收到反馈时 get_total_angle() 为默认零值，减去机械零点后不是有效姿态。
    if (gimbal.timestamp.small_yaw_us == 0u || gimbal.timestamp.small_pitch_us == 0u)
    {
        feedforward_tracker_reset(&gimbal.small_yaw_ff_tracker);
        feedforward_tracker_reset(&gimbal.small_pitch_ff_tracker);
        gimbal.pitch_ff_out = {};
        gimbal.small_yaw_ff_out = {};
    }
}

/**
 * @brief 计算云台电机的控制量
 */
void gimbal_motor_calculate(void)
{
    static uint32_t last_update_us = 0u;
    const uint32_t now_us = (uint32_t)bsp_dwt_get_timeline_us();
    const float dt_s = (last_update_us == 0u) ? 0.0f : (float)(uint32_t)(now_us - last_update_us) * 0.000001f;
    last_update_us = now_us;

    // 前馈依赖本拍目标值, 必须先算
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

        feedforward_tracker_update(&gimbal.small_yaw_ff_tracker, gimbal.target_val.small_yaw_omega,
                                   (isfinite(gimbal.ffd_val.small_yaw_accel) ? gimbal.ffd_val.small_yaw_accel : 0.0f), dt_s);
        feedforward_calculate(&small_yaw_ff, gimbal.now_val.small_yaw_angle,
                              gimbal.small_yaw_ff_tracker.rate_rad_s, gimbal.small_yaw_ff_tracker.accel_rad_s2,
                              SMALL_YAW_TORQUE_LIMIT, &gimbal.small_yaw_ff_out);

        gimbal.rc_small_yaw_pid.speed.set_target(gimbal.target_val.small_yaw_omega);
        gimbal.rc_small_yaw_pid.speed.set_now(gimbal.now_val.small_yaw_omega);
        gimbal.rc_small_yaw_pid.speed.pid_calculate_callback();
        // gimbal.small_yaw_motor.set_control_torque(basic_math_constrain(gimbal.rc_small_yaw_pid.speed.get_out(), -SMALL_YAW_TORQUE_LIMIT, SMALL_YAW_TORQUE_LIMIT));
        gimbal.small_yaw_motor.set_control_torque(basic_math_constrain(gimbal.rc_small_yaw_pid.speed.get_out() + gimbal.small_yaw_ff_out.total_nm, -SMALL_YAW_TORQUE_LIMIT, SMALL_YAW_TORQUE_LIMIT));

        // 小pitch
        gimbal.rc_small_pitch_pid.angle.set_target(gimbal.target_val.small_pitch_angle);
        gimbal.rc_small_pitch_pid.angle.set_now(gimbal.now_val.small_pitch_angle);
        gimbal.rc_small_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_pitch_omega = gimbal.rc_small_pitch_pid.angle.get_out();

        feedforward_tracker_update(&gimbal.small_pitch_ff_tracker, gimbal.target_val.small_pitch_omega,
                                   (isfinite(gimbal.ffd_val.small_pitch_accel) ? gimbal.ffd_val.small_pitch_accel : 0.0f), dt_s);
        feedforward_calculate(&pitch_ff, gimbal.now_val.small_pitch_angle,
                              gimbal.small_pitch_ff_tracker.rate_rad_s, gimbal.small_pitch_ff_tracker.accel_rad_s2,
                              SMALL_PITCH_TORQUE_LIMIT, &gimbal.pitch_ff_out);

        gimbal.rc_small_pitch_pid.speed.set_target(gimbal.target_val.small_pitch_omega);
        gimbal.rc_small_pitch_pid.speed.set_now(gimbal.now_val.small_pitch_omega);
        gimbal.rc_small_pitch_pid.speed.pid_calculate_callback();
        gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.rc_small_pitch_pid.speed.get_out() + gimbal.pitch_ff_out.total_nm, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));

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
        gimbal.rc_big_pitch_pid.angle.set_target(basic_math_constrain(gimbal.big_pit_ang_fil.get_out(), BIG_PITCH_MOTOR_TAG_ANGLE, BIG_PITCH_START_ANGLE));
        gimbal.rc_big_pitch_pid.angle.set_now(gimbal.now_val.big_pitch_angle);
        gimbal.rc_big_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.big_pitch_omega = gimbal.rc_big_pitch_pid.angle.get_out();

        gimbal.rc_big_pitch_pid.speed.set_target(gimbal.target_val.big_pitch_omega);
        gimbal.rc_big_pitch_pid.speed.set_now(gimbal.now_val.big_pitch_omega);
        gimbal.rc_big_pitch_pid.speed.pid_calculate_callback();
        gimbal.big_pitch_motor.set_control_torque(basic_math_constrain(gimbal.rc_big_pitch_pid.speed.get_out() + gimbal.big_pit_gravity_comp, -BIG_PITCH_TORQUE_LIMIT, BIG_PITCH_TORQUE_LIMIT));
    }

    else if (gimbal.now_method == Gimbal_Method_Auto)
    {
        // 小yaw
        gimbal.pc_small_yaw_pid.angle.set_target(gimbal.target_val.small_yaw_angle);
        gimbal.pc_small_yaw_pid.angle.set_now(gimbal.now_val.small_yaw_angle);
        gimbal.pc_small_yaw_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_yaw_omega = gimbal.pc_small_yaw_pid.angle.get_out();

        feedforward_tracker_update(&gimbal.small_yaw_ff_tracker, gimbal.target_val.small_yaw_omega,
                                   (isfinite(gimbal.ffd_val.small_yaw_accel) ? gimbal.ffd_val.small_yaw_accel : 0.0f), dt_s);
        feedforward_calculate(&small_yaw_ff, gimbal.now_val.small_yaw_angle,
                              gimbal.small_yaw_ff_tracker.rate_rad_s, gimbal.small_yaw_ff_tracker.accel_rad_s2,
                              SMALL_YAW_TORQUE_LIMIT, &gimbal.small_yaw_ff_out);

        gimbal.pc_small_yaw_pid.speed.set_target(gimbal.target_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.set_now(gimbal.now_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.pid_calculate_callback();
        gimbal.small_yaw_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_yaw_pid.speed.get_out() + gimbal.small_yaw_ff_out.total_nm, -SMALL_YAW_TORQUE_LIMIT, SMALL_YAW_TORQUE_LIMIT));

        // 小pitch: 力矩 = 速度环 + 总前馈
        gimbal.pc_small_pitch_pid.angle.set_target(gimbal.target_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.set_now(gimbal.now_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_pitch_omega = gimbal.pc_small_pitch_pid.angle.get_out();

        feedforward_tracker_update(&gimbal.small_pitch_ff_tracker, gimbal.target_val.small_pitch_omega,
                                   (isfinite(gimbal.ffd_val.small_pitch_accel) ? gimbal.ffd_val.small_pitch_accel : 0.0f), dt_s);
        feedforward_calculate(&pitch_ff, gimbal.now_val.small_pitch_angle,
                              gimbal.small_pitch_ff_tracker.rate_rad_s, gimbal.small_pitch_ff_tracker.accel_rad_s2,
                              SMALL_PITCH_TORQUE_LIMIT, &gimbal.pitch_ff_out);

        gimbal.pc_small_pitch_pid.speed.set_target(gimbal.target_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.set_now(gimbal.now_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.pid_calculate_callback();
        gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_pitch_pid.speed.get_out() + gimbal.pitch_ff_out.total_nm, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));

        // 大pitch
        gimbal.pc_big_pitch_pid.angle.set_target(basic_math_constrain(gimbal.big_pit_ang_fil.get_out(), BIG_PITCH_MOTOR_TAG_ANGLE, BIG_PITCH_START_ANGLE));
        gimbal.pc_big_pitch_pid.angle.set_now(gimbal.now_val.big_pitch_angle);
        gimbal.pc_big_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.big_pitch_omega = gimbal.pc_big_pitch_pid.angle.get_out();

        gimbal.pc_big_pitch_pid.speed.set_target(gimbal.target_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.set_now(gimbal.now_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.pid_calculate_callback();
        gimbal.big_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_big_pitch_pid.speed.get_out() + gimbal.big_pit_gravity_comp, -BIG_PITCH_TORQUE_LIMIT, BIG_PITCH_TORQUE_LIMIT));

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
    // gimbal.big_yaw_motor.calculate_callback();
    gimbal.big_pitch_motor.calculate_callback();

    gimbal.last_method = gimbal.now_method;
    gimbal.last_mode = gimbal.now_mode;

    gimbal.last_big_pitch_ready = gimbal.big_pitch_ready;
}

/**
 * @brief 更新云台系统辨识参数
 */
void gimbal_joint_identification_update(void)
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
 * @brief 更新大pitch轴系统辨识
 * @note 独立于 gimbal_identification_update(): 小pitch/大yaw 的辨识开关保持原样,
 *       大pitch 单独使能, 避免一次改动同时影响三条轴。
 *       辨识结果(gimbal.ident_big_pitch_output.model)就是 big_pitch_ff 的参数来源。
 */
void gimbal_big_pitch_identification_update(void)
{
    const uint32_t time_us = bsp_dwt_get_timeline_us();
    const bool ident_enable = gimbal.now_method != Gimbal_Method_Zero_Force;

    const float feedback_s = gimbal.timestamp.big_pitch_us == 0u ? 1.0f : (float)(time_us - gimbal.timestamp.big_pitch_us) * 0.000001f;
    const bool valid = (gimbal.big_pitch_motor.get_status() == Motor_DM_Status_Enable) && (feedback_s < 0.005f);

    /* 关节坐标用电机总角度(连续、无跳变), 供速度重构求斜率。
     * 重力相位与关节坐标相同 —— 大pitch 直接绕水平轴摆动, 重力力矩只取决于这一个角;
     * 坐标零点任意, 任意零点都会被 A/C 两个系数吸收。
     */
    const float joint_angle = gimbal.big_pitch_motor.get_total_angle() - BIG_PITCH_START_ANGLE;

    gimbal_identification_sample_fill(&gimbal.ident_big_pitch_sample, time_us, feedback_s, 0u, GIMBAL_IDENTIFICATION_BIG_PITCH,
                                      joint_angle, gimbal.big_pitch_motor.get_now_omega(), gimbal.big_pitch_motor.get_now_torque(),
                                      joint_angle, BIG_PITCH_TORQUE_LIMIT, ident_enable, valid);

    (void)gimbal_identification_update(&gimbal.ident_big_pitch, &gimbal.ident_big_pitch_sample, &gimbal.ident_big_pitch_output);

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
    const float now_small_yaw = gimbal.small_yaw_motor.get_total_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
    // const float now_small_pitch = gimbal.small_pitch_motor.get_total_angle() - SMALL_PITCH_MOTOR_MID_ANGLE;
    // const float now_big_pitch = gimbal.big_pitch_motor.get_total_angle() - BIG_PITCH_START_ANGLE;

    // 小yaw目标值设置
    if(now_small_yaw >= SMALL_YAW_MAX_OFFSET - 0.01f)
    {
        // gimbal.target_val.small_pitch_omega = -target_val;
        // gimbal.target_val.small_yaw_omega = -debug_data;
        // gimbal.target_val.small_pitch_omega = -debug_data;
        // gimbal.target_val.big_pitch_omega = -debug_data;
    }

    else if(now_small_yaw <= SMALL_YAW_MIN_OFFSET + 0.01f)
    {
        // gimbal.target_val.small_pitch_omega = target_val;
        // gimbal.target_val.small_yaw_omega = debug_data;
        // gimbal.target_val.small_pitch_omega = debug_data;
        // gimbal.target_val.big_pitch_omega = debug_data;
    }

    gimbal.target_val.small_pitch_angle = basic_math_constrain(wave_generator_get_value(&test_wave), gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MIN_OFFSET + 0.1f, gimbal.small_pitch_imu_limit_angle + SMALL_PITCH_MAX_OFFSET - 0.2f);
    // gimbal.target_val.big_pitch_angle = basic_math_constrain(wave_generator_get_value(&test_wave) - 1.08681506f, BIG_PITCH_MOTOR_TAG_ANGLE + 0.04f, BIG_PITCH_START_ANGLE - 0.04f);

    if(gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Up)
    {
        gimbal.target_val.big_pitch_angle = BIG_PITCH_MOTOR_TAG_ANGLE;
    }
    else if(gimbal.target_val.big_pitch_status == Gimbal_Big_Pitch_Down)
    {
        gimbal.target_val.big_pitch_angle = BIG_PITCH_START_ANGLE;
    }
    else
    {
        gimbal.target_val.big_pitch_angle = gimbal.now_val.big_pitch_angle;
    }

    // gimbal.target_val.small_yaw_angle = basic_math_constrain(wave_generator_get_value(&test_wave), SMALL_YAW_MIN_OFFSET, SMALL_YAW_MAX_OFFSET);
    gimbal.target_val.small_yaw_angle = basic_math_constrain(debug_data, SMALL_YAW_MIN_OFFSET, SMALL_YAW_MAX_OFFSET);

    gimbal.big_pit_ang_fil.set_now_input(gimbal.target_val.big_pitch_angle);
    gimbal.big_pit_ang_fil.low_calculate_callback();
    // gimbal.target_val.small_pitch_angle = 0.0f;

    // 前馈(重力补偿在函数内部算好)
    gimbal_update_feedforward();

    gimbal.big_pit_gravity_comp = -0.144609526 * arm_sin_f32(gimbal.now_val.big_pitch_angle) - 0.111498073 * arm_cos_f32(gimbal.now_val.big_pitch_angle);

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
        gimbal.pc_small_yaw_pid.angle.set_target(basic_math_constrain(gimbal.target_val.small_yaw_angle, SMALL_YAW_MIN_OFFSET, SMALL_YAW_MAX_OFFSET));
        gimbal.pc_small_yaw_pid.angle.set_now(gimbal.now_val.small_yaw_angle);
        gimbal.pc_small_yaw_pid.angle.pid_calculate_callback();
        // gimbal.target_val.small_yaw_omega = gimbal.pc_small_yaw_pid.angle.get_out();
        gimbal.target_val.small_yaw_omega = gimbal.pc_small_yaw_pid.angle.get_out();

        gimbal.pc_small_yaw_pid.speed.set_target(gimbal.target_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.set_now(gimbal.now_val.small_yaw_omega);
        gimbal.pc_small_yaw_pid.speed.pid_calculate_callback();
        // gimbal.small_yaw_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_yaw_pid.speed.get_out(), -SMALL_YAW_TORQUE_LIMIT, SMALL_YAW_TORQUE_LIMIT));
        gimbal.small_yaw_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_yaw_pid.speed.get_out() + gimbal.small_yaw_ff_out.total_nm, -SMALL_YAW_TORQUE_LIMIT, SMALL_YAW_TORQUE_LIMIT));

        // 小pitch: 力矩 = 速度环 + 总前馈
        gimbal.pc_small_pitch_pid.angle.set_target(gimbal.target_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.set_now(gimbal.now_val.small_pitch_angle);
        gimbal.pc_small_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.small_pitch_omega = gimbal.pc_small_pitch_pid.angle.get_out();

        gimbal.pc_small_pitch_pid.speed.set_target(gimbal.target_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.set_now(gimbal.now_val.small_pitch_omega);
        gimbal.pc_small_pitch_pid.speed.pid_calculate_callback();
        // gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_pitch_pid.speed.get_out() + gravity_torque, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));
        gimbal.small_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_small_pitch_pid.speed.get_out() + gimbal.pitch_ff_out.total_nm, -SMALL_PITCH_TORQUE_LIMIT, SMALL_PITCH_TORQUE_LIMIT));

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
        gimbal.pc_big_pitch_pid.angle.set_target(basic_math_constrain(gimbal.big_pit_ang_fil.get_out(), BIG_PITCH_MOTOR_TAG_ANGLE, BIG_PITCH_START_ANGLE));
        gimbal.pc_big_pitch_pid.angle.set_now(gimbal.now_val.big_pitch_angle);
        gimbal.pc_big_pitch_pid.angle.pid_calculate_callback();
        gimbal.target_val.big_pitch_omega = gimbal.pc_big_pitch_pid.angle.get_out();

        gimbal.pc_big_pitch_pid.speed.set_target(gimbal.target_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.set_now(gimbal.now_val.big_pitch_omega);
        gimbal.pc_big_pitch_pid.speed.pid_calculate_callback();
        gimbal.big_pitch_motor.set_control_torque(basic_math_constrain(gimbal.pc_big_pitch_pid.speed.get_out() + gimbal.big_pit_gravity_comp, -BIG_PITCH_TORQUE_LIMIT, BIG_PITCH_TORQUE_LIMIT));
        // gimbal.big_pitch_motor.set_control_torque(test_torque);
    }


    // else if (gimbal.now_method == Gimbal_Method_Manual) // 直接调试pc模式的pid
    // {
    //     // gimbal.small_pitch_motor.set_control_torque(gravity_torque);
    //     // gimbal.small_pitch_motor.set_control_torque(small_pitch_gravity);
    //     gimbal.big_pitch_motor.set_control_torque(test_torque);
    // }

    gimbal.small_yaw_motor.calculate_callback();
    gimbal.small_pitch_motor.calculate_callback();
    // gimbal.big_yaw_motor.calculate_callback();
    gimbal.big_pitch_motor.calculate_callback();

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

        // 更新目标值更新决策数据
        gimbal_control_decision();

        // 电机辨识
        gimbal_joint_identification_update();

        // 大pitch轴辨识(生成 big_pitch_ff 的参数来源)
        // gimbal_big_pitch_identification_update();

        // 计算电机控制量
        gimbal_motor_calculate();

        // 调试程序
        // gimbal_motor_control_test();

        // 延时
        osDelay(1);
    }
}
