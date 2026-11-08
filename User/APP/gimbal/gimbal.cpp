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

// 波形发生器
wave_generator_t test_wave;

/*------------------------------------云台电机can接收对象(注册到统一分发层）-------------------------------------------*/
static void small_yaw_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.small_yaw_motor.can_rx_callback();     // CAN1 0x11
    gimbal.feedback_timestamp.small_yaw_s = bsp_dwt_get_deltat32(&gimbal.timestamp.small_yaw_us); // 更新时间戳
}

static void small_pitch_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.small_pitch_motor.can_rx_callback();   // CAN1 0x12
    gimbal.feedback_timestamp.small_pitch_s = bsp_dwt_get_deltat32(&gimbal.timestamp.small_pitch_us); // 更新时间戳
}

static void big_pitch_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.big_pitch_motor.can_rx_callback();     // CAN1 0x13
    gimbal.feedback_timestamp.big_pitch_s = bsp_dwt_get_deltat32(&gimbal.timestamp.big_pitch_us); // 更新时间戳
}

static void big_yaw_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.big_yaw_motor.can_rx_callback();       // CAN2 0x14
    gimbal.feedback_timestamp.big_yaw_s = bsp_dwt_get_deltat32(&gimbal.timestamp.big_yaw_us); // 更新时间戳
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
    config->window_s = 0.05f;                 // 50 ms：覆盖约 50 个 1 kHz 控制周期，降低单帧噪声
    config->max_gap_s = 0.02f;                // 20 ms：允许少量任务抖动，但不跨越明显丢帧
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区
    rls_last_default_config(&config->rls, 5u);

    config->rls.initial[0] = 0.01f;          // J 起步量：仅用于 RLS 初值，不是实测标定值
    config->rls.initial[1] = 0.005f;         // B 起步量：保持与小型云台输出轴量级同阶
    config->rls.initial[2] = 0.02f;          // Fc 起步量：给低速摩擦项一个非零可学习初值
    config->rls.initial[3] = 0.0f;       // sin 重力项无先验方向，使用零中心
    config->rls.initial[4] = 0.1f;       // cos 重力项提供轻微非零起点，避免初期完全失激励

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

    config->dt_min_s = 0.0009f;               // 最小采样时间，单位：秒
    config->dt_max_s = 0.005f;                // 最大采样时间，单位：秒

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
    config->max_accel_rad_s2 = 64.0f;        // 最大角加速度，单位：rad/s^2 (不要降低: 它同时决定跟随的加速度能力, 降低会加大跟踪滞后)
    config->max_jerk_rad_s3 = 6850.0f;       // 最大角加加速度，单位：rad/s^3 (≈透明, 仅挡异常尖峰; 太小会削掉正常拐点)
    config->position_gain_per_s = 65.0f;      // 位置比例增益，单位：1/秒 (P滞后≈v/Kp)
    config->rate_servo_gain_per_s = 350.0f;   // 速率环比例增益，单位：1/秒 (wn=13.8Hz, zeta=0.80)

    config->dt_min_s = 0.0009f;               // 最小采样时间，单位：秒
    config->dt_max_s = 0.005f;                // 最大采样时间，单位：秒

    config->wrap_angle = true;                // 是否进行角度环绕处理(最短角规划)
    config->accel_sat_guard_enable = true;    // 轨迹规划器饱和守护函数

    config->sat_gain_scale = 0.5f;           // 饱和时位置增益缩放系数, 0 < scale < 1 (0.4~0.6)
    config->sat_release_per_s = 2.0f;        // 饱和解除后增益恢复速率, 单位 1/s (1~3)
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
    gimbal.small_yaw_motor.init(&hfdcan1, 0x01, 0x11, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);
    gimbal.small_pitch_motor.init(&hfdcan1, 0x02, 0x12, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);
    gimbal.big_pitch_motor.init(&hfdcan1, 0x03, 0x13, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);
    gimbal.big_yaw_motor.init(&hfdcan2, 0x04, 0x14, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 30.0f, 10.0f);

    // 初始化控制低通滤波器
    gimbal.chassis_angular_filter.init(5.0f, 0.001f);

    gimbal.pc_filter.yaw_angle.init(50.0f, 0.001f);
    gimbal.pc_filter.pitch_angle.init(50.0f, 0.001f);

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

    gimbal.pc_small_pitch_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_small_pitch_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.pc_big_yaw_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_big_yaw_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

    gimbal.pc_big_pitch_pid.angle.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);
    gimbal.pc_big_pitch_pid.speed.init(0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE, PID_Anti_Windup_DISABLE);

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
    gimbal.now_val.small_yaw_angle = gimbal.small_yaw_motor.get_now_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
    gimbal.now_val.small_pitch_angle = gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD;
    gimbal.now_val.big_yaw_angle = gimbal.small_yaw_motor.get_now_angle() - SMALL_YAW_MOTOR_MID_ANGLE;
    gimbal.now_val.big_pitch_angle = gimbal.big_pitch_motor.get_now_angle();    

    gimbal.ffd_val.chassis_angular = gimbal.decision->can_board_communication.get_chassis_angular();

    // 编码器到imu角度限制更新
    gimbal.small_pitch_imu_limit_angle = gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD + (gimbal.small_pitch_motor.get_now_angle() - SMALL_PITCH_MOTOR_MID_ANGLE);

    // pitch电机角度是否到位判断
    if (basic_math_abs(BIG_PITCH_MOTOR_TAG_ANGLE - gimbal.now_val.big_pitch_angle) >= BIG_PITCH_MOTOR_FIL_ANGLE)
    {
        gimbal.big_pitch_ready = false;
    }
    else
    {
        gimbal.big_pitch_ready = true;
    }
}

/**
 * @brief 更新云台观测值(低通滤波等状态观测量)
 */
void gimbal_update_observate(void)
{

}

/**
 * @brief 切换模式时，清除控制器和目标值的状态，防止数据冲击和跳变
 */
static void gimbal_switch_clear_condition(void)
{
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
                        
                    }
                    else
                    {

                    }
                }
            }
        }
    }
}

