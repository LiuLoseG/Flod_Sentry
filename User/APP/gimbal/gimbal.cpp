#include "gimbal.h"
#include "arm_math.h"
#include "basic_math.h"
#include "freertos.h"
#include "cmsis_os2.h"

// PID波形调节器
#include "gimbal_identification.h"
#include "pid.h"

// 统一CAN分发层
#include "can_distribution.h"

// 安全守护进程(电机离线检测)
#include "reference_track_plan.h"
#include "safe_daemon_process.h"
#include "wave_generator.h"
#include "bsp_dwt.h"

#include <cmath>
#include <cstring>

// 定义全局云台对象
gimbal_t gimbal;

// 波形发生器
wave_generator_t yaw_wave;

float pitch_c_angle = 0.00065f;
float yaw_c_angle = 0.00112f;


float dt_y = 0.001f;  // 1ms


#define GIMBAL_IDENTIFICATION_CONFIGURATION_PITCH       1u
#define GIMBAL_IDENTIFICATION_CONFIGURATION_SMALL_YAW   2u

/* 小yaw规划角速度输出低通时间常数(s)
 * 规划器在误差超过 max_accel/gain ≈ 0.1° 时就进入 ±max_accel 的 bang-bang,
 * 速度曲线是斜率 60 rad/s^2 的折线, 拐角即力矩冲击。
 * 这里在送速度环前做一阶低通削掉拐角: 8Hz 截止, 对 0.8Hz 级跟踪信号
 * 幅值损失 <0.5%、相位滞后约 5.7°, 而 10ms 尺度上的 |a| 峰值约降 38%。 */
#define YAW_SMALL_PLAN_RATE_LPF_TAU_S  0.020f

/* pitch规划角速度输出低通时间常数(s)
 * pitch轴惯量和阻尼都比小yaw大, 且带重力负载, 同样的 bang-bang 拐角冲击会被
 * 更大的等效质量放大成更长时间的低频晃动, 因此这里取略大的时间常数
 * (截止 8Hz -> 6.4Hz), 对 0.8Hz 级跟踪信号的幅值损失/相位滞后可忽略。 */
#define PITCH_PLAN_RATE_LPF_TAU_S      0.025f

/* ---- pitch轴前馈力矩系数(与小yaw同结构 + 重力项):
 *      tau = J*a + B*w + Fc*tanh(w/eps) + A*sin(th) + C*cos(th)
 *      A/C 为旁路辨识器 identification_pitch 输出的重力 sin/cos 系数(Nm),
 *      其自变量是辨识样本里的 gravity_angle_rad, 即 IMU pitch 角(rad)。
 *      注意: A/C 与 J/B/Fc 同源, 都是"电机反馈力矩"那一侧的系数。 ----*/
#define PITCH_FF_J                    0.0100852549f
#define PITCH_FF_B                    0.0296297334f
#define PITCH_FF_FC                   0.186175033f
#define PITCH_FF_A                    0.00154406193f      // 重力 sin 系数 Nm (辨识值)
#define PITCH_FF_C                    -0.681183577f      // 重力 cos 系数 Nm (辨识值)
#define PITCH_FF_EPS                  0.08f          // 摩擦速度过渡尺度 rad/s (须与辨识器 friction_velocity_rad_s 一致)
#define PITCH_FF_RATE_DEADBAND        0.05f          // 库仑项低速死区 rad/s
#define PITCH_FF_ACC_LPF_ALPHA        0.05f          // 加速度项低通(≈8Hz@1kHz)
#define PITCH_FF_TORQUE_LPF_ALPHA     0.03f          // 前馈力矩输出低通(≈5Hz@1kHz)
#define PITCH_FF_TORQUE_MAX           0.60f          // 仅动态项(J*a+B*w+Fc)的兜底限幅 Nm
/* 重力项必须单独限幅后再与动态项相加: |A*sin+C*cos| 峰值 = sqrt(A^2+C^2) ≈ 0.71 Nm,
 * 已经超过 PITCH_FF_TORQUE_MAX(0.60), 若把重力也塞进 0.6 的限幅里会被直接削掉。 */
#define PITCH_FF_GRAVITY_MAX          1.00f          // 重力项单独限幅 Nm (≈1.4 倍峰值)
#define PITCH_FF_TOTAL_MAX            1.50f          // 合计兜底限幅 Nm (必须小于 PITCH_TORQUE_LIMIT=1.8)

// 轨迹规划器参数配置
reference_config_t yaw_small_track_plan_config = {
    .max_rate_rad_s = 5.0f,           // 最大角速度，单位：rad/s
    .max_accel_rad_s2 = 65.0f,        // 最大角加速度，单位：rad/s^2 (不要降低: 它同时决定跟随的加速度能力, 降低会加大跟踪滞后)
    .max_jerk_rad_s3 = 5000.0f,       // 最大角加加速度，单位：rad/s^3 (≈透明, 仅挡异常尖峰; 太小会削掉正常拐点)
    .position_gain_per_s = 55.0f,      // 位置比例增益，单位：1/秒 (P滞后≈v/Kp)
    .rate_servo_gain_per_s = 450.0f,   // 速率环增益，单位：1/秒 
    .dt_min_s = 0.0009f,               // 最小采样时间，单位：秒
    .dt_max_s = 0.005f,                // 最大采样时间，单位：秒
    .wrap_angle = true,                // 是否进行角度环绕处理(最短角规划)
    .accel_sat_guard_enable = true,    // 轨迹规划器饱和守护函数
    .sat_gain_scale = 0.5f,           // 饱和时位置增益缩放系数, 0 < scale < 1 (建议 0.4~0.6)
    .sat_release_per_s = 2.0f,        // 饱和解除后增益恢复速率, 单位 1/s (建议 1~3)
};

// 轨迹规划器参数配置
reference_config_t pitch_track_plan_config = {
    .max_rate_rad_s = 5.0f,           // 最大角速度，单位：rad/s
    .max_accel_rad_s2 = 64.0f,        // 最大角加速度，单位：rad/s^2 (不要降低: 它同时决定跟随的加速度能力, 降低会加大跟踪滞后)
    .max_jerk_rad_s3 = 6500.0f,       // 最大角加加速度，单位：rad/s^3 (≈透明, 仅挡异常尖峰; 太小会削掉正常拐点)
    .position_gain_per_s = 54.0f,      // 位置比例增益，单位：1/秒 (P滞后≈v/Kp)
    .rate_servo_gain_per_s = 140.0f,   // 速率环增益，单位：1/秒 (wn=13.8Hz, zeta=0.80)
    .dt_min_s = 0.0009f,               // 最小采样时间，单位：秒
    .dt_max_s = 0.005f,                // 最大采样时间，单位：秒
    .wrap_angle = true,                // 是否进行角度环绕处理(最短角规划)
    .accel_sat_guard_enable = true,    // 轨迹规划器饱和守护函数
    .sat_gain_scale = 0.5f,           // 饱和时位置增益缩放系数, 0 < scale < 1 (建议 0.4~0.6)
    .sat_release_per_s = 2.0f,        // 饱和解除后增益恢复速率, 单位 1/s (建议 1~3)
};

// 相关矩阵
// class_matrix_f32<2, 2> A;
// class_matrix_f32<2, 1> B;
// class_matrix_f32<2, 2> H;
// class_matrix_f32<2, 2> Q;
// class_matrix_f32<2, 2> R;
// class_matrix_f32<2, 2> P;

gimbal_identification_config_t identification_small_yaw_config;
gimbal_identification_config_t identification_pitch_config;

/*------------------------------------云台电机can接收对象(注册到统一分发层）-------------------------------------------*/
static void gimbal_pitch_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.pitch_motor.can_rx_callback();     // CAN1 0x11
    gimbal.pitch_feedback_timestamp_us =
        (uint32_t)bsp_dwt_get_timeline_us();
}

static void gimbal_yaw_small_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.yaw_small_motor.can_rx_callback(); // CAN1 0x205
    gimbal.small_yaw_feedback_timestamp_us =
        (uint32_t)bsp_dwt_get_timeline_us();
}

/**
 * @brief 初始化云台单轴的旁路 J/B 候选参数辨识器配置
 * @param axis 轴类型决定参数个数与重力项: YAW 3 参(J,B,Fc), PITCH 5 参(另加 sin/cos 重力)
 * @param configuration_id 必须与该轴送入 sample.configuration_id 完全一致,
 *                         否则每帧都被判 CONFIGURATION_MISMATCH 并直接返回, 估计值永不更新
 * @note 初值是 RLS 的可运行起点, 不是出厂标定值
 */
static void gimbal_identification_config_init(gimbal_identification_config_t *config,
                                              gimbal_identification_axis_t axis,
                                              uint32_t configuration_id)
{
    uint8_t count;
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));

    config->configuration_id = configuration_id;

    config->axis = axis;
    config->window_s = 0.05f;                 // 50 ms：覆盖约 50 个 1 kHz 控制周期，降低单帧噪声
    config->max_gap_s = 0.02f;                // 20 ms：允许少量任务抖动，但不跨越明显丢帧
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区
    count = axis == GIMBAL_IDENTIFICATION_YAW ? 3u : 5u;
    rls_last_default_config(&config->rls, count);

    config->rls.initial[0] = 0.0118676918f;            // J 起步量：仅用于 RLS 初值，不是实测标定值
    config->rls.initial[1] = 0.00934354216f;              // B 起步量：保持与小型云台输出轴量级同阶
    config->rls.initial[2] = 0.0246254914f;             // Fc 起步量：给低速摩擦项一个非零可学习初值

    config->rls.minimum[0] = 0.000001f;      // J 必须为正，禁止 RLS 退化成非物理负惯量
    config->rls.minimum[1] = 0.0f;
    config->rls.minimum[2] = 0.0f;

    config->rls.maximum[0] = 1.0f;           // 覆盖常见云台等效惯量，超出需按实测机构放宽
    config->rls.maximum[1] = 5.0f;           // B 上界避免噪声把阻尼推向异常大值
    config->rls.maximum[2] = 5.0f;           // Fc 上界与最大反馈扭矩同量级

    config->rls.feature_scale[0] = 10.0f;    // 角加速度通常最大，缩放改善 J 列条件数
    config->rls.feature_scale[1] = 1.0f;     // 角速度通常次之，缩放改善 B 列条件数
    config->rls.feature_scale[2] = 1.0f;     // 扭矩通常最小，缩放改善 Fc 列条件数

    if (axis == GIMBAL_IDENTIFICATION_PITCH) {
        config->rls.initial[3] = 0.0f;       // sin 重力项无先验方向，使用零中心
        config->rls.initial[4] = 0.1f;       // cos 重力项提供轻微非零起点，避免初期完全失激励

        config->rls.minimum[3] = -5.0f;
        config->rls.minimum[4] = -5.0f;

        config->rls.maximum[3] = 5.0f;
        config->rls.maximum[4] = 5.0f;

        config->rls.feature_scale[3] = 1.0f;
        config->rls.feature_scale[4] = 1.0f;
    }

    config->rls.dt_min_s = 0.000001f;
    config->rls.dt_max_s = 0.1f;

}

static void gimbal_identification_pitch_config_init(gimbal_identification_config_t *config,
                                              gimbal_identification_axis_t axis,
                                              uint32_t configuration_id)
{
    uint8_t count;
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));

    config->configuration_id = configuration_id;

    config->axis = axis;
    config->window_s = 0.05f;                 // 50 ms：覆盖约 50 个 1 kHz 控制周期，降低单帧噪声
    config->max_gap_s = 0.02f;                // 20 ms：允许少量任务抖动，但不跨越明显丢帧
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区
    /* 必须与 gimbal_identification.cpp::config_valid() 的期望一致:
     * YAW 3 参(J,B,Fc), PITCH 5 参(另加 A,C)。
     * 这里曾误写成 `axis == 5u`(恒 false → count=0)，会让 parameter_count=0，
     * config_valid() 失败 → gimbal_identification_init() 把整个实例 memset 清零并
     * 返回 false，之后 pitch 辨识每帧都走 BAD_CONFIG 分支，参数永不更新且无任何报错。 */
    count = axis == GIMBAL_IDENTIFICATION_YAW ? 3u : 5u;
    rls_last_default_config(&config->rls, count);

    config->rls.initial[0] = PITCH_FF_J;          // J 起步量：仅用于 RLS 初值，不是实测标定值
    config->rls.initial[1] = PITCH_FF_B;         // B 起步量：保持与小型云台输出轴量级同阶
    config->rls.initial[2] = PITCH_FF_FC;          // Fc 起步量：给低速摩擦项一个非零可学习初值

    config->rls.minimum[0] = 0.000001f;      // J 必须为正，禁止 RLS 退化成非物理负惯量
    config->rls.minimum[1] = 0.0f;
    config->rls.minimum[2] = 0.0f;

    config->rls.maximum[0] = 1.0f;           // 覆盖常见云台等效惯量，超出需按实测机构放宽
    config->rls.maximum[1] = 5.0f;           // B 上界避免噪声把阻尼推向异常大值
    config->rls.maximum[2] = 5.0f;           // Fc 上界与最大反馈扭矩同量级

    config->rls.feature_scale[0] = 10.0f;    // 角加速度通常最大，缩放改善 J 列条件数
    config->rls.feature_scale[1] = 1.0f;     // 角速度通常次之，缩放改善 B 列条件数
    config->rls.feature_scale[2] = 1.0f;     // 扭矩通常最小，缩放改善 Fc 列条件数

    config->rls.initial[3] = PITCH_FF_A;       // sin 重力项无先验方向，使用零中心
    config->rls.initial[4] = PITCH_FF_C;       // cos 重力项提供轻微非零起点，避免初期完全失激励

    config->rls.minimum[3] = -5.0f;
    config->rls.minimum[4] = -5.0f;

    config->rls.maximum[3] = 5.0f;
    config->rls.maximum[4] = 5.0f;

    config->rls.feature_scale[3] = 1.0f;
    config->rls.feature_scale[4] = 1.0f;

    config->rls.dt_min_s = 0.000001f;
    config->rls.dt_max_s = 0.1f;

}


// /**
//  * @brief 初始化云台轴的旁路 J/B 候选参数辨识器。
//  * @note 默认配置是可运行的起点，不是出厂标定值。
//  */
// static void gimbal_identification_init_all(void)
// {
//     gimbal_identification_config_t config;

//     /* Pitch 需要同时辨识惯量、阻尼、库仑摩擦和重力正余弦项。 */
//     gimbal_identification_default_config(&config, GIMBAL_IDENTIFICATION_PITCH);
//     config.configuration_id = GIMBAL_IDENTIFICATION_CONFIGURATION_PITCH;
//     (void)gimbal_identification_init(&gimbal.identification_pitch, &config);

//     /* Yaw 没有重力项，使用三参数模型可避免无意义的共线参数。 */
//     gimbal_identification_default_config(&config, GIMBAL_IDENTIFICATION_YAW);
//     config.configuration_id = GIMBAL_IDENTIFICATION_CONFIGURATION_SMALL_YAW;
//     (void)gimbal_identification_init(&gimbal.identification_small_yaw, &config);
// }

/**
 * @brief 填充单轴辨识样本
 * @note 逐字段显式赋值: 两个轴共用一个填充入口, 避免"上一轴的反馈年龄/时间对齐
 *       状态被带进下一轴"或漏赋值(未赋值字段会被 qualify() 判坏并逐帧清窗口)。
 * @param torque_limit 该轴力矩指令限幅 Nm, 仅用于饱和判据
 */
static void gimbal_identification_sample_fill(gimbal_identification_sample_t *sample,
                                              uint32_t timestamp_us,
                                              uint32_t configuration_id,
                                              bool enabled,
                                              bool valid,
                                              float feedback_age_s,
                                              float angle_rad,
                                              float rate_rad_s,
                                              float gravity_angle_rad,
                                              float torque_nm,
                                              float torque_limit)
{
    sample->timestamp_us = timestamp_us;
    sample->segment_id = 0u;
    sample->configuration_id = configuration_id;
    sample->angle_rad = angle_rad;
    sample->rate_rad_s = rate_rad_s;
    sample->gravity_angle_rad = gravity_angle_rad;
    sample->torque_nm = torque_nm;
    sample->feedback_age_s = feedback_age_s;
    sample->enabled = enabled;
    sample->valid = valid;
    sample->saturated = fabsf(torque_nm) >= 0.98f * torque_limit;
    sample->operating_conditions_valid = enabled && valid;
    sample->torque_calibrated = true;
    sample->torque_time_aligned = valid;
}

/* 最近一帧送进辨识器的样本, 仅用于调试观察(两轴各一份, 不参与控制) */
static gimbal_identification_sample_t gimbal_pitch_ident_sample;
static gimbal_identification_sample_t gimbal_yaw_ident_sample;

/**
 * @brief 旁路采集同一控制周期的反馈并更新候选模型。
 * @note 不注入激励、不覆盖 PID/前馈/电机目标；ready 仅代表候选数据门槛通过。
 */
static void gimbal_identification_update_all(void)
{
    const uint32_t timestamp_us = (uint32_t)bsp_dwt_get_timeline_us();
    const bool enabled = gimbal.now_method != Gimbal_Method_Zero_Force;

    const float pitch_feedback_age_s =
        gimbal.pitch_feedback_timestamp_us == 0u
            ? 1.0f
            : (float)(timestamp_us - gimbal.pitch_feedback_timestamp_us) * 0.000001f;

    const float small_yaw_feedback_age_s =
        gimbal.small_yaw_feedback_timestamp_us == 0u
            ? 1.0f
            : (float)(timestamp_us - gimbal.small_yaw_feedback_timestamp_us) * 0.000001f;

    const bool pitch_valid =
        gimbal.imu_attitude_ready != 0u &&
        gimbal.pitch_motor.get_status() == Motor_DM_Status_Enable &&
        pitch_feedback_age_s <= 0.02f;

    const bool small_yaw_valid =
        gimbal.yaw_small_motor.get_status() == Motor_DJI_Status_Enable &&
        small_yaw_feedback_age_s <= 0.02f;

    /* Pitch：角度/角速度/力矩全部取电机输出轴(DM反馈), 三者同坐标系;
     * 角度用 total_angle 保持多圈连续, 避免单圈 ±PI 跳变被当成巨大速率;
     * 重力相位取 IMU pitch(真实仰角), 因此辨识出的 A/C 是电机力矩系下的重力项。 */
    gimbal_identification_sample_fill(&gimbal_pitch_ident_sample,
                                      timestamp_us,
                                      GIMBAL_IDENTIFICATION_CONFIGURATION_PITCH,
                                      enabled,
                                      pitch_valid,
                                      pitch_feedback_age_s,
                                      gimbal.pitch_motor.get_total_angle() - PITCH_MOTOR_MID_ANGLE,
                                      gimbal.pitch_motor.get_now_omega(),
                                      gimbal.now_value.pitch_angle,
                                      gimbal.pitch_motor.get_now_torque(),
                                      PITCH_TORQUE_LIMIT);
    (void)gimbal_identification_update(&gimbal.identification_pitch,
                                       &gimbal_pitch_ident_sample,
                                       &gimbal.identification_pitch_output);

    /* 小 yaw：get_now_angle 是输出轴连续角，避免 single_angle 在 ±PI 处的单圈跳变；
     * 该轴无重力项, gravity_angle_rad 固定 0。 */
    gimbal_identification_sample_fill(&gimbal_yaw_ident_sample,
                                      timestamp_us,
                                      GIMBAL_IDENTIFICATION_CONFIGURATION_SMALL_YAW,
                                      enabled,
                                      small_yaw_valid,
                                      small_yaw_feedback_age_s,
                                      gimbal.yaw_small_motor.get_now_angle() - YAW_SMALL_MOTOR_MID_ANGLE,
                                      gimbal.yaw_small_motor.get_now_omega(),
                                      0.0f,
                                      gimbal.yaw_small_motor.get_now_torque(),
                                      YAW_SMALL_TORQUE_LIMIT);
    (void)gimbal_identification_update(&gimbal.identification_small_yaw,
                                       &gimbal_yaw_ident_sample,
                                       &gimbal.identification_small_yaw_output);
}

/**
 * @brief 云台初始化
 */
void gimbal_init(void)
{
    // 1.绑定陀螺仪对象和遥控器对象
    gimbal.imu_data = &imu_frame;
    gimbal.crsf = &crsf;
    gimbal.source = &gimbal_decision_source;  // 绑定云台控制用于决策的源数据指针
    gimbal.decision = &robot_decision;  // 绑定机器人决策对象指针

    // 2.初始化电机对象
    gimbal.yaw_small_motor.init(&hfdcan1, Motor_DJI_ID_0x205, DJI_Motor_Control_Method_Torque); // DJI电机，内部PID闭环，角度模式
    gimbal.pitch_motor.init(&hfdcan1, 0x11, 0x01, Motor_DM_Control_Method_Normal_MIT, 3.14159f, 30.0f, 10.0f);  // DM电机，外部PID闭环，MIT模式

    // 3.初始化小yaw电机的角度环和角速度环的PID参数
    gimbal.rc_yaw_small_pid.pid_angle.init(11.8f, 0.0f, 0.12f, 0.0f, 1.0f, 3.0f, 0.001f, 0.0f, 0.10f, 0.24f, 0.32f, PID_D_First_ENABLE);
    gimbal.rc_yaw_small_pid.pid_omega.init(0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f, 0.001f, 0.0f, 0.2f, 0.65f, 1.0f, PID_D_First_ENABLE);

    gimbal.pc_yaw_small_pid.pid_angle.init(11.8f, 0.0f, 0.12f, 0.0f, 1.0f, 3.0f, 0.001f, 0.0f, 0.10f, 0.24f, 0.32f, PID_D_First_ENABLE);
    gimbal.pc_yaw_small_pid.pid_omega.init(0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f, 0.001f, 0.0f, 0.2f, 0.65f, 1.0f, PID_D_First_ENABLE);

    // 5.初始化pitch电机的角度环和角速度环的PID参数
    // 角度环Ki=0 (无积分), 前馈通过Feedforward_Calculate独立提供
    // 注意: pid_class的k_f是简单差分前馈, 由Feedforward_Calculate的OLS前馈替代, 设为0避免重复
    gimbal.rc_pitch_pid.pid_angle.init(12.85f, 0.0f, 0.25f, 0.0f, 2.0f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.1f, PID_D_First_ENABLE);
    gimbal.rc_pitch_pid.pid_omega.init(0.58f, 0.0f, 0.0f, 0.0f, 1.0f, 3.2f);

    gimbal.pc_pitch_pid.pid_angle.init(12.85f, 0.0f, 0.25f, 0.0f, 2.0f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.1f, PID_D_First_ENABLE);
    gimbal.pc_pitch_pid.pid_omega.init(0.58f, 0.0f, 0.0f, 0.0f, 1.0f, 3.2f, 0.001f, 0.0f, 0.32f, 0.65f, 0.8f, PID_D_First_ENABLE);

    // 6.初始化上一帧目标角度
    gimbal.prev_target_small_yaw_angle = gimbal.target_value.small_yaw_angle;

    // 7.初始化云台前馈量
    // Feedforward_Init(&gimbal.pc_pitch_ffc, 0.62f, pitch_ff_coeff, 0.008f, PITCH_FF_REF_DOT_OLS_ORDER, 2);
    // Feedforward_Init(&gimbal.pc_yaw_small_ffc, 0.50f, yaw_small_ff_coeff, 0.003f, YAW_SMALL_FF_REF_DOT_OLS_ORDER, 2);

    // 8.滤波器初始化
    gimbal.chassis_angular_lpf.init(5.0f, 0.001f);  // 底盘角速度低通滤波器
    gimbal.filter_yaw.init(12.0f, 0.001f);
    gimbal.filter_pitch.init(6.0f, 0.001f);
    // gimbal.filter_imu_pitch_angle.init(8.0f, 0.001f);
    // gimbal.filter_pitch_pitch_omega.init(8.0f, 0.001f);

    gimbal.pitch_gravity_comp_filter.init(5.0f, 0.001f);
    // gimbal_identification_init_all();
    gimbal_identification_config_init(&identification_small_yaw_config, GIMBAL_IDENTIFICATION_YAW, GIMBAL_IDENTIFICATION_CONFIGURATION_SMALL_YAW);
    (void)gimbal_identification_init(&gimbal.identification_small_yaw, &identification_small_yaw_config);

    gimbal_identification_pitch_config_init(&identification_pitch_config, GIMBAL_IDENTIFICATION_PITCH, GIMBAL_IDENTIFICATION_CONFIGURATION_PITCH);
    (void)gimbal_identification_init(&gimbal.identification_pitch, &identification_pitch_config);

    gimbal.feedforward_value.small_yaw_omega = 0.0f;
    gimbal.feedforward_value.small_yaw_torque = 0.0f;

    gimbal.prev_small_yaw_ff_omega = 0.0f;  // 速率限制器的初始状态
    gimbal.prev_pitch_ff_torque = 0.0f;     // pitch力矩前馈速率限制器初始状态
    gimbal.yaw_small_ff_dc_mean = 0.0f;     // DC blocker 初始状态
    gimbal.yaw_small_ff_enc_lpf = 0.0f;     // 前馈参考用编码器低通初始状态
    gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 前馈参考用视觉误差低通初始状态
    gimbal.yaw_small_plan_target_lpf = 0.0f; // 规划器输入目标低通初始状态
    gimbal.yaw_small_plan_rate_lpf = 0.0f;   // 规划器输出角速度低通初始状态(首帧规划器 rate 亦为0)
    gimbal.yaw_small_ff_acc_lpf = 0.0f;      // 前馈加速度项低通初始状态
    gimbal.yaw_small_ff_torque_lpf = 0.0f;   // 前馈力矩输出低通初始状态
    gimbal.yaw_small_plan_dt_cnt = 0;        // 规划器DWT计数缓存初始状态
    gimbal.pitch_plan_target_lpf = 0.0f;     // pitch规划器输入目标低通初始状态(首帧重播种)
    gimbal.pitch_plan_rate_lpf = 0.0f;       // pitch规划器输出角速度低通初始状态(首帧规划器 rate 亦为0)
    gimbal.pitch_ff_acc_lpf = 0.0f;          // pitch前馈加速度项低通初始状态
    gimbal.pitch_ff_torque_lpf = 0.0f;       // pitch前馈力矩输出低通初始状态
    gimbal.pitch_plan_dt_cnt = 0;            // pitch规划器DWT计数缓存初始状态
    gimbal.feedforward_pitch_torque = 0.0f;  // pitch力矩前馈输出初始状态
    gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 直接播种到当前目标, 避免上电爬升
    gimbal.prev_pitch_ff_omega = 0.0f;      // [拐点缓冲] 加速度限制器初始状态

    gimbal.feedforward_value.pitch_omega = 0.0f;
    gimbal.feedforward_value.pitch_torque = 0.0f;

    // 初始化云台重力/摩擦力补偿量
    gimbal.pitch_gravity_comp.amount = 0.0f;
    gimbal.pitch_gravity_comp.factor[0] = -1.015506f;
    gimbal.pitch_gravity_comp.factor[1] = -0.05028f;
    gimbal.pitch_gravity_comp.factor[2] = 0.081351f;
    gimbal.pitch_gravity_comp.factor[3] = -0.0417713f;

    // gimbal.rls.setParamVector(Matrixf<4, 1>(gimbal.pitch_gravity_comp.factor));

    // 初始化IMU稳定状态
    gimbal.imu_frame_cnt = 0;
    gimbal.imu_attitude_ready = 0;

    gimbal.pit_towards_flag = 0;
    gimbal.yaw_towards_flag = 0;

    // 8.初始化云台模式
    gimbal.now_method = Gimbal_Method_Zero_Force;
    gimbal.last_method = Gimbal_Method_Zero_Force;
    gimbal.now_mode = Gimbal_Auto_Follow;
    gimbal.last_mode = Gimbal_Auto_Follow;

    // 9. 电机波形跟踪调试
    wave_generator_init(&yaw_wave, WAVE_SINE, 0.1f, 0.4f, 0.0f);
    wave_generator_enable(&yaw_wave);

    // 10. 注册 CAN 接收分发规则（统一分发层，各总线仅一个强定义回调）
    can_distribution_register(1, 0x11,  gimbal_pitch_motor_rx_client);
    can_distribution_register(1, 0x205, gimbal_yaw_small_motor_rx_client);

    // 11. 注册电机离线检测（安全守护进程统一调用 alive_check, 只检测状态不做安全动作）
    //     检测周期由 SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS 决定, 即掉线判定时延(单位 ms)
    SAFE_DAEMON_PROCESS_REGISTER("gimbal_pitch_motor", gimbal.pitch_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("gimbal_yaw_small_motor", gimbal.yaw_small_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    // 12. 初始化云台参考轨迹规划器
    reference_track_plan_init(&gimbal.yaw_small_track_plan, &yaw_small_track_plan_config, 0.0f);
    reference_track_plan_init(&gimbal.pitch_track_plan, &pitch_track_plan_config, 0.0f);
    // 卡尔曼滤波器
    // A[0][0] = 1.0f;
    // A[0][1] = 0.001f;
    // A[1][0] = 0.0f;
    // A[1][1] = 1.0f;

    // B[0][0] = 0.0f;
    // B[1][0] = 0.0f;

    // H[0][0] = 1.0f;
    // H[0][1] = 0.0f;
    // H[1][0] = 0.0f;
    // H[1][1] = 1.0f;

    // Q[0][0] = 0.1f;
    // Q[0][1] = 0.0f;
    // Q[1][0] = 0.0f;
    // Q[1][1] = 0.008f;

    // R[0][0] = 0.0001f;
    // R[0][1] = 0.0f;
    // R[1][0] = 0.0f;
    // R[1][1] = 0.1f;

    // P[0][0] = 0.001f;
    // P[0][1] = 0.0f;
    // P[1][0] = 0.0f;
    // P[1][1] = 0.8f;
    // gimbal.filter_yaw_omega.init(A, B, H, Q, R, P);
}

/**
 * @brief 更新云台控制模式
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
 * @note 使用底盘系作为云台的参考系，更新云台的角度和角速度测量值
 */
void gimbal_update_measure(void)
{
    // 更新云台角速度 (来自于 IMU 陀螺仪, 单位已是 rad/s)
    // 旋转矩阵消除z轴偏置导致的角速度分量偏转
    gimbal.now_value.small_yaw_omega = gimbal.imu_data->gyro[2] * arm_cos_f32(gimbal.imu_data->euler_angle.pitch) - gimbal.imu_data->gyro[0] * arm_sin_f32(gimbal.imu_data->euler_angle.pitch);

    gimbal.now_value.pitch_omega = gimbal.imu_data->gyro[1];        //  pitch电机角速度来自于IMU陀螺仪

    // 小yaw电机编码器角度反馈值
    gimbal.now_s_yaw_angle = gimbal.yaw_small_motor.get_now_angle() - YAW_SMALL_MOTOR_MID_ANGLE;

    /* ---- 姿态角需要等待 IMU 滤波器收敛 ---- */
    if (gimbal.imu_data->status == Imu_Status_Running)
    {
        if (gimbal.imu_frame_cnt < 2000)
        {
            gimbal.imu_frame_cnt++;
        }

        if (gimbal.imu_frame_cnt >= 2000 && !gimbal.imu_attitude_ready)
        {
            // IMU 滤波器已收敛, 解锁姿态角更新 
            gimbal.imu_attitude_ready = 1;

            // 首次使用时, 将当前 IMU 姿态一次性设置到 now_value, 避免后续突然跳变
            gimbal.now_value.small_yaw_angle = gimbal.imu_data->euler_angle.yaw;
            gimbal.now_value.pitch_angle = gimbal.imu_data->euler_angle.pitch;
        }
    }

    if (gimbal.imu_attitude_ready)
    {
        // IMU 已稳定, 正常更新姿态角
        gimbal.now_value.small_yaw_angle = gimbal.imu_data->euler_angle.yaw;
        gimbal.now_value.pitch_angle = gimbal.imu_data->euler_angle.pitch;

        gimbal.pitch_limit_angle = (gimbal.imu_data->euler_angle.roll + (gimbal.pitch_motor.get_total_angle() - PITCH_MOTOR_MID_ANGLE));
        gimbal.small_yaw_limit_angle = (gimbal.imu_data->euler_angle.yaw - (gimbal.yaw_small_motor.get_now_angle() - YAW_SMALL_MOTOR_MID_ANGLE));
    }

    // 更新滤波值 (用于观测, 不参与控制)
    gimbal.filter_value.small_yaw_angle   = gimbal.now_value.small_yaw_angle;
    gimbal.filter_value.small_yaw_omega   = gimbal.now_value.small_yaw_omega;

    gimbal.filter_value.pitch_angle = gimbal.now_value.pitch_angle;
    gimbal.filter_value.pitch_omega = gimbal.now_value.pitch_omega;

    // 获取当前真实值
    gimbal.filter_yaw.set_now_input(gimbal.decision->pc_to_mcu_aim.get_yaw_angle());
    gimbal.filter_pitch.set_now_input(gimbal.decision->pc_to_mcu_aim.get_pitch_angle());
    gimbal.chassis_angular_lpf.set_now_input(gimbal.decision->can_board_communication.get_chassis_angular());

    // gimbal.filter_imu_pitch_angle.set_now_input(gimbal.now_value.pitch_angle);
    // gimbal.filter_pitch_pitch_omega.set_now_input(gimbal.now_value.pitch_omega);

    // 对当前数据进行滤波处理
    gimbal.filter_yaw.low_calculate_callback();
    gimbal.filter_pitch.low_calculate_callback();
    gimbal.chassis_angular_lpf.low_calculate_callback();

    if(fabs(gimbal.chassis_angular_lpf.get_out()) < 0.1f)
    {
        gimbal.feedforward_value.chassis_angular = 0.0f;
    }

    else
    {
        gimbal.feedforward_value.chassis_angular = basic_math_constrain(gimbal.chassis_angular_lpf.get_out(), -PI, PI);
    }

    // gimbal.filter_imu_pitch_angle.low_calculate_callback();
    // gimbal.filter_pitch_pitch_omega.low_calculate_callback();
}

/**
 * @brief 更新云台决策源数据
 */
void gimbal_robot_decision_source_update(void)
{
    gimbal.source->small_yaw_angle = gimbal.yaw_small_motor.get_now_angle();
    gimbal.source->vision_gyr_z = gimbal.now_value.small_yaw_omega;
}

/**
 * @brief 更新云台控制决策(控制目标设定)
 */
void gimbal_control_decision(void)
{
    if (gimbal.now_method == Gimbal_Method_Zero_Force)  // 零力矩模式(电机失能)
    {
        gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
        gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
        gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
        gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

        gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
        gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
        gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
        gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

        gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
        gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;

        gimbal.target_value.small_yaw_omega = gimbal.now_value.small_yaw_omega;
        gimbal.target_value.pitch_omega = gimbal.now_value.pitch_omega;
    }

    else if (gimbal.now_method == Gimbal_Method_Manual) // 遥控器控制模式(操作手接管)
    {
        if (gimbal.last_method != Gimbal_Method_Manual)
        {
            gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

            gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

            gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
            gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;

            // 同步前馈状态, 防止模式切换后前馈微分项产生瞬态跳变
            gimbal.prev_target_small_yaw_angle = gimbal.target_value.small_yaw_angle;
            gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 delta=0, 状态同步为0
            gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种到当前目标, 防切换瞬态
            gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
        }

        else
        {
            float yaw_small_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_y(), -1.0f, 1.0f, 0.065f) * YAW_SMALL_ANGLE_INCREMENT_RATION;
            float pitch_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_x(), -1.0f, 1.0f, 0.065f) * PITCH_ANGLE_INCREMENT_RATION;

            gimbal.target_value.small_yaw_angle = basic_math_constrain(gimbal.target_value.small_yaw_angle + yaw_small_angle_increment, gimbal.small_yaw_limit_angle + YAW_SMALL_MIN_OFFSET, gimbal.small_yaw_limit_angle + YAW_SMALL_MAX_OFFSET);
            
            gimbal.target_value.pitch_angle = basic_math_constrain(gimbal.target_value.pitch_angle + pitch_angle_increment, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET);
        }
    }

    else if (gimbal.now_method == Gimbal_Method_Auto) // 自瞄控制模式(视觉接管)
    {
        if(gimbal.last_method != Gimbal_Method_Auto)
        {
            gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

            gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

            gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
            gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;

            // 重播种前馈参考用的编码器低通, 避免模式切换时因滤波器状态残留产生前馈台阶
            gimbal.yaw_small_ff_enc_lpf = gimbal.now_value.small_yaw_angle;
            gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 target=now ⇒ delta=0
            gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种, 防切换瞬态
            gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
        }

        else 
        {
            // 自瞄未识别到目标，进行索敌扫频
            if(gimbal.now_mode == Gimbal_Auto_Scan)
            {
                // 编码器进行当前角度的参考
                gimbal.now_value.small_yaw_angle = gimbal.now_s_yaw_angle;

                if(gimbal.last_mode != Gimbal_Auto_Scan)
                {
                    gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

                    gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

                    wave_generator_reset(&yaw_wave);

                    gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
                    gimbal.target_value.small_yaw_angle = gimbal.now_s_yaw_angle;
                    // 重播种前馈参考用的编码器低通, 避免模式切换时因滤波器状态残留产生前馈台阶
                    gimbal.yaw_small_ff_enc_lpf = gimbal.now_value.small_yaw_angle;
                    gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 target=now ⇒ delta=0
                    gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种, 防切换瞬态
                    gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
                }

                else
                {
                    if(gimbal.yaw_towards_flag == 0)
                    {
                        gimbal.target_value.small_yaw_angle += yaw_c_angle;

                        if(gimbal.target_value.small_yaw_angle >= (gimbal.small_yaw_limit_angle + YAW_SMALL_MAX_OFFSET - 0.05f))
                        {
                            gimbal.yaw_towards_flag = 1;
                        }
                    }

                    else 
                    {
                        gimbal.target_value.small_yaw_angle -= yaw_c_angle;

                        if(gimbal.target_value.small_yaw_angle <= (gimbal.small_yaw_limit_angle + YAW_SMALL_MIN_OFFSET + 0.05f))
                        {
                            gimbal.yaw_towards_flag = 0;
                        }
                    }

                    gimbal.target_value.small_yaw_angle = basic_math_constrain(gimbal.target_value.small_yaw_angle, gimbal.small_yaw_limit_angle + YAW_SMALL_MIN_OFFSET, gimbal.small_yaw_limit_angle + YAW_SMALL_MAX_OFFSET);

                    if(gimbal.pit_towards_flag == 0)
                    {
                        gimbal.target_value.pitch_angle += pitch_c_angle;

                        if(gimbal.target_value.pitch_angle >= (gimbal.pitch_limit_angle + PITCH_MAX_OFFSET - 0.18f))
                        {
                            gimbal.pit_towards_flag = 1;
                        }
                    }

                    else 
                    {
                        gimbal.target_value.pitch_angle -= pitch_c_angle;

                        if(gimbal.target_value.pitch_angle <= (gimbal.pitch_limit_angle + PITCH_MIN_OFFSET + 0.15f))
                        {
                            gimbal.pit_towards_flag = 0;
                        }
                    }

                    gimbal.target_value.pitch_angle = basic_math_constrain(gimbal.target_value.pitch_angle, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET);
                }
            }

            // 自瞄识别到目标，进行跟随
            else if(gimbal.now_mode == Gimbal_Auto_Follow)  
            {
                if(gimbal.last_mode != Gimbal_Auto_Follow)
                {
                    gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

                    gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);

                    wave_generator_reset(&yaw_wave);

                    gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
                    gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
                    // 重播种前馈参考用的编码器低通, 避免模式切换时因滤波器状态残留产生前馈台阶
                    gimbal.yaw_small_ff_enc_lpf = gimbal.now_value.small_yaw_angle;
                    gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 target=now ⇒ delta=0
                    gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种, 防切换瞬态
                    gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
                }

                else
                {
                    /*----------------------------参数调节段------------------------------*/
                    // gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;  // pitch保持当前角度
                    // gimbal.target_value.pitch_angle = basic_math_constrain(wave_generator_calc(&yaw_wave), gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET);   
                    gimbal.target_value.pitch_angle = basic_math_constrain(gimbal.decision->pc_to_mcu_aim.get_pitch_angle(), gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET);
                    // 波形直接作为小yaw目标, 限幅基准与手动模式一致
                    gimbal.target_value.small_yaw_angle = basic_math_constrain(gimbal.decision->pc_to_mcu_aim.get_yaw_angle(), gimbal.small_yaw_limit_angle + YAW_SMALL_MIN_OFFSET, gimbal.small_yaw_limit_angle + YAW_SMALL_MAX_OFFSET);
                    // gimbal.target_value.small_yaw_angle = basic_math_constrain(wave_generator_calc(&yaw_wave), gimbal.small_yaw_limit_angle + YAW_SMALL_MIN_OFFSET, gimbal.small_yaw_limit_angle + YAW_SMALL_MAX_OFFSET);
                }
            }

            // 自瞄未识别到目标，且不在索敌扫频模式下，保持当前角度不动
            else
            {
                gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
                gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
            }
        }
    }
}

/**
 * @brief 更新云台补偿量
 * @note 更新云台的重力补偿、摩擦力补偿等
 */
static void gimbal_update_compensation(void)
{
    if(gimbal.now_value.pitch_angle > -0.6f && gimbal.now_value.pitch_angle < 0.5f && gimbal.imu_attitude_ready == 1)
    {
        gimbal.pitch_gravity_comp.amount =  gimbal.pitch_gravity_comp.factor[3] * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle +
                                            gimbal.pitch_gravity_comp.factor[2] * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle +
                                            gimbal.pitch_gravity_comp.factor[1] * gimbal.target_value.pitch_angle +
                                            gimbal.pitch_gravity_comp.factor[0];
    }
    else
    {
        gimbal.pitch_gravity_comp.amount = 0.0f;
    }

    gimbal.pitch_gravity_comp_filter.set_now_input(gimbal.pitch_gravity_comp.amount);
    gimbal.pitch_gravity_comp_filter.low_calculate_callback();

    // gimbal.samples[0][0]  = 1;
    // gimbal.samples[1][0]  = gimbal.target_value.pitch_angle;
    // gimbal.samples[2][0]  = gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle;
    // gimbal.samples[3][0]  = gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle;

    // gimbal.params = gimbal.rls.update(gimbal.samples, gimbal.pitch_motor.get_now_torque());
    // gimbal.pitch_gravity_comp.factor[0] = gimbal.params[0][0];
    // gimbal.pitch_gravity_comp.factor[1] = gimbal.params[1][0];
    // gimbal.pitch_gravity_comp.factor[2] = gimbal.params[2][0];
    // gimbal.pitch_gravity_comp.factor[3] = gimbal.params[3][0];
}

/**
 * @brief 非线性静态补偿查表 (线性插值)
 * @param angle 目标角度 (rad)
 * @return 补偿扭矩 (Nm)
 */
// static float pitch_static_lookup(float angle)
// {
//     // 边界处理
//     if (angle <= pitch_ff_static_angle[0])
//         return pitch_ff_static_torque[0];
//     if (angle >= pitch_ff_static_angle[PITCH_FF_STATIC_NUM - 1])
//         return pitch_ff_static_torque[PITCH_FF_STATIC_NUM - 1];
    
//     // 线性插值
//     for (int i = 0; i < PITCH_FF_STATIC_NUM - 1; i++)
//     {
//         if (angle >= pitch_ff_static_angle[i] && angle < pitch_ff_static_angle[i + 1])
//         {
//             float t = (angle - pitch_ff_static_angle[i]) / (pitch_ff_static_angle[i + 1] - pitch_ff_static_angle[i]);
//             return pitch_ff_static_torque[i] + t * (pitch_ff_static_torque[i + 1] - pitch_ff_static_torque[i]);
//         }
//     }
//     return 0.0f;
// }

/**
 * @brief 更新云台前馈控制量
 * @note 更新云台的前馈控制量, 用于补偿系统的动态响应
 *       前馈策略:
 *         1. 动态前馈: Feedforward_Calculate 计算 K*θ + B*ω + J*α
 *         2. 速度前馈: Ref_dot 经增益后给角度环, 减小PID负担
 *         3. 静态补偿: 查表修正弹簧/重力的非线性残差
 *         4. 常数偏置: 补偿系统零位偏移
 */
// static void gimbal_update_feedforward(void)
// {
//     gimbal.pitch_ff_ref_lpf += PITCH_FF_REF_LPF_ALPHA *
//         (gimbal.target_value.pitch_angle - gimbal.pitch_ff_ref_lpf);
//     Feedforward_Calculate(&gimbal.pc_pitch_ffc, gimbal.pitch_ff_ref_lpf);
    
//     {
//         float raw_ff_omega = basic_math_constrain(
//             gimbal.pc_pitch_ffc.Ref_dot * PITCH_FF_VELOCITY_GAIN,
//             -PITCH_FF_OMEGA_MAX, PITCH_FF_OMEGA_MAX);
//         float dt = gimbal.pc_pitch_ffc.dt;
//         if (dt > 0.05f) dt = 0.015f;  // 安全钳位, 防止首次调用或长时间暂停后的dt异常
//         float max_delta = PITCH_FF_ACC_LIMIT * dt;
//         gimbal.feedforward_value.pitch_omega = basic_math_constrain(
//             raw_ff_omega,
//             gimbal.prev_pitch_ff_omega - max_delta,
//             gimbal.prev_pitch_ff_omega + max_delta);
//         gimbal.prev_pitch_ff_omega = gimbal.feedforward_value.pitch_omega;
//     }
    
//     float dynamic_ff = gimbal.pc_pitch_ffc.Output;
//     float coulomb_ff = (gimbal.pc_pitch_ffc.Ref_dot > 0.001f) ? PITCH_FF_COULOMB : 
//                        ((gimbal.pc_pitch_ffc.Ref_dot < -0.001f) ? -PITCH_FF_COULOMB : 0.0f);
//     float static_ff = pitch_static_lookup(gimbal.target_value.pitch_angle);
//     float raw_torque = dynamic_ff + coulomb_ff + static_ff + PITCH_FF_BIAS_C;
//     {
//         float dt = gimbal.pc_pitch_ffc.dt;
//         if (dt > 0.05f) dt = 0.015f;
//         float max_delta = 20.0f * dt;  // 20 Nm/s 速率限制
//         gimbal.feedforward_value.pitch_torque = basic_math_constrain(
//             raw_torque,
//             gimbal.prev_pitch_ff_torque - max_delta,
//             gimbal.prev_pitch_ff_torque + max_delta);
//         gimbal.prev_pitch_ff_torque = gimbal.feedforward_value.pitch_torque;
//     }

//     gimbal.yaw_small_ff_enc_lpf += YAW_SMALL_FF_ENC_LPF_ALPHA *
//         (gimbal.now_value.small_yaw_angle - gimbal.yaw_small_ff_enc_lpf);
//     gimbal.yaw_small_ff_delta_lpf += YAW_SMALL_FF_DELTA_LPF_ALPHA *
//         ((gimbal.target_value.small_yaw_angle - gimbal.now_value.small_yaw_angle)
//          - gimbal.yaw_small_ff_delta_lpf);
//     float yaw_small_ff_ref = gimbal.yaw_small_ff_delta_lpf + gimbal.yaw_small_ff_enc_lpf;
//     float dc_blocked = yaw_small_ff_ref - gimbal.yaw_small_ff_dc_mean;
//     gimbal.yaw_small_ff_dc_mean += 0.0003f * (yaw_small_ff_ref - gimbal.yaw_small_ff_dc_mean);
//     Feedforward_Calculate(&gimbal.pc_yaw_small_ffc, dc_blocked);
//     // 角速度前馈: 增益缩放 + 安全硬限幅 + 速率限制(平滑拐点冲击)
//     {
//         float raw_ff_omega = basic_math_constrain(
//             gimbal.pc_yaw_small_ffc.Ref_dot * YAW_SMALL_FF_VELOCITY_GAIN,
//             -YAW_SMALL_FF_OMEGA_MAX, YAW_SMALL_FF_OMEGA_MAX);
//         // 速率限制: 限制每帧变化量, 平滑三角波拐点处ff_omega方向突变
//         // v3(260916): 阈值 3.5 → 12.0 rad/s², 详见 YAW_SMALL_FF_RATE_LIMIT 注释
//         float dt = gimbal.pc_yaw_small_ffc.dt;
//         if (dt > 0.05f) dt = 0.015f;  // 安全钳位, 防止首次调用或长时间暂停后的dt异常
//         float max_delta = YAW_SMALL_FF_RATE_LIMIT * dt;
//         gimbal.feedforward_value.small_yaw_omega = basic_math_constrain(
//             raw_ff_omega,
//             gimbal.prev_small_yaw_ff_omega - max_delta,
//             gimbal.prev_small_yaw_ff_omega + max_delta);
//         gimbal.prev_small_yaw_ff_omega = gimbal.feedforward_value.small_yaw_omega;
//     }
//     float yaw_coulomb_ff = (gimbal.pc_yaw_small_ffc.Ref_dot > 0.001f) ? YAW_SMALL_FF_COULOMB : 
//                            ((gimbal.pc_yaw_small_ffc.Ref_dot < -0.001f) ? -YAW_SMALL_FF_COULOMB : 0.0f);
//     gimbal.feedforward_value.small_yaw_torque = gimbal.pc_yaw_small_ffc.Output + yaw_coulomb_ff + YAW_SMALL_FF_BIAS_C;
// }

/**
 * @brief 更新云台电机控制
 */
void gimbal_motor_calculate(void)
{
    if (gimbal.now_method == Gimbal_Method_Zero_Force)  // 零力矩模式
    {
        gimbal.yaw_small_motor.set_target_torque(0.0f);
        gimbal.pitch_motor.set_control_torque(0.0f);
    }

    else if (gimbal.now_method == Gimbal_Method_Manual) // 遥控器控制模式
    {
        // 更新云台前馈量
        // gimbal_update_feedforward();

/*----------------------------------------------小yaw电机rc--------------------------------------------------------*/
        // gimbal.yaw_small_motor.set_target_angle(gimbal.target_value.small_yaw_angle);     // 设置小yaw电机的目标角度
        // gimbal.yaw_small_motor.set_feedforward_omega(gimbal.feedforward_value.small_yaw_omega);
        // gimbal.yaw_small_motor.set_feedforward_torque(gimbal.feedforward_value.small_yaw_torque);
        // gimbal.yaw_small_motor.set_filter_angle(gimbal.now_value.small_yaw_angle);        // 设置小yaw电机的滤波角度
        // gimbal.yaw_small_motor.set_filter_omega(gimbal.now_value.small_yaw_omega);        // 设置小yaw电机的滤波角速度

        // gimbal.rc_yaw_small_pid.pid_angle.set_target(gimbal.yaw_small_plan_output.angle_rad);
        gimbal.rc_yaw_small_pid.pid_angle.set_target(gimbal.target_value.small_yaw_angle);
        gimbal.rc_yaw_small_pid.pid_angle.set_now(gimbal.now_value.small_yaw_angle);
        gimbal.rc_yaw_small_pid.pid_angle.pid_calculate_callback();
        gimbal.target_value.small_yaw_omega = gimbal.rc_yaw_small_pid.pid_angle.get_out();

        // gimbal.rc_yaw_small_pid.pid_omega.set_target(gimbal.target_value.small_yaw_omega);
        gimbal.rc_yaw_small_pid.pid_omega.set_target(gimbal.target_value.small_yaw_omega + gimbal.yaw_small_plan_rate_lpf * 0.5f);
        gimbal.rc_yaw_small_pid.pid_omega.set_now(gimbal.now_value.small_yaw_omega);
        gimbal.rc_yaw_small_pid.pid_omega.pid_calculate_callback();
        gimbal.yaw_small_motor.set_target_torque(basic_math_constrain(gimbal.rc_yaw_small_pid.pid_omega.get_out() + gimbal.feedforward_small_yaw_torque, -YAW_SMALL_TORQUE_LIMIT, YAW_SMALL_TORQUE_LIMIT));
        // gimbal.yaw_small_motor.set_target_torque(gimbal.rc_yaw_small_pid.pid_omega.get_out());

/*----------------------------------------------pitch电机rc--------------------------------------------------------*/
        // pitch电机使用外部PID闭环控制，计算角度环和角速度环的输出扭矩
        gimbal.rc_pitch_pid.pid_angle.set_target(basic_math_constrain(gimbal.target_value.pitch_angle, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET));
        gimbal.rc_pitch_pid.pid_angle.set_now(gimbal.now_value.pitch_angle);
        gimbal.rc_pitch_pid.pid_angle.pid_calculate_callback();
        gimbal.target_value.pitch_omega = gimbal.rc_pitch_pid.pid_angle.get_out() + gimbal.pitch_plan_rate_lpf * 0.5f;

        gimbal.rc_pitch_pid.pid_omega.set_target(gimbal.target_value.pitch_omega);
        gimbal.rc_pitch_pid.pid_omega.set_now(gimbal.now_value.pitch_omega);
        gimbal.rc_pitch_pid.pid_omega.pid_calculate_callback();
        gimbal.pitch_motor.set_control_torque(basic_math_constrain(-gimbal.rc_pitch_pid.pid_omega.get_out() - gimbal.feedforward_pitch_torque, -PITCH_TORQUE_LIMIT, PITCH_TORQUE_LIMIT));
    }

    else if (gimbal.now_method == Gimbal_Method_Auto)   // 自瞄控制模式
    {
/*----------------------------------------------小yaw电机pc--------------------------------------------------------*/
        // 更新云台前馈量
        // gimbal_update_feedforward();

/*----------------------------------------------小yaw电机pc--------------------------------------------------------*/

        gimbal.pc_yaw_small_pid.pid_angle.set_target(gimbal.target_value.small_yaw_angle);
        gimbal.pc_yaw_small_pid.pid_angle.set_now(gimbal.now_value.small_yaw_angle);
        gimbal.pc_yaw_small_pid.pid_angle.pid_calculate_callback();
        // gimbal.target_value.small_yaw_omega = gimbal.pc_yaw_small_pid.pid_angle.get_out() + gimbal.feedforward_value.small_yaw_omega;
        gimbal.target_value.small_yaw_omega = gimbal.pc_yaw_small_pid.pid_angle.get_out() + gimbal.yaw_small_plan_rate_lpf * 0.5f;
        // gimbal.target_value.small_yaw_omega = gimbal.pc_yaw_small_pid.pid_angle.get_out();

        gimbal.pc_yaw_small_pid.pid_omega.set_target(gimbal.target_value.small_yaw_omega);
        gimbal.pc_yaw_small_pid.pid_omega.set_now(gimbal.now_value.small_yaw_omega);
        gimbal.pc_yaw_small_pid.pid_omega.pid_calculate_callback();
        // gimbal.yaw_small_motor.set_target_torque(basic_math_constrain(gimbal.pc_yaw_small_pid.pid_omega.get_out() + gimbal.feedforward_value.small_yaw_torque, -YAW_SMALL_TORQUE_LIMIT, YAW_SMALL_TORQUE_LIMIT));
        gimbal.yaw_small_motor.set_target_torque(basic_math_constrain(gimbal.pc_yaw_small_pid.pid_omega.get_out() + gimbal.feedforward_small_yaw_torque, -YAW_SMALL_TORQUE_LIMIT, YAW_SMALL_TORQUE_LIMIT));
        // gimbal.yaw_small_motor.set_target_torque(basic_math_constrain(gimbal.pc_yaw_small_pid.pid_omega.get_out(), -YAW_SMALL_TORQUE_LIMIT, YAW_SMALL_TORQUE_LIMIT));

/*----------------------------------------------pitch电机pc--------------------------------------------------------*/
        // pitch电机使用外部PID闭环控制，计算角度环和角速度环的输出扭矩
        gimbal.pc_pitch_pid.pid_angle.set_target(basic_math_constrain(gimbal.target_value.pitch_angle, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET));
        gimbal.pc_pitch_pid.pid_angle.set_now(gimbal.now_value.pitch_angle);
        // gimbal.pc_pitch_pid.pid_angle.set_now(gimbal.filter_imu_pitch_angle.get_out());
        gimbal.pc_pitch_pid.pid_angle.pid_calculate_callback();
        gimbal.target_value.pitch_omega = gimbal.pc_pitch_pid.pid_angle.get_out() + gimbal.pitch_plan_rate_lpf * 0.5f;
        // gimbal.target_value.pitch_omega = gimbal.pc_pitch_pid.pid_angle.get_out();
        // gimbal.target_value.pitch_omega = gimbal.pc_pitch_pid.pid_angle.get_out();

        gimbal.pc_pitch_pid.pid_omega.set_target(basic_math_constrain(gimbal.target_value.pitch_omega, -1.8f, 1.8f));
        gimbal.pc_pitch_pid.pid_omega.set_now(gimbal.now_value.pitch_omega);
        // gimbal.pc_pitch_pid.pid_omega.set_now(gimbal.filter_pitch_pitch_omega.get_out());
        gimbal.pc_pitch_pid.pid_omega.pid_calculate_callback();
        gimbal.pitch_motor.set_control_torque(basic_math_constrain(-gimbal.pc_pitch_pid.pid_omega.get_out() - gimbal.feedforward_pitch_torque, -PITCH_TORQUE_LIMIT, PITCH_TORQUE_LIMIT));
        // gimbal.pitch_motor.set_control_torque(-gimbal.pc_pitch_pid.pid_omega.get_out());
        // gimbal.pitch_motor.set_control_torque(-gimbal.pc_pitch_pid.pid_omega.get_out());
    }

    gimbal.yaw_small_motor.external_calculate_callback();                       // 执行小yaw电机的外部PID闭环计算和控制
    bsp_can_1_to_4_transmit(&hfdcan1, 0x1fe, can1_0x1fe_tx_data, Can_Transmit_Frequency_Div1);  // 发送小yaw电机的CAN数据
    gimbal.pitch_motor.calculate_callback();

    gimbal.last_method = gimbal.now_method;
    gimbal.last_mode = gimbal.now_mode;
}

/**
 * @brief 云台测试任务用于调试和验证云台电机参数
 * @note 该任务在实际应用中可用于验证云台电机的控制性能和参数设置，确保云台在不同控制模式下的稳定性和响应速度。
 */
void gimbal_motor_parameter_test()
{
    // 首帧标志: yaw_small_plan_dt_cnt 尚未被 bsp_dwt_get_deltat32 更新过
    const bool first_call = (gimbal.yaw_small_plan_dt_cnt == 0u);

    // 规划器更新周期: 用实测dt替代写死的0.001f
    // 写死dt会让 accel = Δrate/dt 的放大倍数不可控, 也是量化噪声被放大的原因之一
    // float dt = bsp_dwt_get_deltat32(&gimbal.yaw_small_plan_dt_cnt);
    // if (!(dt >= yaw_small_track_plan_config.dt_min_s &&
    //       dt <= yaw_small_track_plan_config.dt_max_s))
    // {
    //     dt = 0.001f;    // 首次调用/异常周期兜底, 不跨大间隔积分
    // }

    dt_y = bsp_dwt_get_deltat32(&gimbal.yaw_small_plan_dt_cnt);
    if (!(dt_y >= yaw_small_track_plan_config.dt_min_s &&
          dt_y <= yaw_small_track_plan_config.dt_max_s))
    {
        dt_y = 0.001f;    // 首次调用/异常周期兜底, 不跨大间隔积分
    }

    {
        const float alpha = 0.03f;
        if (first_call)
        {
            /* 首帧把规划器对齐到当前目标, 避免从 init 时的 0 起跳造成一次大冲击 */
            gimbal.yaw_small_plan_target_lpf = gimbal.target_value.small_yaw_angle;
            reference_track_plan_reset(&gimbal.yaw_small_track_plan,
                                      gimbal.target_value.small_yaw_angle);
        }
        gimbal.yaw_small_plan_target_lpf += alpha * (gimbal.target_value.small_yaw_angle - gimbal.yaw_small_plan_target_lpf);
    }

    /* 2. 轨迹规划: 用滤波后的目标 + 实测dt */
    reference_track_plan_update(&gimbal.yaw_small_track_plan, gimbal.yaw_small_plan_target_lpf, dt_y, &gimbal.yaw_small_plan_output);

    /* 3. 规划角速度低通: 速度环前馈只用规划器算出的 rate, 该 rate 在误差超过
     *    max_accel/gain 后就是斜率 ±max_accel 的折线, 拐角处加速度阶跃会直接
     *    经速度环 k_p 放大成力矩冲击。这里先做一阶低通圆化拐角, 再送速度环。
     *    时间常数采用 dt/(tau+dt) 形式, 与采样周期解耦。 */
    {
        const float rate_lpf_alpha = dt_y / (YAW_SMALL_PLAN_RATE_LPF_TAU_S + dt_y);
        gimbal.yaw_small_plan_rate_lpf += rate_lpf_alpha * (gimbal.yaw_small_plan_output.rate_rad_s - gimbal.yaw_small_plan_rate_lpf);
    }

    /* 4. 前馈力矩: 用真实物理系数, 并抑制二阶微分噪声
     *    tau = J*accel + B*rate + Fc*tanh(rate/eps)
     *    - J 由实测辨识约 0.0018 kg·m^2 (旧值 0.01 偏大约 5.7 倍)
     *    - epsilon 0.08 -> 0.30 且加低速死区, 避免 tanh 退化成继电器特性
     *    - accel 是二阶微分, 在1kHz下对量化极敏感, 必须低通
     */
    {
        const float J_ff  = 0.0118676918f;    // 等效惯量 Nm/(rad/s^2)
        const float B_ff  = 0.00934354216f;    // 阻尼 Nm/(rad/s)
        const float Fc_ff = 0.0246254914f;    // 库仑摩擦 Nm
        const float EPS   = 0.08f;      // 摩擦速度过渡尺度 rad/s

        const float rate = gimbal.yaw_small_plan_output.rate_rad_s;

        /* 加速度项低通: ~8Hz, 抑制量化台阶被二阶微分放大 */
        gimbal.yaw_small_ff_acc_lpf += 0.05f * (gimbal.yaw_small_plan_output.accel_rad_s2 - gimbal.yaw_small_ff_acc_lpf);

        /* 库仑项: 低速死区, 避免速度过零时符号翻转造成方波抖动 */
        float coulomb = 0.0f;
        if (fabsf(rate) > 0.05f)
        {
            coulomb = Fc_ff * tanhf(rate / EPS);
        }

        float ff_raw = J_ff * gimbal.yaw_small_ff_acc_lpf
                     + B_ff * rate
                     + coulomb;

        /* 输出低通兜底: ~5Hz, 削掉残余高频尖峰(不改变低频幅值) */
        gimbal.yaw_small_ff_torque_lpf += 0.03f * (ff_raw - gimbal.yaw_small_ff_torque_lpf);

        gimbal.feedforward_small_yaw_torque = gimbal.yaw_small_ff_torque_lpf;
    }

    // gimbal.filter_yaw_omega.vector_z[0][0] = gimbal.yaw_small_motor.get_now_angle();
    // gimbal.filter_yaw_omega.vector_z[1][0] = gimbal.yaw_small_motor.get_now_omega();
    // gimbal.filter_yaw_omega.tim_predict_periodelapsedcallback();
    // gimbal.filter_yaw_omega.tim_update_periodelapsedcallback();
    // gimbal.filter_angle = gimbal.filter_yaw_omega.vector_x[0][0];
    // gimbal.filter_omega = gimbal.filter_yaw_omega.vector_x[1][0];
}

/**
 * @brief pitch轴轨迹规划与前馈力矩计算(与小yaw同结构)
 * @note 1. 目标预滤波: 抑制PC零阶保持/目标量化台阶
 *       2. 轨迹规划: 限速/限加速度, 输出 rate/accel 供前馈使用
 *       3. 规划角速度低通: 圆化 bang-bang 拐角
 *       4. 力矩前馈: tau = J*a + B*w + Fc*tanh(w/eps) + A*sin(th) + C*cos(th)
 *          其中 A/C 为辨识器输出的重力项系数, th 取规划器输入的参考角。
 *       约定: 前馈量定义在"增大pitch角"方向, 与PID输出同向, 送电机时整体取负。
 */
void gimbal_motor_parameter_pitch_test()
{
    // 首帧标志: pitch_plan_dt_cnt 尚未被 bsp_dwt_get_deltat32 更新过
    const bool first_call = (gimbal.pitch_plan_dt_cnt == 0u);

    float dt = bsp_dwt_get_deltat32(&gimbal.pitch_plan_dt_cnt);
    if (!(dt >= pitch_track_plan_config.dt_min_s && dt <= pitch_track_plan_config.dt_max_s))
    {
        dt = 0.001f;    // 首次调用/异常周期兜底, 不跨大间隔积分
    }

    /* 1. 规划器输入预滤波 */
    {
        const float alpha = 0.03f;
        if (first_call)
        {
            /* 首帧把规划器对齐到当前目标, 避免从 init 时的 0 起跳造成一次大冲击 */
            gimbal.pitch_plan_target_lpf = gimbal.target_value.pitch_angle;
            reference_track_plan_reset(&gimbal.pitch_track_plan, gimbal.target_value.pitch_angle);
        }
        gimbal.pitch_plan_target_lpf += alpha * (gimbal.target_value.pitch_angle - gimbal.pitch_plan_target_lpf);
    }

    /* 2. 轨迹规划: 用滤波后的目标 + 实测dt */
    reference_track_plan_update(&gimbal.pitch_track_plan, gimbal.pitch_plan_target_lpf, dt, &gimbal.pitch_plan_output);

    /* 3. 规划角速度低通: 拐角圆化后再送速度环 */
    {
        const float rate_lpf_alpha = dt / (PITCH_PLAN_RATE_LPF_TAU_S + dt);
        gimbal.pitch_plan_rate_lpf += rate_lpf_alpha * (gimbal.pitch_plan_output.rate_rad_s - gimbal.pitch_plan_rate_lpf);
    }

    /* 4. 前馈力矩: tau = J*a + B*w + Fc*tanh(w/eps) + 重力项 */
    {
        const float rate = gimbal.pitch_plan_output.rate_rad_s;

        /* 加速度项低通: ~8Hz, 抑制量化台阶被二阶微分放大 */
        gimbal.pitch_ff_acc_lpf += PITCH_FF_ACC_LPF_ALPHA * (gimbal.pitch_plan_output.accel_rad_s2 - gimbal.pitch_ff_acc_lpf);

        /* 库仑项: 低速死区, 避免速度过零时符号翻转造成方波抖动 */
        float coulomb = 0.0f;
        if (fabsf(rate) > PITCH_FF_RATE_DEADBAND)
        {
            coulomb = PITCH_FF_FC * tanhf(rate / PITCH_FF_EPS);
        }

        /* 4.1 动态项(随参考速率/加速度变化), 单独限幅 */
        float ff_dynamic = PITCH_FF_J * gimbal.pitch_ff_acc_lpf
                         + PITCH_FF_B * rate
                         + coulomb;
        ff_dynamic = basic_math_constrain(ff_dynamic, -PITCH_FF_TORQUE_MAX, PITCH_FF_TORQUE_MAX);

        /* 4.2 重力项 A*sin(th) + C*cos(th)
         *   - 相位角取"规划器输入参考角"而不是实测 IMU pitch: 重力补偿是开环通道,
         *     用参考角不吃量化/噪声, 也不会把实测值经前馈再灌回反馈环;
         *     pitch_plan_target_lpf 已做过一阶低通, 本身足够平滑, 无需再加滤波。
         *   - 符号: 辨识回归是 tau_motor(电机反馈力矩) = ... + A*sin + C*cos;
         *     本函数所有前馈量定义在"增大pitch角"方向(送电机时整体取负),
         *     所以重力项要整体取负再加进来。旁证: 辨识出的常值项 C = -0.697 Nm,
         *     与手动模式里手工补的那 -0.65 Nm 偏置同号同量级(该偏置是直接加在
         *     set_control_torque 上的), 说明 A/C 与电机指令侧同号 => 前馈侧取负。
         *   - 上机校验: 让 pitch 在 0 附近缓慢扫动, 观察 feedforward_pitch_torque
         *     的符号是否与"托住负载所需方向"一致; 若明显反向, 只改这一个负号。 */
        const float gravity_angle = gimbal.pitch_plan_target_lpf;
        float ff_gravity = -(PITCH_FF_A * sinf(gravity_angle) + PITCH_FF_C * cosf(gravity_angle));
        ff_gravity = basic_math_constrain(ff_gravity, -PITCH_FF_GRAVITY_MAX, PITCH_FF_GRAVITY_MAX);

        /* 4.3 合成 + 总兜底: 防止识别参数偏差把异常大前馈直接注入电机的 */
        float ff_raw = ff_dynamic + ff_gravity;
        ff_raw = basic_math_constrain(ff_raw, -PITCH_FF_TOTAL_MAX, PITCH_FF_TOTAL_MAX);

        /* 输出低通兜底: ~5Hz, 削掉残余高频尖峰(不改变低频幅值) */
        gimbal.pitch_ff_torque_lpf += PITCH_FF_TORQUE_LPF_ALPHA * (ff_raw - gimbal.pitch_ff_torque_lpf);

        gimbal.feedforward_pitch_torque = gimbal.pitch_ff_torque_lpf;
    }
}

extern "C" void gimbal_task(void *argument)
{
    // 初始化云台
    gimbal_init();
    osDelay(1000);

    for(;;)
    {
        // 更新云台测量值
        gimbal_update_measure();

        // 更新云台控制模式
        gimbal_update_mode();

        // 更新云台决策源数据
        gimbal_robot_decision_source_update();

        // 更新云台控制决策
        gimbal_control_decision();

        // 更新云台轨迹规划与前馈力矩
        // 必须放在 control_decision 之后(拿到本周期的 target_value)、
        // 且放在 motor_calculate 之前(本周期即可使用新规划量), 避免多一个周期延迟
        gimbal_motor_parameter_test();

        // pitch轴轨迹规划与前馈力矩(与小yaw同结构), 与yaw一样夹在决策与电机计算之间
        gimbal_motor_parameter_pitch_test();

        // 更新云台前馈量
        // gimbal_update_feedforward();

        gimbal_update_compensation();

        /* 旁路辨识只读取反馈，必须保持在控制链路之外。 */
        gimbal_identification_update_all();

        // 更新云台电机控制
        gimbal_motor_calculate();

        // 云台测试任务
        // gimbal_motor_parameter_test();

        // 任务执行频率1000Hz
        osDelay(1);
    }
}
