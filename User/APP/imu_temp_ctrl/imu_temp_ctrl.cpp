/**
 * @file imu_temp_ctrl.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 板载 BMI088 IMU: SPI 采样 → 恒温控制 → 零漂补偿 → 四元数 EKF 姿态解算
 * @date 2026-10-26
 * @version 0.8
 * @copyright HNAU-Robot (c) 2026
 */

#include "main.h"
#include "cmsis_os.h"

#include "BMI088driver.h"
#include "QuaternionEKF.h"
#include "imu_temp_ctrl.h"
#include "pid_fuse.h"
#include "basic_math.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"

#include <math.h>

/*------------------------------ 采样与调度 ------------------------------*/

// 无法取得有效 DRDY 周期时使用的初始化/恢复周期, 单位 s
#define IMU_DEFAULT_DT_S                0.001f
// BMI088 陀螺仪配置为 1 kHz, 对实际 dt 做上下限保护, 防止丢中断污染 EKF
#define IMU_DT_MIN_S                    0.0005f
#define IMU_DT_MAX_S                    0.0030f
// DRDY 超时保护. 连续超时后进入 Init_Fail, 由任务周期性重试 BMI088
#define IMU_DRDY_TIMEOUT_TICKS          10U
#define IMU_DRDY_TIMEOUT_LIMIT          3U
#define IMU_DRDY_EVENT_FLAG             (1U << 0)

// 温度读取与温控周期 = 采样周期 x 分频 = 10ms/100Hz
#define IMU_TEMP_CTRL_DIVIDER           10U

/*------------------------------ 姿态解算(EKF) ------------------------------*/

// EKF 参数,过程噪声/量测噪声/渐消因子/加速度低通时间常数
#define IMU_EKF_PROCESS_NOISE_Q1        10.0f          // 四元数过程噪声
#define IMU_EKF_PROCESS_NOISE_Q2        0.001f         // 陀螺零偏过程噪声
#define IMU_EKF_MEASURE_NOISE_R         10000000.0f    // 加速度计量测噪声
#define IMU_EKF_FADING_LAMBDA           1.0f           // 渐消因子, 1 表示不渐消
#define IMU_EKF_ACCEL_LPF_S             0.001f         // 加速度低通时间常数, 单位 s

/*------------------------------ 传感器初始化 ------------------------------*/

#define IMU_INIT_RETRY_TIMES            3U             // 单次初始化重试次数
#define IMU_INIT_RETRY_DELAY_MS         20U            // 重试间隔, 单位 ms
#define IMU_INIT_RETRY_PERIOD_TICKS     500U           // 初始化失败后的重试周期(=500ms)
#define IMU_READ_FAIL_LIMIT             100U           // 连续读取失败上限(100ms), 超过判定传感器离线

/*------------------------------ 陀螺零漂 ------------------------------*/

#define IMU_GYRO_CALIB_SAMPLES          1000U          // 标定有效帧数
#define IMU_GYRO_CALIB_MAX_RATE         0.15f          // rad/s, 超过认为车体在动, 该帧丢弃
#define IMU_GYRO_CALIB_TIMEOUT_MS       5000U          // 标定超时, 超时后按已采样本结算
#define IMU_GYRO_CALIB_MIN_SAMPLES      100U           // 超时结算所需的最少有效帧
#define IMU_GYRO_CALIB_REJECT_LIMIT     2000U          // 累计被丢弃帧数上限, 超过则放弃标定

// 重力加速度
#define IMU_GRAVITY_ACCEL               9.80665f       // 重力加速度, m/s^2

// 静止判定与残差修正参数
#define IMU_STILL_GYRO_THRESHOLD        0.020f         // rad/s, 静止判定的角速度阈值
#define IMU_STILL_GYRO_LPF_S            0.20f          // s, 静止判定用角速度低通时间常数(滤除高频噪声)
#define IMU_STILL_ACCEL_TOL             0.5f           // m/s^2, 静止判定加速度模长偏差

#define IMU_STILL_ACCEL_LPF_S           0.20f          // s, 静止判定用加速度模长低通时间常数
#define IMU_STILL_CONFIRM_SAMPLES       500U           // 连续满足静止的帧数(500ms 去抖)
#define IMU_STILL_TRIM_GAIN             0.00005f       // 残差积分增益
#define IMU_YAW_TRIM_LIMIT              0.02f          // rad/s, 残差修正上限

#define IMU_TRIM_SEED_SAMPLES           500U           // 静止确认后再平均 500 帧(0.5s)定标
#define IMU_TRIM_SEED_RATE_MAX          0.15f          // rad/s, 单帧角速度模长上限

/*------------------------------ 陀螺野值抑制 ------------------------------*/

// 野值抑制策略: 通过限制角速度的变化率和绝对值来识别并剔除野值
#define IMU_GYRO_SLEW_LIMIT_RAD_S2      10000.0f       // rad/s^2, 1ms 内允许的最大角速度变化(10 rad/s)
#define IMU_GYRO_ABS_LIMIT_RAD_S        36.0f          // rad/s, 超出 BMI088 ±2000dps 量程即判野值

/*------------------------------ 温度控制 ------------------------------*/

#define IMU_TEMP_REF_C                  50.0f          // 目标温度, ℃
#define IMU_TEMP_VALID_MIN_C            (-20.0f)       // 温度有效下限, ℃
#define IMU_TEMP_VALID_MAX_C            100.0f         // 温度有效上限, ℃
#define IMU_TEMP_OVERHEAT_C             60.0f          // 过温保护, ℃
#define IMU_TEMP_STABLE_BAND_C          0.8f           // 判定温度稳定的带宽, ℃
#define IMU_TEMP_STABLE_MS              3000U          // 温度稳定持续时长, ms

#define IMU_TEMP_PID_KP                 180.0f
#define IMU_TEMP_PID_KI                 1.2f
#define IMU_TEMP_PID_KD                 150.0f
#define IMU_TEMP_PID_OUT_MAX            1500.0f        // 加热输出上限(与 TIM3 ARR 同量纲)
#define IMU_TEMP_PID_INTEGRAL_LIMIT     500.0f        // 积分限幅, 防止积分风up
#define IMU_HEATER_PWM_PERIOD_COUNTS    10000.0f       // TIM3 ARR + 1, 用于把输出换算成占空比

volatile imu_data_resolve_t imu_frame = {0};

// 温度PID控制器
static pid_t temperature_pid = {0};
static pid_config temperature_ctrl_config = {0};

// 加热电阻 PWM(TIM3_CH4), 上电占空比为 0
static pwm_class heater_pwm;
// 最近一次下发给 TIM3_CH4 的占空比, 用于避免每帧重复写 CCR
static float heater_duty_cmd = 0.0f;

// 陀螺零漂补偿值(参数校准), rad/s
static float gyro_bias[3] = {0.001526523f, 0.00049747858f, 0.003300000f};

static imu_status_e imu_status = Imu_Status_Uninit;
static uint32_t init_retry_ticks = 0;
static uint32_t read_fail_count = 0;
static uint32_t drdy_timeout_count = 0;
static osThreadId_t imu_task_handle = 0;
static volatile uint32_t gyro_drdy_sequence = 0;
static volatile float gyro_drdy_dt = 0.0f;
static uint32_t gyro_drdy_dwt_last = 0;
static uint8_t gyro_drdy_dwt_started = 0;

// EKF 矩阵缓冲区是否分配成功(FreeRTOS 堆不足时为 0)
static uint8_t ekf_ready = 0U;
// 姿态输出是否可用(与传感器数据有效性解耦: EKF 异常不应连带关闭温控)
static uint8_t attitude_valid = 0U;

// 运行时静止残差零漂修正
static float yaw_bias_trim = 0.0f;
static uint32_t still_count = 0;
// 静止判定专用的低通角速度(抑制采样噪声, 避免噪声把 still_count 持续清零)
static float still_gyro_lpf[3] = {0.0f, 0.0f, 0.0f};
static uint8_t still_gyro_lpf_started = 0U;
// 静止判定专用的低通加速度模长(同理, 抑制瞬时跳变)
static float still_accel_lpf = 0.0f;
static uint8_t still_accel_lpf_started = 0U;

// 上电/重连后的静止残差快速定标
static float trim_seed_sum = 0.0f;
static uint32_t trim_seed_n = 0U;
static uint8_t trim_seeded = 0U;

// 陀螺野值抑制: 上一拍原始角速度 + 待确认标志(逐轴)
static float gyro_prev[3] = {0.0f, 0.0f, 0.0f};
static uint8_t gyro_prev_valid = 0U;
static uint8_t gyro_suspect[3] = {0U, 0U, 0U};

#if IMU_GYRO_AUTO_CALIB_ENABLE
static uint32_t temp_stable_ticks = 0;
static float calib_sum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t calib_count = 0;
static uint32_t calib_reject = 0;
static uint32_t calib_elapsed_ms = 0;
#endif

static void imu_set_status(imu_status_e status);
static void imu_heater_safe_off(void);
static void imu_sensor_init(void);
static void imu_temperature_ctrl_step(void);
static uint8_t imu_attitude_step(const float *gyro, const float *accel);
static void INS_Init(void);
static void INS_Task(float sample_dt);

#if IMU_GYRO_AUTO_CALIB_ENABLE
static void imu_gyro_calib_start(void);
static void imu_gyro_calib_step(const float *gyro);
#endif

/**
 * @brief 更新工作状态并同步到观测结构体
 */
static void imu_set_status(imu_status_e status)
{
    imu_status = status;
    imu_frame.status = status;
}

/**
 * @brief 读取当前工作状态
 */
static imu_status_e imu_get_status(void)
{
    return imu_status;
}

static uint8_t imu_array_is_finite(const float *values, uint8_t count)
{
    for (uint8_t i = 0U; i < count; i++)
    {
        if (!isfinite(values[i]))
        {
            return 0U;
        }
    }
    return 1U;
}

static float imu_limit_dt(float dt)
{
    if (!isfinite(dt))
    {
        return IMU_DEFAULT_DT_S;
    }

    if (dt < IMU_DT_MIN_S)
    {
        return IMU_DT_MIN_S;
    }

    if (dt > IMU_DT_MAX_S)
    {
        return IMU_DT_MAX_S;
    }

    return dt;
}

/**
 * @brief Force the heater to its safe state without stopping the timer.
 * @note  The PWM class applies the final TIM3_CH4 hardware-side clamp.
 *        1kHz 采样下 data_valid=0 会频繁进这里, 占空比已经是 0 时跳过寄存器写入.
 */
static void imu_heater_safe_off(void)
{
    temperature_pid.error_sum = 0.0f;
    temperature_pid.output = 0.0f;

    if ((imu_frame.heater_out == 0.0f) && (heater_duty_cmd == 0.0f))
    {
        return;
    }

    imu_frame.heater_out = 0.0f;
    heater_duty_cmd = 0.0f;
    heater_pwm.set_duty_ration(0.0f);
}

/**
 * @brief 陀螺零漂标定开始, 清空累加器
 */
#if IMU_GYRO_AUTO_CALIB_ENABLE
static void imu_gyro_calib_start(void)
{
    calib_sum[0] = 0.0f;
    calib_sum[1] = 0.0f;
    calib_sum[2] = 0.0f;
    calib_count = 0;
    calib_reject = 0;
    calib_elapsed_ms = 0;
}
#endif

/**
 * @brief 陀螺零漂标定单步
 * @param gyro 原始角速度, rad/s
 * @note  丢弃"车体在动"的帧, 避免把运动角速度统计成零漂;
 *        累计被丢弃帧数超过上限或超时后, 用已采样本(或默认值)结算并进入解算状态
 */
#if IMU_GYRO_AUTO_CALIB_ENABLE
static void imu_gyro_calib_step(const float *gyro)
{
    const float norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);

    calib_elapsed_ms++;

    if (norm > IMU_GYRO_CALIB_MAX_RATE)
    {
        if (++calib_reject >= IMU_GYRO_CALIB_REJECT_LIMIT)
        {
            // 长时间被扰动: 放弃本次标定, 沿用默认补偿值
            imu_set_status(Imu_Status_Running);
        }
        return;
    }

    calib_sum[0] += gyro[0];
    calib_sum[1] += gyro[1];
    calib_sum[2] += gyro[2];
    calib_count++;

    if ((calib_count >= IMU_GYRO_CALIB_SAMPLES) ||
        ((calib_elapsed_ms >= IMU_GYRO_CALIB_TIMEOUT_MS) && (calib_count >= IMU_GYRO_CALIB_MIN_SAMPLES)))
    {
        const float inv_count = 1.0f / (float)calib_count;
        for (uint8_t i = 0U; i < 3U; i++)
        {
            gyro_bias[i] = calib_sum[i] * inv_count;
        }
        imu_set_status(Imu_Status_Running);
    }
}
#endif

/**
 * @brief BMI088 初始化(带重试); 失败时置 Init_Fail, 由任务周期重试
 */
static void imu_sensor_init(void)
{
    imu_heater_safe_off();

    for (uint8_t retry = 0U; retry < IMU_INIT_RETRY_TIMES; retry++)
    {
        if (BMI088_init() == BMI088_NO_ERROR)
        {
            // 掉线恢复后必须丢弃旧的姿态状态: 传感器已重新软复位, 旧四元数/圈数不再连续
            IMU_QuaternionEKF_Reset();
            still_count = 0U;
            still_gyro_lpf_started = 0U;
            still_accel_lpf_started = 0U;
            // 软复位后零偏会变, 需要重新做静止残差定标; 野值抑制的上一拍基准同样作废
            trim_seeded = 0U;
            trim_seed_sum = 0.0f;
            trim_seed_n = 0U;
            gyro_prev_valid = 0U;
            gyro_suspect[0] = 0U;
            gyro_suspect[1] = 0U;
            gyro_suspect[2] = 0U;
            imu_frame.data_valid = 1U;
#if IMU_GYRO_AUTO_CALIB_ENABLE
            temp_stable_ticks = 0U;
            imu_set_status(Imu_Status_Wait_Temp);
#else
            imu_set_status(Imu_Status_Running);
#endif
            return;
        }
        osDelay(IMU_INIT_RETRY_DELAY_MS);
    }

    imu_frame.data_valid = 0U;
    imu_set_status(Imu_Status_Init_Fail);
}

/**
 * @brief 温度 PID 控制(100Hz), 输出换算为加热 PWM 占空比
 * @note  1) 单向执行器(只能加热): 输出被压到 0 时撤销本拍积分, 防止过度积负造成降温后加热迟滞
 *        2) 条件积分抗饱和: 输出顶到上限时同样撤销本拍积分. 升温阶段误差很大,
 *           若不冻结积分, Ki 会在一两秒内把积分推到限幅值, 之后必然大超调
 *        3) 传感器无效/过温时立即断开加热, 并清积分(安全优先)
 */
static void imu_temperature_ctrl_step(void)
{
    const float temperature = imu_frame.temperature;

    if ((imu_frame.data_valid == 0U) || (temperature < IMU_TEMP_VALID_MIN_C) ||
        (temperature > IMU_TEMP_VALID_MAX_C) || (temperature > IMU_TEMP_OVERHEAT_C))
    {
        imu_heater_safe_off();
        return;
    }

    const float error_sum_before = temperature_pid.error_sum;
    float out = pid_calc(&temperature_pid, imu_frame.ref_temperature, temperature);

    // NaN/Inf 无法被 basic_math_constrain 拦住(NaN 与任何值比较都为假), 必须单独判掉
    if (!isfinite(out))
    {
        imu_heater_safe_off();
        return;
    }

    if (out > IMU_TEMP_PID_OUT_MAX)
    {
        // 输出饱和: 撤销本拍积分累加, 防止升温阶段积分饱和(条件积分)
        temperature_pid.error_sum = error_sum_before;
        out = IMU_TEMP_PID_OUT_MAX;
    }
    else if (out <= 0.0f)
    {
        // 偏热: 撤销本拍积分, 避免积分越积越负, 降温后长时间不加热
        temperature_pid.error_sum = error_sum_before;
        out = 0.0f;
    }

    // pid_calc 的 PID_POSITION 分支不做输出限幅, 这里必须自己限幅,
    temperature_pid.output = out;

    imu_frame.heater_out = out;
    heater_duty_cmd = out / IMU_HEATER_PWM_PERIOD_COUNTS;
    heater_pwm.set_duty_ration(heater_duty_cmd);
}

/**
 * @brief 陀螺野值抑制
 * @param gyro 原始角速度, rad/s, 就地修正
 * @note  单帧 SPI 位错/DRDY 竞争会给出几百~上千 °/s 的假角速度.
 *        EKF 对 z 轴没有绝对观测, 一次野值就把 yaw 永久推偏且不可恢复, 因此必须在此拦截.
 *        判据为"相邻帧变化率"而不是绝对量程: 真实机动连续两帧都会远离上一拍值, 于是被放行;
 *        孤立野值下一帧即回到正常值, 会被完全剔除. 真实阶跃只损失 1 帧(1ms)积分.
 */
static void imu_gyro_reject_glitch(float *gyro)
{
    const float slew_limit = IMU_GYRO_SLEW_LIMIT_RAD_S2 * imu_frame.dt;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        const float abs_raw = fabsf(gyro[i]);

        if (gyro_prev_valid == 0U)
        {
            continue;
        }

        const uint8_t out_of_range = (abs_raw > IMU_GYRO_ABS_LIMIT_RAD_S) ? 1U : 0U;
        const uint8_t too_fast = (fabsf(gyro[i] - gyro_prev[i]) > slew_limit) ? 1U : 0U;

        // 超出物理量程的读数永远不是真实机动, 必须一直保持上一拍(否则连续两帧同样的
        // 饱和/位错会被当成真实机动放行). 变化率越限才允许"连续两帧"确认放行.
        if ((out_of_range != 0U) || ((too_fast != 0U) && (gyro_suspect[i] == 0U)))
        {
            gyro[i] = gyro_prev[i];
            gyro_suspect[i] = 1U;
            continue;
        }

        // 与上一拍一致, 或连续两帧偏离(真实机动) -> 清除待确认标志
        gyro_suspect[i] = 0U;
    }

    gyro_prev[0] = gyro[0];
    gyro_prev[1] = gyro[1];
    gyro_prev[2] = gyro[2];
    gyro_prev_valid = 1U;
}

/**
 * @brief 姿态解算单步: 零漂补偿 → 静止残差修正 → 四元数 EKF → 发布欧拉角
 * @param gyro  原始角速度, rad/s
 * @param accel 加速度, m/s^2
 */
static uint8_t imu_attitude_step(const float *gyro, const float *accel)
{
    if (ekf_ready == 0U)
    {
        // EKF 内存分配失败: 禁止解算与输出, 避免发布垃圾姿态
        still_count = 0U;
        attitude_valid = 0U;
        return 0U;
    }

    if ((gyro == 0) || (accel == 0) || !imu_array_is_finite(gyro, 3U) ||
        !imu_array_is_finite(accel, 3U) || !isfinite(imu_frame.dt))
    {
        IMU_QuaternionEKF_Reset();
        still_count = 0U;
        attitude_valid = 0U;
        return 0U;
    }

    float gx = gyro[0] - gyro_bias[0];
    float gy = gyro[1] - gyro_bias[1];
    float gz = gyro[2] - gyro_bias[2] - yaw_bias_trim;

    // 静止判定: 低通后的角速度足够小且加速度模长接近 1g
    const float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    if (still_gyro_lpf_started == 0U)
    {
        still_gyro_lpf[0] = gx;
        still_gyro_lpf[1] = gy;
        still_gyro_lpf[2] = gz;
        still_gyro_lpf_started = 1U;
    }
    else
    {
        const float alpha = imu_frame.dt / (IMU_STILL_GYRO_LPF_S + imu_frame.dt);
        still_gyro_lpf[0] += alpha * (gx - still_gyro_lpf[0]);
        still_gyro_lpf[1] += alpha * (gy - still_gyro_lpf[1]);
        still_gyro_lpf[2] += alpha * (gz - still_gyro_lpf[2]);
    }

    const float gyro_norm = sqrtf(still_gyro_lpf[0] * still_gyro_lpf[0] +
                                 still_gyro_lpf[1] * still_gyro_lpf[1] +
                                 still_gyro_lpf[2] * still_gyro_lpf[2]);

    if (!isfinite(gyro_norm) || !isfinite(accel_norm))
    {
        IMU_QuaternionEKF_Reset();
        still_count = 0U;
        still_gyro_lpf_started = 0U;
        still_accel_lpf_started = 0U;
        attitude_valid = 0U;
        return 0U;
    }

    if (still_accel_lpf_started == 0U)
    {
        still_accel_lpf = accel_norm;
        still_accel_lpf_started = 1U;
    }
    else
    {
        const float accel_alpha = imu_frame.dt / (IMU_STILL_ACCEL_LPF_S + imu_frame.dt);
        still_accel_lpf += accel_alpha * (accel_norm - still_accel_lpf);
    }

    if ((gyro_norm < IMU_STILL_GYRO_THRESHOLD) && (fabsf(still_accel_lpf - IMU_GRAVITY_ACCEL) < IMU_STILL_ACCEL_TOL))
    {
        if (still_count < IMU_STILL_CONFIRM_SAMPLES)
        {
            still_count++;
        }
    }
    else
    {
        still_count = 0U;
    }

    // 静止时修正 z 轴残余零漂(只修正"静止"时的残差, 不影响真实低速转动)
    //
    // 逐帧门控: 静止判定用的是 0.2s 低通角速度, 它比真实运动滞后 3~4 拍.
    // 实测机动起始的那几帧 |ω| 可达 1 rad/s, 却仍满足"低通值 < 0.02"的静止条件,
    // 直接积分进 trim 会累加 gain×Σ|ω| ≈ 4e-4 rad/s(0.023°/s), 之后要几十秒才回落.
    // 因此 trim 的积分再叠一层瞬时角速度门控(与 seed 阶段共用阈值).
    const float gyro_norm_now = sqrtf(gx * gx + gy * gy + gz * gz);
    const uint8_t trim_gate_ok =
        (isfinite(gyro_norm_now) && (gyro_norm_now < IMU_TRIM_SEED_RATE_MAX)) ? 1U : 0U;

    if (still_count >= IMU_STILL_CONFIRM_SAMPLES)
    {
        if (trim_seeded == 0U)
        {
            // 首次确认静止: 直接对残差取平均定标, 而不是从 0 用 20s 时间常数慢积分.
            // 硬编码 gyro_bias[2] 只有 0.000085 rad/s, 而本机实测零偏约 0.0033 rad/s,
            // 慢积分会让 yaw 先以 0.18°/s 漂移好几分钟才收敛.
            if (trim_gate_ok != 0U)
            {
                trim_seed_sum += gz;
                if (++trim_seed_n >= IMU_TRIM_SEED_SAMPLES)
                {
                    yaw_bias_trim = basic_math_constrain(yaw_bias_trim + trim_seed_sum / (float)trim_seed_n,
                                                         -IMU_YAW_TRIM_LIMIT, IMU_YAW_TRIM_LIMIT);
                    trim_seeded = 1U;
                }
            }
            else
            {
                trim_seed_sum = 0.0f;
                trim_seed_n = 0U;
            }
        }
        else if (trim_gate_ok != 0U)
        {
            yaw_bias_trim = basic_math_constrain(yaw_bias_trim + IMU_STILL_TRIM_GAIN * gz,
                                                 -IMU_YAW_TRIM_LIMIT, IMU_YAW_TRIM_LIMIT);
        }
        gz = gyro[2] - gyro_bias[2] - yaw_bias_trim;
    }

    QEKF_INS.dt = imu_frame.dt;
    IMU_QuaternionEKF_Update(gx, gy, gz, accel[0], accel[1], accel[2]);

    imu_frame.gyro[0] = gx;
    imu_frame.gyro[1] = gy;
    imu_frame.gyro[2] = gz;
    imu_frame.euler_angle.roll = Get_Roll() * BASIC_MATH_DEG_TO_RAD;
    imu_frame.euler_angle.pitch = Get_Pitch() * BASIC_MATH_DEG_TO_RAD;
    imu_frame.euler_angle.yaw = Get_Yaw() * BASIC_MATH_DEG_TO_RAD;
    imu_frame.yaw_total = QEKF_INS.YawTotalAngle * BASIC_MATH_DEG_TO_RAD;

    if (!isfinite(imu_frame.euler_angle.roll) || !isfinite(imu_frame.euler_angle.pitch) ||
        !isfinite(imu_frame.euler_angle.yaw) || !isfinite(imu_frame.yaw_total))
    {
        IMU_QuaternionEKF_Reset();
        still_count = 0U;
        attitude_valid = 0U;
        return 0U;
    }

    attitude_valid = 1U;
    return 1U;
}

/**
 * @brief 模块初始化: 加热 PWM 置安全态 → EKF/温控参数 → 传感器
 */
static void INS_Init(void)
{
    // 加热 PWM 先以 0 占空比起动, 保证上电不会满功率加热
    heater_pwm.set_tim_handle(&htim3);
    heater_pwm.set_pwm_channel(TIM_CHANNEL_4);
    // The generic PWM class defaults to 100%; only this heater instance is
    // restricted to 15%, and the limit cannot be raised after configuration.
    heater_pwm.set_max_duty_ratio(0.15f);
    heater_pwm.start_pwm();
    heater_pwm.set_duty_ration(0.0f);

    IMU_QuaternionEKF_Init(IMU_EKF_PROCESS_NOISE_Q1, IMU_EKF_PROCESS_NOISE_Q2, IMU_EKF_MEASURE_NOISE_R,
                           IMU_EKF_FADING_LAMBDA, IMU_DEFAULT_DT_S, IMU_EKF_ACCEL_LPF_S);
    // Kalman 矩阵用 pvPortMalloc 分配, FreeRTOS 堆不足时会返回 NULL.
    // 此时矩阵运算会写到地址 0 并产出垃圾姿态, 必须显式标记为不可用, 禁止输出.
    ekf_ready = IMU_QuaternionEKF_IsReady();

    pid_set_config_pos(&temperature_ctrl_config, IMU_TEMP_PID_KP, IMU_TEMP_PID_KI, IMU_TEMP_PID_KD,
                       IMU_TEMP_PID_INTEGRAL_LIMIT, IMU_TEMP_PID_OUT_MAX);
    pid_init(&temperature_pid, &temperature_ctrl_config);

    imu_frame.ref_temperature = IMU_TEMP_REF_C;

    imu_sensor_init();
}

/**
 * @brief DRDY 同步的采样与解算单步
 * @note  1) 陀螺仪 + 加速度计每帧必读(姿态解算需要), 只有 2 次 SPI 突发事务
 *        2) 温度按 IMU_TEMP_CTRL_DIVIDER 降频读取, 与温控同拍, 保证 PID 用的是最新温度
 *        3) 温度读取失败只关闭加热, 不影响本拍姿态(姿态与温控解耦)
 */
static void INS_Task(float sample_dt)
{
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 0.0f};

    imu_frame.dt = imu_limit_dt(sample_dt);

    const imu_status_e status = imu_get_status();
    if ((status == Imu_Status_Uninit) || (status == Imu_Status_Init_Fail))
    {
        // 传感器不可用: 加热保持关闭, 周期性重试初始化
        imu_heater_safe_off();
        if (++init_retry_ticks >= IMU_INIT_RETRY_PERIOD_TICKS)
        {
            init_retry_ticks = 0U;
            imu_sensor_init();
        }
        return;
    }

    const uint8_t imu_status_code = BMI088_read_imu(gyro, accel);
    if (imu_status_code != BMI088_NO_ERROR)
    {
        imu_frame.data_valid = 0U;
        // 温度反馈已经不可信, 任何一次通信失败都必须立即断开 24V 加热
        imu_heater_safe_off();

        if (++read_fail_count >= IMU_READ_FAIL_LIMIT)
        {
            imu_set_status(Imu_Status_Init_Fail);
        }
        return;
    }

    if (!imu_array_is_finite(gyro, 3U) || !imu_array_is_finite(accel, 3U))
    {
        imu_frame.data_valid = 0U;
        imu_heater_safe_off();
        return;
    }

    imu_frame.data_valid = 1U;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        imu_frame.accel[i] = accel[i];
    }

    // 野值抑制必须在任何积分之前
    imu_gyro_reject_glitch(gyro);

    // 温控节拍: 先读温度再跑 PID, 避免用到上一次(10ms 前)的旧值
    const uint8_t temp_due = ((imu_frame.sample_count % IMU_TEMP_CTRL_DIVIDER) == 0U) ? 1U : 0U;
    uint8_t temp_ok = 1U;

    if (temp_due != 0U)
    {
        float temperature = imu_frame.temperature;

        if ((BMI088_read_temperature(&temperature) != BMI088_NO_ERROR) || !isfinite(temperature) ||
            (temperature < IMU_TEMP_VALID_MIN_C) || (temperature > IMU_TEMP_VALID_MAX_C))
        {
            // 温度不可信: 立即断开加热, 但姿态继续用本拍的陀螺/加速度.
            // 温度与加速度计同 die、同一组 SPI 事务, 不可能单独长期失效, 因此不计入
            // read_fail_count(那是"整片掉线"判据)
            temp_ok = 0U;
            imu_heater_safe_off();
        }
        else
        {
            imu_frame.temperature = temperature;
        }
    }

    read_fail_count = 0U;
    drdy_timeout_count = 0U;

    imu_frame.sample_count++;

    if ((temp_due != 0U) && (temp_ok != 0U))
    {
        imu_temperature_ctrl_step();
    }

    switch (imu_get_status())
    {
#if IMU_GYRO_AUTO_CALIB_ENABLE
        case Imu_Status_Wait_Temp:
            // 温度进入目标带宽并持续 IMU_TEMP_STABLE_MS 后才开始标定
            if (fabsf(imu_frame.temperature - imu_frame.ref_temperature) < IMU_TEMP_STABLE_BAND_C)
            {
                if (++temp_stable_ticks >= IMU_TEMP_STABLE_MS)
                {
                    temp_stable_ticks = 0U;
                    imu_gyro_calib_start();
                    imu_set_status(Imu_Status_Calibrating);
                }
            }
            else
            {
                temp_stable_ticks = 0U;
            }
            break;

        case Imu_Status_Calibrating:
            imu_gyro_calib_step(gyro);
            break;
#endif

        case Imu_Status_Running:
            // 姿态解算失败只影响姿态输出, 不关闭加热也不清传感器有效标志:
            // 温度闭环只依赖 temperature, 与姿态无关, 整链失效会掩盖真正的温控状态
            (void)imu_attitude_step(gyro, accel);
            break;

        default:
            break;
    }
}

/**
 * @brief IMU 任务(DRDY 同步)
 * @param argument 未使用
 * @note  ISR 只记录时间并通知任务, SPI 读取和姿态解算均在任务上下文执行
 */
extern "C" void imu_task(void *argument)
{
    (void)argument;

    imu_task_handle = osThreadGetId();
    INS_Init();

    uint32_t last_drdy_sequence = gyro_drdy_sequence;
    for (;;)
    {
        const imu_status_e status = imu_get_status();
        if ((status == Imu_Status_Uninit) || (status == Imu_Status_Init_Fail))
        {
            // 初始化失败时没有可靠的 DRDY, 用短周期维护重试计时器
            INS_Task(IMU_DEFAULT_DT_S);
            osDelay(1U);

            // 重试期间传感器可能已恢复并在持续产生 DRDY(SPI 故障但 INT3 正常),
            // 序号会一直累加. 恢复时必须重新对齐序号与 DWT 基准, 否则这段离线时间
            // 会被当成一帧巨大的 dt
            last_drdy_sequence = gyro_drdy_sequence;
            gyro_drdy_dwt_started = 0U;
            continue;
        }

        const uint32_t flags = osThreadFlagsWait(IMU_DRDY_EVENT_FLAG, osFlagsWaitAny,
                                                   IMU_DRDY_TIMEOUT_TICKS);
        if ((flags & osFlagsError) != 0U)
        {
            imu_frame.data_valid = 0U;
            imu_heater_safe_off();

            if (++drdy_timeout_count >= IMU_DRDY_TIMEOUT_LIMIT)
            {
                drdy_timeout_count = 0U;
                imu_set_status(Imu_Status_Init_Fail);
            }
            continue;
        }

        const uint32_t sequence = gyro_drdy_sequence;
        const uint32_t pending = sequence - last_drdy_sequence;
        if (pending == 0U)
        {
            continue;
        }
        last_drdy_sequence = sequence;

        // 多个中断合并成一个通知时, 用总间隔推进一次 EKF
        INS_Task(gyro_drdy_dt * (float)pending);
    }
}

float imu_get_roll_rad(void)
{
    return imu_frame.euler_angle.roll;
}

float imu_get_pitch_rad(void)
{
    return imu_frame.euler_angle.pitch;
}

float imu_get_yaw_rad(void)
{
    return imu_frame.euler_angle.yaw;
}

float imu_get_yaw_total_rad(void)
{
    return imu_frame.yaw_total;
}

uint8_t imu_is_ready(void)
{
    // 姿态可用 = 进入解算状态 且 传感器数据有效 且 EKF 正常产出
    return ((imu_frame.status == Imu_Status_Running) && (imu_frame.data_valid != 0U) &&
            (attitude_valid != 0U)) ? 1U : 0U;
}

/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	BMI088 数据就绪(ACC_INT / GYRO_INT)中断回调
* @note        这里只做时间戳和线程通知, 不在中断中执行 SPI 或浮点姿态解算
************************************************************************
**/
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin != GYRO_INT_Pin)
   {
       return;
   }

   if (gyro_drdy_dwt_started == 0U)
   {
       (void)bsp_dwt_get_deltat32(&gyro_drdy_dwt_last);
       gyro_drdy_dwt_started = 1U;
       gyro_drdy_dt = IMU_DEFAULT_DT_S;
   }
   else
   {
       gyro_drdy_dt = bsp_dwt_get_deltat32(&gyro_drdy_dwt_last);
   }

   gyro_drdy_sequence++;
   if (imu_task_handle != 0)
   {
       (void)osThreadFlagsSet(imu_task_handle, IMU_DRDY_EVENT_FLAG);
   }
}
