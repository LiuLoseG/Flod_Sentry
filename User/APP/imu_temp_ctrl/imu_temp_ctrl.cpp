/**
 * @file imu_temp_ctrl.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 板载 BMI088 IMU: SPI 采样 → 恒温控制 → 零漂/刻度/杆臂补偿 → 时间账本 → 四元数 EKF 姿态解算
 * @date 2026-11-03
 * @version 2.0
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

/*------------------------------ 陀螺积分时间基准 ------------------------------*/

// yaw 在 EKF 中无绝对观测, 只能靠 z 轴角速度积分, 丢时间即丢转角.
// 账本累计"已到达但未积分"的时间, 每拍只取受限步长, 余量留到后续拍排空.

// ===== IMU_TIME_BASE_BEGIN (主机侧测试按此标记抽取, 勿改名) =====

// 单次积分的最大步长, s. 必须小于 QuaternionEKF 内部 0.01s 的 dt 复位阈值
#define IMU_TIME_BASE_STEP_MAX_S        0.009f
// 单帧 DRDY 周期的合理下限, s; 低于它由 imu_limit_dt() 抬回
#define IMU_TIME_BASE_STEP_MIN_S        0.0005f
// 待积分时间的累计上限, s
#define IMU_TIME_BASE_PENDING_MAX_S     0.020f

typedef struct
{
    float pending_s;    // 已到达但尚未积分的时间, s
} imu_time_base_t;

/**
 * @brief 清空账本(姿态复位/退出解算/传感器重连时调用)
 */
static inline void imu_time_base_reset(imu_time_base_t *tb)
{
    tb->pending_s = 0.0f;
}

/**
 * @brief 记账: 本拍新增 frames 个采样周期, 每周期 frame_dt 秒
 * @param frame_dt 单帧名义周期, s, 非正值视为无效
 * @param frames   本拍合并的帧数, >= 1
 * @note  读失败/超时的那一帧同样要记账, 否则永久丢失该帧转角
 */
static inline void imu_time_base_accumulate(imu_time_base_t *tb, float frame_dt, uint32_t frames)
{
    if ((frame_dt > 0.0f) && (frames > 0U))
    {
        tb->pending_s += frame_dt * (float)frames;
    }

    if (tb->pending_s > IMU_TIME_BASE_PENDING_MAX_S)
    {
        tb->pending_s = IMU_TIME_BASE_PENDING_MAX_S;
    }
}

/**
 * @brief 取出本拍要积分的时间并结算
 * @return 积分步长, s, 落在 (0, IMU_TIME_BASE_STEP_MAX_S]; 返回 0 表示本拍无时间可积
 * @note  余额留在账本里由后续几拍排空; 账本为空时绝不返回最小步长(等于凭空造时间)
 */
static inline float imu_time_base_take(imu_time_base_t *tb)
{
    float dt = tb->pending_s;

    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    if (dt > IMU_TIME_BASE_STEP_MAX_S)
    {
        dt = IMU_TIME_BASE_STEP_MAX_S;
    }

    tb->pending_s -= dt;

    if (tb->pending_s < 0.0f)
    {
        tb->pending_s = 0.0f;
    }

    return dt;
}

// ===== IMU_TIME_BASE_END =====

/*------------------------------ 采样与调度 ------------------------------*/

// ===== IMU_DT_LIMIT_BEGIN (主机侧测试按此标记抽取, 勿改名) =====

// 无法取得有效 DRDY 周期时使用的初始化/恢复周期, s
#define IMU_DEFAULT_DT_S                0.001f
// 单帧 DRDY 周期(1kHz)的合理区间, s
#define IMU_DT_MIN_S                    IMU_TIME_BASE_STEP_MIN_S
#define IMU_DT_MAX_S                    IMU_TIME_BASE_STEP_MAX_S

/**
 * @brief 单帧 DRDY 周期的合理性限幅
 * @param dt 实测单帧周期, s
 * @return 限幅后的周期, s, 落在 [IMU_DT_MIN_S, IMU_DT_MAX_S]
 * @note  只用于"单帧"语义(野值判据、未解算时的 dt 发布), 不能拿它截短积分时间
 */
static inline float imu_limit_dt(float dt)
{
    if (!(dt > 0.0f))
    {
        // 同时拦住 dt <= 0 与 NaN(NaN 比较恒为假)
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

// ===== IMU_DT_LIMIT_END =====

// DRDY 超时保护. 连续超时后进入 Init_Fail, 由任务周期性重试 BMI088
#define IMU_DRDY_TIMEOUT_TICKS          10U
#define IMU_DRDY_TIMEOUT_LIMIT          3U
#define IMU_DRDY_EVENT_FLAG             (1U << 0)

// 温度读取与温控周期 = 采样周期 x 分频 = 10ms/100Hz
#define IMU_TEMP_CTRL_DIVIDER           10U

/*------------------------------ 姿态解算(EKF) ------------------------------*/

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

// 自标定按真实时间计, 不按调用次数(IMU_GYRO_AUTO_CALIB_ENABLE = 0 时本段不参与编译)
#define IMU_GYRO_CALIB_S                1.0f           // s, 有效采样时长(1kHz 下 = 1000 帧)
#define IMU_GYRO_CALIB_MAX_RATE         0.15f          // rad/s, 超过认为车体在动, 该帧丢弃
#define IMU_GYRO_CALIB_TIMEOUT_S        5.0f           // s, 标定超时, 超时后按已采样本结算
#define IMU_GYRO_CALIB_MIN_S            0.1f           // s, 超时结算所需的最少有效时长
#define IMU_GYRO_CALIB_REJECT_S         2.0f           // s, 累计被丢弃时长上限, 超过则放弃标定

#define IMU_GRAVITY_ACCEL               9.80665f       // 重力加速度, m/s^2

// 静止判定与残差修正参数
#define IMU_STILL_GYRO_THRESHOLD        0.020f         // rad/s, 静止判定的角速度阈值
#define IMU_STILL_GYRO_LPF_S            0.20f          // s, 静止判定用角速度低通时间常数(滤除高频噪声)
#define IMU_STILL_ACCEL_TOL             0.5f           // m/s^2, 静止判定加速度模长偏差

#define IMU_STILL_ACCEL_LPF_S           0.20f          // s, 静止判定用加速度模长低通时间常数
#define IMU_STILL_CONFIRM_S             0.5f           // s, 连续静止时长(去抖)
#define IMU_STILL_TRIM_RATE             0.05f          // 1/s, 残差积分速率(时间常数 ~20s)
#define IMU_YAW_TRIM_LIMIT              0.02f          // rad/s, 残差修正上限

#define IMU_TRIM_SEED_S                 0.5f           // s, 静止确认后再平均这么久定标
#define IMU_TRIM_SEED_RATE_MAX          0.15f          // rad/s, 单帧角速度模长上限

/*------------------------------ 陀螺野值抑制 ------------------------------*/

// ===== IMU_GYRO_GLITCH_BEGIN (主机侧测试按此标记抽取, 勿改名) =====

#define IMU_GYRO_SLEW_LIMIT_RAD_S2      10000.0f       // rad/s^2, 单帧允许的最大角速度变化
#define IMU_GYRO_ABS_LIMIT_RAD_S        36.0f          // rad/s, 超出 BMI088 ±2000dps 量程即判野值
// 距上一拍超过这么多帧, 上一拍样本已过期, 不能作为参考
#define IMU_GYRO_GLITCH_GAP_FRAMES      4.0f

typedef struct
{
    float prev[3];          // 上一拍(修正后)的原始角速度, rad/s
    uint8_t prev_valid;     // prev 是否可用
    uint8_t suspect[3];     // 逐轴"已因变化率越限而保持上一拍"标志
} imu_gyro_glitch_t;

static inline void imu_gyro_glitch_reset(imu_gyro_glitch_t *st)
{
    st->prev[0] = 0.0f;
    st->prev[1] = 0.0f;
    st->prev[2] = 0.0f;
    st->prev_valid = 0U;
    st->suspect[0] = 0U;
    st->suspect[1] = 0U;
    st->suspect[2] = 0U;
}

/**
 * @brief 陀螺野值抑制
 * @param st         滤波器状态
 * @param gyro       原始角速度, rad/s, 就地修正
 * @param frame_dt   单帧采样周期, s
 * @param elapsed_dt 距上一拍真实经过的时间, s
 * @note  1) 单帧 SPI 位错会给出几百~上千 °/s 的假角速度, 而 EKF 对 z 轴无绝对观测,
 *           一次野值就把 yaw 永久推偏, 必须在此拦截
 *        2) 变化率判据针对相邻 1kHz 采样点, frame_dt 必须是单帧周期, 不能用累计积分步长
 *            (后者合并时可达 9ms, 阈值会被放宽 9 倍)
 *        3) 超量程读数一律挡住; elapsed_dt 远大于 frame_dt 时 prev 已过期, 只保留量程判据
 */
static inline void imu_gyro_glitch_filter(imu_gyro_glitch_t *st, float *gyro, float frame_dt, float elapsed_dt)
{
    const float slew_limit = IMU_GYRO_SLEW_LIMIT_RAD_S2 * frame_dt;

    if (elapsed_dt > (IMU_GYRO_GLITCH_GAP_FRAMES * frame_dt))
    {
        st->prev_valid = 0U;
        st->suspect[0] = 0U;
        st->suspect[1] = 0U;
        st->suspect[2] = 0U;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (fabsf(gyro[i]) > IMU_GYRO_ABS_LIMIT_RAD_S)
        {
            gyro[i] = (st->prev_valid != 0U) ? st->prev[i] : 0.0f;
            st->suspect[i] = 1U;
            continue;
        }

        if (st->prev_valid == 0U)
        {
            st->suspect[i] = 0U;
            continue;
        }

        // 变化率越限: 先保持上一拍; 下一帧仍偏离才认定为真实机动并放行
        if ((fabsf(gyro[i] - st->prev[i]) > slew_limit) && (st->suspect[i] == 0U))
        {
            gyro[i] = st->prev[i];
            st->suspect[i] = 1U;
            continue;
        }

        st->suspect[i] = 0U;
    }

    st->prev[0] = gyro[0];
    st->prev[1] = gyro[1];
    st->prev[2] = gyro[2];
    st->prev_valid = 1U;
}

// ===== IMU_GYRO_GLITCH_END =====

/*------------------------------ 陀螺标定(零偏 + 刻度) ------------------------------*/

// ===== IMU_GYRO_CALIB_BEGIN (主机侧测试按此标记抽取, 勿改名) =====

/**
 * @brief 原始角速度换算成物理角速度: (raw - bias) * scale
 * @param gyro  原始角速度, rad/s, 就地换算
 * @param bias  零偏, rad/s, 原始域标定量
 * @param scale 刻度因子, 无量纲, 原始域标定量
 * @note  bias 与 scale 同处原始域, 必须先扣零偏再乘刻度, 否则残留 (scale-1)*bias;
 *        yaw_bias_trim 属标定域, 由调用方之后单独扣除
 */
static inline void imu_gyro_calibrate(float *gyro, const float *bias, const float *scale)
{
    gyro[0] = (gyro[0] - bias[0]) * scale[0];
    gyro[1] = (gyro[1] - bias[1]) * scale[1];
    gyro[2] = (gyro[2] - bias[2]) * scale[2];
}

// ===== IMU_GYRO_CALIB_END =====

/*------------------------------ 加速度计偏心补偿(杆臂补偿) ------------------------------*/

// IMU 不在刚体旋转中心时, 加速度计除重力外还会读到旋转带来的两项附加加速度:
//     a_meas = a_ref + α × r + ω × (ω × r)

// 角加速度估计: 陀螺先一阶低通再差分, 避免 1kHz 差分把噪声放大 1/dt 倍
// (0.004 rad/s 的陀螺噪声直接差分就是 4 rad/s², 乘 0.1 m 偏心等于 0.4 m/s² 假加速度)
#define IMU_LEVER_GYRO_LPF_S        0.004f        // s, 供差分用的角速度低通时间常数
#define IMU_LEVER_ALPHA_LPF_S       0.008f        // s, 角加速度低通时间常数
// 角加速度限幅, rad/s². 200 rad/s² 相当于 0.17s 从静止拉到 ±2000dps 满量程
#define IMU_LEVER_ALPHA_MAX_RAD_S2  200.0f
// 补偿量限幅, m/s². 0.1 m 偏心 @ 6 rad/s 向心项 3.6 m/s², 留足余量;
// 只为拦住异常数据把重力方向带飞, 正常工况远达不到
#define IMU_LEVER_CORR_MAX_M_S2     30.0f
// 距上一拍超过这么多帧, 上一拍样本已过期, 不能再拿它差分
// (与野值抑制同口径: 差分的语义是"相邻两个 1kHz 采样点")
#define IMU_LEVER_GAP_FRAMES        4.0f

typedef struct
{
    float gyro_lpf[3];      // 供角加速度差分用的角速度低通值, rad/s
    float alpha_lpf[3];     // 角加速度低通值, rad/s^2
    float prev[3];          // 上一拍的陀螺低通值, rad/s
    uint8_t started;        // 滤波器是否已初始化
} imu_accel_lever_t;

static inline void imu_accel_lever_reset(imu_accel_lever_t *st)
{
    st->gyro_lpf[0] = 0.0f;
    st->gyro_lpf[1] = 0.0f;
    st->gyro_lpf[2] = 0.0f;
    st->alpha_lpf[0] = 0.0f;
    st->alpha_lpf[1] = 0.0f;
    st->alpha_lpf[2] = 0.0f;
    st->prev[0] = 0.0f;
    st->prev[1] = 0.0f;
    st->prev[2] = 0.0f;
    st->started = 0U;
}

/**
 * @brief 对称限幅, 非有限值(除零/溢出产生的 NaN/Inf)一律归 0
 */
static inline float imu_lever_clamp(float value, float limit)
{
    if (!isfinite(value))
    {
        return 0.0f;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

/**
 * @brief 加速度计偏心(杆臂)补偿: 把加速度计读数折算到刚体旋转中心
 * @param st      滤波器状态
 * @param accel   加速度, m/s^2, 就地修正
 * @param gyro    已扣零偏/刻度的角速度, rad/s
 * @param lever_m 旋转中心 → IMU 的位置矢量, m, **BMI088 芯片自身坐标系**(与 gyro/accel 同系)
 * @param frame_dt   单帧 DRDY 周期, s —— 角加速度差分的时间基
 * @param elapsed_dt 距上一拍真实经过的时间, s —— 只用于长间隔判据
 */
static inline void imu_accel_lever_compensate(imu_accel_lever_t *st, float *accel,
                                              const float *gyro, const float *lever_m,
                                              float frame_dt, float elapsed_dt)
{
    if (!isfinite(frame_dt) || !(frame_dt > 0.0f))
    {
        imu_accel_lever_reset(st);
        return;
    }

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!isfinite(gyro[i]) || !isfinite(accel[i]) || !isfinite(lever_m[i]))
        {
            imu_accel_lever_reset(st);
            return;
        }
    }

    // 长间隔: 样本已不连续, 差分基准作废(与野值抑制的长间隔判据同口径)
    const uint8_t gap = (isfinite(elapsed_dt) &&
                         (elapsed_dt > (IMU_LEVER_GAP_FRAMES * frame_dt))) ? 1U : 0U;

    if ((st->started == 0U) || (gap != 0U))
    {
        // 没有可用的上一拍(首拍/重连/长间隔): 角加速度按 0 处理, 本拍只做向心项
        for (uint8_t i = 0U; i < 3U; i++)
        {
            st->gyro_lpf[i] = gyro[i];
            st->alpha_lpf[i] = 0.0f;
            st->prev[i] = gyro[i];
        }
        st->started = 1U;
    }
    else
    {
        const float gyro_coef = frame_dt / (IMU_LEVER_GYRO_LPF_S + frame_dt);
        const float alpha_coef = frame_dt / (IMU_LEVER_ALPHA_LPF_S + frame_dt);

        for (uint8_t i = 0U; i < 3U; i++)
        {
            st->gyro_lpf[i] += gyro_coef * (gyro[i] - st->gyro_lpf[i]);

            const float alpha = imu_lever_clamp((st->gyro_lpf[i] - st->prev[i]) / frame_dt,
                                                IMU_LEVER_ALPHA_MAX_RAD_S2);
            st->prev[i] = st->gyro_lpf[i];
            st->alpha_lpf[i] += alpha_coef * (alpha - st->alpha_lpf[i]);
        }
    }

    // ω × (ω × r) = ω(ω·r) − r|ω|², 向心项是 ω 的二次量, 用瞬时值信噪比反而更好
    const float w_dot_r = gyro[0] * lever_m[0] + gyro[1] * lever_m[1] + gyro[2] * lever_m[2];
    const float w_sq = gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2];

    for (uint8_t i = 0U; i < 3U; i++)
    {
        const uint8_t j = (uint8_t)((i + 1U) % 3U);
        const uint8_t k = (uint8_t)((i + 2U) % 3U);

        // 切向项 (α × r)_i
        const float tangential = st->alpha_lpf[j] * lever_m[k] - st->alpha_lpf[k] * lever_m[j];
        // 向心项 (ω × (ω × r))_i
        const float centripetal = gyro[i] * w_dot_r - lever_m[i] * w_sq;

        accel[i] -= imu_lever_clamp(tangential + centripetal, IMU_LEVER_CORR_MAX_M_S2);
    }
}

// ===== IMU_ACCEL_LEVER_END =====

/*------------------------------ 温度控制 ------------------------------*/

#define IMU_TEMP_REF_C                  50.0f          // 目标温度, ℃
#define IMU_TEMP_VALID_MIN_C            (-20.0f)       // 温度有效下限, ℃
#define IMU_TEMP_VALID_MAX_C            100.0f         // 温度有效上限, ℃
#define IMU_TEMP_OVERHEAT_C             60.0f          // 过温保护, ℃
#define IMU_TEMP_STABLE_BAND_C          0.8f           // 判定温度稳定的带宽, ℃
#define IMU_TEMP_STABLE_S               3.0f           // s, 温度稳定持续时长

#define IMU_TEMP_PID_KP                 180.0f
#define IMU_TEMP_PID_KI                 1.2f
#define IMU_TEMP_PID_KD                 150.0f
#define IMU_TEMP_PID_OUT_MAX            1500.0f        // 加热输出上限(与 TIM3 ARR 同量纲)
#define IMU_TEMP_PID_INTEGRAL_LIMIT     500.0f         // 积分限幅, 防止积分饱和(windup)
#define IMU_HEATER_PWM_PERIOD_COUNTS    10000.0f       // TIM3 ARR + 1, 用于把输出换算成占空比

// ===== IMU_HEATER_POLICY_BEGIN (主机侧测试按此标记抽取, 勿改名) =====

/**
 * @brief 单向执行器的条件积分判决
 * @return 1 = 保留本拍积分累加; 0 = 撤销本拍积分累加
 * @note  加热器只能加热: 输出顶到上限时冻结积分防大超调; 输出被压到 0 时撤销积分,
 *        避免降温后先消化负积分造成"过冲却长时间不加热"的迟滞.
 *        NaN 比较恒为假, 会落入"撤销"分支, 方向安全.
 */
static inline uint8_t imu_heater_keep_integral(float out, float out_max)
{
    return ((out > 0.0f) && (out <= out_max)) ? 1U : 0U;
}

/**
 * @brief 把 PID 输出夹进执行器可行域 [0, out_max]
 */
static inline float imu_heater_clamp_out(float out, float out_max)
{
    if (out > out_max)
    {
        return out_max;
    }

    return (out > 0.0f) ? out : 0.0f;
}

// ===== IMU_HEATER_POLICY_END =====

volatile imu_data_resolve_t imu_frame = {0};

// 温度PID控制器
static pid_t temperature_pid = {0};
static pid_config temperature_ctrl_config = {0};

// 加热电阻 PWM(TIM3_CH4), 上电占空比为 0
static pwm_class heater_pwm;
// 最近一次下发给 TIM3_CH4 的占空比, 用于避免每帧重复写 CCR
static float heater_duty_cmd = 0.0f;

// 陀螺零漂补偿值(参数校准), rad/s
static float gyro_bias[3] = {0.00317657646f, -0.00223845663f, 0.00285839941f};

// 陀螺刻度因子(标度因数修正), 无量纲, 1.0 = 不修正(参数校准)
static float gyro_scale[3] = {1.0f, 1.0f, 0.9936f};

// IMU 安装位置相对刚体旋转中心的位置矢量(参数校准), m.
static float imu_lever_arm_m[3] = {0.08f, 0.015f, 0.0f};

// 陀螺积分的时间账本(定义见本文件 IMU_TIME_BASE 段)
static imu_time_base_t gyro_time_base = {0.0f};

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
// 连续静止时长, s
static float still_confirm_s = 0.0f;
// 静止判定用低通角速度(抑制采样噪声打断判定)
static float still_gyro_lpf[3] = {0.0f, 0.0f, 0.0f};
static uint8_t still_gyro_lpf_started = 0U;
// 静止判定用低通加速度模长
static float still_accel_lpf = 0.0f;
static uint8_t still_accel_lpf_started = 0U;

// 上电/重连后的静止残差快速定标(时间加权平均)
static float trim_seed_sum = 0.0f;      // Σ gz*dt
static float trim_seed_span = 0.0f;     // Σ dt
static uint8_t trim_seeded = 0U;

// 陀螺野值抑制状态
static imu_gyro_glitch_t gyro_glitch = {{0.0f, 0.0f, 0.0f}, 0U, {0U, 0U, 0U}};

// 加速度计偏心补偿状态(角加速度估计)
static imu_accel_lever_t accel_lever = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0U};

// 积分时基: 上一次进入 INS_Task 时刻的 DWT 计数
static uint32_t imu_elapsed_dwt_last = 0U;
static uint8_t imu_elapsed_dwt_started = 0U;

#if IMU_GYRO_AUTO_CALIB_ENABLE
static float temp_stable_s = 0.0f;
static float calib_sum[3] = {0.0f, 0.0f, 0.0f};   // Σ raw*dt
static float calib_span_s = 0.0f;                  // Σ dt (有效)
static float calib_elapsed_s = 0.0f;               // Σ dt (全部)
static float calib_reject_s = 0.0f;                // Σ dt (被丢弃)
#endif

static void imu_set_status(imu_status_e status);
static void imu_heater_output_off(void);
static void imu_heater_safe_off(void);
static void imu_sensor_init(void);
static void imu_temperature_ctrl_step(void);
static uint8_t imu_attitude_step(const float *gyro, const float *accel, float dt);
static void INS_Init(void);
static void INS_Task(float drdy_dt, float elapsed_dt);

#if IMU_GYRO_AUTO_CALIB_ENABLE
static void imu_gyro_calib_start(void);
static void imu_gyro_calib_step(const float *gyro, float dt);
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

/**
 * @brief 量取距上一次进入 INS_Task 真实经过的时间
 * @return 间隔, s; 首次调用返回名义周期
 * @note  用 DWT 硬件计数直接量, 与帧数、DRDY 抖动、通知延迟无关
 */
static float imu_measure_elapsed_dt(void)
{
    if (imu_elapsed_dwt_started == 0U)
    {
        (void)bsp_dwt_get_deltat32(&imu_elapsed_dwt_last);
        imu_elapsed_dwt_started = 1U;
        return IMU_DEFAULT_DT_S;
    }

    return bsp_dwt_get_deltat32(&imu_elapsed_dwt_last);
}

/**
 * @brief 立即把加热输出置 0, 但保留积分器
 * @note  用于单帧通信毛刺/瞬时数据无效: 安全优先先断加热, 但清积分器会让温控
 *        退回冷启动(稳态输出重建需 10s 量级)
 */
static void imu_heater_output_off(void)
{
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
 * @brief 硬故障: 立即断开加热并清空积分器
 * @note  适用传感器掉线/初始化失败/温度超范围/过温; 恢复后必须从零重新积分,
 *        否则会拿掉线期间攒的积分去猛加热
 */
static void imu_heater_safe_off(void)
{
    temperature_pid.error_sum = 0.0f;
    imu_heater_output_off();
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
    calib_span_s = 0.0f;
    calib_elapsed_s = 0.0f;
    calib_reject_s = 0.0f;
}
#endif

/**
 * @brief 陀螺零漂标定单步
 * @param gyro 原始角速度, rad/s
 * @param dt   本拍代表的时间, s
 * @note  丢弃运动帧; 累加按时间加权(Σraw·dt / Σdt), 不是逐帧算术平均; 超时按真实时间
 */
#if IMU_GYRO_AUTO_CALIB_ENABLE
static void imu_gyro_calib_step(const float *gyro, float dt)
{
    const float norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);

    calib_elapsed_s += dt;

    if (norm > IMU_GYRO_CALIB_MAX_RATE)
    {
        calib_reject_s += dt;
        if (calib_reject_s >= IMU_GYRO_CALIB_REJECT_S)
        {
            // 长时间被扰动: 放弃本次标定, 沿用默认补偿值
            imu_set_status(Imu_Status_Running);
        }
        return;
    }

    calib_sum[0] += gyro[0] * dt;
    calib_sum[1] += gyro[1] * dt;
    calib_sum[2] += gyro[2] * dt;
    calib_span_s += dt;

    if ((calib_span_s >= IMU_GYRO_CALIB_S) ||
        ((calib_elapsed_s >= IMU_GYRO_CALIB_TIMEOUT_S) && (calib_span_s >= IMU_GYRO_CALIB_MIN_S)))
    {
        const float inv_span = 1.0f / calib_span_s;
        for (uint8_t i = 0U; i < 3U; i++)
        {
            gyro_bias[i] = calib_sum[i] * inv_span;
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
            imu_time_base_reset(&gyro_time_base);
            still_confirm_s = 0.0f;
            still_gyro_lpf_started = 0U;
            still_accel_lpf_started = 0U;
            // 软复位后零偏会变, 需要重新做静止残差定标; 野值抑制的上一拍基准同样作废
            trim_seeded = 0U;
            trim_seed_sum = 0.0f;
            trim_seed_span = 0.0f;
            imu_gyro_glitch_reset(&gyro_glitch);
            // 离线前后的角速度不连续, 角加速度差分基准同样作废
            imu_accel_lever_reset(&accel_lever);
            // 重连期间经历的时间不能算进账本, 否则会拿一帧新数据去"补"整段离线时间
            imu_elapsed_dwt_started = 0U;
            imu_frame.data_valid = 1U;
#if IMU_GYRO_AUTO_CALIB_ENABLE
            temp_stable_s = 0.0f;
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
 * @note  单向执行器条件积分抗饱和; 传感器无效/过温时立即断加热并清积分
 */
static void imu_temperature_ctrl_step(void)
{
    const float temperature = imu_frame.temperature;

    if ((imu_frame.data_valid == 0U) || (temperature < IMU_TEMP_VALID_MIN_C) ||
        (temperature > IMU_TEMP_VALID_MAX_C) || (temperature > IMU_TEMP_OVERHEAT_C))
    {
        // 读数不可信/过温: 硬故障, 积分器一并清零
        imu_heater_safe_off();
        return;
    }

    const float error_sum_before = temperature_pid.error_sum;
    float out = pid_calc(&temperature_pid, imu_frame.ref_temperature, temperature);

    // pid_calc 不拦 NaN/Inf, basic_math_constrain 也拦不住 NaN, 必须单独判掉
    if (!isfinite(out))
    {
        imu_heater_safe_off();
        return;
    }

    // 条件积分: 输出冲出可行域时撤销本拍积分累加, 避免 Windup
    if (imu_heater_keep_integral(out, IMU_TEMP_PID_OUT_MAX) == 0U)
    {
        temperature_pid.error_sum = error_sum_before;
    }

    // pid_calc 的 PID_POSITION 分支不做输出限幅, 这里必须自己限幅
    out = imu_heater_clamp_out(out, IMU_TEMP_PID_OUT_MAX);
    temperature_pid.output = out;

    imu_frame.heater_out = out;
    heater_duty_cmd = out / IMU_HEATER_PWM_PERIOD_COUNTS;
    heater_pwm.set_duty_ration(heater_duty_cmd);
}

/**
 * @brief 姿态解算单步: 静止残差修正 → 四元数 EKF → 发布欧拉角
 * @param gyro  已换算的角速度, rad/s
 * @param accel 加速度, m/s^2
 * @param dt    本拍积分步长, s
 * @return 1 = 姿态有效, 0 = 已复位且姿态不可用
 */
static uint8_t imu_attitude_step(const float *gyro, const float *accel, float dt)
{
    if (ekf_ready == 0U)
    {
        // EKF 内存分配失败: 禁止解算与输出, 避免发布垃圾姿态
        still_confirm_s = 0.0f;
        attitude_valid = 0U;
        return 0U;
    }

    if ((gyro == 0) || (accel == 0) || !imu_array_is_finite(gyro, 3U) ||
        !imu_array_is_finite(accel, 3U) || !isfinite(dt))
    {
        IMU_QuaternionEKF_Reset();
        still_confirm_s = 0.0f;
        attitude_valid = 0U;
        return 0U;
    }

    // 零偏与刻度已在 INS_Task 里换算完, 这里只再扣除标定域的静止残差
    const float gx = gyro[0];
    const float gy = gyro[1];
    float gz = gyro[2] - yaw_bias_trim;

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
        const float alpha = dt / (IMU_STILL_GYRO_LPF_S + dt);
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
        still_confirm_s = 0.0f;
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
        const float accel_alpha = dt / (IMU_STILL_ACCEL_LPF_S + dt);
        still_accel_lpf += accel_alpha * (accel_norm - still_accel_lpf);
    }

    if ((gyro_norm < IMU_STILL_GYRO_THRESHOLD) && (fabsf(still_accel_lpf - IMU_GRAVITY_ACCEL) < IMU_STILL_ACCEL_TOL))
    {
        // 按真实时间累计, 不按调用次数
        still_confirm_s += dt;
        if (still_confirm_s > IMU_STILL_CONFIRM_S)
        {
            still_confirm_s = IMU_STILL_CONFIRM_S;
        }
    }
    else
    {
        still_confirm_s = 0.0f;
    }

    // 静止时修正 z 轴残余零漂: 静止判定用的 0.2s 低通比真实运动滞后 3~4 拍,
    // 机动起始的几帧瞬时 |ω| 仍可能很大, 因此 trim 积分再叠一层瞬时角速度门控
    const float gyro_norm_now = sqrtf(gx * gx + gy * gy + gz * gz);
    const uint8_t trim_gate_ok =
        (isfinite(gyro_norm_now) && (gyro_norm_now < IMU_TRIM_SEED_RATE_MAX)) ? 1U : 0U;

    if (still_confirm_s >= IMU_STILL_CONFIRM_S)
    {
        if (trim_seeded == 0U)
        {
            // 首次确认静止: 直接对残差取时间加权平均定标, 避免从 0 慢积分导致
            // yaw 上电后漂移数分钟才收敛
            if (trim_gate_ok != 0U)
            {
                trim_seed_sum += gz * dt;
                trim_seed_span += dt;
                if (trim_seed_span >= IMU_TRIM_SEED_S)
                {
                    yaw_bias_trim = basic_math_constrain(yaw_bias_trim + trim_seed_sum / trim_seed_span,
                                                         -IMU_YAW_TRIM_LIMIT, IMU_YAW_TRIM_LIMIT);
                    trim_seeded = 1U;
                }
            }
            else
            {
                trim_seed_sum = 0.0f;
                trim_seed_span = 0.0f;
            }
        }
        else if (trim_gate_ok != 0U)
        {
            // 增益按 速率 × dt 归一, 收敛时间常数与通知合并率无关
            yaw_bias_trim = basic_math_constrain(yaw_bias_trim + IMU_STILL_TRIM_RATE * gz * dt,
                                                 -IMU_YAW_TRIM_LIMIT, IMU_YAW_TRIM_LIMIT);
        }
        gz = gyro[2] - yaw_bias_trim;
    }

    QEKF_INS.dt = dt;
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
        still_confirm_s = 0.0f;
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
    // 通用 PWM 类默认 100%, 只对加热实例限到 15%, 且配置后无法放宽
    heater_pwm.set_max_duty_ratio(0.15f);
    heater_pwm.start_pwm();
    heater_pwm.set_duty_ration(0.0f);

    IMU_QuaternionEKF_Init(IMU_EKF_PROCESS_NOISE_Q1, IMU_EKF_PROCESS_NOISE_Q2, IMU_EKF_MEASURE_NOISE_R,
                           IMU_EKF_FADING_LAMBDA, IMU_DEFAULT_DT_S, IMU_EKF_ACCEL_LPF_S);
    // Kalman 矩阵用 pvPortMalloc 分配, 堆不足时返回 NULL(后续运算会写到地址 0),
    // 必须显式标记不可用并禁止输出姿态
    ekf_ready = IMU_QuaternionEKF_IsReady();

    pid_set_config_pos(&temperature_ctrl_config, IMU_TEMP_PID_KP, IMU_TEMP_PID_KI, IMU_TEMP_PID_KD,
                       IMU_TEMP_PID_INTEGRAL_LIMIT, IMU_TEMP_PID_OUT_MAX);
    pid_init(&temperature_pid, &temperature_ctrl_config);

    imu_frame.ref_temperature = IMU_TEMP_REF_C;

    imu_sensor_init();
}

/**
 * @brief DRDY 同步的采样与解算单步
 * @param drdy_dt    单帧 DRDY 周期(名义值), s —— 只用于野值判据
 * @param elapsed_dt 距上一拍真实经过的时间, s —— 只用于时间账本
 * @note  两个时间量必须分开传: 合成一个数会同时放宽野值阈值并把真实时间截短
 */
static void INS_Task(float drdy_dt, float elapsed_dt)
{
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 0.0f};

    // 单帧名义周期: 只用于野值判据与"本拍未解算"时的 dt 发布
    const float frame_dt = imu_limit_dt(drdy_dt);

    imu_frame.dt = frame_dt;

    const imu_status_e status = imu_get_status();
    if ((status == Imu_Status_Uninit) || (status == Imu_Status_Init_Fail))
    {
        // 传感器不可用: 加热保持关闭 + 时间账本作废, 周期性重试初始化
        imu_time_base_reset(&gyro_time_base);
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
        // 通信失败立即断开加热, 但不清积分器(避免偶发 SPI 错把温控打回冷启动)
        imu_heater_output_off();

        if (imu_get_status() == Imu_Status_Running)
        {
            // 本拍无数据可积分, 但时间确实过去了: 记进账本, 由下一个有效帧补上
            imu_time_base_accumulate(&gyro_time_base, elapsed_dt, 1U);
        }

        if (++read_fail_count >= IMU_READ_FAIL_LIMIT)
        {
            imu_set_status(Imu_Status_Init_Fail);
        }
        return;
    }

    if (!imu_array_is_finite(gyro, 3U) || !imu_array_is_finite(accel, 3U))
    {
        imu_frame.data_valid = 0U;
        // 与读失败同处理: 时间照样过去了, 记账等下一个有效帧补上
        imu_heater_output_off();
        if (imu_get_status() == Imu_Status_Running)
        {
            imu_time_base_accumulate(&gyro_time_base, elapsed_dt, 1U);
        }
        return;
    }

    imu_frame.data_valid = 1U;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        imu_frame.accel[i] = accel[i];
    }

    // 野值抑制必须在本拍任何积分之前
    imu_gyro_glitch_filter(&gyro_glitch, gyro, frame_dt, elapsed_dt);

    // 零偏/刻度换算放在 Running 分支内: 自标定需要使用原始域样本

    // 温控节拍: 先读温度再跑 PID, 避免用到上一次(10ms 前)的旧值
    const uint8_t temp_due = ((imu_frame.sample_count % IMU_TEMP_CTRL_DIVIDER) == 0U) ? 1U : 0U;
    uint8_t temp_ok = 1U;

    if (temp_due != 0U)
    {
        float temperature = imu_frame.temperature;

        if ((BMI088_read_temperature(&temperature) != BMI088_NO_ERROR) || !isfinite(temperature) ||
            (temperature < IMU_TEMP_VALID_MIN_C) || (temperature > IMU_TEMP_VALID_MAX_C))
        {
            // 温度不可信: 立即断加热, 姿态继续用本拍数据; 不计入 read_fail_count(那是整片掉线判据)
            temp_ok = 0U;
            imu_heater_safe_off();
        }
        else
        {
            imu_frame.temperature = temperature;
        }
    }

    read_fail_count = 0U;

    imu_frame.sample_count++;

    if ((temp_due != 0U) && (temp_ok != 0U))
    {
        imu_temperature_ctrl_step();
    }

    switch (imu_get_status())
    {
#if IMU_GYRO_AUTO_CALIB_ENABLE
        case Imu_Status_Wait_Temp:
            // 温度进入目标带宽并持续 IMU_TEMP_STABLE_S 后才开始标定(按真实时间)
            if (fabsf(imu_frame.temperature - imu_frame.ref_temperature) < IMU_TEMP_STABLE_BAND_C)
            {
                temp_stable_s += elapsed_dt;
                if (temp_stable_s >= IMU_TEMP_STABLE_S)
                {
                    temp_stable_s = 0.0f;
                    imu_gyro_calib_start();
                    imu_set_status(Imu_Status_Calibrating);
                }
            }
            else
            {
                temp_stable_s = 0.0f;
            }
            break;

        case Imu_Status_Calibrating:
            // 标定阶段不积分姿态, 账本保持空
            imu_time_base_reset(&gyro_time_base);
            imu_gyro_calib_step(gyro, elapsed_dt);
            break;
#endif

        case Imu_Status_Running:
        {
            // 先记账再取步长: 已到达的时间一帧都不丢, 余额留到后续几拍排空
            imu_time_base_accumulate(&gyro_time_base, elapsed_dt, 1U);
            const float integrate_dt = imu_time_base_take(&gyro_time_base);

            if (integrate_dt <= 0.0f)
            {
                // 账本为空: 不推进 EKF, 也不凭空造时间
                break;
            }

            imu_frame.dt = integrate_dt;

            // 零偏/刻度换算(两者均为原始域标定量, 先扣零偏再乘刻度)
            imu_gyro_calibrate(gyro, gyro_bias, gyro_scale);

            // 偏心补偿: 用已标定的角速度把加速度折算到刚体旋转中心(杆臂全 0 时直通)
            // 差分时间基用单帧周期, 真实间隔只用于长间隔判据(不能用积分步长, 见函数注释)
            imu_accel_lever_compensate(&accel_lever, accel, gyro, imu_lever_arm_m, frame_dt, elapsed_dt);
            for (uint8_t i = 0U; i < 3U; i++)
            {
                imu_frame.accel[i] = accel[i];
            }

            // 姿态失败只影响姿态输出: 温控只依赖 temperature, 不应被连带关闭
            if (imu_attitude_step(gyro, accel, integrate_dt) == 0U)
            {
                // imu_attitude_step 内部已复位 EKF, 旧时间不再有意义
                imu_time_base_reset(&gyro_time_base);
                imu_frame.dt = frame_dt;
            }
            break;
        }

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
            INS_Task(IMU_DEFAULT_DT_S, IMU_DEFAULT_DT_S);
            osDelay(1U);

            // 恢复时必须重新对齐序号与 DWT 基准, 否则离线期间累积的序号会被当成巨大的 dt
            last_drdy_sequence = gyro_drdy_sequence;
            gyro_drdy_dwt_started = 0U;
            imu_elapsed_dwt_started = 0U;
            continue;
        }

        const uint32_t flags = osThreadFlagsWait(IMU_DRDY_EVENT_FLAG, osFlagsWaitAny,
                                                   IMU_DRDY_TIMEOUT_TICKS);

        // 每次回到循环都量一次 DWT 间隔, 否则超时/假唤醒期间的时间会凭空消失
        const float elapsed_dt = imu_measure_elapsed_dt();

        const uint32_t sequence = gyro_drdy_sequence;
        const uint32_t pending = sequence - last_drdy_sequence;
        last_drdy_sequence = sequence;

        if ((flags & osFlagsError) != 0U)
        {
            imu_frame.data_valid = 0U;
            imu_heater_safe_off();

            if (++drdy_timeout_count >= IMU_DRDY_TIMEOUT_LIMIT)
            {
                drdy_timeout_count = 0U;
                imu_set_status(Imu_Status_Init_Fail);
                continue;
            }

            // DRDY 丢失不代表传感器停了, 照常读一次: 读成功则正常积分, 读失败由 INS_Task 记账
            INS_Task(IMU_DEFAULT_DT_S, elapsed_dt);
            continue;
        }

        if (pending == 0U)
        {
            // 假唤醒: 没有新帧可读, 但时间照样过去了, 记账等下一个有效帧
            imu_time_base_accumulate(&gyro_time_base, elapsed_dt, 1U);
            continue;
        }

        drdy_timeout_count = 0U;

        // 多个中断合并成一个通知时, 用 DWT 实测的总间隔推进一次 EKF
        INS_Task(gyro_drdy_dt, elapsed_dt);
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
 * @brief BMI088 陀螺数据就绪中断回调
 * @param GPIO_Pin 触发中断的引脚
 * @note  只做时间戳记录与线程通知, 不在 ISR 中执行 SPI 或浮点姿态解算
 */
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
