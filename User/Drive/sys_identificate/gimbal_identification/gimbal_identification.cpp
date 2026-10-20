/**
 * @file gimbal_identification.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-10-19
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
 
#include "gimbal_identification.h"

#include <math.h>
#include <stddef.h>
#include <string.h>


#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "gimbal_identification requires finite-value checks; disable fast-math and finite-math-only"
#endif

#define MICROSECONDS_PER_SECOND_F 1000000.0f
#define SECONDS_PER_MICROSECOND_F 0.000001f

// "有限且为正" 的判据
static bool positive_finite(float value)
{
    return isfinite(value) && value > 0.0f;
}

/**
 * @brief 验证配置是否有效
 * @param config 配置指针
 * @return true 表示配置有效，false 表示配置无效
 * @note Yaw 模型 3 参（J,B,Fc），Pitch 5 参（+A,C）
 */
static bool config_valid(const gimbal_identification_config_t *config)
{
    float span;
    uint8_t count;
    if (config == NULL ||
        (unsigned)config->axis > (unsigned)GIMBAL_IDENTIFICATION_PITCH ||
        !positive_finite(config->window_s) ||
        !positive_finite(config->max_gap_s) ||
        !positive_finite(config->feedback_timeout_s) ||
        !positive_finite(config->friction_velocity_rad_s) ||
        config->window_s < SECONDS_PER_MICROSECOND_F ||
        config->max_gap_s < SECONDS_PER_MICROSECOND_F)
        return false;
    /* Leave the entire possible window inside the unambiguous half of the
     * uint32 clock range. This also bounds the float-to-integer conversions.
     */
    span = config->window_s + config->max_gap_s;
    if (!isfinite(span) || span >= 2147.0f)
        return false;
    count = config->axis == GIMBAL_IDENTIFICATION_YAW ? 3u : 5u;
    if (config->rls.parameter_count != count ||
        !positive_finite(config->rls.minimum[0]) ||
        !isfinite(config->rls.minimum[1]) || config->rls.minimum[1] < 0.0f ||
        !isfinite(config->rls.minimum[2]) || config->rls.minimum[2] < 0.0f ||
        !positive_finite(config->rls.dt_min_s) ||
        !positive_finite(config->rls.dt_max_s) ||
        config->rls.dt_min_s > config->window_s ||
        config->rls.dt_max_s < span)
        return false;
    return true;
}

/**
 * @brief 清掉当前积分窗口的全部中间量
 * @param identification 
 */
static void clear_window(gimbal_identification_t *identification)
{
    memset(&identification->previous, 0, sizeof(identification->previous));
    identification->start_angle_rad = 0.0f;
    identification->start_rate_rad_s = 0.0f;
    identification->integral_torque = 0.0f;
    identification->integral_friction = 0.0f;
    identification->integral_gravity_sin = 0.0f;
    identification->integral_gravity_cos = 0.0f;
    identification->window_start_us = 0u;
    identification->window_active = false;
}

/**
 * @brief 发生不连续（坏样本、超时、断段、故障）时的标准收尾：清窗口 + 让内层 RLS 清掉激励/创新历史，但保留已学参数与 P，然后刷新诊断快照。
 * @param identification 
 */
static void break_observations(gimbal_identification_t *identification)
{
    clear_window(identification);
    rls_last_clear_observations(&identification->estimator);
    (void)rls_last_snapshot(&identification->estimator,
                             &identification->diagnostics);
}

/**
 * @brief 锚定一个新的积分窗口
 * @param identification
 * @param sample
 */
static void anchor(gimbal_identification_t *identification,
                   const gimbal_identification_sample_t *sample)
{
    clear_window(identification);
    identification->previous = *sample;
    identification->start_angle_rad = sample->angle_rad;
    identification->start_rate_rad_s = sample->rate_rad_s;
    identification->window_start_us = sample->timestamp_us;
    identification->window_active = true;
}

/**
 * @brief 完成当前积分窗口并生成输出
 * @param identification
 * @param output
 * @param flags
 * @param window_duration_s
 * @return
 */
static uint32_t finish(const gimbal_identification_t *identification,
                       gimbal_identification_output_t *output,
                       uint32_t flags, float window_duration_s)
{
    if (output == NULL) return flags;
    memset(output, 0, sizeof(*output));
    output->flags = flags;
    output->window_duration_s = window_duration_s;
    if (identification == NULL || !identification->initialized)
        return flags;
    output->rls = identification->diagnostics;
    output->model.inertia_kg_m2 = output->rls.parameters[0];
    output->model.viscous_nm_s_rad = output->rls.parameters[1];
    output->model.coulomb_nm = output->rls.parameters[2];
    output->model.friction_velocity_rad_s =
        identification->config.friction_velocity_rad_s;
    if (identification->config.axis == GIMBAL_IDENTIFICATION_PITCH) {
        output->model.gravity_sin_nm = output->rls.parameters[3];
        output->model.gravity_cos_nm = output->rls.parameters[4];
    }
    output->window_updated = output->rls.updated;
    output->ready = output->rls.updated && output->rls.ready;
    return flags;
}

/**
 * @brief 给出一套可编译、可跑通的起步值(测试用)
 * @param config 
 * @param axis 
 */
void gimbal_identification_default_config(gimbal_identification_config_t *config,
                                          gimbal_identification_axis_t axis)
{
    uint8_t count;
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->axis = axis;
    config->window_s = 0.05f;                 // 50 ms：覆盖约 50 个 1 kHz 控制周期，降低单帧噪声
    config->max_gap_s = 0.02f;                // 20 ms：允许少量任务抖动，但不跨越明显丢帧
    config->feedback_timeout_s = 0.02f;       // 反馈超过 20 ms 未更新就丢弃，避免错位回归
    config->friction_velocity_rad_s = 0.08f;  // tanh 平滑 sign(omega)，约覆盖低速摩擦过渡区
    count = axis == GIMBAL_IDENTIFICATION_YAW ? 3u : 5u;
    rls_last_default_config(&config->rls, count);
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
    if (axis == GIMBAL_IDENTIFICATION_PITCH) {
        config->rls.initial[3] = 0.0f;       // sin 重力项无先验方向，使用零中心
        config->rls.initial[4] = 0.1f;       // cos 重力项提供轻微非零起点，避免初期完全失激励
        config->rls.minimum[3] = -5.0f;
        config->rls.minimum[4] = -5.0f;
        config->rls.maximum[3] = 5.0f;
        config->rls.maximum[4] = 5.0f;
    }
    config->rls.dt_min_s = 0.000001f;
    config->rls.dt_max_s = 0.1f;
}

/**
 * @brief 初始化 gimbal_identification 实例
 * @param identification
 * @param config
 * @return
 */
bool gimbal_identification_init(gimbal_identification_t *identification,
                                const gimbal_identification_config_t *config)
{
    gimbal_identification_config_t copy;
    if (identification == NULL) return false;
    if (!config_valid(config)) {
        memset(identification, 0, sizeof(*identification));
        return false;
    }
    copy = *config; /* Permit reinitialization with &identification->config. */
    memset(identification, 0, sizeof(*identification));
    identification->config = copy;
    identification->window_us =
        (uint32_t)ceilf(copy.window_s * MICROSECONDS_PER_SECOND_F);
    identification->max_gap_us =
        (uint32_t)floorf(copy.max_gap_s * MICROSECONDS_PER_SECOND_F);
    if (identification->window_us == 0u || identification->max_gap_us == 0u ||
        !rls_last_init(&identification->estimator, &copy.rls) ||
        !rls_last_snapshot(&identification->estimator,
                             &identification->diagnostics)) {
        memset(identification, 0, sizeof(*identification));
        return false;
    }
    identification->initialized = true;
    return true;
}

/**
 * @brief 重置 gimbal_identification 实例
 * @param identification
 * @note 把 RLS 恢复成配置初值
 */
void gimbal_identification_reset(gimbal_identification_t *identification)
{
    if (identification == NULL || !identification->initialized) return;
    clear_window(identification);
    identification->last_timestamp_us = 0u;
    identification->have_timestamp = false;
    rls_last_reset(&identification->estimator);
    (void)rls_last_snapshot(&identification->estimator,
                             &identification->diagnostics);
}

/**
 * @brief 清除 gimbal_identification 实例的观测数据
 * @param identification
 * @note 保留已学参数/P，只清窗口、激励/创新历史与时间连续性。用于"工况不连续后重新积累数据"，
 *       但不想丢掉之前学到的模型。
 */
void gimbal_identification_clear_observations(gimbal_identification_t *identification)
{
    if (identification == NULL || !identification->initialized) return;
    break_observations(identification);
    identification->last_timestamp_us = 0u;
    identification->have_timestamp = false;
}

/**
 * @brief 对观测数据进行质量检查
 * @param config
 * @param sample
 * @return
 */
static uint32_t qualify(const gimbal_identification_config_t *config,
                        const gimbal_identification_sample_t *sample)
{
    uint32_t flags = 0u;
    if (!sample->enabled) flags |= GIMBAL_IDENTIFICATION_DISABLED;
    if (!sample->valid || !isfinite(sample->angle_rad) ||
        !isfinite(sample->rate_rad_s) || !isfinite(sample->gravity_angle_rad) ||
        !isfinite(sample->torque_nm) || !isfinite(sample->feedback_age_s) ||
        sample->feedback_age_s < 0.0f)
        flags |= GIMBAL_IDENTIFICATION_BAD_INPUT;
    if (sample->saturated) flags |= GIMBAL_IDENTIFICATION_SATURATED;
    if (isfinite(sample->feedback_age_s) &&
        sample->feedback_age_s > config->feedback_timeout_s)
        flags |= GIMBAL_IDENTIFICATION_STALE_FEEDBACK;
    if (!sample->operating_conditions_valid)
        flags |= GIMBAL_IDENTIFICATION_UNQUALIFIED_OPERATION;
    if (!sample->torque_calibrated)
        flags |= GIMBAL_IDENTIFICATION_TORQUE_UNCALIBRATED;
    if (!sample->torque_time_aligned)
        flags |= GIMBAL_IDENTIFICATION_TORQUE_UNALIGNED;
    if (sample->configuration_id != config->configuration_id)
        flags |= GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH;
    return flags;
}

/**
 * @brief 对观测数据进行积分
 * @param identification
 * @param sample
 * @param dt_s
 * @note 对窗口内每个区间做梯形法累加四个积分
    ∫τ dt（力矩）
    ∫tanh(ω/ε) dt（库仑摩擦项的同构积分）
    ∫sin(q_g) dt、∫cos(q_g) dt（重力项，仅 Pitch）
    tanh 必须逐样本算再积分，不能先平均速度再 tanh
 * @return
 */
static bool integrate(gimbal_identification_t *identification,
                       const gimbal_identification_sample_t *sample,
                       float dt_s)
{
    const gimbal_identification_sample_t *previous = &identification->previous;
    const float epsilon = identification->config.friction_velocity_rad_s;
    const float previous_argument = previous->rate_rad_s / epsilon;
    const float argument = sample->rate_rad_s / epsilon;
    float torque, friction, gravity_sin, gravity_cos;
    if (!isfinite(previous_argument) || !isfinite(argument)) return false;
    /* Half each operand before adding: avoid an avoidable overflow in a+b. */
    torque = identification->integral_torque +
        (0.5f * previous->torque_nm + 0.5f * sample->torque_nm) * dt_s;
    friction = identification->integral_friction +
        (0.5f * tanhf(previous_argument) + 0.5f * tanhf(argument)) * dt_s;
    gravity_sin = identification->integral_gravity_sin;
    gravity_cos = identification->integral_gravity_cos;
    if (identification->config.axis == GIMBAL_IDENTIFICATION_PITCH) {
        gravity_sin += (0.5f * sinf(previous->gravity_angle_rad) +
                        0.5f * sinf(sample->gravity_angle_rad)) * dt_s;
        gravity_cos += (0.5f * cosf(previous->gravity_angle_rad) +
                        0.5f * cosf(sample->gravity_angle_rad)) * dt_s;
    }
    if (!isfinite(torque) || !isfinite(friction) ||
        !isfinite(gravity_sin) || !isfinite(gravity_cos)) return false;
    identification->integral_torque = torque;
    identification->integral_friction = friction;
    identification->integral_gravity_sin = gravity_sin;
    identification->integral_gravity_cos = gravity_cos;
    return true;
}

/**
 * @brief 更新 gimbal_identification 实例
 * @param identification
 * @param sample
 * @param output
 * @return
 */
uint32_t gimbal_identification_update(gimbal_identification_t *identification,
                                      const gimbal_identification_sample_t *sample,
                                      gimbal_identification_output_t *output)
{
    rls_last_input_t regression;
    uint32_t flags, elapsed_us, interval_us, rls_flags;
    float duration_s;
    unsigned index;
    const uint32_t rls_faults = RLS_LAST_DISABLED | RLS_LAST_BAD_CONFIG |
        RLS_LAST_BAD_INPUT | RLS_LAST_BAD_DT | RLS_LAST_NUMERIC_FAULT |
        RLS_LAST_COVARIANCE_FAULT;

    if (identification == NULL || !identification->initialized)
        return finish(NULL, output, GIMBAL_IDENTIFICATION_BAD_CONFIG, 0.0f);
    if (!config_valid(&identification->config)) {
        break_observations(identification);
        return finish(NULL, output, GIMBAL_IDENTIFICATION_BAD_CONFIG, 0.0f);
    }
    if (!rls_last_snapshot(&identification->estimator,
                             &identification->diagnostics)) {
        break_observations(identification);
        return finish(identification, output, GIMBAL_IDENTIFICATION_RLS_REJECTED, 0.0f);
    }
    if (sample == NULL || output == NULL) {
        break_observations(identification);
        return finish(identification, output, GIMBAL_IDENTIFICATION_BAD_INPUT, 0.0f);
    }
    flags = qualify(&identification->config, sample);
    if ((flags & GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH) != 0u) {
        /* An unrelated configuration must not advance this instance's clock. */
        break_observations(identification);
        return finish(identification, output, flags, 0.0f);
    }
    if (identification->have_timestamp) {
        interval_us = sample->timestamp_us - identification->last_timestamp_us;
        if (interval_us == 0u || interval_us > UINT32_MAX / 2u) {
            /* Keep the last accepted clock position; do not rewind on error. */
            flags |= GIMBAL_IDENTIFICATION_BAD_TIMESTAMP;
            break_observations(identification);
            return finish(identification, output, flags, 0.0f);
        }
        if (interval_us > identification->max_gap_us)
            flags |= GIMBAL_IDENTIFICATION_GAP;
    }
    identification->last_timestamp_us = sample->timestamp_us;
    identification->have_timestamp = true;
    if ((flags & ~GIMBAL_IDENTIFICATION_GAP) != 0u) {
        break_observations(identification);
        return finish(identification, output, flags, 0.0f);
    }
    if (identification->window_active &&
        sample->segment_id != identification->previous.segment_id)
        flags |= GIMBAL_IDENTIFICATION_SEGMENT_CHANGE;
    if (flags != 0u)
        break_observations(identification);
    if (!identification->window_active) {
        anchor(identification, sample);
        return finish(identification, output, flags, 0.0f);
    }

    interval_us = sample->timestamp_us - identification->previous.timestamp_us;
    if (interval_us == 0u || interval_us > identification->max_gap_us ||
        !integrate(identification, sample,
                    (float)interval_us * SECONDS_PER_MICROSECOND_F)) {
        break_observations(identification);
        return finish(identification, output,
                       flags | GIMBAL_IDENTIFICATION_NUMERIC_FAULT, 0.0f);
    }
    identification->previous = *sample;
    elapsed_us = sample->timestamp_us - identification->window_start_us;
    if (elapsed_us < identification->window_us)
        return finish(identification, output, flags, 0.0f);

    duration_s = (float)elapsed_us * SECONDS_PER_MICROSECOND_F;
    memset(&regression, 0, sizeof(regression));
    regression.features[0] =
        (sample->rate_rad_s - identification->start_rate_rad_s) / duration_s;
    regression.features[1] =
        (sample->angle_rad - identification->start_angle_rad) / duration_s;
    regression.features[2] = identification->integral_friction / duration_s;
    if (identification->config.axis == GIMBAL_IDENTIFICATION_PITCH) {
        regression.features[3] = identification->integral_gravity_sin / duration_s;
        regression.features[4] = identification->integral_gravity_cos / duration_s;
    }
    regression.target = identification->integral_torque / duration_s;
    regression.dt_s = duration_s;
    regression.enabled = true;
    regression.valid = isfinite(regression.target) && positive_finite(duration_s);
    for (index = 0u; index < identification->config.rls.parameter_count; ++index)
        regression.valid = regression.valid && isfinite(regression.features[index]);
    if (!regression.valid) {
        break_observations(identification);
        return finish(identification, output,
                       flags | GIMBAL_IDENTIFICATION_NUMERIC_FAULT, 0.0f);
    }
    rls_flags = rls_last_update(&identification->estimator, &regression,
                                  &identification->diagnostics);
    if ((rls_flags & rls_faults) != 0u) {
        break_observations(identification);
        identification->diagnostics.flags = rls_flags;
        return finish(identification, output,
                       flags | GIMBAL_IDENTIFICATION_RLS_REJECTED, duration_s);
    }
    /* Low excitation must retain the RLS feature history so PE can eventually
     * accumulate. It is not a discontinuity. Projection similarly retains the
     * accepted bounded estimate, but the core marks that update not ready.
     */
    anchor(identification, sample);
    return finish(identification, output, flags, duration_s);
}
