/**
 * @file reference_track_plan.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 带速度、加速度及jerk约束的参考轨迹跟踪
 * @date 2026-10-18
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#include "reference_track_plan.h"
#include <math.h>
#include <cstring>
#include "basic_math.h"

// 估算后续制动距离时只计入 99% 的 jerk 能力，给浮点运算留出余量。
static const float REFERENCE_BRAKING_JERK_RATIO = 0.99f;
// 拍末制动计算将速度上限向内收缩的比例。拍内极值仍按原速度上限检查。
static const float REFERENCE_RATE_MARGIN_RATIO = 1e-5f;
// 加速度上下界允许的倒置量（rad/s²）。在此容差内取上界，超过则拒绝本拍。
static const float REFERENCE_ACCEL_INTERVAL_TOLERANCE = 1e-4f;
// 目标速度前馈指向“背离目标”一侧时，按越过误差衰减的尺度，单位 rad。
// 硬置零相当于该尺度为 0，会在目标过零点把速度参考一步踩到 0，产生加速度脉冲。
static const float REFERENCE_FF_SUPPRESS_ERROR_RAD = 0.01f;
// 目标加速度“同号且超过能力上限”需要持续这么久才判定目标超出规划能力。
// 噪声(逐拍变号)与阶跃/零阶保持(单帧脉冲)都不会持续这么久; 例外的只有大周期(>10ms)下
// 单帧就满足时长, 故判定时另要求至少两帧。
static const float REFERENCE_OVER_CAPABILITY_TIME_S = 0.01f;

/** @brief 目标角减当前规划角；环绕模式下取 (-π, π]。 */
static float angle_error(float target_angle_rad, float now_angle_rad, bool wrap_angle)
{
    if (!wrap_angle)
    {
        return target_angle_rad - now_angle_rad;
    }

    // 分别归一化后再相减，避免大角度直接相减溢出。
    const float period = 2.0f * PI;
    float error = remainderf(remainderf(target_angle_rad, period) - remainderf(now_angle_rad, period), period);
    if (error <= -PI)
    {
        error += period;
    }
    return error;
}

/**
 * @brief 计算单侧速度边界允许的加速度
 * @note 由 a_next²/(2*j) + a_next*dt/2 <= headroom 求正根，避免相减消去。
 */
static float braking_accel_limit(float headroom, float braking_jerk, float dt)
{
    if (headroom < 0.0f)
    {
        return 2.0f * headroom / dt;
    }

    const float half_delta = 0.5f * braking_jerk * dt;
    const float braking_term = 2.0f * braking_jerk * headroom;
    return braking_term / (sqrtf(half_delta * half_delta + braking_term) + half_delta);
}

/** @brief 将拍末加速度需求限制在可行区间内；区间不可达时不写输出。 */
static bool limit_step_accel(const reference_config_t *c, const reference_value_t *current,
                             float dt, float requested_accel, float *accel)
{
    const float jerk = c->max_jerk_rad_s3;
    const float previous_accel = current->accel_rad_s2;
    // 从上一拍加速度出发，本拍最多改变 jerk * dt，再与绝对加速度范围取交集。
    const float delta = jerk * dt;
    float lower = fmaxf(-c->max_accel_rad_s2, previous_accel - delta);
    float upper = fminf(c->max_accel_rad_s2, previous_accel + delta);

    // 保留浮点裕量，避免变周期下落到下一拍不可制动的边界外。
    const float braking_jerk = jerk * REFERENCE_BRAKING_JERK_RATIO;
    const float safe_rate = c->max_rate_rad_s * (1.0f - REFERENCE_RATE_MARGIN_RATIO);
    const float half_rate_step = 0.5f * previous_accel * dt;
    upper = fminf(upper, braking_accel_limit(safe_rate - current->rate_rad_s - half_rate_step, braking_jerk, dt));
    lower = fmaxf(lower, -braking_accel_limit(safe_rate + current->rate_rad_s + half_rate_step, braking_jerk, dt));

    // 加速度拍内过零时速度存在极值，拍末可达性不足以约束该极值。
    if (previous_accel != 0.0f)
    {
        const bool positive = previous_accel > 0.0f;
        const float headroom = positive ? c->max_rate_rad_s - current->rate_rad_s
                                        : c->max_rate_rad_s + current->rate_rad_s;
        if (headroom <= 0.0f) return false;

        const float crossing_limit = fabsf(previous_accel)
            - previous_accel * previous_accel * dt / (2.0f * headroom);
        if (crossing_limit < 0.0f)
        {
            if (positive) upper = fminf(upper, crossing_limit);
            else lower = fmaxf(lower, -crossing_limit);
        }
    }

    if (!isfinite(lower) || !isfinite(upper) || lower > upper + REFERENCE_ACCEL_INTERVAL_TOLERANCE)
    {
        return false;
    }
    // 容差内的轻微倒置统一取上界，与第 3.2 节的选择规则一致。
    *accel = basic_math_constrain(requested_accel, fminf(lower, upper), upper);
    return true;
}

/** @brief 按跟踪误差计算 jerk 需求，再用可行的拍末加速度作恒 jerk 积分。
 *  @param control_error 进入位置/速率环节的误差；位置项暂挂时为 0。
 */
static bool jerk_step(const reference_config_t *c, const reference_value_t *current,
                      float control_error, float gain,
                      float target_rate, float target_accel,
                      float dt, reference_value_t *next, bool *limited)
{
    // 带宽同时受跟踪增益和采样周期限制。误差较大时再限制位置项 bandwidth³ * error。
    float bandwidth = fminf(gain, c->rate_servo_gain_per_s / 3.0f);
    bandwidth = fminf(bandwidth, 0.25f / dt);
    if (fabsf(control_error) > 1e-9f)
    {
        bandwidth = fminf(bandwidth, cbrtf(c->max_jerk_rad_s3 / fabsf(control_error)));
    }

    basic_math_constrain(&target_rate, -c->max_rate_rad_s, c->max_rate_rad_s);
    basic_math_constrain(&target_accel, -c->max_accel_rad_s2, c->max_accel_rad_s2);
    const float previous_accel = current->accel_rad_s2;
    const float requested_jerk = bandwidth * bandwidth * bandwidth * control_error
                               + 3.0f * bandwidth * bandwidth * (target_rate - current->rate_rad_s)
                               + 3.0f * bandwidth * (target_accel - previous_accel);
    if (!isfinite(requested_jerk)) return false;

    const float requested_accel = previous_accel + requested_jerk * dt;
    if (!limit_step_accel(c, current, dt, requested_accel, &next->accel_rad_s2)) return false;
    *limited = fabsf(next->accel_rad_s2 - requested_accel) > 1e-6f;
    next->rate_rad_s = current->rate_rad_s +
        0.5f * (previous_accel + next->accel_rad_s2) * dt;
    next->angle_rad = current->angle_rad + current->rate_rad_s * dt +
        (previous_accel / 3.0f + next->accel_rad_s2 / 6.0f) * dt * dt;
    return true;
}

/**
 * @brief 未启用jerk约束时的二阶兼容跟踪
 * @param control_error 进入位置/速率环节的误差；位置项暂挂时为 0。
 */
static bool accel_step(const reference_config_t *c, const reference_value_t *current,
                       float control_error, float gain,
                       float target_rate, float target_accel,
                       float dt, reference_value_t *next, bool *limited)
{
    float desired_rate = gain * control_error + target_rate;
    if (!isfinite(desired_rate)) return false;
    desired_rate = basic_math_constrain(desired_rate, -c->max_rate_rad_s, c->max_rate_rad_s);
    const float desired_accel = c->rate_servo_gain_per_s *
        (desired_rate - current->rate_rad_s) + target_accel;
    if (!isfinite(desired_accel)) return false;
    next->accel_rad_s2 = basic_math_constrain(desired_accel, -c->max_accel_rad_s2, c->max_accel_rad_s2);
    *limited = fabsf(next->accel_rad_s2 - desired_accel) > 1e-6f;
    const float raw_rate = current->rate_rad_s + next->accel_rad_s2 * dt;
    next->rate_rad_s = basic_math_constrain(raw_rate, -c->max_rate_rad_s, c->max_rate_rad_s);
    // 速度被截断时，按实际速度增量回算加速度，使两个输出描述同一拍运动。
    if (next->rate_rad_s != raw_rate)
    {
        *limited = true;
        next->accel_rad_s2 = (next->rate_rad_s - current->rate_rad_s) / dt;
    }
    next->angle_rad = current->angle_rad + 0.5f * (current->rate_rad_s + next->rate_rad_s) * dt;
    return true;
}

/**
 * @brief 成功更新后推进饱和保护与位置项暂挂状态
 * @param position_hold 本拍位置项是否暂挂; 暂挂期间不恢复位置增益, 避免解除瞬间以满增益冲击
 */
static void update_saturation(reference_track_plan_t *plan, bool position_hold, bool limited, float dt)
{
    const reference_config_t *c = &plan->config;
    plan->accel_saturated = limited;
    if (!c->accel_sat_guard_enable)
    {
        plan->gain_scale = 1.0f;
        plan->sat_frames = 0;
        plan->sat_time_s = 0.0f;
        return;
    }

    if (limited)
    {
        plan->gain_scale = fmaxf(c->sat_gain_scale, plan->gain_scale - c->sat_release_per_s * dt);
        plan->sat_time_s += dt;
        if (plan->sat_frames < 0xFFFFFFFFu) plan->sat_frames++;
    }
    else
    {
        if (!position_hold)
        {
            plan->gain_scale = fminf(1.0f, plan->gain_scale + c->sat_release_per_s * dt);
        }
        plan->sat_time_s = 0.0f;
        plan->sat_frames = 0;
    }
}

/**
 * @brief 验证参考配置是否有效
 * @param c 待检查的配置；关闭饱和保护时忽略其专用参数
 * @retval true: 配置有效
 * @retval false: 配置无效
 */
static bool reference_config_valid(const reference_config_t *c)
{
    if (c == nullptr) return false;

    if (!isfinite(c->max_rate_rad_s) || c->max_rate_rad_s <= 0.0f ||
        !isfinite(c->max_accel_rad_s2) || c->max_accel_rad_s2 <= 0.0f ||
        !isfinite(c->max_jerk_rad_s3)) return false;

    if (!isfinite(c->position_gain_per_s) || c->position_gain_per_s <= 0.0f ||
        !isfinite(c->rate_servo_gain_per_s) || c->rate_servo_gain_per_s <= 0.0f) return false;

    if (!isfinite(c->dt_min_s) || c->dt_min_s <= 0.0f ||
        !isfinite(c->dt_max_s) || c->dt_max_s < c->dt_min_s) return false;

    if (!c->accel_sat_guard_enable) return true;
    return isfinite(c->sat_gain_scale) && c->sat_gain_scale > 0.0f && c->sat_gain_scale <= 1.0f &&
           isfinite(c->sat_release_per_s) && c->sat_release_per_s >= 0.0f &&
           isfinite(c->sat_hold_time_s) && c->sat_hold_time_s >= 0.0f;
}

static bool reference_state_valid(const reference_track_plan_t *plan)
{
    return isfinite(plan->value.angle_rad) && isfinite(plan->value.rate_rad_s) &&
        isfinite(plan->value.accel_rad_s2) && isfinite(plan->gain_scale) &&
        plan->gain_scale > 0.0f && plan->gain_scale <= 1.0f &&
        isfinite(plan->sat_time_s) && plan->sat_time_s >= 0.0f &&
        isfinite(plan->position_hold_s) && plan->position_hold_s >= 0.0f &&
        isfinite(plan->over_capability_time_s) && plan->over_capability_time_s >= 0.0f &&
        isfinite(plan->last_rate_increment_rad_s) &&
        (!plan->config.accel_sat_guard_enable || plan->gain_scale >= plan->config.sat_gain_scale) &&
        (!plan->config.estimate_target_rate || !plan->target_history_valid ||
         (isfinite(plan->previous_target_angle_rad) && isfinite(plan->estimated_target_rate_rad_s) &&
          isfinite(plan->estimated_target_accel_rad_s2) && isfinite(plan->previous_measured_rate_rad_s))) &&
        fabsf(plan->value.rate_rad_s) <= plan->config.max_rate_rad_s &&
        fabsf(plan->value.accel_rad_s2) <= plan->config.max_accel_rad_s2;
}

/**
 * @brief 初始化参考轨迹规划
 * @param plan 总体轨迹规划结构体
 * @param config 轨迹规划参数配置
 * @param initial_angle_rad 初始角度(生成器的初始参考角),避免上电瞬间的目标跳变 / 冲击
 * @retval true: 初始化成功
 * @retval false: 初始化失败
 */
bool reference_track_plan_init(reference_track_plan_t *plan, const reference_config_t *config, float initial_angle_rad)
{
    reference_config_t copy;

    if (plan == NULL || config == NULL)
    {
        return false;
    }

    if(!reference_config_valid(config) || !isfinite(initial_angle_rad))
    {
        memset(plan, 0, sizeof(reference_track_plan_t));
        return false;
    }

    copy = *config;

    // 清空结构体，避免未初始化的成员变量导致的不可预期行为
    memset(plan, 0, sizeof(reference_track_plan_t));

    plan->config = copy;
    plan->is_initialized = true;
    return reference_track_plan_reset(plan, initial_angle_rad);
}

/**
 * @brief 重置参考轨迹规划
 * @param plan 总体轨迹规划结构体
 * @param angle_rad 目标角度(生成器的初始参考角),避免上电瞬间的目标跳变 / 冲击
 * @retval true: 重置成功
 * @retval false: 重置失败
 */
bool reference_track_plan_reset(reference_track_plan_t *plan, float angle_rad)
{
    if(plan == NULL || !plan->is_initialized || !isfinite(angle_rad))
    {
        return false;
    }

    // wrap 模式下把重置角归一化到 [-pi, pi]
    plan->value.angle_rad = plan->config.wrap_angle ? remainderf(angle_rad, 2.0f * PI)
                                                    : angle_rad;
    plan->value.rate_rad_s = 0.0f;
    plan->value.accel_rad_s2 = 0.0f;

    plan->gain_scale      = 1.0f;
    plan->accel_saturated = false;
    plan->sat_frames      = 0;
    plan->sat_time_s      = 0.0f;
    plan->position_hold   = false;
    plan->position_hold_s = 0.0f;
    plan->over_capability_time_s = 0.0f;
    plan->last_rate_increment_rad_s = 0.0f;
    plan->dt_used_s       = 0.0f;
    plan->previous_target_angle_rad = 0.0f;
    plan->estimated_target_rate_rad_s = 0.0f;
    plan->estimated_target_accel_rad_s2 = 0.0f;
    plan->previous_measured_rate_rad_s = 0.0f;
    plan->target_history_valid = false;

    return true;
}

/**
 * @brief 更新参考轨迹规划(含目标角速度/角加速度前馈)
 * @param plan 总体轨迹规划结构体
 * @param target_angle_rad 目标角度
 * @param omega_ff_rad_s 目标角速度前馈(rad/s), 与目标角同向
 * @param accel_ff_rad_s2 目标角加速度前馈(rad/s^2), 与目标角同向
 * @param dt_s 时间步长
 * @param output 输出的参考值
 * @retval true: 更新成功
 * @retval false: 更新失败
 */
bool reference_track_plan_update(reference_track_plan_t *plan, float target_angle_rad,
                                 float omega_ff_rad_s, float accel_ff_rad_s2,
                                 float dt_s, reference_value_t *output,
                                 bool rate_valid, bool accel_valid)
{
    reference_value_t next;
    bool limited = false;

    if (plan == NULL || output == NULL || output == &plan->value ||
        !plan->is_initialized || !reference_config_valid(&plan->config) ||
        !reference_state_valid(plan) || !isfinite(target_angle_rad) || !isfinite(dt_s))
    {
        return false;
    }

    if ((rate_valid && !isfinite(omega_ff_rad_s)) ||
        (accel_valid && !isfinite(accel_ff_rad_s2))) return false;
    if (!rate_valid) omega_ff_rad_s = 0.0f;
    if (!accel_valid) accel_ff_rad_s2 = 0.0f;

    const reference_config_t *c = &plan->config;
    if (dt_s < c->dt_min_s || dt_s > c->dt_max_s)
    {
        return false;
    }

    // 约束和积分统一使用真实时间。
    const float dt = dt_s;

    // 当前目标角度与参考轨迹规划点的角度误差
    const float error = angle_error(target_angle_rad, plan->value.angle_rad, c->wrap_angle);

    if (!isfinite(error)) return false;

    float estimated_rate = 0.0f;
    float estimated_accel = 0.0f;
    float measured_rate_history = 0.0f;
    float over_capability_time = 0.0f;
    float rate_increment = plan->last_rate_increment_rad_s;
    if (c->estimate_target_rate && plan->target_history_valid)
    {
        const float target_delta = angle_error(target_angle_rad, plan->previous_target_angle_rad, c->wrap_angle);
        if (!isfinite(target_delta)) return false;
        const float raw_rate = target_delta / dt;
        if (!isfinite(raw_rate)) return false;
        measured_rate_history = raw_rate;
        const float measured_rate = basic_math_constrain(raw_rate,
            -c->max_rate_rad_s, c->max_rate_rad_s);
        // 精确离散一阶滤波，不对角度做二次差分；滤波带宽不随饱和降增益改变。
        // 目标速度估计用于抵消输入变化造成的滞后，不应被位置环带宽限制；
        // 使用速度伺服带宽可减少换向时的过时前馈，避免参考角越过目标。
        const float alpha = -expm1f(-(c->rate_servo_gain_per_s / 3.0f) * dt);
        estimated_rate = plan->estimated_target_rate_rad_s +
            alpha * (measured_rate - plan->estimated_target_rate_rad_s);

        if (!rate_valid)
        {
            // 目标角加速度估计: 只给速度前馈时, 误差中会残留 3w*a_t/(s+w)³,
            // 在目标频段与目标同向相加, 表现为参考角幅值大于目标幅值(超调)。
            // 加速度由相邻两拍目标速度的一阶差分得到, 并做同带宽滤波。
            const float increment = raw_rate - plan->previous_measured_rate_rad_s;
            const float raw_accel = increment / dt;
            rate_increment = increment;
            if (isfinite(raw_accel) && fabsf(raw_accel) <= c->max_accel_rad_s2)
            {
                estimated_accel = plan->estimated_target_accel_rad_s2 +
                    alpha * (raw_accel - plan->estimated_target_accel_rad_s2);
            }
            else
            {
                // 阶跃/零阶保持/噪声使目标在速度上不可微, 此时二阶差分是脉冲而非加速度。
                // 不做加速度前馈, 并按滤波时间常数衰减到 0, 避免虚假加速度被反复注入。
                estimated_accel = plan->estimated_target_accel_rad_s2 * (1.0f - alpha);
            }

            // 超能力检测: 二阶差分“同号且超过加速度上限”必须持续一小段时间。
            // 测角噪声逐拍变号、阶跃与 100 Hz 阶梯只给出单帧脉冲, 都不会累计; 只有目标
            // 本身持续需要超能力的加速度(如 3 Hz / 0.3 rad 需 104 rad/s²)才会触发。
            if (isfinite(raw_accel) && increment * plan->last_rate_increment_rad_s > 0.0f &&
                fabsf(raw_accel) > c->max_accel_rad_s2)
            {
                over_capability_time = plan->over_capability_time_s + dt;
            }
        }
    }
    if (c->estimate_target_rate && !rate_valid) omega_ff_rad_s = estimated_rate;
    if (c->estimate_target_rate && !accel_valid && !rate_valid) accel_ff_rad_s2 = estimated_accel;
    if (c->estimate_target_rate && rate_valid)
    {
        // 用可信速度校准回退状态，避免短暂掉线时退回尚未收敛的角度差分。
        estimated_rate = basic_math_constrain(omega_ff_rad_s,
            -c->max_rate_rad_s, c->max_rate_rad_s);
        // 外部速度逐帧切换时不做二次差分，避免把源切换当成巨大的加速度。
        estimated_accel = 0.0f;
        measured_rate_history = estimated_rate;
    }

    if (c->estimate_target_rate && !rate_valid && error * omega_ff_rad_s < 0.0f)
    {
        // 前馈指向背离目标的另一侧(参考点已越过目标): 不平滑抑制而不硬置零 —— 置零会把速率参考
        // 一步踩到 0, 经 3w²(v_t-v) 放大成阶跃式 jerk(实测每过一次目标就产生一次加速度尖峰)。
        // 抑制力度随角误差平滑增长: 误差小(过零附近)几乎不抑制, 误差大(真正越过后)接近全抑制。
        const float ratio = error / REFERENCE_FF_SUPPRESS_ERROR_RAD;
        omega_ff_rad_s *= 1.0f / (1.0f + ratio * ratio);
    }

    // 位置项暂挂(超能力守护): 目标所需加速度已超出规划能力(加速度幅值低通接近上限), 或位置项
    // 连续受限使位置增益已压到下限。此时位置环不可能跟上目标, 继续用它追逐只会把参考点撞在
    // 限幅边界上来回穿越目标, 幅值反而大过目标(实测 2Hz->3Hz、0.3rad 时参考峰值 0.45 对目标 0.30)。
    // 暂挂期间只保留速度/加速度前馈, 让参考点"跟着目标走"而不去追它; 目标回到能力范围后再保持
    // sat_hold_time_s 即解除, 由位置项消除暂挂期间累积的偏差。
    const bool target_over_capability = c->estimate_target_rate && !rate_valid && !accel_valid &&
        over_capability_time >= fmaxf(REFERENCE_OVER_CAPABILITY_TIME_S, 2.0f * dt);
    const bool saturation_bottomed = plan->accel_saturated &&
        plan->gain_scale <= c->sat_gain_scale + 1e-4f;
    bool position_hold = plan->position_hold_s > 0.0f;
    bool position_hold_start = false;
    if (c->accel_sat_guard_enable && c->sat_hold_time_s > 0.0f && !position_hold &&
        (target_over_capability || saturation_bottomed))
    {
        position_hold = true;
        position_hold_start = true;
    }

    // 位置环增益; 暂挂时位置项不参与, 速率环仍需完整带宽
    float gain = c->position_gain_per_s;
    if (c->accel_sat_guard_enable && !position_hold)
    {
        gain *= plan->gain_scale;      // gain_scale ∈ [sat_gain_scale, 1.0]
    }

    if (c->max_jerk_rad_s3 > 0.0f)
    {
        if (!jerk_step(c, &plan->value, position_hold ? 0.0f : error, gain,
                       omega_ff_rad_s, accel_ff_rad_s2, dt, &next, &limited)) return false;
    }
    else
    {
        if (!accel_step(c, &plan->value, position_hold ? 0.0f : error, gain,
                        omega_ff_rad_s, accel_ff_rad_s2, dt, &next, &limited)) return false;
    }

    if (c->wrap_angle)
    {
        next.angle_rad = remainderf(next.angle_rad, 2.0f * PI);
    }

    if (!isfinite(next.angle_rad) || !isfinite(next.rate_rad_s) || !isfinite(next.accel_rad_s2))
    {
        return false;
    }

    update_saturation(plan, position_hold, limited, dt);
    if (position_hold)
    {
        if (position_hold_start || target_over_capability)
        {
            // 本拍刚进入, 或目标仍超能力: 续期, 暂挂期间不恢复位置增益
            plan->position_hold_s = c->sat_hold_time_s;
        }
        else
        {
            // 目标已回到能力范围: 再保持 sat_hold_time_s 后解除
            plan->position_hold_s = fmaxf(0.0f, plan->position_hold_s - dt);
        }
    }
    plan->position_hold = position_hold;
    plan->over_capability_time_s = over_capability_time;
    plan->last_rate_increment_rad_s = rate_increment;
    plan->dt_used_s = dt;
    plan->previous_target_angle_rad = target_angle_rad;
    plan->estimated_target_rate_rad_s = estimated_rate;
    plan->estimated_target_accel_rad_s2 = estimated_accel;
    plan->previous_measured_rate_rad_s = measured_rate_history;
    plan->target_history_valid = c->estimate_target_rate;
    plan->value = next;
    *output = next;
    return true;
}

