/**
 * @file feedforward.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 轴模型前馈实现
 * @date 2026-09-01
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "feedforward.h"

/* 参考速率差分的合法调用间隔上限, s
 * 超过它说明任务被长时间抢占, 这一拍的差分结果没有物理意义(会把整段速度变化压进一拍)
 */
#define FEEDFORWARD_REF_DT_MAX_S 0.02f

/* tanh 速度尺度下限, rad/s
 * eps 太小时 tanh 退化成 sign(阶跃), 摩擦前馈变成 bang-bang, 换向会抖
 */
#define FEEDFORWARD_EPS_MIN 0.001f

void feedforward_tracker_init(feedforward_tracker_t *tracker,
                              const float &angle_rad,
                              const float &rate_rad_s,
                              const float &max_rate_rad_s,
                              const float &max_accel_rad_s2,
                              const float &rate_filter_alpha,
                              const float &accel_filter_alpha,
                              const float &accel_jerk_limit_rad_s3)
{
    if (tracker == NULL) return;
    tracker->max_rate_rad_s = isfinite(max_rate_rad_s) && max_rate_rad_s > 0.0f ? max_rate_rad_s : 0.0f;
    tracker->max_accel_rad_s2 = isfinite(max_accel_rad_s2) && max_accel_rad_s2 > 0.0f ? max_accel_rad_s2 : 0.0f;
    tracker->rate_filter_alpha = isfinite(rate_filter_alpha) ? basic_math_constrain(rate_filter_alpha, 0.0f, 1.0f) : 0.0f;
    tracker->accel_filter_alpha = isfinite(accel_filter_alpha) ? basic_math_constrain(accel_filter_alpha, 0.0f, 1.0f) : 0.0f;
    tracker->accel_jerk_limit_rad_s3 = isfinite(accel_jerk_limit_rad_s3) && accel_jerk_limit_rad_s3 > 0.0f ? accel_jerk_limit_rad_s3 : 0.0f;
    tracker->last_angle_rad = isfinite(angle_rad) ? angle_rad : 0.0f;
    feedforward_tracker_reset(tracker, rate_rad_s);
}

void feedforward_tracker_reset(feedforward_tracker_t *tracker, const float &rate_rad_s)
{
    if (tracker == NULL) return;
    tracker->rate_rad_s = isfinite(rate_rad_s) ? rate_rad_s : 0.0f;
    tracker->accel_rad_s2 = 0.0f;
    tracker->accel_filtered_rad_s2 = 0.0f;
    tracker->primed = false;
}

void feedforward_tracker_update(feedforward_tracker_t *tracker,
                                const float &target_angle_rad,
                                const float &measured_rate_rad_s,
                                const float &dt_s)
{
    if (tracker == NULL || !isfinite(target_angle_rad) || !isfinite(measured_rate_rad_s) ||
        !isfinite(dt_s)) return;
    if (!tracker->primed)
    {
        tracker->last_angle_rad = target_angle_rad;
        tracker->primed = true;
        tracker->accel_rad_s2 = 0.0f;
        tracker->accel_filtered_rad_s2 = 0.0f;
        return;
    }
    if (dt_s <= 0.0f || dt_s > FEEDFORWARD_REF_DT_MAX_S)
    {
        tracker->last_angle_rad = target_angle_rad;
        tracker->accel_rad_s2 = 0.0f;
        tracker->accel_filtered_rad_s2 = 0.0f;
        tracker->primed = false;
        return;
    }
    const float raw_rate = tracker->max_rate_rad_s > 0.0f
                               ? basic_math_constrain((target_angle_rad - tracker->last_angle_rad) / dt_s, -tracker->max_rate_rad_s, tracker->max_rate_rad_s)
                               : 0.0f;
    tracker->rate_rad_s += tracker->rate_filter_alpha * (raw_rate - tracker->rate_rad_s);

    tracker->last_angle_rad = target_angle_rad;
    tracker->accel_rad_s2 = 0.0f;
    if (tracker->max_accel_rad_s2 <= 0.0f) return;
    const float max_step = tracker->max_accel_rad_s2 * dt_s;

    const float delta = basic_math_constrain(tracker->rate_rad_s - measured_rate_rad_s, -max_step, max_step);

    const float next_rate = measured_rate_rad_s + delta;

    tracker->accel_rad_s2 = basic_math_constrain((next_rate - tracker->rate_rad_s) / dt_s, -tracker->max_accel_rad_s2, tracker->max_accel_rad_s2);

    tracker->rate_rad_s = next_rate;
    // 按实际采样周期归一化低通系数，避免周期抖动改变滤波器动态。
    const float normalized_alpha = (tracker->accel_filter_alpha <= 0.0f) ? 0.0f : (tracker->accel_filter_alpha >= 1.0f) ? 1.0f
                                             : 1.0f - powf(1.0f - tracker->accel_filter_alpha, dt_s / 0.01f);

    const float filtered_target = tracker->accel_filtered_rad_s2 + normalized_alpha * (tracker->accel_rad_s2 - tracker->accel_filtered_rad_s2);

    float filtered_delta = filtered_target - tracker->accel_filtered_rad_s2;

    if (tracker->accel_jerk_limit_rad_s3 > 0.0f)
    {
        const float max_delta = tracker->accel_jerk_limit_rad_s3 * dt_s;
        filtered_delta = basic_math_constrain(filtered_delta, -max_delta, max_delta);
    }
    
    tracker->accel_filtered_rad_s2 += filtered_delta;
    if (!isfinite(tracker->accel_filtered_rad_s2)) tracker->accel_filtered_rad_s2 = 0.0f;
}

/**
 * @brief 计算单轴力矩前馈
 */
void feedforward_calculate(const feedforward_param_t *param,
                           const float &angle_rad,
                           const float &rate_rad_s,
                           const float &accel_rad_s2,
                           const float &torque_limit_nm,
                           feedforward_output_t *output)
{
    if (output == NULL) return;

    output->dynamic_nm = 0.0f;
    output->gravity_nm = 0.0f;
    output->total_nm = 0.0f;

    if (param == NULL) return;
    if (!isfinite(angle_rad) || !isfinite(rate_rad_s) || !isfinite(accel_rad_s2) || !isfinite(torque_limit_nm)) return;
    if (!isfinite(param->j) || !isfinite(param->b) || !isfinite(param->fc) || !isfinite(param->eps) ||
        !isfinite(param->grav_sin) || !isfinite(param->grav_cos) || !isfinite(param->dynamic_max)) return;

    const float gravity = param->grav_sin * sinf(angle_rad) + param->grav_cos * cosf(angle_rad);
    if (!isfinite(gravity)) return;

    float limited_dynamic = 0.0f;
    if (param->dynamic_max > 0.0f)
    {
        const float eps = (param->eps > FEEDFORWARD_EPS_MIN) ? param->eps : FEEDFORWARD_EPS_MIN;
        // tanh 在 0 附近本身连续, 不能再叠死区: 死区会在边界处凭空产生力矩台阶
        const float coulomb = param->fc * tanhf(rate_rad_s / eps);
        const float dynamic = param->j * accel_rad_s2 + param->b * rate_rad_s + coulomb;
        if (!isfinite(dynamic)) return;
        limited_dynamic = basic_math_constrain(dynamic, -param->dynamic_max, param->dynamic_max);
    }

    const float total = limited_dynamic + gravity;
    if (!isfinite(total)) return;

    output->dynamic_nm = limited_dynamic;
    output->gravity_nm = gravity;
    output->total_nm = (torque_limit_nm > 0.0f)
                           ? basic_math_constrain(total, -torque_limit_nm, torque_limit_nm)
                           : total;
}
