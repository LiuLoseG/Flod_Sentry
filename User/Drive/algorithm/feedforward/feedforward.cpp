/**
 * @file feedforward.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 轴模型前馈实现
 * @date 2026-09-01
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "feedforward.h"

/**
 * @brief 验证正数, 非有限或 <=0 时返回回退值
 */
static float feedforward_positive_verify(const float &value, const float &fallback)
{
    return (isfinite(value) && value > 0.0f) ? value : fallback;
}

/**
 * @brief 验证单位区间, 非有限时返回 0, 否则收口到 [0,1]
 */
static float feedforward_unit_alpha(const float &alpha)
{
    return isfinite(alpha) ? basic_math_constrain(alpha, 0.0f, 1.0f) : 0.0f;
}

/**
 * @brief 按 dt 归一化低通系数
 * @note alpha = 1 - (1 - alpha)^(dt / base_dt)
 */
static float feedforward_normalize_alpha(const float &alpha, const float &dt_s, const float &base_dt_s)
{
    if (alpha <= 0.0f) return 0.0f;
    if (alpha >= 1.0f) return 1.0f;
    return 1.0f - powf(1.0f - alpha, dt_s / base_dt_s);
}

/**
 * @brief 限幅
 */
static float feedforward_limit(const float &value, const float &limit)
{
    return (limit > 0.0f) ? basic_math_constrain(value, -limit, limit) : value;
}

/**
 * @brief 把配置收口到合法范围
 * @note init/set_config 时写回一次, 便于在 Ozone 中看到实际生效值;
 *       update 时再对本地副本校验一次, 防止运行中被改成非法值。
 */
static feedforward_tracker_config_t feedforward_tracker_config_sanitize(const feedforward_tracker_config_t &config)
{
    feedforward_tracker_config_t out;
    out.max_rate_rad_s = feedforward_positive_verify(config.max_rate_rad_s, 0.0f);
    out.max_accel_rad_s2 = feedforward_positive_verify(config.max_accel_rad_s2, 0.0f);
    out.rate_filter_alpha = feedforward_unit_alpha(config.rate_filter_alpha);
    out.accel_filter_alpha = feedforward_unit_alpha(config.accel_filter_alpha);
    out.accel_jerk_limit_rad_s3 = feedforward_positive_verify(config.accel_jerk_limit_rad_s3, 0.0f);
    out.ref_dt_max_s = feedforward_positive_verify(config.ref_dt_max_s, FEEDFORWARD_DEFAULT_REF_DT_MAX_S);
    out.filter_base_dt_s = feedforward_positive_verify(config.filter_base_dt_s, FEEDFORWARD_DEFAULT_FILTER_BASE_DT_S);
    return out;
}

void feedforward_tracker_set_config(feedforward_tracker_t *tracker, const feedforward_tracker_config_t *config)
{
    if (tracker == NULL) return;
    static const feedforward_tracker_config_t default_config = {
        0.0f,
        0.0f,
        1.0f,
        1.0f,
        0.0f,
        FEEDFORWARD_DEFAULT_REF_DT_MAX_S,
        FEEDFORWARD_DEFAULT_FILTER_BASE_DT_S,
    };
    tracker->config = feedforward_tracker_config_sanitize((config != NULL) ? *config : default_config);
}

void feedforward_tracker_reset(feedforward_tracker_t *tracker)
{
    if (tracker == NULL) return;
    tracker->rate_rad_s = 0.0f;
    tracker->accel_rad_s2 = 0.0f;
    tracker->primed = false;
}

/**
 * @brief 初始化参考信号调理器(写入配置并清零输出, 首拍直接对齐外部给定)
 * @note 目标角速度/角加速度由外部(轨迹规划、自瞄预测、遥控斜坡等)直接给定, 本模块不再对目标角差分,
 * @param tracker 
 * @param config 
 */
void feedforward_tracker_init(feedforward_tracker_t *tracker, const feedforward_tracker_config_t *config)
{
    if (tracker == NULL) return;
    feedforward_tracker_set_config(tracker, config);
    feedforward_tracker_reset(tracker);
}

/**
 * @brief 更新前馈参考信号调理器
 * @param tracker 
 * @param target_rate_rad_s 
 * @param target_accel_rad_s2 
 * @param dt_s 
 */
void feedforward_tracker_update(feedforward_tracker_t *tracker, const float &target_rate_rad_s, const float &target_accel_rad_s2, const float &dt_s)
{
    if (tracker == NULL) return;
    // 外部给定非法时宁可不补, 也不能沿用可能已过期的参考
    if (!isfinite(target_rate_rad_s) || !isfinite(target_accel_rad_s2) || !isfinite(dt_s))
    {
        feedforward_tracker_reset(tracker);
        return;
    }

    // 配置允许在线改写, 每拍按本地副本校验, 不信任字段当前值
    const feedforward_tracker_config_t cfg = feedforward_tracker_config_sanitize(tracker->config);

    const float rate_cmd = feedforward_limit(target_rate_rad_s, cfg.max_rate_rad_s);
    const float accel_cmd = feedforward_limit(target_accel_rad_s2, cfg.max_accel_rad_s2);

    // 首拍或调度异常: 速度直接对齐给定, 加速度从 0 起步, 之后由 jerk 限制爬升
    if (!tracker->primed || dt_s <= 0.0f || dt_s > cfg.ref_dt_max_s)
    {
        tracker->rate_rad_s = rate_cmd;
        tracker->accel_rad_s2 = 0.0f;
        tracker->primed = true;
        return;
    }

    tracker->rate_rad_s += feedforward_normalize_alpha(cfg.rate_filter_alpha, dt_s, cfg.filter_base_dt_s) * (rate_cmd - tracker->rate_rad_s);

    float accel_delta = feedforward_normalize_alpha(cfg.accel_filter_alpha, dt_s, cfg.filter_base_dt_s) * (accel_cmd - tracker->accel_rad_s2);
    if (cfg.accel_jerk_limit_rad_s3 > 0.0f)
    {
        accel_delta = feedforward_limit(accel_delta, cfg.accel_jerk_limit_rad_s3 * dt_s);
    }
    tracker->accel_rad_s2 += accel_delta;
    // 限幅在线收紧时, 已有的参考值也要立刻落回新范围
    tracker->rate_rad_s = feedforward_limit(tracker->rate_rad_s, cfg.max_rate_rad_s);
    tracker->accel_rad_s2 = feedforward_limit(tracker->accel_rad_s2, cfg.max_accel_rad_s2);

    if (!isfinite(tracker->rate_rad_s) || !isfinite(tracker->accel_rad_s2)) feedforward_tracker_reset(tracker);
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

    // 重力项 A*sin(q) + C*cos(q), 未限幅 
    const float gravity = param->grav_sin * sinf(angle_rad) + param->grav_cos * cosf(angle_rad);
    if (!isfinite(gravity)) return;

    float limited_dynamic = 0.0f;
    if (param->dynamic_max > 0.0f)
    {
        // eps越大 tanh 越平滑, 但摩擦补偿越弱; eps 越小 tanh 越接近 sign, 摩擦补偿越强, 但换向抖动越大
        const float eps_min = feedforward_positive_verify(param->eps_min_rad_s, FEEDFORWARD_DEFAULT_EPS_MIN_RAD_S);
        const float eps = (param->eps > eps_min) ? param->eps : eps_min;
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
    output->total_nm = (torque_limit_nm > 0.0f) ? basic_math_constrain(total, -torque_limit_nm, torque_limit_nm) : total;
}
