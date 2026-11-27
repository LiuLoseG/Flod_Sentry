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

// 调度周期相对变化超过该比例才重算滤波系数: 兼顾截止频率精度与 tanf 开销
#define FEEDFORWARD_FILTER_PERIOD_TOLERANCE 0.02f

/**
 * @brief 把低通滤波器的系数同步到当前截止频率与实际调度周期(不动滤波状态)
 * @note 系数由 (截止频率, 采样周期) 双线性算出, 采样周期取实际 dt, 因此调用频率变化不改变截止频率;
 *       只有参数确实变化时才重算, 避免每拍调用一次超越函数。
 */
static void feedforward_filter_prepare(low_pass_filter_class *filter, const float &cutoff_hz, const float &dt_s)
{
    if (filter->get_cutoff_frequency() != cutoff_hz)
    {
        filter->set_cutoff_frequency(cutoff_hz);
    }

    const float applied_period = filter->get_sampling_period();
    if (dt_s > 0.0f && fabsf(applied_period - dt_s) > applied_period * FEEDFORWARD_FILTER_PERIOD_TOLERANCE)
    {
        filter->set_sampling_period(dt_s);
    }
}

/**
 * @brief 送入一个新样本并取回滤波输出
 */
static float feedforward_filter_run(low_pass_filter_class *filter, const float &cutoff_hz, const float &dt_s, const float &input)
{
    feedforward_filter_prepare(filter, cutoff_hz, dt_s);
    filter->set_now_input(input);
    filter->low_calculate_callback();
    return filter->get_out();
}

/**
 * @brief 前馈限幅
 */
static float feedforward_limit(const float &value, const float &limit)
{
    return (limit > 0.0f) ? basic_math_constrain(value, -limit, limit) : value;
}

/**
 * @brief 配置合法范围检查
 */
static feedforward_tracker_config_t feedforward_tracker_config_sanitize(const feedforward_tracker_config_t &config)
{
    feedforward_tracker_config_t out;
    out.max_rate_rad_s = feedforward_positive_verify(config.max_rate_rad_s, 0.0f);
    out.max_accel_rad_s2 = feedforward_positive_verify(config.max_accel_rad_s2, 0.0f);
    // 截止频率非法时按"不滤波"处理: 参考信号宁可直通也不要被冻结在某个旧值上
    out.rate_filter_cutoff_hz = feedforward_positive_verify(config.rate_filter_cutoff_hz, 0.0f);
    out.accel_filter_cutoff_hz = feedforward_positive_verify(config.accel_filter_cutoff_hz, 0.0f);
    out.accel_jerk_limit_rad_s3 = feedforward_positive_verify(config.accel_jerk_limit_rad_s3, 0.0f);
    out.ref_dt_max_s = feedforward_positive_verify(config.ref_dt_max_s, FEEDFORWARD_DEFAULT_REF_DT_MAX_S);
    return out;
}

void feedforward_tracker_set_config(feedforward_tracker_t *tracker, const feedforward_tracker_config_t *config)
{
    if (tracker == NULL) return;
    static const feedforward_tracker_config_t default_config = 
    {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        FEEDFORWARD_DEFAULT_REF_DT_MAX_S,
    };
    tracker->config = feedforward_tracker_config_sanitize((config != NULL) ? *config : default_config);

    // 只刷新系数不动状态: 运行中替换配置不会让参考值跳变
    tracker->rate_filter.set_cutoff_frequency(tracker->config.rate_filter_cutoff_hz);
    tracker->accel_filter.set_cutoff_frequency(tracker->config.accel_filter_cutoff_hz);
}

void feedforward_tracker_reset(feedforward_tracker_t *tracker)
{
    if (tracker == NULL) return;
    tracker->rate_rad_s = 0.0f;
    tracker->accel_rad_s2 = 0.0f;
    tracker->primed = false;
    tracker->rate_filter.reset(0.0f);
    tracker->accel_filter.reset(0.0f);
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

        // 滤波器一起播种到当前稳态, 否则下一拍会因 x[n-1]/y[n-1] 缺历史出现半个阶跃
        if (cfg.rate_filter_cutoff_hz > 0.0f)
        {
            feedforward_filter_prepare(&tracker->rate_filter, cfg.rate_filter_cutoff_hz, dt_s);
            tracker->rate_filter.reset(rate_cmd);
        }
        if (cfg.accel_filter_cutoff_hz > 0.0f)
        {
            feedforward_filter_prepare(&tracker->accel_filter, cfg.accel_filter_cutoff_hz, dt_s);
            tracker->accel_filter.reset(0.0f);
        }
        return;
    }

    // 角速度参考: 一阶低通; 未配置截止频率时直通
    tracker->rate_rad_s = (cfg.rate_filter_cutoff_hz > 0.0f)
                              ? feedforward_filter_run(&tracker->rate_filter, cfg.rate_filter_cutoff_hz, dt_s, rate_cmd)
                              : rate_cmd;

    // 角加速度参考: 一阶低通后再限 jerk
    const float accel_target = (cfg.accel_filter_cutoff_hz > 0.0f)
                                   ? feedforward_filter_run(&tracker->accel_filter, cfg.accel_filter_cutoff_hz, dt_s, accel_cmd)
                                   : accel_cmd;

    float accel_delta = accel_target - tracker->accel_rad_s2;
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
