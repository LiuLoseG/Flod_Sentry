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

/**
 * @brief 用给定速率播种参考跟踪器
 */
void feedforward_ref_reset(feedforward_ref_t *ref, const float &rate_rad_s)
{
    if (ref == NULL) return;
    ref->rate_rad_s = isfinite(rate_rad_s) ? rate_rad_s : 0.0f;
    ref->accel_rad_s2 = 0.0f;
    ref->primed = false;
}

/**
 * @brief 限斜率跟踪目标速率
 * @note 首拍只对齐不产生加速度: 播种速率可能来自实测(与目标差很远), 第一拍就差分会得到假加速度。
 */
void feedforward_ref_track(feedforward_ref_t *ref, const float &target_rate_rad_s, const float &dt_s, const float &max_accel_rad_s2)
{
    if (ref == NULL) return;

    ref->accel_rad_s2 = 0.0f;

    if (!ref->primed)
    {
        // 首拍: 保持播种值, 下一拍才开始跟踪
        ref->primed = true;
        return;
    }

    if (!isfinite(target_rate_rad_s) || !isfinite(max_accel_rad_s2))
    {
        return; // 目标或加速度上限非法: 保持上一拍速率, 不产生加速度
    }

    if (!(max_accel_rad_s2 > 0.0f))
    {
        // 未配置斜率限制: 参考直接跟随目标, 不输出加速度前馈
        ref->rate_rad_s = target_rate_rad_s;
        return;
    }

    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > FEEDFORWARD_REF_DT_MAX_S)
    {
        return; // 调度异常: 本拍不推进参考, 也不产生加速度
    }

    const float max_step = max_accel_rad_s2 * dt_s;
    const float delta = basic_math_constrain(target_rate_rad_s - ref->rate_rad_s, -max_step, max_step);
    const float next_rate = ref->rate_rad_s + delta;
    // 用实际可表示的速率增量差分, 并抑制浮点舍入造成的加速度越界。
    const float accel = (next_rate - ref->rate_rad_s) / dt_s;
    if (!isfinite(next_rate) || !isfinite(accel)) return;
    ref->rate_rad_s = next_rate;
    ref->accel_rad_s2 = basic_math_constrain(accel, -max_accel_rad_s2, max_accel_rad_s2);
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
