/**
 * @file reference_track_plan.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-10-18
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#include "reference_track_plan.h"
#include <math.h>
#include <cstring>
#include "basic_math.h"

/**
 * @brief 计算角度误差
 * @param angle 
 * @param target 
 * @param wrap do wrap the error to [-pi, pi] or not
 * @retval float: 
 */
static float angle_error(float angle, float target, bool wrap)
{
    if (wrap) 
    {
        /* Reducing each operand first also avoids overflow in subtraction. */
        float error = remainderf(remainderf(angle, 2.0f * PI) -
                          remainderf(target, 2.0f * PI), 2.0f * PI);
        if(!isfinite(error))
        {
            return 0.0f;
        }

        if(error <= -PI)
        {
            error += 2.0f * PI;
        }
        
        return error;
    }
    return angle - target;
}

/**
 * @brief 验证参考配置是否有效
 * @param c 
 * @retval true: 配置有效
 * @retval false: 配置无效
 */
static bool reference_config_valid(const reference_config_t *c)
{
    return c != NULL && isfinite(c->max_rate_rad_s) && c->max_rate_rad_s > 0.0f &&
    isfinite(c->max_accel_rad_s2) && c->max_accel_rad_s2 > 0.0f &&
    isfinite(c->position_gain_per_s) && c->position_gain_per_s > 0.0f &&
    isfinite(c->dt_min_s) && c->dt_min_s > 0.0f &&
    isfinite(c->dt_max_s) && c->dt_max_s >= c->dt_min_s;
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

    // 复制配置参数
    plan->config = copy;
    // wrap 模式下把初始参考角归一化到 [-pi, pi]，保证内部状态与"最短角"语义一致
    plan->value.angle_rad = copy.wrap_angle ? remainderf(initial_angle_rad, 2.0f * PI)
                                            : initial_angle_rad;
    plan->is_initialized = true;
    return true;
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
    return true;
}

/**
 * @brief 更新参考轨迹规划
 * @param plan 总体轨迹规划结构体
 * @param target_angle_rad 目标角度
 * @param dt_s 时间步长
 * @param out_value 输出的参考值
 * @retval true: 更新成功
 * @retval false: 更新失败
 */
bool reference_track_plan_update(reference_track_plan_t *plan, float target_angle_rad, 
                                float dt_s, reference_value_t *output)
{
    const reference_config_t *c;
    reference_value_t next;

    float error, desired_rate, desired_accel;

    // 检查输入参数
    if(output == NULL)
    {
        return false;
    }

    // 检测输出缓冲区"和"参考生成器的内部状态"是不是同一块内存。
    if(plan != NULL && output == &plan->value)
    {
        return false;
    }

    if(plan == NULL || !plan->is_initialized || !reference_config_valid(&plan->config) 
       || !isfinite(target_angle_rad) || !isfinite(dt_s))
    {
        return false;
    }

    c = &plan->config;

    if(dt_s < c->dt_min_s || dt_s > c->dt_max_s
       || !isfinite(plan->value.angle_rad) || !isfinite(plan->value.rate_rad_s))
    {
        return false;
    }

    // 当前目标角度与参考轨迹规划点的角度误差
    error = angle_error(target_angle_rad, plan->value.angle_rad, c->wrap_angle);

    // 根据角度误差计算出角速度(拟定的角速度)
    desired_rate = c->position_gain_per_s * error;

    // 防止非正常的角速度值(无理数)
    if (!isfinite(error) || !isfinite(desired_rate)) 
    {
        return false;
    }

    // 限制角速度在最大角速度范围内
    desired_rate = basic_math_constrain(desired_rate, -c->max_rate_rad_s, c->max_rate_rad_s);
    // 参考轨迹规划点的角速度与拟定的角速度之间的差值，计算出所需的角加速度
    desired_accel = (desired_rate - plan->value.rate_rad_s) / dt_s;
    if (!isfinite(desired_accel)) {
        return false;
    }

    // 限制角加速度在最大角加速度范围内
    next.accel_rad_s2 = basic_math_constrain(desired_accel, -c->max_accel_rad_s2, c->max_accel_rad_s2);
    // 角速度更新，当前角速度加上角加速度乘以时间步长
    next.rate_rad_s = basic_math_constrain(plan->value.rate_rad_s + next.accel_rad_s2 * dt_s, -c->max_rate_rad_s, c->max_rate_rad_s);
    // 实际的速率差反算加速度
    next.accel_rad_s2 = (next.rate_rad_s - plan->value.rate_rad_s) / dt_s;
    // 角度更新，梯形积分求解角度增量
    next.angle_rad = plan->value.angle_rad + 0.5f * (plan->value.rate_rad_s + next.rate_rad_s) * dt_s;

    // wrap 模式下把参考角归一化到 [-pi, pi]，避免内部角度随时间无界增长导致精度劣化，
    // 并保证输出角与"最短角"语义一致(跨 ±pi 时角度通道会有 2pi 跳变，速度/加速度连续不受影响)
    if (c->wrap_angle)
    {
        next.angle_rad = remainderf(next.angle_rad, 2.0f * PI);
    }

    // 规划结果的有效性检查，确保计算出的角度、角速度和角加速度都是有限的数值
    if (!isfinite(next.angle_rad) || !isfinite(next.rate_rad_s) ||
    !isfinite(next.accel_rad_s2)) 
    {
        return false;
    }

    // 更新参考轨迹规划点的状态为计算出的下一个状态，并将其输出
    plan->value = next;
    *output = next;
    return true;
}

