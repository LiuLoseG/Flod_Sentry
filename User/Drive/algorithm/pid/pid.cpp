/**
 * @file pid.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-07-16
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

/* Includes ------------------------------------------------------------------*/

#include "pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID初始化
 *
 * @param __k_p P值
 * @param __k_i I值
 * @param __k_d D值
 * @param __k_f 前馈
 * @param __i_out_max 积分限幅
 * @param __out_max 输出限幅
 * @param __d_t 时间片长度
 * @param __dead_zone 死区误差阈值
 * @param __i_variable_speed_a 变速积分误差阈值A
 * @param __i_variable_speed_b 变速积分误差阈值B
 * @param __i_separate_threshold 积分分离误差阈值
 * @param __d_first 是否开启微分先行
 * @param __d_filter 微分滤波器使能
 * @param __anti_windup 抗积分饱和方案
 * @param __anti_windup_gain 反算增益Kb, 单位1/s, 仅BACK_CALC使用
 * @param __anti_windup_out_max 抗饱和跟踪上限, 仅用于抗饱和判定, 0为沿用out_max
 */
void pid_class::init(const float &__k_p, const float &__k_i, const float &__k_d, const float &__k_f, 
                     const float &__i_out_max, const float &__out_max, const float &__d_t, const float &__dead_zone, 
                     const float &__i_variable_speed_a, const float &__i_variable_speed_b, const float &__i_separate_threshold, 
                     const pid_d_first_e &__d_first, const pid_d_filter_e &__d_filter, 
                     const pid_anti_windup_e &__anti_windup, const float &__anti_windup_gain, const float &__anti_windup_out_max)
{
    k_p = __k_p;
    k_i = __k_i;
    k_d = __k_d;
    k_f = __k_f;
    i_out_max = __i_out_max;
    out_max = __out_max;
    d_t = __d_t;
    dead_zone = __dead_zone;
    i_variable_speed_a = __i_variable_speed_a;
    i_variable_speed_b = __i_variable_speed_b;
    i_separate_threshold = __i_separate_threshold;
    d_first = __d_first;
    d_filter = __d_filter;
    anti_windup = __anti_windup;
    anti_windup_gain = __anti_windup_gain;
    anti_windup_out_max = __anti_windup_out_max;
}

/**
 * @brief PID复位, 清空积分、微分滤波器与历史状态, 参数保持不变
 */
void pid_class::pid_clear()
{
    integral_error = 0.0f;
    d_filter_out = 0.0f;

    term = {};

    out = 0.0f;
    out_unsaturated = 0.0f;
    pre_out = 0.0f;

    // 历史值对齐当前输入, 避免复位后首帧的微分/前馈冲击
    if (basic_math_abs(target - now) <= dead_zone)
    {
        pre_error = 0.0f;
    }
    else if (target - now > 0.0f)
    {
        pre_error = (target - now) - dead_zone;
    }
    else
    {
        pre_error = (target - now) + dead_zone;
    }

    // 历史值对齐当前输入, 避免复位后首帧的微分/前馈冲击
    pre_now = now;
    pre_target = target;
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 */
void pid_class::pid_calculate_callback()
{
    // 本步实际累加进积分的增量, 供抗积分饱和回退
    float integ_delta = 0.0f;
    float integral_limit = 0.0f;

    term.error = target - now;  // 当前误差
    term.abs_error = basic_math_abs(term.error);  // 计算绝对值误差

    // 判断死区
    if (term.abs_error <= dead_zone)
    {
        term.error = 0.0f;
    }
    // 减少控制器因微小扰动反复动作、降低抖振和积分累积（wind-up）风险，同时对正负误差对称处理。
    else if (term.error > 0.0f && term.abs_error > dead_zone)
    {
        term.error -= dead_zone;
    }
    else if (term.error < 0.0f && term.abs_error > dead_zone)
    {
        term.error += dead_zone;
    }
    term.abs_error = basic_math_abs(term.error);

    // 计算p项
    term.p_out = k_p * term.error;

    // 计算i项

    // 非变速积分
    if (i_variable_speed_b == 0.0f)
    {
        term.speed_ratio = 1.0f;
    }
    // 变速积分
    else
    {
        // 误差小于i_variable_speed_a时，积分全速工作
        if (term.abs_error <= i_variable_speed_a)
        {
            term.speed_ratio = 1.0f;
        }
        // 误差在两者之间时，积分按比例线性减速。
        else if (i_variable_speed_a < term.abs_error && term.abs_error < i_variable_speed_b)
        {
            term.speed_ratio = (i_variable_speed_b - term.abs_error) / (i_variable_speed_b - i_variable_speed_a);
        }
        // 误差大于i_variable_speed_b时，积分完全停止；
        else if (term.abs_error >= i_variable_speed_b)
        {
            term.speed_ratio = 0.0f;
        }
    }
    
    // 积分限幅 (k_i==0 时积分项恒为0, 不做除法, 避免除零)
    if (i_out_max != 0.0f && k_i != 0.0f)
    {
        integral_limit = basic_math_abs(i_out_max / k_i);
    }
    else
    {
        integral_limit = 0.0f;
    }

    // 限制积分值在合理范围内
    if (i_out_max != 0.0f && k_i != 0.0f)
    {
        basic_math_constrain(&integral_error, -integral_limit, integral_limit);
    }

    // 记录积分值变化前的值, 供抗积分饱和回退
    const float integral_before = integral_error;

    // 积分分离判断, 仅在积分分离阈值不为0时生效, 且误差超出阈值时清空积分并暂停抗积分饱和, 回到阈值内重新开始积分
    const bool integral_separated = (i_separate_threshold != 0.0f && term.abs_error >= i_separate_threshold);

    if (integral_separated)
    {
        // 在积分分离区间上
        integral_error = 0.0f;
        integ_delta = 0.0f;
    }
    else if (k_i != 0.0f)
    {
        // 未启用积分分离或误差位于允许积分区间
        integ_delta = term.speed_ratio * d_t * term.error;
        integral_error += integ_delta;
    }
    else
    {
        // k_i为0时无积分作用, 不累积, 避免启用积分瞬间输出跳变
        integ_delta = 0.0f;
    }

    // 积分限幅并更新积分项输出
    if (i_out_max != 0.0f && k_i != 0.0f)
    {
        basic_math_constrain(&integral_error, -integral_limit, integral_limit);
    }
    term.i_out = k_i * integral_error;

    // 记录限幅后的实际积分增量，供抗积分饱和回退
    if (!integral_separated)
    {
        integ_delta = integral_error - integral_before;
    }

    // 计算d项

    // 没有微分先行
    if (d_first == PID_D_First_DISABLE)
    {
        if(d_filter == PID_D_Filter_ENABLE)
        {
            float d_raw = (term.error - pre_error) / d_t;
            float alpha_d = d_t * d_filter_n / (1.0f + d_t * d_filter_n); // 微分滤波器系数

            d_filter_out = alpha_d * d_raw + (1.0f - alpha_d) * d_filter_out; // 微分滤波器输出
            
            term.d_out = k_d * d_filter_out;
        }

        else 
        {
            term.d_out = k_d * (term.error - pre_error) / d_t;
        }
    }
    
    else
    {
        // 微分先行使能
        if(d_filter == PID_D_Filter_ENABLE)
        {
            float d_raw = -(now - pre_now) / d_t;
            float alpha_d = d_t * d_filter_n / (1.0f + d_t * d_filter_n); // 微分滤波器系数

            d_filter_out = alpha_d * d_raw + (1.0f - alpha_d) * d_filter_out; // 微分滤波器输出
            
            term.d_out = k_d * d_filter_out;
        }

        else 
        {
            term.d_out = -k_d * (now - pre_now) / d_t;
        }
    }

    // 计算前馈
    term.f_out = k_f * (target - pre_target);

    // 计算输出
    out_unsaturated = term.p_out + term.i_out + term.d_out + term.f_out;

    // 输出命令只由 out_max 硬限幅；anti_windup_out_max 仅用于抗饱和跟踪判定。
    float out_limit = basic_math_abs(out_max);

    out = out_unsaturated;

    if (out_limit != 0.0f)
    {
        out = basic_math_constrain(out_unsaturated, -out_limit, out_limit);
    }

    // 抗积分饱和阈值可比硬输出限幅更严格，但不能改变下发输出。
    float anti_windup_limit = out_limit;
    if (anti_windup != PID_Anti_Windup_DISABLE && anti_windup_out_max != 0.0f &&
        (anti_windup_limit == 0.0f || basic_math_abs(anti_windup_out_max) < anti_windup_limit))
    {
        anti_windup_limit = basic_math_abs(anti_windup_out_max);
    }

    // 积分分离期间积分已被强制清零, 抗饱和不得再写回, 否则积分分离失效
    if (!integral_separated && anti_windup_limit != 0.0f && (out_unsaturated > anti_windup_limit || out_unsaturated < -anti_windup_limit)
        && anti_windup != PID_Anti_Windup_DISABLE && k_i != 0.0f)
    {
        // 跟踪输出与硬限幅输出分开，允许按下游限幅反馈积分。
        const float tracking_out = basic_math_constrain(out_unsaturated, -anti_windup_limit, anti_windup_limit);
        if (anti_windup == PID_Anti_Windup_CLAMP)
        {
            // 如果这一帧输出被限幅了，而且误差方向还在往限幅的方向推，那就把这一帧刚加进积分的量撤销掉
            if (k_i * integ_delta * (out_unsaturated - tracking_out) > 0.0f)
            {
                integral_error = integral_before;
                term.i_out = k_i * integral_error;
                out_unsaturated = term.p_out + term.i_out + term.d_out + term.f_out;
                if (out_limit != 0.0f)
                {
                    out = basic_math_constrain(out_unsaturated, -out_limit, out_limit);
                }
                else
                {
                    out = out_unsaturated;
                }
            }
        }

        // 输出被削掉多少，就把积分按比例往回拉多少
        else if (anti_windup == PID_Anti_Windup_BACK_CALC)
        {
            // 反算: integral += (Kb/k_i) * (u_sat - u_unsat) * d_t
            integral_error += (anti_windup_gain / k_i) * (tracking_out - out_unsaturated) * d_t;
            if (i_out_max != 0.0f)
            {
                basic_math_constrain(&integral_error, -integral_limit, integral_limit);
            }
            term.i_out = k_i * integral_error;

            // 反算改写了积分项, 同步本帧各项与本帧输出, 避免调试量互相矛盾
            out_unsaturated = term.p_out + term.i_out + term.d_out + term.f_out;
            if (out_limit != 0.0f)
            {
                out = basic_math_constrain(out_unsaturated, -out_limit, out_limit);
            }
            else
            {
                out = out_unsaturated;
            }
        }
    }

    // 善后工作
    pre_now = now;
    pre_target = target;
    pre_out = out;
    pre_error = term.error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/