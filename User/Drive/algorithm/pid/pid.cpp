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
 */
void pid_class::init(const float &__k_p, const float &__k_i, const float &__k_d, const float &__k_f, const float &__i_out_max, const float &__out_max, const float &__d_t, const float &__dead_zone, const float &__i_variable_speed_a, const float &__i_variable_speed_b, const float &__i_separate_threshold, const pid_d_first_e &__d_first)
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
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 */
void pid_class::pid_calculate_callback()
{
    term.error = target - now;  // 当前误差
    term.abs_error = basic_math_abs(term.error);  // 计算绝对值误差

    // 判断死区
    if (term.abs_error < dead_zone)
    {
        target = now;
        term.error = 0.0f;
        term.abs_error = 0.0f;
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

    // 计算p项

    term.p_out = k_p * term.error;

    // 计算i项

    // 非变速积分
    if (i_variable_speed_a == 0.0f && i_variable_speed_b == 0.0f)
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
    
    // 积分限幅
    if (i_out_max != 0.0f)
    {
        basic_math_constrain(&integral_error, -i_out_max / k_i, i_out_max / k_i);
    }

    if (i_separate_threshold == 0.0f)
    {
        // 没有积分分离
        integral_error += term.speed_ratio * d_t * term.error;
        term.i_out = k_i * integral_error;
    }
    else
    {
        // 有积分分离
        if (term.abs_error < i_separate_threshold)
        {
            // 不在积分分离区间上
            integral_error += term.speed_ratio * d_t * term.error;
            term.i_out = k_i * integral_error;
        }
        else
        {
            // 在积分分离区间上
            integral_error = 0.0f;
            term.i_out = 0.0f;
        }
    }

    // 计算d项

    if (d_first == PID_D_First_DISABLE)
    {
        // 没有微分先行
        term.d_out = k_d * (term.error - pre_error) / d_t;
    }
    
    else
    {

        // 微分先行使能
        float d_raw = -(now - pre_now) / d_t;
        float alpha_d = d_t * d_filter_n / (1.0f + d_t * d_filter_n); // 微分滤波器系数

        d_filter_out = alpha_d * d_raw + (1.0f - alpha_d) * d_filter_out; // 微分滤波器输出
        
        term.d_out = k_d * d_filter_out;
    }

    // 计算前馈

    term.f_out = k_f * (target - pre_target);

    // 计算输出

    out = term.p_out + term.i_out + term.d_out + term.f_out;

    // 输出限幅
    if (out_max != 0.0f)
    {
        basic_math_constrain(&out, -out_max, out_max);
    }

    // 善后工作
    pre_now = now;
    pre_target = target;
    pre_out = out;
    pre_error = term.error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/