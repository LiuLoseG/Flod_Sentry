/**
 * @file alg_pid.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PID算法
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "basic_math.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 微分先行
 *
 */
typedef enum
{
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
} pid_d_first_e;

/**
 * @brief 抗积分饱和(anti-windup)方案
 * @note 均在输出被限幅时才生效; 默认 DISABLE, 行为与原逻辑逐位一致
 */
typedef enum
{
    PID_Anti_Windup_DISABLE = 0,    // 关闭
    PID_Anti_Windup_CLAMP,          // 条件积分: 饱和且误差仍朝限幅方向时冻结本步积分
    PID_Anti_Windup_BACK_CALC,      // 反算: 按输出超出量回退积分
} pid_anti_windup_e;

/**
 * @brief PID各项输出结构体
 */
typedef struct
{
    float p_out;        // P输出
    float i_out;        // I输出
    float d_out;        // D输出
    float f_out;        // 前馈输出
    
    float error;        // 当前误差
    float abs_error;     // 当前误差绝对值
    float speed_ratio;    // 积分变速比例系数
} pid_term_t;

/**
 * @brief Reusable, PID算法
 *
 */
class pid_class
{
public:
    void init(const float &__k_p, const float &__k_i, const float &__k_d, const float &__k_f = 0.0f, const float &__i_out_max = 0.0f, const float &__out_max = 0.0f, const float &__d_t = 0.001f, const float &__dead_zone = 0.0f, const float &__i_variable_speed_a = 0.0f, const float &__i_variable_speed_b = 0.0f, const float &__i_separate_threshold = 0.0f, const pid_d_first_e &__d_first = PID_D_First_DISABLE, const pid_anti_windup_e &__anti_windup = PID_Anti_Windup_DISABLE, const float &__anti_windup_gain = 10.0f, const float &__anti_windup_out_max = 0.0f);

    inline float get_integral_error() const;

    inline float get_out() const;

    inline float get_out_unsaturated() const;

    inline void set_k_p(const float &__k_p);

    inline void set_k_i(const float &__k_i);

    inline void set_k_d(const float &__k_d);

    inline void set_k_f(const float &__k_f);

    inline void set_i_out_max(const float &__i_out_max);

    inline void set_out_max(const float &__out_max);

    inline void set_i_variable_speed_a(const float &__i_variable_speed_a);

    inline void set_i_variable_speed_b(const float &__i_variable_speed_b);

    inline void set_i_separate_threshold(const float &__i_separate_threshold);

    inline void set_d_filter_n(const float &__d_filter_n);

    inline void set_target(const float &__target);

    inline void set_now(const float &__now);

    inline void set_integral_error(const float &__integral_error);

    inline void set_anti_windup(const pid_anti_windup_e &__anti_windup);

    inline void set_anti_windup_gain(const float &__anti_windup_gain);

    inline void set_anti_windup_out_max(const float &__anti_windup_out_max);

    void pid_calculate_callback();

protected:
    // 初始化相关常量

    // PID计时器周期, s
    float d_t;
    // 死区, Error在其绝对值内不输出
    float dead_zone;
    // 微分先行
    pid_d_first_e d_first;

    // 常量

    // 内部变量

    // 之前的当前值
    float pre_now = 0.0f;
    // 之前的目标值
    float pre_target = 0.0f;
    // 之前的输出值
    float pre_out = 0.0f;
    // 前向误差
    float pre_error = 0.0f;

    // 读变量

    // 输出值
    float out = 0.0f;

    // 写变量

    // PID的P
    float k_p = 0.0f;
    // PID的I
    float k_i = 0.0f;
    // PID的D
    float k_d = 0.0f;
    // 前馈
    float k_f = 0.0f;

    // 积分限幅, 0为不限制
    float i_out_max = 0;
    // 输出限幅, 0为不限制
    float out_max = 0;

    // 变速积分定速内段阈值, 0为不限制
    float i_variable_speed_a = 0.0f;
    // 变速积分变速区间, 0为不限制
    float i_variable_speed_b = 0.0f;
    // 积分分离阈值，需为正数, 0为不限制
    float i_separate_threshold = 0.0f;

    // 微分滤波器补偿
    float d_filter_n = 10.0f;  // 微分滤波器截止频率, Hz
    float d_filter_out = 0.0f; // 微分滤波器输出

    // 抗积分饱和
    pid_anti_windup_e anti_windup = PID_Anti_Windup_DISABLE;
    // 反算增益Kb, 单位1/s
    float anti_windup_gain = 10.0f;      
    // 抗饱和判定用输出上限, 0=沿用out_max, 单位同out
    float anti_windup_out_max = 0.0f;    
    // 限幅前的原始输出
    float out_unsaturated = 0.0f;        

    // 目标值
    float target = 0.0f;
    // 当前值
    float now = 0.0f;

    // 读写变量

    // 积分值
    float integral_error = 0.0f;

    // pid各项输出结构体
    pid_term_t term;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float pid_class::get_integral_error() const
{
    return (integral_error);
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
inline float pid_class::get_out() const
{
    return (out);
}

/**
 * @brief 获取限幅前的原始输出(AW调试观察用)
 *
 * @return float 限幅前输出
 */
inline float pid_class::get_out_unsaturated() const
{
    return (out_unsaturated);
}

/**
 * @brief 设定PID的P
 *
 * @param __k_p PID的P
 */
inline void pid_class::set_k_p(const float &__k_p)
{
    k_p = __k_p;
}

/**
 * @brief 设定PID的I
 *
 * @param __k_i PID的I
 */
inline void pid_class::set_k_i(const float &__k_i)
{
    k_i = __k_i;
}

/**
 * @brief 设定PID的D
 *
 * @param __k_d PID的D
 */
inline void pid_class::set_k_d(const float &__k_d)
{
    k_d = __k_d;
}

/**
 * @brief 设定前馈
 *
 * @param __k_f 前馈
 */
inline void pid_class::set_k_f(const float &__k_f)
{
    k_f = __k_f;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param __i_out_max 积分限幅, 0为不限制
 */
inline void pid_class::set_i_out_max(const float &__i_out_max)
{
    i_out_max = __i_out_max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param __out_max 输出限幅, 0为不限制
 */
inline void pid_class::set_out_max(const float &__out_max)
{
    out_max = __out_max;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 *
 * @param __i_variable_speed_a 定速内段阈值, 0为不限制
 */
inline void pid_class::set_i_variable_speed_a(const float &__i_variable_speed_a)
{
    i_variable_speed_a = __i_variable_speed_a;
}

/**
 * @brief 设定变速区间, 0为不限制
 *
 * @param __i_variable_speed_b 变速区间, 0为不限制
 */
inline void pid_class::set_i_variable_speed_b(const float &__i_variable_speed_b)
{
    i_variable_speed_b = __i_variable_speed_b;
}

/**
 * @brief 设定积分分离阈值，需为正数, 0为不限制
 *
 * @param __i_separate_threshold 积分分离阈值，需为正数, 0为不限制
 */
inline void pid_class::set_i_separate_threshold(const float &__i_separate_threshold)
{
    i_separate_threshold = __i_separate_threshold;
}

/**
 * @brief 设定微分滤波器截止频率, Hz
 *
 * @param __d_filter_n 微分滤波器截止频率, Hz
 */
inline void pid_class::set_d_filter_n(const float &__d_filter_n)
{
    d_filter_n = __d_filter_n;
}

/**
 * @brief 设定目标值
 *
 * @param __target 目标值
 */
inline void pid_class::set_target(const float &__target)
{
    target = __target;
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
inline void pid_class::set_now(const float &__now)
{
    now = __now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param __set_integral_error 积分值
 */
inline void pid_class::set_integral_error(const float &__set_integral_error)
{
    integral_error = __set_integral_error;
}

/**
 * @brief 设定抗积分饱和方案
 *
 * @param __anti_windup 抗积分饱和方案
 */
inline void pid_class::set_anti_windup(const pid_anti_windup_e &__anti_windup)
{
    anti_windup = __anti_windup;
}

/**
 * @brief 设定反算增益Kb, 单位1/s, 仅PID_Anti_Windup_BACK_CALC使用
 *
 * @param __anti_windup_gain 反算增益Kb
 */
inline void pid_class::set_anti_windup_gain(const float &__anti_windup_gain)
{
    anti_windup_gain = __anti_windup_gain;
}

/**
 * @brief 设定抗饱和判定用输出上限, 0为沿用out_max
 *
 * @param __anti_windup_out_max 抗饱和判定用输出上限
 */
inline void pid_class::set_anti_windup_out_max(const float &__anti_windup_out_max)
{
    anti_windup_out_max = __anti_windup_out_max;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
