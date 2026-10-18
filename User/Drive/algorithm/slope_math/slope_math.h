/**
 * @file alg_slope.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 斜坡函数, 用于速度规划等
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "basic_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 规划优先类型, 分为目标值优先和真实值优先
 * 目标值优先, 即硬规划
 * 真实值优先, 即当前真实值夹在当前规划值和目标值之间, 当前规划值转为当前真实值
 *
 */
enum Enum_Slope_First
{
    Slope_First_REAL = 0,   // 真实值优先
    Slope_First_TARGET,     // 目标值优先
};

/**
 * @brief Reusable, 斜坡函数本体
 *
 */
class slope_class
{
public:
    void init(const float &__increase_value, const float &__decrease_value, const Enum_Slope_First &__slope_first = Slope_First_REAL);

    inline float get_out() const;

    inline void set_now_real(const float &__now_real);

    inline void set_increase_value(const float &__increase_value);

    inline void set_decrease_value(const float &__decrease_value);

    inline void set_target(const float &__target);

    void slope_calculate_callback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 输出值
    float out = 0.0f;

    // 写变量

    // 规划优先类型
    Enum_Slope_First slope_first = Slope_First_REAL;
    // 当前规划值
    float now_planning = 0.0f;
    // 当前真实值
    float now_real = 0.0f;

    // 绝对值增量, 一次计算周期改变值
    float increase_value = 0.0f;
    // 绝对值减量, 一次计算周期改变值
    float decrease_value = 0.0f;
    // 目标值
    float target = 0.0f;

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取输出值
 *
 * @return 输出值
 */
inline float slope_class::get_out() const
{
    return (out);
}

/**
 * @brief 设定当前规划值
 *
 * @param __Now_Real 当前规划值
 */
inline void slope_class::set_now_real(const float &__now_real)
{
    now_real = __now_real;
}

/**
 * @brief 设定绝对值增量, 一次计算周期改变值
 *
 * @param __Increase_Value 绝对值增量, 一次计算周期改变值
 */
inline void slope_class::set_increase_value(const float &__increase_value)
{
    increase_value = __increase_value;
}

/**
 * @brief 设定绝对值减量, 一次计算周期改变值
 *
 * @param __Decrease_Value 绝对值减量, 一次计算周期改变值
 */
inline void slope_class::set_decrease_value(const float &__decrease_value)
{
    decrease_value = __decrease_value;
}

/**
 * @brief 设定目标值
 *
 * @param __Target 目标值
 */
inline void slope_class::set_target(const float &__target)
{
    target = __target;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/