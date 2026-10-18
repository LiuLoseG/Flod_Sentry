/**
 * @file alg_slope.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 斜坡函数, 用于速度规划等
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-06-03 1.1 规划引入优先级方式
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "slope_math.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化
 *
 * @param __increase_value 增长最大幅度
 * @param __decrease_value 降低最大幅度
 * @param __slope_first 规划优先类型, 分为目标值优先和真实值优先
 */
void slope_class::init(const float &__increase_value, const float &__decrease_value, const Enum_Slope_First &__slope_first)
{
    increase_value = __increase_value;
    decrease_value = __decrease_value;
    slope_first = __slope_first;
}

/**
 * @brief 斜坡函数调整值, 计算周期取决于调用者
 *
 */
void slope_class::slope_calculate_callback()
{
    // 规划为当前真实值优先的额外逻辑
    if (slope_first == Slope_First_REAL)
    {
        if ((target >= now_real && now_real >= now_planning) || (target <= now_real && now_real <= now_planning))
        {
            out = now_real;
        }
    }

    if (now_planning > 0.0f)
    {
        if (target > now_planning)
        {
            // 正值加速
            if (basic_math_abs(now_planning - target) > increase_value)
            {
                out += increase_value;
            }
            else
            {
                out = target;
            }
        }
        else if (target < now_planning)
        {
            // 正值减速
            if (basic_math_abs(now_planning - target) > decrease_value)
            {
                out -= decrease_value;
            }
            else
            {
                out = target;
            }
        }
    }
    else if (now_planning < 0.0f)
    {
        if (target < now_planning)
        {
            // 负值加速
            if (basic_math_abs(now_planning - target) > increase_value)
            {
                out -= increase_value;
            }
            else
            {
                out = target;
            }
        }
        else if (target > now_planning)
        {
            // 负值减速
            if (basic_math_abs(now_planning - target) > decrease_value)
            {
                out += decrease_value;
            }
            else
            {
                out = target;
            }
        }
    }
    else
    {
        if (target > now_planning)
        {
            // 0值正加速
            if (basic_math_abs(now_planning - target) > increase_value)
            {
                out += increase_value;
            }
            else
            {
                out = target;
            }
        }
        else if (target < now_planning)
        {
            // 0值负加速
            if (basic_math_abs(now_planning - target) > decrease_value)
            {
                out -= decrease_value;
            }
            else
            {
                out = target;
            }
        }
    }

    // 善后工作
    now_planning = out;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/