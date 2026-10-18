#pragma once

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"
#include "float.h"

/* Exported macros -----------------------------------------------------------*/
extern const float BASIC_MATH_RPM_TO_RADPS;
extern const float BASIC_MATH_DEG_TO_RAD;
extern const float BASIC_MATH_CELSIUS_TO_KELVIN;

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void basic_math_bool_reverse(bool *value);

void basic_math_endian_swap_16(uint16_t *address);

uint16_t basic_math_endian_swap_16(void *source, void *destination);

void basic_math_endian_swap_32(uint32_t *address);

uint32_t basic_math_endian_swap_32(void *source, void *destination);

uint8_t basic_math_sum_8(const uint8_t *address, uint32_t length);

uint16_t basic_math_sum_16(const uint16_t *address, uint32_t length);

uint32_t basic_math_sum_32(const uint32_t *address, uint32_t length);

float basic_math_sinc(float x);

int32_t basic_math_float_to_int(float x, float float_1, float float_2, int32_t int_1, int32_t int_2);

float basic_math_int_to_float(int32_t x, int32_t int_1, int32_t int_2, float float_1, float float_2);

bool basic_math_is_invalid_float(float x);

float basic_math_modulus_normalization(float x, float modulus);

/**
 * @brief 限幅函数, 传入值版本
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param min 最小值
 * @param max 最大值
 * @return 输出值
 */
template<typename Type>
Type basic_math_constrain(Type x, Type min, Type max)
{
    if (x < min)
    {
        x = min;
    }
    else if (x > max)
    {
        x = max;
    }
    return (x);
}

/**
 * @brief 限幅函数, 传入指针版本
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @param min 最小值
 * @param max 最大值
 * @return 输出值
 */
template<typename Type>
Type basic_math_constrain(Type *x, Type min, Type max)
{
    if (*x < min)
    {
        *x = min;
    }
    else if (*x > max)
    {
        *x = max;
    }
    return (*x);
}

/**
 * @brief 求绝对值
 *
 * @tparam Type 类型
 * @param x 传入数据
 * @return Type x的绝对值
 */
template<typename Type>
Type basic_math_abs(Type x)
{
    return ((x > 0) ? x : -x);
}

/**
    * @brief 死区函数, 传入值版本
    * @tparam Type 类型
    * @param x 传入数据
    * @param min 最小值
    * @param max 最大值
    * @param dead_zone 死区范围,死区/不响应区域（防止抖动）
    * @note  当输入值在死区范围内时，输出为0；当输入值超过死区范围时，输出根据限幅函数进行调整
 */
template<typename Type>
Type basic_math_dead_constrain(Type x, Type min, Type max, Type dead_zone)
{
    if (x > dead_zone)
    {
        x += dead_zone;  // 正向越过死区，补偿死区宽度，保证电机在低速时获得足够的初始驱动力
    }

    else if (x < -dead_zone)
    {
        x -= dead_zone;  // 负向越过死区，补偿死区宽度，保证电机在低速时获得足够的初始驱动力
    }
    else
    {
        x = 0;  // 死区内输出为零
    }

    return (basic_math_constrain(x, min, max));

}

/**
    * @brief 死区函数, 传入指针版本
    * @tparam Type 类型
    * @param x 传入数据
    * @param min 最小值
    * @param max 最大值
    * @param dead_zone 死区范围,死区/不响应区域（防止抖动）
    * @note  当输入值在死区范围内时，输出为0；当输入值超过死区范围时，输出根据限幅函数进行调整
 */
template<typename Type>
Type basic_math_dead_constrain(Type *x, Type min, Type max, Type dead_zone)
{
    if (*x > dead_zone)
    {
        *x += dead_zone;  // 正向越过死区，补偿死区宽度，保证电机在低速时获得足够的初始驱动力
    }

    else if (*x < -dead_zone)
    {
        *x -= dead_zone;  // 负向越过死区，补偿死区宽度，保证电机在低速时获得足够的初始驱动力
    }
    else
    {
        *x = 0;  // 死区内输出为零
    }

    return (basic_math_constrain(x, min, max));

}

/**
 * @brief 循环限幅函数
 * @param x 限幅的目标参数
 * @param min 最小值
 * @param max 最大值
 * @retval Type: 
 */
template<typename Type>
Type basic_math_loop_constrain(Type x, Type min, Type max)
{
    if (max < min)
    {
        return x;  // 如果最大值小于最小值，直接返回原值
    }

    if (x > max)
    {
        Type len = max - min;
        while (x > max)
        {
            x -= len;
        }
    }

    else if (x < min)
    {
        Type len = max - min;
        while (x < min)
        {
            x += len;
        }
    }

    return x;
}

