/**
 * @file alg_complex.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 复数计算相关支持库
 * @version 0.1
 * @date 2025-08-22 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "math_matrix.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief reusable, 复数运算, 由于不涉及批处理因此用硬算版本即可
 *
 */
class class_complex_f32 : public class_matrix_f32<2, 1>
{
public:
    // 构造函数
    using class_matrix_f32<2, 1>::class_matrix_f32;

    class_complex_f32(float real = 1.0f, float imaginary = 0.0f)
    {
        data[0] = real;
        data[1] = imaginary;
    }

    class_complex_f32(class_matrix_f32<2, 1> matrix)
    {
        data[0] = matrix[0][0];
        data[1] = matrix[1][0];
    }

    // 析构函数
    ~class_complex_f32() = default;

    // 赋值函数
    using class_matrix_f32<2, 1>::operator=;

    inline float &operator[](const int &index)
    {
        return (data[index]);
    }

    using class_matrix_f32<2, 1>::operator+;

    using class_matrix_f32<2, 1>::operator-;

    inline class_complex_f32 operator*(const float &value) const
    {
        class_complex_f32 result;
        result.data[0] = data[0] * value;
        result.data[1] = data[1] * value;
        return (result);
    }

    inline friend class_complex_f32 operator*(const float &value, const class_complex_f32 &complex)
    {
        class_complex_f32 result;
        result.data[0] = complex.data[0] * value;
        result.data[1] = complex.data[1] * value;
        return (result);
    }

    inline class_complex_f32 operator*(const class_complex_f32 &complex) const
    {
        class_complex_f32 result;
        float c0c0 = data[0] * complex.data[0];
        float c0c1 = data[0] * complex.data[1];
        float c1c0 = data[1] * complex.data[0];
        float c1c1 = data[1] * complex.data[1];
        result.data[0] = c0c0 - c1c1;
        result.data[1] = c0c1 + c1c0;
        return (result);
    }

    using class_matrix_f32<2, 1>::operator/;

    inline class_complex_f32 operator/(const class_complex_f32 &complex) const
    {
        class_complex_f32 result;
        class_complex_f32 divisor = complex;
        float modulus_square = divisor.data[0] * divisor.data[0] + divisor.data[1] * divisor.data[1];
        if (modulus_square <= matrix_compare_epsilon)
        {
            return (*this);
        }
        modulus_square = 1.0f / modulus_square;
        float c0c0 = data[0] * divisor.data[0];
        float c0c1 = data[0] * -divisor.data[1];
        float c1c0 = data[1] * divisor.data[0];
        float c1c1 = data[1] * -divisor.data[1];
        result.data[0] = (c0c0 - c1c1) * modulus_square;
        result.data[1] = (c0c1 + c1c0) * modulus_square;
        return (result);
    }

    using class_matrix_f32<2, 1>::operator+=;

    using class_matrix_f32<2, 1>::operator-=;

    inline class_complex_f32 &operator*=(const float &value)
    {
        data[0] *= value;
        data[1] *= value;
        return (*this);
    }

    inline class_complex_f32 &operator*=(const class_complex_f32 &complex)
    {
        float c0c0 = data[0] * complex.data[0];
        float c0c1 = data[0] * complex.data[1];
        float c1c0 = data[1] * complex.data[0];
        float c1c1 = data[1] * complex.data[1];
        data[0] = c0c0 - c1c1;
        data[1] = c0c1 + c1c0;
        return (*this);
    }

    using class_matrix_f32<2, 1>::operator/=;

    inline class_complex_f32 &operator/=(const class_complex_f32 &complex)
    {
        class_complex_f32 divisor = complex;
        float modulus_square = divisor.data[0] * divisor.data[0] + divisor.data[1] * divisor.data[1];
        if (modulus_square <= matrix_compare_epsilon)
        {
            return (*this);
        }
        modulus_square = 1.0f / modulus_square;
        float c0c0 = data[0] * divisor.data[0];
        float c0c1 = data[0] * -divisor.data[1];
        float c1c0 = data[1] * divisor.data[0];
        float c1c1 = data[1] * -divisor.data[1];
        data[0] = (c0c0 - c1c1) * modulus_square;
        data[1] = (c0c1 + c1c0) * modulus_square;
        return (*this);
    }

    using class_matrix_f32<2, 1>::operator==;

    using class_matrix_f32<2, 1>::operator!=;

    inline float get_real() const;

    inline float get_imaginary() const;

    inline class_complex_f32 get_conjugate() const;

    inline class_complex_f32 get_inverse() const;

    using class_matrix_f32<2, 1>::get_modulus;

    inline float get_angle() const;

    using class_matrix_f32<2, 1>::get_normalization;

    inline class_matrix_f32<2, 2> get_self_matrix() const;

    inline class_matrix_f32<2, 2> get_rotation_matrix() const;

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/* exported variables --------------------------------------------------------*/

namespace namespace_alg_complex
{
    class_complex_f32 zero();

    class_complex_f32 unit_real();

    class_complex_f32 unit_imaginary();

    class_complex_f32 angle_unit(const float &angle);

    class_complex_f32 polar(const float &modulus, const float &angle);
}

/* exported function declarations --------------------------------------------*/

/**
 * @brief 获取复数的实部
 *
 * @return float 实部
 */
inline float class_complex_f32::get_real() const
{
    return (data[0]);
}

/**
 * @brief 获取复数的虚部
 *
 * @return float 虚部
 */
inline float class_complex_f32::get_imaginary() const
{
    return (data[1]);
}

/**
 * @brief 获取复数的共轭复数
 *
 * @return class_complex_f32 共轭复数
 */
inline class_complex_f32 class_complex_f32::get_conjugate() const
{
    class_complex_f32 result;
    result.data[0] = data[0];
    result.data[1] = -data[1];
    return (result);
}

/**
 * @brief 获取复数的逆元
 *
 * @return class_complex_f32 倒数
 */
inline class_complex_f32 class_complex_f32::get_inverse() const
{
    class_complex_f32 result;
    float modulus_square = data[0] * data[0] + data[1] * data[1];
    if (modulus_square <= matrix_compare_epsilon)
    {
        return (class_complex_f32(1.0f, 0.0f));
    }
    modulus_square = 1.0f / modulus_square;
    result.data[0] = data[0] * modulus_square;
    result.data[1] = -data[1] * modulus_square;
    return (result);
}

/**
 * @brief 获取复数的幅角, 范围[-pi, pi]
 *
 * @return float 幅角
 */
inline float class_complex_f32::get_angle() const
{
    return (atan2f(data[1], data[0]));
}

/**
 * @brief 获取复数对应的2x2矩阵表示
 *
 * @return class_matrix_f32<2, 2> 2x2矩阵
 */
inline class_matrix_f32<2, 2> class_complex_f32::get_self_matrix() const
{
    class_matrix_f32 < 2, 2 > result;
    result[0][0] = data[0];
    result[0][1] = -data[1];
    result[1][0] = data[1];
    result[1][1] = data[0];
    return (result);
}

/**
 * @brief 获取复数对应的旋转矩阵表示
 *
 * @return class_matrix_f32<2, 2> 2x2旋转矩阵
 */
inline class_matrix_f32<2, 2> class_complex_f32::get_rotation_matrix() const
{
    class_matrix_f32 < 2, 2 > result;
    float modulus_square = data[0] * data[0] + data[1] * data[1];
    if (modulus_square <= matrix_compare_epsilon)
    {
        return (namespace_alg_matrix::identity<2, 2>());
    }
    float modulus = 1.0f / sqrtf(modulus_square);
    float cos_theta = data[0] * modulus;
    float sin_theta = data[1] * modulus;
    result[0][0] = cos_theta;
    result[0][1] = -sin_theta;
    result[1][0] = sin_theta;
    result[1][1] = cos_theta;
    return (result);
}

/************************ copyright(c) ustc-robovalker **************************/