/**
 * @file alg_quaternion.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 四元数计算相关支持库
 * @version 0.1
 * @date 2025-08-22 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef alg_quaternion_h
#define alg_quaternion_h

/* Includes ------------------------------------------------------------------*/

#include "math_matrix.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 四元数运算
 *
 */
class class_quaternion_f32 : public class_matrix_f32<4, 1>
{
public:
    // 构造函数
    using class_matrix_f32<4, 1>::class_matrix_f32;

    class_quaternion_f32(float real = 1.0f, float imaginary_x = 0.0f, float imaginary_y = 0.0f, float imaginary_z = 0.0f)
    {
        data[0] = real;
        data[1] = imaginary_x;
        data[2] = imaginary_y;
        data[3] = imaginary_z;
    }

    class_quaternion_f32(class_matrix_f32<4, 1> matrix)
    {
        data[0] = matrix[0][0];
        data[1] = matrix[1][0];
        data[2] = matrix[2][0];
        data[3] = matrix[3][0];
    }

    // 析构函数
    ~class_quaternion_f32() = default;

    // 赋值函数
    using class_matrix_f32<4, 1>::operator=;

    inline float &operator[](const int &index)
    {
        return (data[index]);
    }

    using class_matrix_f32<4, 1>::operator+;

    using class_matrix_f32<4, 1>::operator-;

    inline class_quaternion_f32 operator*(const float &value) const
    {
        class_quaternion_f32 result;
        result.data[0] = data[0] * value;
        result.data[1] = data[1] * value;
        result.data[2] = data[2] * value;
        result.data[3] = data[3] * value;
        return (result);
    }

    inline friend class_quaternion_f32 operator*(const float &value, const class_quaternion_f32 &quaternion)
    {
        class_quaternion_f32 result;
        result.data[0] = quaternion.data[0] * value;
        result.data[1] = quaternion.data[1] * value;
        result.data[2] = quaternion.data[2] * value;
        result.data[3] = quaternion.data[3] * value;
        return (result);
    }

    inline class_quaternion_f32 operator*(const class_quaternion_f32 &quaternion) const
    {
        class_quaternion_f32 result;
        float q0q0 = data[0] * quaternion.data[0];
        float q0q1 = data[0] * quaternion.data[1];
        float q0q2 = data[0] * quaternion.data[2];
        float q0q3 = data[0] * quaternion.data[3];
        float q1q0 = data[1] * quaternion.data[0];
        float q1q1 = data[1] * quaternion.data[1];
        float q1q2 = data[1] * quaternion.data[2];
        float q1q3 = data[1] * quaternion.data[3];
        float q2q0 = data[2] * quaternion.data[0];
        float q2q1 = data[2] * quaternion.data[1];
        float q2q2 = data[2] * quaternion.data[2];
        float q2q3 = data[2] * quaternion.data[3];
        float q3q0 = data[3] * quaternion.data[0];
        float q3q1 = data[3] * quaternion.data[1];
        float q3q2 = data[3] * quaternion.data[2];
        float q3q3 = data[3] * quaternion.data[3];
        result.data[0] = q0q0 - q1q1 - q2q2 - q3q3;
        result.data[1] = q0q1 + q1q0 + q2q3 - q3q2;
        result.data[2] = q0q2 - q1q3 + q2q0 + q3q1;
        result.data[3] = q0q3 + q1q2 - q2q1 + q3q0;
        return (result);
    }

    using class_matrix_f32<4, 1>::operator/;

    inline class_quaternion_f32 operator/(const class_quaternion_f32 &quaternion) const
    {
        class_quaternion_f32 result;
        class_quaternion_f32 divisor = quaternion;
        float modulus_square = divisor.data[0] * divisor.data[0] + divisor.data[1] * divisor.data[1] + divisor.data[2] * divisor.data[2] + divisor.data[3] * divisor.data[3];
        if (modulus_square <= matrix_compare_epsilon)
        {
            return (*this);
        }
        modulus_square = 1.0f / modulus_square;
        float q0q0 = data[0] * divisor.data[0];
        float q0q1 = data[0] * -divisor.data[1];
        float q0q2 = data[0] * -divisor.data[2];
        float q0q3 = data[0] * -divisor.data[3];
        float q1q0 = data[1] * divisor.data[0];
        float q1q1 = data[1] * -divisor.data[1];
        float q1q2 = data[1] * -divisor.data[2];
        float q1q3 = data[1] * -divisor.data[3];
        float q2q0 = data[2] * divisor.data[0];
        float q2q1 = data[2] * -divisor.data[1];
        float q2q2 = data[2] * -divisor.data[2];
        float q2q3 = data[2] * -divisor.data[3];
        float q3q0 = data[3] * divisor.data[0];
        float q3q1 = data[3] * -divisor.data[1];
        float q3q2 = data[3] * -divisor.data[2];
        float q3q3 = data[3] * -divisor.data[3];
        result.data[0] = (q0q0 - q1q1 - q2q2 - q3q3) * modulus_square;
        result.data[1] = (q0q1 + q1q0 + q2q3 - q3q2) * modulus_square;
        result.data[2] = (q0q2 - q1q3 + q2q0 + q3q1) * modulus_square;
        result.data[3] = (q0q3 + q1q2 - q2q1 + q3q0) * modulus_square;
        return (result);
    }

    using class_matrix_f32<4, 1>::operator+=;

    using class_matrix_f32<4, 1>::operator-=;

    inline class_quaternion_f32 &operator*=(const float &value)
    {
        data[0] *= value;
        data[1] *= value;
        data[2] *= value;
        data[3] *= value;
        return (*this);
    }

    inline class_quaternion_f32 &operator*=(const class_quaternion_f32 &quaternion)
    {
        float q0q0 = data[0] * quaternion.data[0];
        float q0q1 = data[0] * quaternion.data[1];
        float q0q2 = data[0] * quaternion.data[2];
        float q0q3 = data[0] * quaternion.data[3];
        float q1q0 = data[1] * quaternion.data[0];
        float q1q1 = data[1] * quaternion.data[1];
        float q1q2 = data[1] * quaternion.data[2];
        float q1q3 = data[1] * quaternion.data[3];
        float q2q0 = data[2] * quaternion.data[0];
        float q2q1 = data[2] * quaternion.data[1];
        float q2q2 = data[2] * quaternion.data[2];
        float q2q3 = data[2] * quaternion.data[3];
        float q3q0 = data[3] * quaternion.data[0];
        float q3q1 = data[3] * quaternion.data[1];
        float q3q2 = data[3] * quaternion.data[2];
        float q3q3 = data[3] * quaternion.data[3];
        data[0] = q0q0 - q1q1 - q2q2 - q3q3;
        data[1] = q0q1 + q1q0 + q2q3 - q3q2;
        data[2] = q0q2 - q1q3 + q2q0 + q3q1;
        data[3] = q0q3 + q1q2 - q2q1 + q3q0;
        return (*this);
    }

    inline class_quaternion_f32 &operator/=(const class_quaternion_f32 &quaternion)
    {
        class_quaternion_f32 divisor = quaternion;
        float modulus_square = divisor.data[0] * divisor.data[0] + divisor.data[1] * divisor.data[1] + divisor.data[2] * divisor.data[2] + divisor.data[3] * divisor.data[3];
        if (modulus_square <= matrix_compare_epsilon)
        {
            return (*this);
        }
        modulus_square = 1.0f / modulus_square;
        float q0q0 = data[0] * divisor.data[0];
        float q0q1 = data[0] * -divisor.data[1];
        float q0q2 = data[0] * -divisor.data[2];
        float q0q3 = data[0] * -divisor.data[3];
        float q1q0 = data[1] * divisor.data[0];
        float q1q1 = data[1] * -divisor.data[1];
        float q1q2 = data[1] * -divisor.data[2];
        float q1q3 = data[1] * -divisor.data[3];
        float q2q0 = data[2] * divisor.data[0];
        float q2q1 = data[2] * -divisor.data[1];
        float q2q2 = data[2] * -divisor.data[2];
        float q2q3 = data[2] * -divisor.data[3];
        float q3q0 = data[3] * divisor.data[0];
        float q3q1 = data[3] * -divisor.data[1];
        float q3q2 = data[3] * -divisor.data[2];
        float q3q3 = data[3] * -divisor.data[3];
        data[0] = (q0q0 - q1q1 - q2q2 - q3q3) * modulus_square;
        data[1] = (q0q1 + q1q0 + q2q3 - q3q2) * modulus_square;
        data[2] = (q0q2 - q1q3 + q2q0 + q3q1) * modulus_square;
        data[3] = (q0q3 + q1q2 - q2q1 + q3q0) * modulus_square;
        return (*this);
    }

    using class_matrix_f32<4, 1>::operator==;

    using class_matrix_f32<4, 1>::operator!=;

    inline float get_real() const;

    inline class_matrix_f32<3, 1> get_imaginary() const;

    inline class_quaternion_f32 get_conjugate() const;

    inline class_quaternion_f32 get_inverse() const;

    using class_matrix_f32<4, 1>::get_modulus;

    inline class_matrix_f32<3, 1> get_euler_angle() const;

    using class_matrix_f32<4, 1>::get_normalization;

    inline class_matrix_f32<4, 4> get_self_matrix() const;

    inline class_matrix_f32<3, 3> get_rotation_matrix() const;

    inline class_matrix_f32<4, 1> get_axis_angle() const;

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

namespace namespace_alg_quaternion
{
    class_quaternion_f32 zero();

    class_quaternion_f32 unit_real();

    class_quaternion_f32 unit_imaginary_x();

    class_quaternion_f32 unit_imaginary_y();

    class_quaternion_f32 unit_imaginary_z();

    class_quaternion_f32 from_vector(const class_matrix_f32<3, 1> &vector);

    class_quaternion_f32 from_euler_angle(const class_matrix_f32<3, 1> &euler_angle);

    class_quaternion_f32 from_rotation_matrix(const class_matrix_f32<3, 3> &rotation_matrix);

    class_quaternion_f32 from_axis_angle(const class_matrix_f32<3, 1> &axis, const float &angle);
}

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取四元数的实部
 *
 * @return float 实部
 */
inline float class_quaternion_f32::get_real() const
{
    return (data[0]);
}

/**
 * @brief 获取四元数的虚部
 *
 * @return Class_Matrix_f32<3, 1> 虚部
 */
inline class_matrix_f32<3, 1> class_quaternion_f32::get_imaginary() const
{
    return (class_matrix_f32 < 3, 1 > (&data[1]));
}

/**
 * @brief 获取四元数的共轭四元数
 *
 * @return class_quaternion_f32 共轭四元数
 */
inline class_quaternion_f32 class_quaternion_f32::get_conjugate() const
{
    class_quaternion_f32 result;
    result.data[0] = data[0];
    result.data[1] = -data[1];
    result.data[2] = -data[2];
    result.data[3] = -data[3];
    return (result);
}

/**
 * @brief 获取四元数的逆元
 *
 * @return class_quaternion_f32 逆四元数
 */
inline class_quaternion_f32 class_quaternion_f32::get_inverse() const
{
    class_quaternion_f32 result;
    float modulus_square = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
    if (modulus_square <= matrix_compare_epsilon)
    {
        return (class_quaternion_f32(1.0f, 0.0f, 0.0f, 0.0f));
    }
    modulus_square = 1.0f / modulus_square;
    result.data[0] = data[0] * modulus_square;
    result.data[1] = -data[1] * modulus_square;
    result.data[2] = -data[2] * modulus_square;
    result.data[3] = -data[3] * modulus_square;
    return (result);
}

/**
 * @brief 获取四元数对应的欧拉角, Z-Y-X顺序
 *
 * @return class_matrix_f32<3, 1> 欧拉角, Yaw-Pitch-Roll顺序
 * 注意, 这只是用矩阵形式存储, 不可参与矩阵计算
 */
inline class_matrix_f32<3, 1> class_quaternion_f32::get_euler_angle() const
{
    class_matrix_f32 < 3, 1 > result;
    float modulus_square = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
    if (modulus_square <= matrix_compare_epsilon)
    {
        return (result);
    }
    float modulus = 1.0f / sqrtf(modulus_square);
    float q0 = data[0] * modulus;
    float q1 = data[1] * modulus;
    float q2 = data[2] * modulus;
    float q3 = data[3] * modulus;
    result[0][0] = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sin_pitch) >= 1.0f)
    {
        if (sin_pitch > 0.0f)
        {
            result[1][0] = PI / 2.0f;
        }
        else
        {
            result[1][0] = -PI / 2.0f;
        }
    }
    else
    {
        result[1][0] = asinf(sin_pitch);
    }
    result[2][0] = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    return (result);
}

/**
 * @brief 获取四元数的自乘矩阵
 *
 * @return class_matrix_f32<4, 4> 自乘矩阵
 */
inline class_matrix_f32<4, 4> class_quaternion_f32::get_self_matrix() const
{
    class_matrix_f32 < 4, 4 > result;
    result[0][0] = data[0];
    result[0][1] = -data[1];
    result[0][2] = -data[2];
    result[0][3] = -data[3];
    result[1][0] = data[1];
    result[1][1] = data[0];
    result[1][2] = -data[3];
    result[1][3] = data[2];
    result[2][0] = data[2];
    result[2][1] = data[3];
    result[2][2] = data[0];
    result[2][3] = -data[1];
    result[3][0] = data[3];
    result[3][1] = -data[2];
    result[3][2] = data[1];
    result[3][3] = data[0];
    return (result);
}

/**
 * @brief 获取四元数对应的旋转矩阵
 *
 * @return class_matrix_f32<3, 3> 旋转矩阵
 */
inline class_matrix_f32<3, 3> class_quaternion_f32::get_rotation_matrix() const
{
    class_matrix_f32 < 3, 3 > result;
    float modulus_square = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
    if (modulus_square <= matrix_compare_epsilon)
    {
        return (namespace_alg_matrix::identity<3, 3>());
    }
    float modulus = 1.0f / sqrtf(modulus_square);
    float q0 = data[0] * modulus;
    float q1 = data[1] * modulus;
    float q2 = data[2] * modulus;
    float q3 = data[3] * modulus;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    result[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    result[0][1] = 2.0f * (q1q2 - q0q3);
    result[0][2] = 2.0f * (q1q3 + q0q2);
    result[1][0] = 2.0f * (q1q2 + q0q3);
    result[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    result[1][2] = 2.0f * (q2q3 - q0q1);
    result[2][0] = 2.0f * (q1q3 - q0q2);
    result[2][1] = 2.0f * (q2q3 + q0q1);
    result[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
    return (result);
}

/**
 * @brief 获取四元数对应的轴角表示, Axis-Angle顺序, 轴与x轴正方向夹角为正
 *
 * @return class_matrix_f32<4, 1> 轴角表示, Axis-Angle顺序
 * 注意, 这只是用矩阵形式存储, 不可参与矩阵计算
 */
inline class_matrix_f32<4, 1> class_quaternion_f32::get_axis_angle() const
{
    class_matrix_f32 < 4, 1 > result;
    float modulus_square = data[0] * data[0] + data[1] * data[1] + data[2] * data[2] + data[3] * data[3];
    if (modulus_square <= matrix_compare_epsilon)
    {
        result[0][0] = 1.0f;
        return (result);
    }
    float modulus = 1.0f / sqrtf(modulus_square);
    float q0 = data[0] * modulus;
    float q1 = data[1] * modulus;
    float q2 = data[2] * modulus;
    float q3 = data[3] * modulus;
    result[3][0] = 2.0f * acosf(q0);
    float sin_half_angle = sqrtf(1.0f - q0 * q0);
    if (sin_half_angle <= matrix_compare_epsilon)
    {
        result[0][0] = 1.0f;
        result[1][0] = 0.0f;
        result[2][0] = 0.0f;
    }
    else
    {
        float inv_sin_half_angle = 1.0f / sin_half_angle;
        if (q1 < 0.0f)
        {
            inv_sin_half_angle = -inv_sin_half_angle;
        }
        result[0][0] = q1 * inv_sin_half_angle;
        result[1][0] = q2 * inv_sin_half_angle;
        result[2][0] = q3 * inv_sin_half_angle;
    }
    return (result);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/