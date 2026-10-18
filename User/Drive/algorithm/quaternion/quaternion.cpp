/**
 * @file alg_quaternion.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 四元数计算相关支持库
 * @version 0.1
 * @date 2025-08-22 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#include "quaternion.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**brief 获取零四元数
 *
 */
class_quaternion_f32 namespace_alg_quaternion::zero()
{
    class_quaternion_f32 result;
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位实四元数
 *
 */
class_quaternion_f32 namespace_alg_quaternion::unit_real()
{
    class_quaternion_f32 result;
    result[0] = 1.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位虚四元数X
 *
 */
class_quaternion_f32 namespace_alg_quaternion::unit_imaginary_x()
{
    class_quaternion_f32 result;
    result[0] = 1.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位虚四元数Y
 *
 */
class_quaternion_f32 namespace_alg_quaternion::unit_imaginary_y()
{
    class_quaternion_f32 result;
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 1.0f;
    result[3] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位虚四元数Z
 *
 */
class_quaternion_f32 namespace_alg_quaternion::unit_imaginary_z()
{
    class_quaternion_f32 result;
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 1.0f;
    return (result);
}

/**brief 根据三维向量获取纯四元数
 *
 * @param Vector 三维向量
 * @return class_quaternion_f32 纯四元数
 */
class_quaternion_f32 namespace_alg_quaternion::from_vector(const class_matrix_f32<3, 1> &vector)
{
    class_quaternion_f32 result;
    result[0] = 0.0f;
    result[1] = vector[0][0];
    result[2] = vector[1][0];
    result[3] = vector[2][0];
    return (result);
}

/**
 * @brief 根据旋转矩阵获取四元数
 *
 * @param Rotation_Matrix 3x3旋转矩阵
 * @return class_quaternion_f32 四元数
 */
class_quaternion_f32 namespace_alg_quaternion::from_rotation_matrix(const class_matrix_f32<3, 3> &rotation_matrix)
{
    class_quaternion_f32 result;
    float trace = rotation_matrix.get_trace();
    if (trace > 0.0f)
    {
        // 矩阵迹是正的

        float s = sqrtf(trace + 1.0f) * 2.0f;
        result[0] = 0.25f * s;

        result[1] = (rotation_matrix[2][1] - rotation_matrix[1][2]) / s;
        result[2] = (rotation_matrix[0][2] - rotation_matrix[2][0]) / s;
        result[3] = (rotation_matrix[1][0] - rotation_matrix[0][1]) / s;
    }
    else if ((rotation_matrix[0][0] >= rotation_matrix[1][1]) && (rotation_matrix[0][0] >= rotation_matrix[2][2]))
    {
        // r00是最大值

        float s = sqrtf(1.0f + rotation_matrix[0][0] - rotation_matrix[1][1] - rotation_matrix[2][2]) * 2.0f;

        result[0] = (rotation_matrix[2][1] - rotation_matrix[1][2]) / s;
        result[1] = 0.25f * s;
        result[2] = (rotation_matrix[0][1] + rotation_matrix[1][0]) / s;
        result[3] = (rotation_matrix[0][2] + rotation_matrix[2][0]) / s;
    }
    else if (rotation_matrix[1][1] >= rotation_matrix[2][2] && rotation_matrix[1][1] >= rotation_matrix[0][0])
    {
        // r11是最大值

        float s = sqrtf(1.0f + rotation_matrix[1][1] - rotation_matrix[0][0] - rotation_matrix[2][2]) * 2.0f;

        result[0] = (rotation_matrix[0][2] - rotation_matrix[2][0]) / s;
        result[1] = (rotation_matrix[0][1] + rotation_matrix[1][0]) / s;
        result[2] = 0.25f * s;
        result[3] = (rotation_matrix[1][2] + rotation_matrix[2][1]) / s;
    }
    else if (rotation_matrix[2][2] >= rotation_matrix[0][0] && rotation_matrix[2][2] >= rotation_matrix[1][1])
    {
        // r22是最大值

        float s = sqrtf(1.0f + rotation_matrix[2][2] - rotation_matrix[0][0] - rotation_matrix[1][1]) * 2.0f;

        result[0] = (rotation_matrix[1][0] - rotation_matrix[0][1]) / s;
        result[1] = (rotation_matrix[0][2] + rotation_matrix[2][0]) / s;
        result[2] = (rotation_matrix[1][2] + rotation_matrix[2][1]) / s;
        result[3] = 0.25f * s;
    }
    else
    {
        // 其他情况, 返回单位实四元数
        result = namespace_alg_quaternion::unit_real();
    }

    return (result);
}

/**
 * @brief 根据旋转轴与旋转角度获取四元数
 *
 * @param Axis 旋转轴, 单位向量
 * @param Angle 旋转角度, 单位弧度
 * @return class_quaternion_f32 四元数
 */
class_quaternion_f32 namespace_alg_quaternion::from_axis_angle(const class_matrix_f32<3, 1> &axis, const float &angle)
{
    class_quaternion_f32 result;
    float half_angle = angle * 0.5f;
    float cos_half_angle = arm_cos_f32(half_angle);
    float sin_half_angle = arm_sin_f32(half_angle);
    result[0] = cos_half_angle;
    result[1] = axis[0][0] * sin_half_angle;
    result[2] = axis[1][0] * sin_half_angle;
    result[3] = axis[2][0] * sin_half_angle;
    return (result);
}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/