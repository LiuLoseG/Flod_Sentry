/**
 * @file alg_matrix.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 矩阵计算相关支持库
 * @version 0.1
 * @date 2025-08-21 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#include "math_matrix.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 矩阵比较大小时允许的最大误差
float matrix_compare_epsilon = 1e-6f;

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 获取2D空间的X轴单位向量
 *
 */
class_matrix_f32<2, 1> namespace_alg_matrix::axis_x_2d()
{
    class_matrix_f32<2, 1> result;
    result[0][0] = 1.0f;
    result[1][0] = 0.0f;
    return (result);
}

/**
 * @brief 获取2D空间的Y轴单位向量
 *
 */
class_matrix_f32<2, 1> namespace_alg_matrix::axis_y_2d()
{
    class_matrix_f32<2, 1> result;
    result[0][0] = 0.0f;
    result[1][0] = 1.0f;
    return (result);
}

/**
 * @brief 获取X轴单位向量
 *
 */
class_matrix_f32<3, 1> namespace_alg_matrix::axis_x_3d()
{
    class_matrix_f32<3, 1> result;
    result[0][0] = 1.0f;
    result[1][0] = 0.0f;
    result[2][0] = 0.0f;
    return (result);
}

/**
 * @brief 获取Y轴单位向量
 *
 */
class_matrix_f32<3, 1> namespace_alg_matrix::axis_y_3d()
{
    class_matrix_f32<3, 1> result;
    result[0][0] = 0.0f;
    result[1][0] = 1.0f;
    result[2][0] = 0.0f;
    return (result);
}

/**
 * @brief 获取Z轴单位向量
 *
 */
class_matrix_f32<3, 1> namespace_alg_matrix::axis_z_3d()
{
    class_matrix_f32<3, 1> result;
    result[0][0] = 0.0f;
    result[1][0] = 0.0f;
    result[2][0] = 1.0f;
    return (result);
}

/**
 * @brief 获取二维空间的旋转矩阵
 *
 * @param Angle 旋转角度, 单位弧度
 * @return Class_Matrix_f32<2, 2> 旋转矩阵
 */
class_matrix_f32<2, 2> namespace_alg_matrix::from_angle(const float &angle)
{
    class_matrix_f32<2, 2> result;
    float tmp_cos, tmp_sin;
    tmp_cos = arm_cos_f32(angle);
    tmp_sin = arm_sin_f32(angle);
    result[0][0] = tmp_cos;
    result[0][1] = -tmp_sin;
    result[1][0] = tmp_sin;
    result[1][1] = tmp_cos;
    return (result);
}

/**
 * @brief 获取三维空间的旋转矩阵
 *
 * @param Angle 旋转角度, 单位弧度
 * @param Axis 旋转轴, 单位向量
 * @return Class_Matrix_f32<3, 3> 旋转矩阵
 */
class_matrix_f32<3, 3> namespace_alg_matrix::from_axis_angle(const float &angle, const class_matrix_f32<3, 1> &axis)
{
    class_matrix_f32<3, 3> result;
    float tmp_cos, tmp_sin;
    tmp_cos = arm_cos_f32(angle);
    tmp_sin = arm_sin_f32(angle);
    result[0][0] = tmp_cos + axis[0][0] * axis[0][0] * (1.0f - tmp_cos);
    result[0][1] = axis[0][0] * axis[1][0] * (1.0f - tmp_cos) - axis[2][0] * tmp_sin;
    result[0][2] = axis[0][0] * axis[2][0] * (1.0f - tmp_cos) + axis[1][0] * tmp_sin;
    result[1][0] = axis[1][0] * axis[0][0] * (1.0f - tmp_cos) + axis[2][0] * tmp_sin;
    result[1][1] = tmp_cos + axis[1][0] * axis[1][0] * (1.0f - tmp_cos);
    result[1][2] = axis[1][0] * axis[2][0] * (1.0f - tmp_cos) - axis[0][0] * tmp_sin;
    result[2][0] = axis[2][0] * axis[0][0] * (1.0f - tmp_cos) - axis[1][0] * tmp_sin;
    result[2][1] = axis[2][0] * axis[1][0] * (1.0f - tmp_cos) + axis[0][0] * tmp_sin;
    result[2][2] = tmp_cos + axis[2][0] * axis[2][0] * (1.0f - tmp_cos);
    return (result);
}

/**
 * @brief 获取三维空间的旋转矩阵
 *
 * @param Yaw 偏航角, 单位弧度
 * @param Pitch 俯仰角, 单位弧度
 * @param Roll 横滚角, 单位弧度
 * @return Class_Matrix_f32<3, 3> 旋转矩阵
 */
class_matrix_f32<3, 3> namespace_alg_matrix::from_euler_angle(const float &yaw, const float &pitch, const float &roll)
{
    class_matrix_f32<3, 3> result;
    float cy = arm_cos_f32(yaw);
    float sy = arm_sin_f32(yaw);
    float cp = arm_cos_f32(pitch);
    float sp = arm_sin_f32(pitch);
    float cr = arm_cos_f32(roll);
    float sr = arm_sin_f32(roll);

    result[0][0] = cy * cp;
    result[0][1] = cy * sp * sr - sy * cr;
    result[0][2] = cy * sp * cr + sy * sr;

    result[1][0] = sy * cp;
    result[1][1] = sy * sp * sr + cy * cr;
    result[1][2] = sy * sp * cr - cy * sr;

    result[2][0] = -sp;
    result[2][1] = cp * sr;
    result[2][2] = cp * cr;

    return (result);
}

/**
 * @brief 计算两个三维列向量的叉乘
 *
 * @param Vector_1 向量1
 * @param Vector_2 向量2
 * @return Class_Matrix_f32<3, 1> 叉乘结果
 */
class_matrix_f32<3, 1> namespace_alg_matrix::operator_cross_3d(const class_matrix_f32<3, 1> &vector_1, const class_matrix_f32<3, 1> &vector_2)
{
    class_matrix_f32<3, 1> result;
    result[0][0] = vector_1[1][0] * vector_2[2][0] - vector_1[2][0] * vector_2[1][0];
    result[1][0] = vector_1[2][0] * vector_2[0][0] - vector_1[0][0] * vector_2[2][0];
    result[2][0] = vector_1[0][0] * vector_2[1][0] - vector_1[1][0] * vector_2[0][0];
    return (result);
}

/**
 * @brief 计算两个二维列向量的叉乘
 *
 * @param Vector_1 向量1
 * @param Vector_2 向量2
 * @return float 叉乘结果
 */
float namespace_alg_matrix::operator_cross_2d(const class_matrix_f32<2, 1> &vector_1, const class_matrix_f32<2, 1> &vector_2)
{
    float result;
    result = vector_1[0][0] * vector_2[1][0] - vector_1[1][0] * vector_2[0][0];
    return (result);
}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
