/**
 * @file alg_complex.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 复数计算相关支持库
 * @version 0.1
 * @date 2025-08-22 0.1 新建文档
 *
 * @copyright ustc-robovalker (c) 2025
 *
 */

#include "complex.h"

/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/**
 * @brief 获取零复数
 *
 */
class_complex_f32 namespace_alg_complex::zero()
{
    class_complex_f32 result;
    result[0] = 0.0f;
    result[1] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位实复数
 *
 */
class_complex_f32 namespace_alg_complex::unit_real()
{
    class_complex_f32 result;
    result[0] = 1.0f;
    result[1] = 0.0f;
    return (result);
}

/**
 * @brief 获取单位虚复数
 *
 */
class_complex_f32 namespace_alg_complex::unit_imaginary()
{
    class_complex_f32 result;
    result[0] = 0.0f;
    result[1] = 1.0f;
    return (result);
}

/**
 * @brief 获取模长为1的旋转复数
 *
 * @param angle 旋转角度, 单位: 弧度
 */
class_complex_f32 namespace_alg_complex::angle_unit(const float &angle)
{
    class_complex_f32 result;
    result[0] = arm_cos_f32(angle);
    result[1] = arm_sin_f32(angle);
    return (result);
}

/**
 * @brief 获取根据极坐标的表示来定义的复数
 *
 * @param modulus 复数模长
 * @param angle 复数幅角
 */
class_complex_f32 namespace_alg_complex::polar(const float &modulus, const float &angle)
{
    class_complex_f32 result;
    result[0] = modulus * arm_cos_f32(angle);
    result[1] = modulus * arm_sin_f32(angle);
    return (result);
}

/* function prototypes -------------------------------------------------------*/

/************************ copyright(c) ustc-robovalker **************************/