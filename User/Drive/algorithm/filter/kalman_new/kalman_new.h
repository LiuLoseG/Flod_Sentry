/**
 * @file alg_filter_kalman.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief kalman滤波器
 * @version 0.1
 * @date 2025-09-07 0.1 新增
 *
 * @copyright copyright (c) 2025
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "math_matrix.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief reusable, kalman滤波器算法
 *
 * 系统模型
 * x_k = a * x_{k-1} + b * u_k + w_k
 * z_k = h * x_k + v_k
 * 其中, w_k ~ n(0, q), v_k ~ n(0, r)
 *
 * 每轮调用预测和更新函数前, 需要先更新输入向量vector_u和测量向量vector_z
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 */
template<uint32_t state_dimension = 2, uint32_t input_dimension = 1, uint32_t measurement_dimension = 2>
class class_filter_kalman
{
public:
    // 随着系统确定的
    // 系统状态矩阵
    class_matrix_f32<state_dimension, state_dimension> matrix_a;
    class_matrix_f32<state_dimension, input_dimension> matrix_b;
    // 测量矩阵
    class_matrix_f32<measurement_dimension, state_dimension> matrix_h;

    // 需要自己调参的
    // 过程噪声协方差矩阵
    class_matrix_f32<state_dimension, state_dimension> matrix_q;
    // 测量噪声协方差矩阵
    class_matrix_f32<measurement_dimension, measurement_dimension> matrix_r;

    // 内部运算需要的
    // 误差协方差矩阵
    class_matrix_f32<state_dimension, state_dimension> matrix_p;
    // 状态向量
    class_matrix_f32<state_dimension, 1> vector_x;
    // 输入向量
    class_matrix_f32<input_dimension, 1> vector_u;
    // 先验估计状态向量
    class_matrix_f32<state_dimension, 1> vector_x_prior;
    // 先验误差协方差矩阵
    class_matrix_f32<state_dimension, state_dimension> matrix_p_prior;
    // kalman增益矩阵
    class_matrix_f32<state_dimension, measurement_dimension> matrix_k;

    // 传感器拿到的
    // 测量向量
    class_matrix_f32<measurement_dimension, 1> vector_z;

    void init(const class_matrix_f32<state_dimension, state_dimension> &__matrix_a, const class_matrix_f32<state_dimension, input_dimension> &__matrix_b, const class_matrix_f32<measurement_dimension, state_dimension> &__matrix_h, const class_matrix_f32<state_dimension, state_dimension> &__matrix_q, const class_matrix_f32<measurement_dimension, measurement_dimension> &__matrix_r, const class_matrix_f32<state_dimension, state_dimension> &__matrix_p = namespace_alg_matrix::identity<state_dimension, state_dimension>(), const class_matrix_f32<state_dimension, 1> &__vector_x = namespace_alg_matrix::zero<state_dimension, 1>(), const class_matrix_f32<input_dimension, 1> &__matrix_u = namespace_alg_matrix::zero<input_dimension, 1>());

    void tim_predict_periodelapsedcallback();

    void tim_update_periodelapsedcallback();

protected:
    // 初始化相关常量

    // 常量

    // 单位矩阵
    const class_matrix_f32<state_dimension, state_dimension> matrix_i_state = namespace_alg_matrix::identity<state_dimension, state_dimension>();

    // 内部变量

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

/* exported variables --------------------------------------------------------*/

/* exported function declarations --------------------------------------------*/

/**
 * @brief 初始化kalman滤波器
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 * @param __matrix_a 系统状态矩阵
 * @param __matrix_b 控制矩阵
 * @param __matrix_h 测量矩阵
 * @param __matrix_q 过程噪声协方差矩阵
 * @param __matrix_r 测量噪声协方差矩阵
 * @param __vector_x 初始状态向量
 * @param __matrix_u 初始输入向量
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
void class_filter_kalman<state_dimension, input_dimension, measurement_dimension>::init(const class_matrix_f32<state_dimension, state_dimension> &__matrix_a, const class_matrix_f32<state_dimension, input_dimension> &__matrix_b, const class_matrix_f32<measurement_dimension, state_dimension> &__matrix_h, const class_matrix_f32<state_dimension, state_dimension> &__matrix_q, const class_matrix_f32<measurement_dimension, measurement_dimension> &__matrix_r, const class_matrix_f32<state_dimension, state_dimension> &__matrix_p, const class_matrix_f32<state_dimension, 1> &__vector_x, const class_matrix_f32<input_dimension, 1> &__matrix_u)
{
    matrix_a = __matrix_a;
    matrix_b = __matrix_b;
    matrix_h = __matrix_h;

    matrix_q = __matrix_q;
    matrix_r = __matrix_r;

    matrix_p = __matrix_p;
    vector_x = __vector_x;
    vector_u = __matrix_u;
}

/**
 * @brief kalman滤波器预测步骤, 周期与采样周期相同
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
void class_filter_kalman<state_dimension, input_dimension, measurement_dimension>::tim_predict_periodelapsedcallback()
{
    // 预测状态向量
    vector_x_prior = matrix_a * vector_x + matrix_b * vector_u;

    // 预测误差协方差矩阵
    matrix_p_prior = matrix_a * matrix_p * matrix_a.get_transpose() + matrix_q;
}

/**
 * @brief kalman滤波器更新步骤, 周期与采样周期相同
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
void class_filter_kalman<state_dimension, input_dimension, measurement_dimension>::tim_update_periodelapsedcallback()
{
    // 计算kalman增益矩阵
    matrix_k = matrix_p_prior * matrix_h.get_transpose() * (matrix_h * matrix_p_prior * matrix_h.get_transpose() + matrix_r).get_inverse();

    // 更新状态向量
    vector_x = vector_x_prior + matrix_k * (vector_z - matrix_h * vector_x_prior);

    // 更新误差协方差矩阵, 为获得更好的数值稳定性, 采用约瑟夫形式
    class_matrix_f32 < state_dimension, state_dimension > matrix_tmp = matrix_i_state - matrix_k * matrix_h;
    // matrix_p = matrix_tmp * matrix_p_prior;
    matrix_p = matrix_tmp * matrix_p_prior * matrix_tmp.get_transpose() + matrix_k * matrix_r * matrix_k.get_transpose();
}

/************************ copyright(c) ustc-robovalker **************************/