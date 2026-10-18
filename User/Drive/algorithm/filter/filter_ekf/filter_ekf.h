/**
 * @file alg_filter_ekf.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 扩展kalman滤波器
 * @version 0.1
 * @date 2025-09-15 0.1 新增
 *
 * @copyright ustc-robovalker (c) 2025
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "math_matrix.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief reusable, 扩展kalman滤波器算法
 *
 * 系统模型
 * x_k = f(x_{k-1}, u_k, w_k)
 * z_k = h(x_k, v_k)
 * 其中, w_k ~ n(0, q), v_k ~ n(0, r)
 *
 * 每轮调用预测和更新函数前, 需要先更新输入向量vector_u和测量向量vector_z
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 */
template<uint32_t state_dimension = 2, uint32_t input_dimension = 1, uint32_t measurement_dimension = 2>
class class_filter_ekf
{
public:
    // 随着系统确定的
    // 系统状态函数
    class_matrix_f32<state_dimension, 1> (*function_f)(const class_matrix_f32<state_dimension, 1> &vector_x, const class_matrix_f32<input_dimension, 1> &vector_u, const float &d_t);

    // 系统状态函数对状态的雅可比矩阵
    class_matrix_f32<state_dimension, state_dimension> (*function_jacobian_f_x)(const class_matrix_f32<state_dimension, 1> &vector_x, const class_matrix_f32<input_dimension, 1> &vector_u, const float &d_t);

    // 系统状态函数对过程噪声的雅可比矩阵
    class_matrix_f32<state_dimension, input_dimension> (*function_jacobian_f_w)(const class_matrix_f32<state_dimension, 1> &vector_x, const class_matrix_f32<input_dimension, 1> &vector_u, const float &d_t);

    // 系统测量函数
    class_matrix_f32<measurement_dimension, 1> (*function_h)(const class_matrix_f32<state_dimension, 1> &vector_x, const float &d_t);

    // 系统测量函数对状态的雅可比矩阵
    class_matrix_f32<measurement_dimension, state_dimension> (*function_jacobian_h_x)(const class_matrix_f32<state_dimension, 1> &vector_x, const float &d_t);

    // 系统测量函数对测量噪声的雅可比矩阵
    class_matrix_f32<measurement_dimension, measurement_dimension> (*function_jacobian_h_v)(const class_matrix_f32<state_dimension, 1> &vector_x, const float &d_t);

    // 需要自己调参的
    // 过程噪声协方差矩阵
    class_matrix_f32<input_dimension, input_dimension> matrix_q;
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

    void init(const class_matrix_f32<input_dimension, input_dimension> &__matrix_q, const class_matrix_f32<measurement_dimension, measurement_dimension> &__matrix_r, const class_matrix_f32<state_dimension, state_dimension> &__matrix_p = namespace_alg_matrix::identity<state_dimension, state_dimension>(), const class_matrix_f32<state_dimension, 1> &__vector_x = namespace_alg_matrix::zero<state_dimension, 1>(), const class_matrix_f32<input_dimension, 1> &__matrix_u = namespace_alg_matrix::zero<input_dimension, 1>());

    inline void config_nonlinear_state_model(class_matrix_f32<state_dimension, 1> (*__function_f)(const class_matrix_f32<state_dimension, 1> &vector_x, const class_matrix_f32<input_dimension, 1> &vector_u, const float &d_t), class_matrix_f32<state_dimension, state_dimension> (*__function_jacobian_f_x)(const class_matrix_f32<state_dimension, 1> &vector_x, const class_matrix_f32<input_dimension, 1> &vector_u, const float &d_t), class_matrix_f32<state_dimension, input_dimension> (*__function_jacobian_f_w)(const class_matrix_f32<state_dimension, 1> &vector_x, const class_matrix_f32<input_dimension, 1> &vector_u, const float &d_t));

    inline void config_nonlinear_measurement_model(class_matrix_f32<measurement_dimension, 1> (*__function_h)(const class_matrix_f32<state_dimension, 1> &vector_x, const float &d_t), class_matrix_f32<measurement_dimension, state_dimension> (*__function_jacobian_h_x)(const class_matrix_f32<state_dimension, 1> &vector_x, const float &d_t), class_matrix_f32<measurement_dimension, measurement_dimension> (*__function_jacobian_h_v)(const class_matrix_f32<state_dimension, 1> &vector_x, const float &d_t));

    void set_d_t(const float &__d_t);

    void tim_predict_periodelapsedcallback();

    void tim_update_periodelapsedcallback();

protected:
    // 初始化相关常量

    // 常量

    // 单位矩阵
    const class_matrix_f32<state_dimension, state_dimension> matrix_i_state = namespace_alg_matrix::identity<state_dimension, state_dimension>();
    // 零矩阵
    const class_matrix_f32<state_dimension, state_dimension> matrix_zero_state = namespace_alg_matrix::zero<state_dimension, state_dimension>();

    // 内部变量

    // 读变量

    // 写变量

    // 时间差, 单位s
    float d_t = 0.001f;

    // 读写变量

    // 内部函数
};

/* exported variables --------------------------------------------------------*/

/* exported function declarations --------------------------------------------*/

/**
 * @brief 初始化扩展kalman滤波器
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 * @param __matrix_q 过程噪声协方差矩阵
 * @param __matrix_r 测量噪声协方差矩阵
 * @param __vector_x 初始状态向量
 * @param __matrix_u 初始输入向量
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
void class_filter_ekf<state_dimension, input_dimension, measurement_dimension>::init(const class_matrix_f32<input_dimension, input_dimension> &__matrix_q, const class_matrix_f32<measurement_dimension, measurement_dimension> &__matrix_r, const class_matrix_f32<state_dimension, state_dimension> &__matrix_p, const class_matrix_f32<state_dimension, 1> &__vector_x, const class_matrix_f32<input_dimension, 1> &__matrix_u)
{
    matrix_q = __matrix_q;
    matrix_r = __matrix_r;

    matrix_p = __matrix_p;
    vector_x = __vector_x;
    vector_u = __matrix_u;
}

/**
 * @brief 设置非线性系统状态模型
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 * @param __function_f 系统状态函数
 * @param __function_jacobian_f_x 系统状态函数对状态的雅可比矩阵
 * @param __function_jacobian_f_w 系统状态函数对过程噪声的雅可比矩阵
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
inline void class_filter_ekf<state_dimension, input_dimension, measurement_dimension>::config_nonlinear_state_model(class_matrix_f32 < state_dimension, 1 > (*__function_f)(const class_matrix_f32 < state_dimension, 1 > &vector_x, 
const class_matrix_f32<input_dimension, 1> &vector_u, 
const float &d_t
)
,
class_matrix_f32<state_dimension, state_dimension> (*__function_jacobian_f_x)(const class_matrix_f32<state_dimension, 1> &vector_x, const class_matrix_f32<input_dimension, 1> &vector_u, const float &d_t), class_matrix_f32<state_dimension, input_dimension>(*__function_jacobian_f_w)(
const class_matrix_f32<state_dimension, 1> &vector_x, 
const class_matrix_f32<input_dimension, 1> &vector_u, 
const float &d_t
)
)
{
    function_f = __function_f;
    function_jacobian_f_x = __function_jacobian_f_x;
    function_jacobian_f_w = __function_jacobian_f_w;
}

/**
 * @brief 设置非线性系统测量模型
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 * @param __function_h 系统测量函数
 * @param __function_jacobian_h_x 系统测量函数对状态的雅可比矩阵
 * @param __function_jacobian_h_v 系统测量函数对测量噪声的雅可比矩阵
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
inline void class_filter_ekf<state_dimension, input_dimension, measurement_dimension>::config_nonlinear_measurement_model(class_matrix_f32 < measurement_dimension, 1 > (*__function_h)(const class_matrix_f32 < state_dimension, 1 > &vector_x, const float &d_t), class_matrix_f32 < measurement_dimension, state_dimension > (*__function_jacobian_h_x)(const class_matrix_f32 < state_dimension, 1 > &vector_x, const float &d_t), class_matrix_f32 < measurement_dimension, measurement_dimension > (*__function_jacobian_h_v)(const class_matrix_f32 < state_dimension, 1 > &vector_x, const float &d_t))
{
    function_h = __function_h;
    function_jacobian_h_x = __function_jacobian_h_x;
    function_jacobian_h_v = __function_jacobian_h_v;
}

/**
 * @brief 设置计算周期
 *
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
void class_filter_ekf<state_dimension, input_dimension, measurement_dimension>::set_d_t(const float &__d_t)
{
    d_t = __d_t;
}

/**
 * @brief 扩展kalman滤波器预测步骤, 周期与采样周期相同
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
void class_filter_ekf<state_dimension, input_dimension, measurement_dimension>::tim_predict_periodelapsedcallback()
{
    // 计算jacobi矩阵
    class_matrix_f32 < state_dimension, state_dimension > matrix_f_x = function_jacobian_f_x(vector_x, vector_u, d_t);
    class_matrix_f32 < state_dimension, input_dimension > matrix_f_w = function_jacobian_f_w(vector_x, vector_u, d_t);

    // 预测状态向量
    vector_x_prior = function_f(vector_x, vector_u, d_t);

    // 预测误差协方差矩阵
    matrix_p_prior = matrix_f_x * matrix_p * matrix_f_x.get_transpose() + matrix_f_w * matrix_q * matrix_f_w.get_transpose();

    // 在未更新观测时, 也能有后验
    vector_x = vector_x_prior;
    matrix_p = matrix_p_prior;
}

/**
 * @brief 扩展kalman滤波器更新步骤, 周期与采样周期相同
 *
 * @tparam state_dimension 状态维度
 * @tparam input_dimension 输入维度
 * @tparam measurement_dimension 测量维度
 */
template<uint32_t state_dimension, uint32_t input_dimension, uint32_t measurement_dimension>
void class_filter_ekf<state_dimension, input_dimension, measurement_dimension>::tim_update_periodelapsedcallback()
{
    // 计算jacobi矩阵
    class_matrix_f32 < measurement_dimension, state_dimension > matrix_h_x = function_jacobian_h_x(vector_x_prior, d_t);
    class_matrix_f32 < measurement_dimension, measurement_dimension > matrix_h_v = function_jacobian_h_v(vector_x_prior, d_t);

    // 计算kalman增益矩阵
    matrix_k = matrix_p_prior * matrix_h_x.get_transpose() * (matrix_h_x * matrix_p_prior * matrix_h_x.get_transpose() + matrix_h_v * matrix_r * matrix_h_v.get_transpose()).get_inverse();

    // 更新状态向量
    vector_x = vector_x_prior + matrix_k * (vector_z - function_h(vector_x_prior, d_t));

    // 更新误差协方差矩阵, 为获得更好的数值稳定性, 采用约瑟夫形式
    class_matrix_f32 < state_dimension, state_dimension > matrix_tmp = matrix_i_state - matrix_k * matrix_h_x;
    matrix_p = matrix_tmp * matrix_p_prior * matrix_tmp.get_transpose() + matrix_k * matrix_h_v * matrix_r * matrix_h_v.get_transpose() * matrix_k.get_transpose();
}

/************************ copyright(c) ustc-robovalker **************************/