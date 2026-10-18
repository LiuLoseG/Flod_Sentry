/**
 * @file alg_matrix.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 矩阵计算相关支持库
 * @version 0.1
 * @date 2025-08-21 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "basic_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

extern float matrix_compare_epsilon;

/**
 * @brief Reusable, 矩阵运算
 *
 */
template<int row, int column>
class class_matrix_f32
{
public:
    // 矩阵数据区域
    float data[row * column] = {0.0f};

    // 构造函数
    class_matrix_f32()
    {
        for (int i = 0; i < row * column; i++)
        {
            data[i] = 0.0f;
        }
    }

    class_matrix_f32(const float __data[row * column])
    {
        memcpy(data, __data, sizeof(float) * row * column);
    }

    // 复制构造函数
    class_matrix_f32(const class_matrix_f32<row, column> &matrix)
    {
        memcpy(data, matrix.data, sizeof(float) * row * column);
    }

    // 移动构造函数
    class_matrix_f32(class_matrix_f32<row, column> &&matrix) noexcept
    {
        memcpy(data, matrix.data, sizeof(float) * row * column);
    }

    // 析构函数
    ~class_matrix_f32() = default;

    // 复制赋值函数
    class_matrix_f32<row, column> &operator=(const class_matrix_f32<row, column> &matrix)
    {
        if (this != &matrix)
        {
            memcpy(data, matrix.data, sizeof(float) * row * column);
        }
        return (*this);
    }

    // 移动赋值函数
    class_matrix_f32<row, column> &operator=(class_matrix_f32<row, column> &&matrix) noexcept
    {
        memcpy(data, matrix.data, sizeof(float) * row * column);
        return (*this);
    }

    inline float *operator[](const int &index)
    {
        return (&data[index * column]);
    }

    inline const float *operator[](const int &index) const
    {
        return (&data[index * column]);
    }

    inline class_matrix_f32<row, column> operator+() const
    {
        return (*this);
    }

    inline class_matrix_f32<row, column> operator+(const float &value) const
    {
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = data[i] + value;
        }
        return (result);
    }

    inline class_matrix_f32<row, column> operator+(const class_matrix_f32<row, column> &matrix) const
    {
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = data[i] + matrix.data[i];
        }
        return (result);
    }

    inline class_matrix_f32<row, column> operator-() const
    {
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = -data[i];
        }
        return (result);
    }

    inline class_matrix_f32<row, column> operator-(const float &value) const
    {
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = data[i] - value;
        }
        return (result);
    }

    inline class_matrix_f32<row, column> operator-(const class_matrix_f32<row, column> &matrix) const
    {
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = data[i] - matrix.data[i];
        }
        return (result);
    }

    inline class_matrix_f32<row, column> operator*(const float &value) const
    {
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = data[i] * value;
        }
        return (result);
    }

    inline friend class_matrix_f32<row, column> operator*(const float &value, const class_matrix_f32<row, column> &matrix)
    {
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = value * matrix.data[i];
        }
        return (result);
    }

    template<int tmp>
    inline class_matrix_f32<row, tmp> operator*(const class_matrix_f32<column, tmp> &matrix)
    {
        class_matrix_f32<row, tmp> result;
        for (int i = 0; i < row; i++)
        {
            for (int k = 0; k < column; k++)
            {
                float data_ik = data[i * column + k];
                int result_index = i * tmp;
                int matrix_index = k * tmp;
                for (int j = 0; j < tmp; j++)
                {
                    result.data[result_index + j] += data_ik * matrix.data[matrix_index + j];
                }
            }
        }
        return (result);
    }

    inline class_matrix_f32<row, column> operator/(const float &value) const
    {
        if (fabs(value) <= matrix_compare_epsilon)
        {
            return (*this);
        }
        class_matrix_f32<row, column> result;
        for (int i = 0; i < row * column; i++)
        {
            result.data[i] = data[i] / value;
        }
        return (result);
    }

    inline class_matrix_f32<row, column> &operator+=(const float &value)
    {
        for (int i = 0; i < row * column; i++)
        {
            data[i] += value;
        }
        return (*this);
    }

    inline class_matrix_f32<row, column> &operator+=(const class_matrix_f32<row, column> &matrix)
    {
        for (int i = 0; i < row * column; i++)
        {
            data[i] += matrix.data[i];
        }
        return (*this);
    }

    inline class_matrix_f32<row, column> &operator-=(const float &value)
    {
        for (int i = 0; i < row * column; i++)
        {
            data[i] -= value;
        }
        return (*this);
    }

    inline class_matrix_f32<row, column> &operator-=(const class_matrix_f32<row, column> &matrix)
    {
        for (int i = 0; i < row * column; i++)
        {
            data[i] -= matrix.data[i];
        }
        return (*this);
    }

    inline class_matrix_f32<row, column> &operator*=(const float &value)
    {
        for (int i = 0; i < row * column; i++)
        {
            data[i] *= value;
        }
        return (*this);
    }

    inline class_matrix_f32<row, column> &operator/=(const float &value)
    {
        if (fabs(value) <= matrix_compare_epsilon)
        {
            return (*this);
        }
        for (int i = 0; i < row * column; i++)
        {
            data[i] /= value;
        }
        return (*this);
    }

    template<int tmp_row, int tmp_column>
    inline bool operator==(const class_matrix_f32<tmp_row, tmp_column> &matrix) const
    {
        if (row == tmp_row && column == tmp_column)
        {
            for (int i = 0; i < row * column; i++)
            {
                if (fabs(data[i] - matrix.data[i]) > matrix_compare_epsilon)
                {
                    return (false);
                }
            }
            return (true);
        }
        else
        {
            return (false);
        }
    }

    template<int tmp_row, int tmp_column>
    inline class_matrix_f32<row, column> operator!=(const class_matrix_f32<row, column> &matrix) const
    {
        if (row != tmp_row || column != tmp_column)
        {
            return (true);
        }
        else
        {
            for (int i = 0; i < row * column; i++)
            {
                if (fabs(data[i] - matrix.data[i]) > matrix_compare_epsilon)
                {
                    return (true);
                }
            }
            return (false);
        }
    }

    inline class_matrix_f32<column, row> get_transpose() const;

    inline float get_trace() const;

    inline class_matrix_f32<1, column> get_row(const int &row_index) const;

    inline class_matrix_f32<row, 1> get_column(const int &column_index) const;

    // 方阵特有

    template<int tmp_row = row, int tmp_column = column>
    inline std::enable_if_t<tmp_row == tmp_column, class_matrix_f32<tmp_row, tmp_row>> &operator*=(const class_matrix_f32<tmp_row, tmp_row> &matrix)
    {
        class_matrix_f32<tmp_row, tmp_row> result;
        for (int i = 0; i < tmp_row; i++)
        {
            for (int k = 0; k < tmp_row; k++)
            {
                float data_ik = data[i * tmp_row + k];
                int result_index = i * tmp_row;
                int matrix_index = k * tmp_row;
                for (int j = 0; j < tmp_row; j++)
                {
                    result.data[result_index + j] += data_ik * matrix.data[matrix_index + j];
                }
            }
        }
        *this = result;
        return (*this);
    }

    template<int tmp_row = row, int tmp_column = column>
    inline std::enable_if_t<tmp_row == tmp_column, class_matrix_f32<tmp_row, tmp_row>> get_inverse() const;

    // 列向量特有

    template<int tmp_row = row, int tmp_column = column>
    inline std::enable_if_t<tmp_column == 1, float> get_modulus() const;

    template<int tmp_row = row, int tmp_column = column>
    inline std::enable_if_t<tmp_column == 1, class_matrix_f32<tmp_row, 1>> get_normalization() const;

    // 三维列向量特有

    template<int tmp_row = row, int tmp_column = column>
    inline std::enable_if_t<tmp_row == 3 && tmp_column == 1, class_matrix_f32<3, 3>> get_cross_matrix() const;

    // 单个元素特有

    template<int tmp_row = row, int tmp_column = column>
    inline std::enable_if_t<tmp_row == 1 && tmp_column == 1, float> get_single_value() const;

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

namespace namespace_alg_matrix
{
    class_matrix_f32<2, 1> axis_x_2d();

    class_matrix_f32<2, 1> axis_y_2d();

    class_matrix_f32<3, 1> axis_x_3d();

    class_matrix_f32<3, 1> axis_y_3d();

    class_matrix_f32<3, 1> axis_z_3d();

    class_matrix_f32<2, 2> from_angle(const float &angle);

    class_matrix_f32<3, 3> from_axis_angle(const float &angle, const class_matrix_f32<3, 1> &axis);

    class_matrix_f32<3, 3> from_euler_angle(const float &yaw, const float &pitch, const float &roll);

    template<int row, int column>
    class_matrix_f32<row, column> zero();

    template<int row, int column>
    class_matrix_f32<row, column> identity();

    template<int row, int column>
    class_matrix_f32<row, column> constant(const float &value);

    template<int row>
    float operator_dot(const class_matrix_f32<row, 1> &vector_1, const class_matrix_f32<row, 1> &vector_2);

    class_matrix_f32<3, 1> operator_cross_3d(const class_matrix_f32<3, 1> &vector_1, const class_matrix_f32<3, 1> &vector_2);

    float operator_cross_2d(const class_matrix_f32<2, 1> &vector_1, const class_matrix_f32<2, 1> &vector_2);
}

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 矩阵转置, 返回转置矩阵
 *
 * @tparam row 行数
 * @tparam column 列数
 * @return Class_Matrix_f32<column, row> 转置矩阵
 */
template<int row, int column>
inline class_matrix_f32<column, row> class_matrix_f32<row, column>::get_transpose() const
{
    class_matrix_f32<column, row> result;

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < column; j++)
        {
            result[j][i] = data[i * column + j];
        }
    }

    return (result);
}

/**
 * @brief 矩阵迹, 返回矩阵对角线元素之和
 *
 * @tparam row 行数
 * @tparam column 列数
 * @return float 矩阵迹
 */
template<int row, int column>
inline float class_matrix_f32<row, column>::get_trace() const
{
    float result = 0.0f;
    for (int i = 0; i < fmin(row, column); i++)
    {
        result += data[i * column + i];
    }
    return (result);
}

/**
 * @brief 获取矩阵的某一行
 *
 * @tparam row 行数
 * @tparam column 列数
 * @param Row_Index 行索引
 * @return Class_Matrix_f32<row, 1> 行向量
 */
template<int row, int column>
inline class_matrix_f32<1, column> class_matrix_f32<row, column>::get_row(const int &row_index) const
{
    class_matrix_f32<1, column> result;

    memcpy(result.data, &data[row_index * column], sizeof(float) * column);

    return (result);
}

/**
 * @brief 获取矩阵的某一列
 *
 * @tparam row 行数
 * @tparam column 列数
 * @param Column_Index 列索引
 * @return Class_Matrix_f32<1, column> 列向量
 */
template<int row, int column>
inline class_matrix_f32<row, 1> class_matrix_f32<row, column>::get_column(const int &column_index) const
{
    class_matrix_f32<row, 1> result;

    for (int i = 0; i < row; i++)
    {
        result.data[i] = data[i * column + column_index];
    }

    return (result);
}

/**
 * @brief 获取矩阵的逆矩阵
 *
 * @tparam row 行数
 * @tparam column 列数
 * @tparam tmp_row 行数
 * @tparam tmp_column 列数
 * @return Class_Matrix_f32<tmp_row, tmp_column> 逆矩阵
 */
template<int row, int column>
template<int tmp_row, int tmp_column>
inline std::enable_if_t<tmp_row == tmp_column, class_matrix_f32<tmp_row, tmp_row>> class_matrix_f32<row, column>::get_inverse() const
{
    // 扩展矩阵 [A|I]
    class_matrix_f32<tmp_row, 2 * tmp_row> extended_matrix = namespace_alg_matrix::zero<tmp_row, 2 * tmp_row>();

    // 扩展矩阵初始化
    for (int i = 0; i < tmp_row; i++)
    {
        // 矩阵A部分
        memcpy(&extended_matrix.data[i * 2 * tmp_row], &data[i * tmp_row], sizeof(float) * tmp_row);
        // 单位矩阵部分
        extended_matrix[i][i + tmp_row] = 1.0f;
    }

    // 高斯消元法
    for (int i = 0; i < tmp_row; i++)
    {
        float max_value = fabs(extended_matrix[i][i]);
        int max_index = i;
        for (int j = i + 1; j < tmp_row; j++)
        {
            if (fabs(extended_matrix[j][i]) > max_value)
            {
                max_value = fabs(extended_matrix[j][i]);
                max_index = j;
            }
        }
        // 最大元太小, 认为矩阵不可逆
        if (max_value <= matrix_compare_epsilon)
        {
            return (namespace_alg_matrix::identity<tmp_row, tmp_row>());
        }
        // 交换行, 将第i行与主元所在行交换
        if (max_index != i)
        {
            float tmp_data[2 * tmp_row];
            memcpy(tmp_data, &extended_matrix.data[i * 2 * tmp_row], sizeof(float) * 2 * tmp_row);
            memcpy(&extended_matrix.data[i * 2 * tmp_row], &extended_matrix.data[max_index * 2 * tmp_row], sizeof(float) * 2 * tmp_row);
            memcpy(&extended_matrix.data[max_index * 2 * tmp_row], tmp_data, sizeof(float) * 2 * tmp_row);
        }

        // 行归一化
        float divisor = 1.0f / extended_matrix[i][i];
        for (int j = 0; j < 2 * tmp_row; j++)
        {
            extended_matrix.data[i * 2 * tmp_row + j] *= divisor;
        }

        // 消元
        for (int j = 0; j < tmp_row; j++)
        {
            // 遍历每一行进行消元, 如果是主元行, 则跳过
            if (j != i)
            {
                float factor = extended_matrix[j][i];
                float tmp_data[2 * tmp_row];
                // 计算消元行
                for (int k = 0; k < 2 * tmp_row; k++)
                {
                    tmp_data[k] = -factor * extended_matrix.data[i * 2 * tmp_row + k];
                }
                // 将消元行与当前行相加
                for (int k = 0; k < 2 * tmp_row; k++)
                {
                    extended_matrix.data[j * 2 * tmp_row + k] += tmp_data[k];
                }
            }
        }
    }

    // 高斯消元法完成, 接下来是提取逆矩阵部分
    class_matrix_f32<tmp_row, tmp_row> result;

    for (int i = 0; i < tmp_row; i++)
    {
        memcpy(&result.data[i * tmp_row], &extended_matrix.data[i * 2 * tmp_row + tmp_row], sizeof(float) * tmp_row);
    }

    return (result);
}

/**
 * @brief 获取向量的模长
 *
 * @tparam row 行数
 * @tparam column 列数
 * @tparam tmp_row 行数
 * @tparam tmp_column 列数
 * @return Class_Matrix_f32<tmp_row, tmp_column> 模长
 */
template<int row, int column>
template<int tmp_row, int tmp_column>
inline std::enable_if_t<tmp_column == 1, float> class_matrix_f32<row, column>::get_modulus() const
{
    float norm = 0.0f;
    for (int i = 0; i < tmp_row; i++)
    {
        norm += data[i] * data[i];
    }
    norm = sqrtf(norm);
    return (norm);
}

/**
 * @brief 获取向量的归一化结果
 *
 * @tparam row 行数
 * @tparam column 列数
 * @tparam tmp_row 行数
 * @tparam tmp_column 列数
 * @return Class_Matrix_f32<tmp_row, tmp_column> 归一化结果
 */
template<int row, int column>
template<int tmp_row, int tmp_column>
inline std::enable_if_t<tmp_column == 1, class_matrix_f32<tmp_row, 1>> class_matrix_f32<row, column>::get_normalization() const
{
    class_matrix_f32<tmp_row, 1> result;

    float norm = 0.0f;
    for (int i = 0; i < tmp_row; i++)
    {
        norm += data[i] * data[i];
    }
    norm = 1.0f / sqrtf(norm);
    for (int i = 0; i < tmp_row; i++)
    {
        result.data[i] = data[i] * norm;
    }
    return (result);
}

/**
 * @brief 获取三维列向量的叉乘矩阵
 *
 * @tparam row 行数
 * @tparam column 列数
 * @tparam tmp_row 行数
 * @tparam tmp_column 列数
 * @return Class_Matrix_f32<tmp_row, tmp_column> 叉乘矩阵
 */
template<int row, int column>
template<int tmp_row, int tmp_column>
inline std::enable_if_t<tmp_row == 3 && tmp_column == 1, class_matrix_f32<3, 3>> class_matrix_f32<row, column>::get_cross_matrix() const
{
    class_matrix_f32<3, 3> result;
    result.data[0] = 0.0f;
    result.data[1] = -data[2];
    result.data[2] = data[1];
    result.data[3] = data[2];
    result.data[4] = 0.0f;
    result.data[5] = -data[0];
    result.data[6] = -data[1];
    result.data[7] = data[0];
    result.data[8] = 0.0f;
    return (result);
}

/**
 * @brief 获取单个元素的值
 *
 * @tparam row 行数
 * @tparam column 列数
 * @tparam tmp_row 行数
 * @tparam tmp_column 列数
 * @return float 元素值
 */
template<int row, int column>
template<int tmp_row, int tmp_column>
inline std::enable_if_t<tmp_row == 1 && tmp_column == 1, float> class_matrix_f32<row, column>::get_single_value() const
{
    return (data[0]);
}

/**
 * @brief 获取对应尺寸的零矩阵
 *
 * @tparam row 行数
 * @tparam column 列数
 * @return Class_Matrix_f32<row, column> 零矩阵
 */
template<int row, int column>
class_matrix_f32<row, column> namespace_alg_matrix::zero()
{
    class_matrix_f32<row, column> result;
    return (result);
}

/**
 * @brief 获取对应尺寸的单位矩阵
 *
 * @tparam row 行数
 * @tparam column 列数
 * @return Class_Matrix_f32<row, column> 单位矩阵
 */
template<int row, int column>
class_matrix_f32<row, column> namespace_alg_matrix::identity()
{
    class_matrix_f32<row, column> result;
    int min_dimension = (row < column) ? row : column;
    for (int i = 0; i < min_dimension; i++)
    {
        result[i][i] = 1.0f;
    }
    return (result);
}

/** * @brief 获取对应尺寸的常量矩阵
 *
 * @tparam row 行数
 * @tparam column 列数
 * @return Class_Matrix_f32<row, column> 常量矩阵
 */
template<int row, int column>
class_matrix_f32<row, column> namespace_alg_matrix::constant(const float &Value)
{
    class_matrix_f32<row, column> result;
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < column; j++)
        {
            result[i][j] = Value;
        }
    }
    return (result);
}

/**
 * @brief 计算两个列向量的点乘
 *
 * @tparam row 向量行数
 * @param Vector_1 向量1
 * @param Vector_2 向量2
 * @return Class_Matrix_f32<1, 1> 点乘结果
 */
template<int row>
float namespace_alg_matrix::operator_dot(const class_matrix_f32<row, 1> &vector_1, const class_matrix_f32<row, 1> &vector_2)
{
    float result = 0.0f;
    for (int i = 0; i < row; i++)
    {
        result += vector_1[i][0] * vector_2[i][0];
    }
    return (result);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
