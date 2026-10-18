/**
 * @file general_filter.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 通用滤波器类实现文件
 * @note 该文件实现了通用滤波器类，包括低通滤波器、高通滤波器、中值滤波器和滑动均值滤波器。
 * 通过这些滤波器类，可以对输入信号进行滤波处理，以减少噪声和干扰，提高信号的质量和稳定性。
 * @date 2026-08-25
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "general_filter.h"

/**
 * @brief 初始化低通滤波器
 *
 * @param __cutoff_frequency 截止频率(Hz)
 * @param __sampling_period 采样周期(s)
 */
void low_pass_filter_class::init(const float &__cutoff_frequency, const float &__sampling_period)
{
    float scaling_factor = 2.0f * PI * __cutoff_frequency * __sampling_period;

    cutoff_frequency = __cutoff_frequency;
    sampling_period = __sampling_period;

    alpha = scaling_factor / (scaling_factor + 1.0f);

    // 重置滤波状态, 避免运行时重新初始化时输出从旧状态跳变
    now_input = 0.0f;
    now_output = 0.0f;
    last_output = 0.0f;
}

/**
 * @brief 低通滤波器处理函数
 */
void low_pass_filter_class::low_calculate_callback(void)
{
    // 计算当前滤波输出值yn = yn-1 + α * (xn - yn-1)
    now_output = last_output + alpha * (now_input - last_output);
    last_output = now_output;
}

/**
 * @brief 初始化高通滤波器
 * @param __low_limit_frequency 
 * @param __sampling_period 
 */
void high_pass_filter_class::init(const float &__low_limit_frequency, const float &__sampling_period)
{
    float scaling_factor = 2.0f * PI * __low_limit_frequency * __sampling_period;

    low_limit_frequency = __low_limit_frequency;
    sampling_period = __sampling_period;

    alpha = 1.0f / (scaling_factor + 1.0f);

    // 重置滤波状态, 避免运行时重新初始化时输出从旧状态跳变
    now_input = 0.0f;
    last_input = 0.0f;
    now_output = 0.0f;
    last_output = 0.0f;
}

/**
 * @brief 高通滤波器处理函数
 */
void high_pass_filter_class::high_calculate_callback(void)
{
    // 计算当前滤波输出值yn = α * (yn-1 + xn - xn-1)
    now_output = alpha * (last_output + now_input - last_input);
    last_input = now_input;
    last_output = now_output;
}

/**
 * @brief 中值滤波器初始化
 * @param __median_filter_size 
 */
void median_filter_class::init(const uint16_t &__median_filter_size)
{
    median_filter_size = __median_filter_size;
    if (median_filter_size == 0)
    {
        median_filter_size = 1;
    }
    else if (median_filter_size > Median_Filter_Max_Size)
    {
        median_filter_size = Median_Filter_Max_Size;
    }

    data_count = 0;
    data_index = 0;
    now_output = 0.0f;
}

/**
 * @brief 中值滤波器处理函数
 * @note 流程: 写入环形缓冲 → 复制有效数据 → 部分选择排序 → 取中值
 *       中值只需窗口中间 1~2 个元素, 用部分选择排序只排到中值位置即可,
 *       避免全量排序, 数据移动次数由 O(N²) 降至 O(N)
 */
void median_filter_class::median_calculate_callback(void)
{
    // 1. 写入环形缓冲并推进下标
    data_buffer[data_index] = now_input;
    data_index = (data_index + 1) % median_filter_size;

    // 2. 有效数据个数(未满窗前按已有数据取部分中值, 避免未初始化垃圾参与排序)
    if (data_count < median_filter_size)
    {
        data_count++;
    }

    // 3. 复制有效数据到排序缓冲(满窗后环形缓冲 [0, n) 均为有效数据)
    for (uint16_t i = 0; i < data_count; i++)
    {
        sort_buffer[i] = data_buffer[i];
    }

    // 4. 部分选择排序: 中值只需窗口中间 1~2 个元素, 无需全量排序
    //    选择排序第 i 轮后 [0, i] 即为最小的 i+1 个元素(位置已确定),
    //    因此只需排到中值位置 mid = N/2 即可(偶数窗多排一个取中间两值)
    const uint16_t mid = data_count / 2;
    for (uint16_t i = 0; i <= mid; i++)
    {
        uint16_t min_idx = i;
        for (uint16_t j = i + 1; j < data_count; j++)
        {
            if (sort_buffer[j] < sort_buffer[min_idx])
            {
                min_idx = j;
            }
        }
        if (min_idx != i)
        {
            const float temp = sort_buffer[i];
            sort_buffer[i] = sort_buffer[min_idx];
            sort_buffer[min_idx] = temp;
        }
    }

    // 5. 取中值(偶数窗口取中间两值平均)
    if (data_count % 2 == 0)
    {
        now_output = (sort_buffer[data_count / 2 - 1] + sort_buffer[data_count / 2]) * 0.5f;
    }
    else
    {
        now_output = sort_buffer[(data_count - 1) / 2];
    }
}

/**
 * @brief 滑动均值滤波器初始化
 * @param __sliding_mean_filter_size 
 */
void sliding_mean_filter_class::init(const uint16_t &__sliding_mean_filter_size)
{
    sliding_mean_filter_size = __sliding_mean_filter_size;
    if (sliding_mean_filter_size == 0)
    {
        sliding_mean_filter_size = 1;
    }
    else if (sliding_mean_filter_size > Sliding_Mean_Filter_Max_Size)
    {
        sliding_mean_filter_size = Sliding_Mean_Filter_Max_Size;
    }

    data_count = 0;
    data_index = 0;
    sum = 0.0f;
    now_output = 0.0f;
}

/**
 * @brief 滑动均值滤波器处理函数
 * @note 运行式累加和(O(1)): 满窗后先减去即将被覆盖的最旧值, 再加上新值,
 *       避免每周期 O(N) 重算整个窗口之和
 */
void sliding_mean_filter_class::sliding_mean_calculate_callback(void)
{
    // 1. 满窗后先从累加和中减去即将被覆盖的最旧值
    if (data_count >= sliding_mean_filter_size)
    {
        sum -= data_buffer[data_index];
    }

    // 2. 写入新值并累加, 推进环形下标
    data_buffer[data_index] = now_input;
    sum += now_input;
    data_index = (data_index + 1) % sliding_mean_filter_size;

    // 3. 有效数据个数(未满窗前按已有数据取部分均值)
    if (data_count < sliding_mean_filter_size)
    {
        data_count++;
    }

    // 4. 输出均值
    now_output = sum / static_cast<float>(data_count);
}

/**
 * @brief 自适应低通滤波器初始化
 * @param __frict_alpha_min 
 * @param __frict_alpha_max 
 * @param __frict_err_high 
 * @param __frict_err_low 
 */
void adaptive_filter_class::init(const float &__frict_alpha_min, const float &__frict_alpha_max, const float &__frict_err_low, const float &__frict_err_high)
{
    frict_alpha_min = __frict_alpha_min;
    frict_alpha_max = __frict_alpha_max;
    frict_err_high = __frict_err_high;
    frict_err_low = __frict_err_low;

    // 检查参数有效性, 避免运行时出现异常
    if(__frict_alpha_min >= __frict_alpha_max)
    {
        // 强行拉开一个小量
        frict_alpha_max = __frict_alpha_min + 1e-6f;  
    }
    if(__frict_err_low >= __frict_err_high)
    {
        // 强行拉开一个小量
        frict_err_high = __frict_err_low + 1e-6f;  
    }

    // 重置滤波状态, 避免运行时重新初始化时输出从旧状态跳变
    now_input = 0.0f;
    last_output = 0.0f;
    alpha = frict_alpha_min; // 初始滤波系数设置为最小值
}

/**
 * @brief 自适应滤波器处理函数
 */
void adaptive_filter_class::adaptive_calculate_callback(void)
{
    // 计算当前误差值
    abs_err = fabsf(now_input - last_output);

    // 根据误差值调整滤波系数
    if(abs_err > frict_err_low)
    {
        if(abs_err >= frict_err_high)
        {
            alpha = frict_alpha_max;
        }
        else
        {
            // 线性插值计算滤波系数
            alpha = frict_alpha_min + (abs_err - frict_err_low) / (frict_err_high - frict_err_low) * (frict_alpha_max - frict_alpha_min);
        }
    }

    else
    {
        alpha = frict_alpha_min;
    }

    // 计算当前滤波输出值yn = alpha * xn + (1 - alpha) * yn-1
    now_output = alpha * now_input + (1.0f - alpha) * last_output;
    last_output = now_output;
}
