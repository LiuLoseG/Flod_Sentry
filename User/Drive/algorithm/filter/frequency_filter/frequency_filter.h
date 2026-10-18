/**
 * @file alg_filter_frequency.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 频率滤波器
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-25 1.1 可自定义滤波器阶数
 *
 * @copyright copyright (c) 2023
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "basic_math.h"

/* Exported macros -----------------------------------------------------------*/

extern const float FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 滤波器类型
 *
 */
enum enum_filter_frequency_type
{
    filter_frequency_type_lowpass = 0,
    filter_frequency_type_highpass,
    filter_frequency_type_bandpass,
    filter_frequency_type_bandstop,
};

/**
 * @brief reusable, frequency滤波器算法
 *
 */
template<uint32_t filter_frequency_order = 50>
class class_filter_frequency
{
public:
    void init(const float &__value_constrain_low = 0.0f, const float &__value_constrain_high = 1.0f, const enum_filter_frequency_type &__filter_frequency_type = filter_frequency_type_lowpass, const float &__frequency_low = 0.0f, const float &__frequency_high = FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f, const float &__sampling_frequency = FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY);

    inline float get_out() const;

    inline void set_now(const float &__now);

    void frequency_filter_calculate_callback();

protected:
    // 初始化相关常量

    // 输入限幅
    float value_constrain_low;
    float value_constrain_high;

    // 滤波器类型
    enum_filter_frequency_type filter_frequency_type;
    // 滤波器特征低频
    float frequency_low;
    // 滤波器特征高频
    float frequency_high;
    // 滤波器采样频率
    float sampling_frequency;

    // 常量

    // 内部变量

    // 卷积系统函数向量
    float system_function[filter_frequency_order + 1]{};

    // 输入信号向量
    float input_signal[filter_frequency_order + 1]{};

    // 新数据指示向量
    uint8_t signal_flag = 0;

    // 读变量

    // 输出值
    float out = 0;

    // 写变量

    // 读写变量

    // 内部函数
};

/* exported variables --------------------------------------------------------*/

/* exported function declarations --------------------------------------------*/

/**
 * @brief 初始化滤波器
 *
 * @tparam filter_frequency_order 滤波器阶数
 * @param __value_constrain_low 滤波器最小值, 全0不限制
 * @param __value_constrain_high 滤波器最大值, 全0不限制
 * @param __filter_frequency_type 滤波器类型
 * @param __frequency_low 滤波器特征低频, 非高通有效
 * @param __frequency_high 滤波器特征高频, 非低通有效
 * @param __sampling_frequency 滤波器采样频率
 */
template<uint32_t filter_frequency_order>
void class_filter_frequency<filter_frequency_order>::init(const float &__value_constrain_low, const float &__value_constrain_high, const enum_filter_frequency_type &__filter_frequency_type, const float &__frequency_low, const float &__frequency_high, const float &__sampling_frequency)
{
    value_constrain_low = __value_constrain_low;
    value_constrain_high = __value_constrain_high;
    filter_frequency_type = __filter_frequency_type;
    frequency_low = __frequency_low;
    frequency_high = __frequency_high;
    sampling_frequency = __sampling_frequency;

    // 将所有计算所得值进行softmax操作成和为1的值
    float system_function_sum = 0.0f;
    // 特征低角速度
    float omega_low;
    // 特征高角速度
    float omega_high;

    omega_low = 2.0f * PI * frequency_low / sampling_frequency;
    omega_high = 2.0f * PI * frequency_high / sampling_frequency;

    // 计算滤波器系统

    switch (filter_frequency_type)
    {
    case (filter_frequency_type_lowpass):
    {
        for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
        {
            system_function[i] = omega_low / PI * basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * omega_low);
        }

        break;
    }
    case (filter_frequency_type_highpass):
    {
        for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
        {
            system_function[i] = basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * PI) - omega_high / PI * basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * omega_high);
        }

        break;
    }
    case (filter_frequency_type_bandpass):
    {
        for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
        {
            system_function[i] = omega_high / PI * basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * omega_high) - omega_low / PI * basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * omega_low);
        }

        break;
    }
    case (filter_frequency_type_bandstop):
    {
        for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
        {
            system_function[i] = basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * PI) + omega_low / PI * basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * omega_low) - omega_high / PI * basic_math_sinc(((float) (i) - filter_frequency_order / 2.0f) * omega_high);
        }

        break;
    }
    }

    // 施加 Blackman 窗抑制吉布斯振铃（相比Hamming窗旁瓣衰减更优，从-43dB提升至-58dB）
    for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
    {
        float window = 0.42f - 0.5f * cosf(2.0f * PI * (float)i / (float)filter_frequency_order) + 0.08f * cosf(4.0f * PI * (float)i / (float)filter_frequency_order);
        system_function[i] *= window;
    }

    for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
    {
        system_function_sum += system_function[i];
    }

    for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
    {
        system_function[i] /= system_function_sum;
    }
}

/**
 * @brief 滤波器调整值, 周期与采样周期相同
 *
 * @tparam filter_frequency_order 滤波器阶数
 */
template<uint32_t filter_frequency_order>
void class_filter_frequency<filter_frequency_order>::frequency_filter_calculate_callback()
{
    out = 0.0f;

    // 执行卷积操作
    for (uint32_t i = 0; i < filter_frequency_order + 1; i++)
    {
        out += system_function[i] * input_signal[(signal_flag + i) % (filter_frequency_order + 1)];
    }
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
template<uint32_t filter_frequency_order>
inline float class_filter_frequency<filter_frequency_order>::get_out() const
{
    return (out);
}

/**
 * @brief 设定当前值
 *
 * @param __now 当前值
 */
template<uint32_t filter_frequency_order>
inline void class_filter_frequency<filter_frequency_order>::set_now(const float &__now)
{
    float now_value = __now;

    // 输入限幅, 全0为不限制
    if (value_constrain_low != 0.0f || value_constrain_high != 0.0f)
    {
        now_value = basic_math_constrain(__now, value_constrain_low, value_constrain_high);
    }

    // 将当前值放入被卷积的信号中
    input_signal[signal_flag] = now_value;
    signal_flag++;

    // 若越界则轮回
    if (signal_flag == filter_frequency_order + 1)
    {
        signal_flag = 0;
    }
}


/************************ copyright(c) ustc-robovalker **************************/