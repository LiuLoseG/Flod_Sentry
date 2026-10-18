/**
 * @file general_filter.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 通用滤波器类头文件
 * @note 该文件定义了通用滤波器类，包括低通滤波器、高通滤波器、中值滤波器和滑动均值滤波器。
 * @date 2026-08-25
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#pragma once

#include "basic_math.h"

/**
 * @brief 低通滤波器类
 */
class low_pass_filter_class
{
public:

    void init(const float &__cutoff_frequency, const float &__sampling_period = 0.001f);

    inline float get_out() const;

    inline void set_now_input(const float &__input);

    void low_calculate_callback(void);

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 输出值

    // 当前滤波输出值
    float now_output = 0.0f; 
    // 上一次滤波输出值
    float last_output = 0.0f; 

    // 写变量

    // 采样周期(s)
    float sampling_period = 0.001f; 
    // 截止频率(Hz)
    float cutoff_frequency = 0.0f; 
    // 滤波系数
    float alpha = 0.0f; 

    // 当前输入值
    float now_input = 0.0f; 

    // 读写变量

    // 内部函数
    
};

/**
 * @brief 高通滤波器类
 */
class high_pass_filter_class
{
public:

    void init(const float &__low_limit_frequency, const float &__sampling_period = 0.001f);

    inline float get_out() const;

    inline void set_now_input(const float &__input);

    void high_calculate_callback(void);

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // 输出值

    // 当前滤波输出值
    float now_output = 0.0f; 
    // 上一次滤波输出值
    float last_output = 0.0f; 

    // 写变量

    // 采样周期(s)
    float sampling_period = 0.001f; 
    // 下限频率(Hz)
    float low_limit_frequency = 0.0f; 
    // 滤波系数
    float alpha = 0.0f; 

    // 当前输入值
    float now_input = 0.0f; 
    // 上一次输入值
    float last_input = 0.0f;

    // 读写变量

    // 内部函数
    
};

/**
 * @brief 中值滤波器类
 * @note 固定最大缓冲 + 运行时窗口长度, 避免C++非标准VLA(数组长度不能为成员变量);
 *       处理流程: 写入环形缓冲 → 复制有效数据 → 部分选择排序 → 取中值,
 *       中值只需中间 1~2 个元素, 只排到中值位置即可(数据移动 O(N))
 */
class median_filter_class
{
public:

    void init(const uint16_t &__median_filter_size);

    inline float get_out() const;

    inline void set_now_input(const float &__input);

    void median_calculate_callback(void);

protected:
    // 初始化相关常量

    // 常量

    // 中值滤波器最大窗口长度(编译期常量, 用于固定数组缓冲)
    static constexpr uint16_t Median_Filter_Max_Size = 16;

    // 内部变量

    // 读变量

    // 输出值

    // 当前滤波输出值
    float now_output = 0.0f;

    // 写变量

    // 中值滤波器窗口长度(≤ Median_Filter_Max_Size)
    uint16_t median_filter_size = 1;

    // 环形数据缓冲区(保存原始输入顺序)
    float data_buffer[Median_Filter_Max_Size];

    // 有序缓冲区(保存窗口内升序排列)
    float sort_buffer[Median_Filter_Max_Size];

    // 当前窗口内有效数据个数(未满窗前用于部分窗口中值)
    uint16_t data_count = 0;

    // 环形缓冲写入下标(指向最旧数据位置)
    uint16_t data_index = 0;

    // 当前输入值
    float now_input = 0.0f;

    // 读写变量

    // 内部函数

};

/**
 * @brief 滑动均值滤波器类
 * @note 
 */
class sliding_mean_filter_class
{
public:

    void init(const uint16_t &__sliding_mean_filter_size);

    inline float get_out() const;

    inline void set_now_input(const float &__input);

    void sliding_mean_calculate_callback(void);

protected:
    // 初始化相关常量

    // 常量

    // 滑动均值滤波器最大窗口长度(编译期常量, 用于固定数组缓冲)
    static constexpr uint16_t Sliding_Mean_Filter_Max_Size = 16;

    // 内部变量
    

    // 读变量

    // 输出值

    // 当前滤波输出值
    float now_output = 0.0f;

    // 写变量

    // 滑动均值滤波器窗口长度(≤ Sliding_Mean_Filter_Max_Size)
    uint16_t sliding_mean_filter_size = 1;
    // 环形数据缓冲区(保存原始输入顺序)
    float data_buffer[Sliding_Mean_Filter_Max_Size];

    // 当前窗口内有效数据个数(未满窗前用于部分窗口中值)
    uint16_t data_count = 0;
    // 环形缓冲写入下标(指向最旧数据位置)
    uint16_t data_index = 0;
    // 当前输入值
    float now_input = 0.0f;
    // 窗口累加和(运行式滑动均值, 每周期 O(1))
    float sum = 0.0f;

    // 读写变量

    // 内部函数

};

/**
 * @brief 自适应低通滤波器类
 * @note 该类用于实现自适应滤波器，能够根据输入信号的特性动态调整滤波参数，以实现更好的滤波效果。
 */
class adaptive_filter_class
{
public:
    void init(const float &__frict_alpha_min, const float &__frict_alpha_max, const float &__frict_err_low, const float &__frict_err_high);

    inline float get_out() const;

    inline void set_now_input(const float &__input);

    void adaptive_calculate_callback(void);

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 自适应低通滤波最小系数
    float frict_alpha_min = 0.0f;  
    // 自适应低通滤波最大系数
    float frict_alpha_max = 0.0f;

    // 自适应低通滤波比较误差高阈值
    float frict_err_high = 0.0f;
    // 自适应低通滤波比较误差低阈值
    float frict_err_low = 0.0f;

    // 当前绝对值误差值
    float abs_err = 0.0f;  

    // 读变量

    // 当前输入值
    float now_input = 0.0f;  

    // 输出值

    // 当前滤波输出值
    float now_output = 0.0f;
    // 上一次滤波输出值
    float last_output = 0.0f;  

    // 写变量

    // 滤波系数
    float alpha = 0.0f;  

    // 读写变量

    // 内部函数

};

/**
 * @brief 获取低通滤波器输出值
 * @return 输出值
 */
inline float low_pass_filter_class::get_out() const
{
    return (now_output);
}

/**
 * @brief 设置低通滤波器当前输入值
 * @param __input 
 */
inline void low_pass_filter_class::set_now_input(const float &__input)
{
    now_input = __input;
}

/**
 * @brief 获取高通滤波器输出值
 * @return 输出值
 */
inline float high_pass_filter_class::get_out() const
{
    return (now_output);
}

/**
 * @brief 设置高通滤波器当前输入值
 * @param __input 
 */
inline void high_pass_filter_class::set_now_input(const float &__input)
{
    now_input = __input;
}

/**
 * @brief 获取中值滤波器输出值
 * @return 输出值
 */
inline float median_filter_class::get_out() const
{
    return (now_output);
}

/**
 * @brief 设置中值滤波器当前输入值
 * @param __input 
 */
inline void median_filter_class::set_now_input(const float &__input)
{
    now_input = __input;
}

/**
 * @brief 获取滑动均值滤波器输出值
 * @retval float: 
 */
inline float sliding_mean_filter_class::get_out() const
{
    return (now_output);
}

/**
 * @brief 设置滑动均值滤波器当前输入值
 * @param __input 
 */
inline void sliding_mean_filter_class::set_now_input(const float &__input)
{
    now_input = __input;
}

inline float adaptive_filter_class::get_out() const
{
    return (now_output);
}

/**
 * @brief 设置自适应滤波器当前输入值
 * @param __input 
 */
inline void adaptive_filter_class::set_now_input(const float &__input)
{
    now_input = __input;
}

