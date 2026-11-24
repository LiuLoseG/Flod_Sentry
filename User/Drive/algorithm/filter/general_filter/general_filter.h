/**
 * @file general_filter.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 通用滤波器类头文件
 * @note 该文件定义了通用滤波器类: 一阶低通、二阶巴特沃斯低通、一阶高通、中值、
 *       滑动均值、自适应滤波器。各类注释里写明了离散化方式、适用边界与已知代价;
 *       完整设计依据与实测数据见: general_filter滤波器设计与验证.md
 * @date 2026-08-25
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#pragma once

#include "basic_math.h"

/**
 * @brief 一阶低通滤波器类
 * @note 离散化采用双线性变换 + 预畸变(bilinear with prewarp), 与二阶 biquad 同一套离散化:
 *         K = tan(pi*fc*T)
 *         H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1),  b0 = b1 = K/(1+K),  a1 = (K-1)/(1+K)
 *         递推写成增量式 y[n] = y[n-1] + b0*((x[n] + x[n-1]) - 2*y[n-1])
 */
class low_pass_filter_class
{
public:

    void init(const float &__cutoff_frequency, const float &__sampling_period = 0.001f);

    void set_cutoff_frequency(const float &__cutoff_frequency);

    void set_sampling_period(const float &__sampling_period);

    void reset(const float &__value);

    inline float get_out() const;

    inline float get_alpha() const;

    inline float get_cutoff_frequency() const;

    inline float get_sampling_period() const;

    void get_coefficients(float *__coeff) const;

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
    // 截止频率(Hz, 已含钳位后的实际生效值)
    float cutoff_frequency = 0.0f; 

    // 双线性一阶系数(增量式递推): y[n] = y[n-1] + b0*((x[n] + x[n-1]) - 2*y[n-1])
    // 前向系数 b0 = b1 = K/(1+K); 反馈系数 1-2*b0 = (1-K)/(1+K) 即极点, 由 b0 直接给出
    float b0 = 0.0f;

    // 当前输入值
    float now_input = 0.0f; 
    // 上一次输入值(双线性需要 x[n-1])
    float last_input = 0.0f;

    // 读写变量

    // 内部函数

    // 由 cutoff_frequency / sampling_period 重算 b0(含参数合法性钳位)
    void recalculate_coefficients(void);
};

/**
 * @brief 二阶巴特沃斯低通滤波器类
 * @note 双线性变换 + 预畸变, 传递函数 H(s) = wc^2 / (s^2 + wc/Q*s + wc^2), 默认 Q = 0.7071(巴特沃斯)
 */
class biquad_low_pass_filter_class
{
public:

    void init(const float &__cutoff_frequency, const float &__sampling_period = 0.001f, const float &__q = 0.70710678f);

    void set_cutoff_frequency(const float &__cutoff_frequency);

    void set_q(const float &__q);

    void set_sampling_period(const float &__sampling_period);

    void reset(const float &__value);

    inline float get_out() const;

    inline float get_cutoff_frequency() const;

    inline float get_q() const;

    inline float get_minus3db_frequency() const;

    inline float get_dc_gain() const;

    void get_coefficients(float *__coeff) const;

    inline void set_now_input(const float &__input);

    void low_calculate_callback(void);

protected:
    // 读变量

    // 输出值
    float now_output = 0.0f;

    // 写变量

    // 采样周期(s)
    float sampling_period = 0.001f;
    // 截止频率(Hz)
    float cutoff_frequency = 0.0f;
    // 品质因数
    float q = 0.70710678f;
    // 实际 -3dB 频率(Hz), init 时算一次; Q=0.7071 时等于 cutoff_frequency
    float minus3db_frequency = 0.0f;

    // 直接II型转置(TDF-II)系数: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
    float b0 = 0.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;

    // TDF-II 状态
    float state_1 = 0.0f;
    float state_2 = 0.0f;

    // 当前输入值
    float now_input = 0.0f;

    // 内部函数

    // 由 cutoff_frequency / sampling_period / q 重算系数(含参数合法性钳位)
    void recalculate_coefficients(void);

    // 按当前 now_output 重播 TDF-II 状态, 供在线改系数后保持输出连续
    void reseed_state(void);
};

/**
 * @brief 一阶高通滤波器类
 * @note 与低通同一套双线性变换 + 预畸变: K = tan(pi*fc*T)
 *         H(z) = b0*(1 - z^-1) / (1 + a1*z^-1),  b0 = 1/(1+K),  a1 = (K-1)/(1+K)
 *         即 y[n] = b0*(x[n] - x[n-1]) - a1*y[n-1], 每拍 2 乘 2 加
 *       零点被双线性映射到 z=1, 直流增益严格为 0(不动点), Nyquist 处增益严格为 1, |H(fc)| = -3dB。
 *       匹配Z法(alpha = e^(-2*pi*fc*T), y = alpha*(y[n-1] + x[n] - x[n-1])) 的 Nyquist 增益只有
 *       2*alpha/(1+alpha) < 1(fc=50Hz,T=1ms 时 -1.5dB), 且 -3dB 点与相位都偏离模拟原型,
 *       因此与低通统一改用双线性。
 */
class high_pass_filter_class
{
public:

    void init(const float &__low_limit_frequency, const float &__sampling_period = 0.001f);

    void set_cutoff_frequency(const float &__low_limit_frequency);

    void set_sampling_period(const float &__sampling_period);

    void reset(const float &__value);

    inline float get_out() const;

    inline float get_cutoff_frequency() const;

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
    // 下限频率(Hz, 已含钳位后的实际生效值)
    float low_limit_frequency = 0.0f; 

    // 双线性一阶系数: y[n] = b0*(x[n] - x[n-1]) + pole*y[n-1]
    // 前向系数 b0 = 1/(1+K)
    float b0 = 0.0f;
    // 反馈极点 pole = -a1 = (1-K)/(1+K)
    float pole = 0.0f;

    // 当前输入值
    float now_input = 0.0f; 
    // 上一次输入值
    float last_input = 0.0f;

    // 读写变量

    // 内部函数

    // 由 low_limit_frequency / sampling_period 重算 b0/pole(含参数合法性钳位)
    void recalculate_coefficients(void);
};

/**
 * @brief 中值滤波器类
 * @note 固定最大缓冲 + 运行时窗口长度, 避免C++非标准VLA(数组长度不能为成员变量);
 *       维护一个"始终升序"的副本: 每个采样只做一次删除 + 一次插入, 都是 O(N) 移位,
 *       取中值直接按下标取, O(1)。相比"每拍复制窗口 + 部分选择排序"的 O(N²) 实现,
 *       N=16 时实测单拍耗时从 ~243ns 降到 ~35ns(PC, g++ -O2), 且耗时与数据分布无关。
 *       重复值不需要区分身份: 删除时删掉任意一个等值元素即可, 后续窗口内容完全一致。
 */
class median_filter_class
{
public:

    void init(const uint16_t &__median_filter_size);

    void reset(const float &__value);

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

    // 环形数据缓冲区(保存原始输入顺序, 用于知道下一拍该淘汰哪个值)
    float data_buffer[Median_Filter_Max_Size];

    // 升序缓冲区(窗口内数据的升序副本, 长度 = data_count)
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
 * @note 运行式累加和(O(1)): 满窗后先减去即将被覆盖的最旧值再加上新值。
 *       纯增量累加有个隐患: float 下 a-b+b != a, 舍入误差会留在 sum 里做随机游走, 永不衰减。
 *       实测输入幅度 ±100、1kHz 连续跑 2e7 拍(≈5.5h) 后, 输出相对理想均值漂移约 7e-3,
 *       虽然慢但会一直长。因此每 Sliding_Mean_Resum_Interval 拍从环形缓冲重新求和一次,
 *       把漂移限制在常数级(实测 <1e-4), 平均代价 (窗口长度/64) 次加/拍。
 */
class sliding_mean_filter_class
{
public:

    void init(const uint16_t &__sliding_mean_filter_size);

    void reset(const float &__value);

    inline float get_out() const;

    inline void set_now_input(const float &__input);

    void sliding_mean_calculate_callback(void);

protected:
    // 初始化相关常量

    // 常量

    // 滑动均值滤波器最大窗口长度(编译期常量, 用于固定数组缓冲)
    static constexpr uint16_t Sliding_Mean_Filter_Max_Size = 16;

    // 累加和重算间隔(拍), 用于消除 float 增量累加的长期漂移
    static constexpr uint16_t Sliding_Mean_Resum_Interval = 64;

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

    // 当前窗口内有效数据个数(未满窗前用于部分窗口均值)
    uint16_t data_count = 0;
    // 环形缓冲写入下标(指向最旧数据位置)
    uint16_t data_index = 0;
    // 距下一次累加和重算的剩余拍数
    uint16_t resum_countdown = Sliding_Mean_Resum_Interval;
    // 当前输入值
    float now_input = 0.0f;
    // 窗口累加和(运行式滑动均值, 每周期 O(1))
    float sum = 0.0f;

    // 读写变量

    // 内部函数

    // 从环形缓冲重新求窗口和, 用于定期清除增量累加的舍入漂移
    void resum_buffer(void);
};

/**
 * @brief 自适应低通滤波器类
 * @note 误差大时用 alpha_max(跟得快), 误差小时用 alpha_min(滤得狠), 中间线性过渡;
 *       递推仍是 y += alpha*(x-y) 的半离散形式, 因为 alpha 需要在线插值, 不是固定截止频率的线性系统。
 *       若需要可预测的相位预算, 请用 low_pass_filter_class / biquad_low_pass_filter_class。
 */
class adaptive_filter_class
{
public:
    void init(const float &__frict_alpha_min, const float &__frict_alpha_max, const float &__frict_err_low, const float &__frict_err_high);

    void reset(const float &__value);

    inline float get_out() const;

    inline float get_alpha() const;

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
 * @brief 获取低通滤波器等效一阶平滑系数(= b0+b1 = 1-极点, 供上位机核对离散化结果)
 */
inline float low_pass_filter_class::get_alpha() const
{
    return (2.0f * b0);
}

/**
 * @brief 获取低通滤波器实际生效的截止频率(可能已被钳位)
 */
inline float low_pass_filter_class::get_cutoff_frequency() const
{
    return (cutoff_frequency);
}

/**
 * @brief 获取低通滤波器当前生效的采样周期(在线改周期后随之更新, 调用方可据此判断是否需要重算系数)
 */
inline float low_pass_filter_class::get_sampling_period() const
{
    return (sampling_period);
}

/**
 * @brief 按 CMSIS-DSP arm_biquad_cascade_df1 顺序导出系数 {b0, b1, b2, a1, a2}
 */
inline void low_pass_filter_class::get_coefficients(float *__coeff) const
{
    __coeff[0] = b0;
    __coeff[1] = b0;
    __coeff[2] = 0.0f;
    __coeff[3] = 1.0f - 2.0f * b0;   // = 极点 = -a1
    __coeff[4] = 0.0f;
}

/**
 * @brief 获取二阶低通滤波器输出值
 */
inline float biquad_low_pass_filter_class::get_out() const
{
    return (now_output);
}

/**
 * @brief 获取二阶低通滤波器实际生效的截止频率(可能已被钳位)
 */
inline float biquad_low_pass_filter_class::get_cutoff_frequency() const
{
    return (cutoff_frequency);
}

inline float biquad_low_pass_filter_class::get_q() const
{
    return (q);
}

inline float biquad_low_pass_filter_class::get_minus3db_frequency() const
{
    return (minus3db_frequency);
}

/**
 * @brief 直流增益: H(z=1) = (b0+b1+b2)/(1+a1+a2)
 */
inline float biquad_low_pass_filter_class::get_dc_gain() const
{
    const float numerator = b0 + b1 + b2;
    const float denominator = 1.0f + a1 + a2;

    // 分母接近 0 或计算结果非有限时增益无意义, 返回 0 让调用方察觉
    if (!isfinite(denominator) || fabsf(denominator) <= 1.0e-12f)
    {
        return (0.0f);
    }
    const float gain = numerator / denominator;
    return isfinite(gain) ? gain : 0.0f;
}

/**
 * @brief 设置二阶低通滤波器当前输入值
 */
inline void biquad_low_pass_filter_class::set_now_input(const float &__input)
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
 * @brief 获取高通滤波器实际生效的截止频率(可能已被钳位)
 */
inline float high_pass_filter_class::get_cutoff_frequency() const
{
    return (low_limit_frequency);
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
 * @brief 获取自适应滤波器当前生效的滤波系数(供遥测)
 */
inline float adaptive_filter_class::get_alpha() const
{
    return (alpha);
}

/**
 * @brief 设置自适应滤波器当前输入值
 * @param __input 
 */
inline void adaptive_filter_class::set_now_input(const float &__input)
{
    now_input = __input;
}

