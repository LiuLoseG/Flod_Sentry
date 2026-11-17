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
    sampling_period = (isfinite(__sampling_period) && __sampling_period > 0.0f) ? __sampling_period : 0.001f;
    cutoff_frequency = __cutoff_frequency;
    recalculate_coefficients();

    // 重置滤波状态, 避免运行时重新初始化时输出从旧状态跳变
    now_input = 0.0f;
    now_output = 0.0f;
    last_output = 0.0f;
    last_input = 0.0f;
}

/**
 * @brief 重算双线性一阶系数
 * @note 双线性变换 + 预畸变: K = tan(pi*fc*T)
 *         b0 = b1 = K/(1+K),  极点 = (1-K)/(1+K) = 1-2*b0
 *       预畸变保证 |H(fc)| 严格 -3dB 且与 fc/fs 无关; 直流增益 = 2*b0/(1-极点) = 1;
 *       零点在 z=-1(Nyquist), 故 Nyquist 处增益为 0。
 */
void low_pass_filter_class::recalculate_coefficients(void)
{
    // 非正/非法截止频率: 冻结输出(b0 = 0), 避免出现负数或 NaN 系数
    if (!(cutoff_frequency > 0.0f) || !(sampling_period > 0.0f))
    {
        b0 = 0.0f;
        return;
    }

    // 钳到 Nyquist 以下, 保证 tan() 有限、极点落在单位圆内
    const float nyquist = 0.5f / sampling_period;
    if (cutoff_frequency > 0.98f * nyquist)
    {
        cutoff_frequency = 0.98f * nyquist;
    }

    // 极小截止频率(时间常数远大于采样周期)时 1-极点 = 2K/(1+K) 会被 float 精度吃掉,
    // 极点吸附到 1 就退化成纯积分器(直流增益无穷); 给 K 设 1e-6 下限避免退化
    const float min_cutoff_frequency = 1.0e-6f / (PI * sampling_period);
    if (cutoff_frequency < min_cutoff_frequency)
    {
        cutoff_frequency = min_cutoff_frequency;
    }

    b0 = tanf(PI * cutoff_frequency * sampling_period);
    b0 = b0 / (1.0f + b0);
}

/**
 * @brief 运行时修改截止频率(保留滤波状态)
 */
void low_pass_filter_class::set_cutoff_frequency(const float &__cutoff_frequency)
{
    cutoff_frequency = __cutoff_frequency;
    recalculate_coefficients();
}

/**
 * @brief 运行时修改调用周期(保留滤波状态)
 */
void low_pass_filter_class::set_sampling_period(const float &__sampling_period)
{
    if (isfinite(__sampling_period) && __sampling_period > 0.0f)
    {
        sampling_period = __sampling_period;
        recalculate_coefficients();
    }
}

/**
 * @brief 用给定值播种滤波状态
 * @note 增量式 y[n] = y[n-1] + b0*((x[n]+x[n-1]) - 2*y[n-1]) 的直流稳态是 x=y=v,
 *       两路状态都要播种, 否则首拍会因 x[n-1]=0 出现 b0*v 的半个阶跃
 */
void low_pass_filter_class::reset(const float &__value)
{
    now_input = __value;
    last_input = __value;
    now_output = __value;
    last_output = __value;
}

/**
 * @brief 低通滤波器处理函数
 */
void low_pass_filter_class::low_calculate_callback(void)
{
    // 非有限输入(NaN/Inf)时保持上一拍输出, 防止单帧故障把滤波器状态污染成 NaN 并扩散到整条控制链
    if (!isfinite(now_input))
    {
        now_output = last_output;
        return;
    }

    // 双线性一阶, 写成增量形式: y[n] = y[n-1] + b0*((x[n] + x[n-1]) - 2*y[n-1])
    // 与 y = b0*(x+x1) + pole*y1 代数等价, 但输入恒定时括号内严格为 0, 输出能精确保持不动点;
    // 直接式在 float 下会有 2*b0+pole != 1 的舍入漂移(实测 reset() 后 2000 拍漂 2e-6)
    now_output = last_output + b0 * ((now_input + last_input) - 2.0f * last_output);
    last_input = now_input;
    last_output = now_output;
}

/**
 * @brief 初始化二阶巴特沃斯低通滤波器
 */
void biquad_low_pass_filter_class::init(const float &__cutoff_frequency, const float &__sampling_period, const float &__q)
{
    sampling_period = (isfinite(__sampling_period) && __sampling_period > 0.0f) ? __sampling_period : 0.001f;
    cutoff_frequency = __cutoff_frequency;
    q = (__q > 0.0f) ? __q : 0.70710678f;
    recalculate_coefficients();

    now_input = 0.0f;
    now_output = 0.0f;
    state_1 = 0.0f;
    state_2 = 0.0f;
}

/**
 * @brief 重算二阶低通系数
 * @note 双线性变换 + 预畸变: K = tan(pi*fc*T), 保证 |H(fc)| = -3dB 且到 Nyquist 全程单调
 */
void biquad_low_pass_filter_class::recalculate_coefficients(void)
{
    if (!(cutoff_frequency > 0.0f) || !(sampling_period > 0.0f) || !(q > 0.0f))
    {
        b0 = 0.0f;
        b1 = 0.0f;
        b2 = 0.0f;
        a1 = 0.0f;
        a2 = 0.0f;
        minus3db_frequency = 0.0f;
        return;
    }

    const float nyquist = 0.5f / sampling_period;
    if (cutoff_frequency > 0.98f * nyquist)
    {
        cutoff_frequency = 0.98f * nyquist;
    }

    const float k = tanf(PI * cutoff_frequency * sampling_period);
    const float norm = 1.0f / (1.0f + k / q + k * k);

    b0 = k * k * norm;
    b1 = 2.0f * b0;
    b2 = b0;
    a1 = 2.0f * (k * k - 1.0f) * norm;
    a2 = (1.0f - k / q + k * k) * norm;

    // 实际 -3dB 频率: 先解模拟原型的 r = f3dB/fc: r^4 + (1/Q^2 - 2)r^2 - 1 = 0
    // Q=0.7071 时 1/Q^2-2 = 0 => r = 1, 即标称值就是 -3dB(巴特沃斯);
    // 再经双线性预畸变弯折回数字频率: f3dB = atan(r*K)/(pi*T), K = tan(pi*fc*T)。
    // 只算模拟比值再乘 fc 会在 fc/fs 较大时偏大(实测 fc/fs=0.05 时 Q=2 偏 1%);
    // 弯折后 Q=0.7071 严格等于 fc, 其余 Q 也与真实 |H|=0.7071 的落点一致。
    const float inv_q2 = 1.0f / (q * q);
    const float linear = inv_q2 - 2.0f;
    // 小 Q 时用有理化表达式避免 sqrt(linear^2+4)-linear 的相消。
    const float root = hypotf(linear, 2.0f);
    const float r = (linear > 0.0f)
                        ? sqrtf(2.0f / root / (1.0f + linear / root))
                        : sqrtf(0.5f * (root - linear));
    minus3db_frequency = atanf(r * k) / (PI * sampling_period);
}

/**
 * @brief 按当前输出重新播种 TDF-II 状态, 供在线改系数后保持输出连续
 * @note TDF-II 的 state_1/state_2 是内部 v 域量, 换系数后与新分子不匹配,
 *       会直接体现为一次跳变(实测换 Q 时约 0.14% 的信号量级)。
 *       按"当前输出即稳态"重播状态后, 只要输入此刻等于当前输出, 下一拍输出严格不变。
 *       代价是丢掉状态里隐含的斜率信息(中途改参时轨迹会有一点折角), 但输出本身连续,
 *       这比留一组过期状态更符合"在线改参不该有冲击"的预期。
 */
void biquad_low_pass_filter_class::reseed_state(void)
{
    state_1 = (1.0f - b0) * now_output;
    state_2 = (b2 - a2) * now_output;
}

/**
 * @brief 运行时修改截止频率(保留滤波状态)
 */
void biquad_low_pass_filter_class::set_cutoff_frequency(const float &__cutoff_frequency)
{
    cutoff_frequency = __cutoff_frequency;
    recalculate_coefficients();
    reseed_state();
}

/**
 * @brief 运行时修改品质因数(保留滤波状态)
 * @note Q 同时进入分子与反馈系数, 必须重播状态才能保持输出连续
 */
void biquad_low_pass_filter_class::set_q(const float &__q)
{
    q = (__q > 0.0f) ? __q : 0.70710678f;
    recalculate_coefficients();
    reseed_state();
}

/**
 * @brief 运行时修改调用周期(保留滤波状态)
 */
void biquad_low_pass_filter_class::set_sampling_period(const float &__sampling_period)
{
    if (isfinite(__sampling_period) && __sampling_period > 0.0f)
    {
        sampling_period = __sampling_period;
        recalculate_coefficients();
        reseed_state();
    }
}

/**
 * @brief 用给定值播种滤波状态
 * @note TDF-II 稳态: y = v, state_1 = (1 - b0)*v, state_2 = (b2 - a2)*v
 */
void biquad_low_pass_filter_class::reset(const float &__value)
{
    now_input = __value;
    now_output = __value;
    reseed_state();
}

/**
 * @brief 按 CMSIS-DSP arm_biquad_cascade_df1 顺序导出系数
 * @note 分母系数取反: 本类递推是 y = b*x - a1*y1 - a2*y2, CMSIS 是 y = b*x + A1*y1 + A2*y2
 */
void biquad_low_pass_filter_class::get_coefficients(float *__coeff) const
{
    __coeff[0] = b0;
    __coeff[1] = b1;
    __coeff[2] = b2;
    __coeff[3] = -a1;
    __coeff[4] = -a2;
}

/**
 * @brief 二阶低通滤波器处理函数
 */
void biquad_low_pass_filter_class::low_calculate_callback(void){
    // 非有限输入(NaN/Inf)时整拍跳过, 保持输出与内部状态不变
    if (!isfinite(now_input))
    {
        return;
    }

    // 直接II型转置: 每拍 5 乘 4 加, 状态数 2, 数值条件优于直接I型
    const float x = now_input;
    const float y = b0 * x + state_1;

    state_1 = b1 * x - a1 * y + state_2;
    state_2 = b2 * x - a2 * y;

    now_output = y;
}

/**
 * @brief 初始化高通滤波器
 * @param __low_limit_frequency 下限截止频率(Hz)
 * @param __sampling_period 采样周期(s)
 */
void high_pass_filter_class::init(const float &__low_limit_frequency, const float &__sampling_period)
{
    sampling_period = (isfinite(__sampling_period) && __sampling_period > 0.0f) ? __sampling_period : 0.001f;
    low_limit_frequency = __low_limit_frequency;
    recalculate_coefficients();

    // 重置滤波状态, 避免运行时重新初始化时输出从旧状态跳变
    now_input = 0.0f;
    last_input = 0.0f;
    now_output = 0.0f;
    last_output = 0.0f;
}

/**
 * @brief 重算双线性一阶高通系数
 * @note 与低通同一套双线性 + 预畸变: K = tan(pi*fc*T)
 *         b0 = 1/(1+K),  pole = (1-K)/(1+K)
 *       零点被映射到 z=1(直流增益严格为 0), Nyquist 增益 = 2*b0/(1+pole) = 1, |H(fc)| = -3dB。
 *       旧式(匹配Z) alpha = e^(-2*pi*fc*T) 的 Nyquist 增益只有 2*alpha/(1+alpha), 高频有下垂。
 */
void high_pass_filter_class::recalculate_coefficients(void)
{
    // 非正/非法截止频率: 退化为差分器 y = x[n] - x[n-1](直流增益仍为 0), 避免 NaN 系数
    if (!(low_limit_frequency > 0.0f) || !(sampling_period > 0.0f))
    {
        b0 = 1.0f;
        pole = 0.0f;
        return;
    }

    // 钳到 Nyquist 以下, 保证 tan() 有限、极点落在单位圆内
    const float nyquist = 0.5f / sampling_period;
    if (low_limit_frequency > 0.98f * nyquist)
    {
        low_limit_frequency = 0.98f * nyquist;
    }

    // 同低通: 极小截止频率下 pole 会吸附到 1, 极点退化为临界稳定, 给 K 设 1e-6 下限
    const float min_cutoff_frequency = 1.0e-6f / (PI * sampling_period);
    if (low_limit_frequency < min_cutoff_frequency)
    {
        low_limit_frequency = min_cutoff_frequency;
    }

    const float k = tanf(PI * low_limit_frequency * sampling_period);
    const float norm = 1.0f / (1.0f + k);

    b0 = norm;
    pole = (1.0f - k) * norm;
}

/**
 * @brief 运行时修改截止频率(保留滤波状态)
 */
void high_pass_filter_class::set_cutoff_frequency(const float &__low_limit_frequency)
{
    low_limit_frequency = __low_limit_frequency;
    recalculate_coefficients();
}

/**
 * @brief 运行时修改调用周期(保留滤波状态)
 */
void high_pass_filter_class::set_sampling_period(const float &__sampling_period)
{
    if (isfinite(__sampling_period) && __sampling_period > 0.0f)
    {
        sampling_period = __sampling_period;
        recalculate_coefficients();
    }
}

/**
 * @brief 用给定值播种滤波状态
 * @note 高通稳态输出为 0, 但 last_input 必须播种成当前输入, 否则首拍会出现 b0*(x-0) 的尖峰
 */
void high_pass_filter_class::reset(const float &__value)
{
    now_input = __value;
    last_input = __value;
    now_output = 0.0f;
    last_output = 0.0f;
}

/**
 * @brief 高通滤波器处理函数
 */
void high_pass_filter_class::high_calculate_callback(void)
{
    // 非有限输入(NaN/Inf)时保持上一拍输出
    if (!isfinite(now_input))
    {
        now_output = last_output;
        return;
    }

    // 双线性一阶高通: y[n] = b0*(x[n] - x[n-1]) + pole*y[n-1]
    // 输入恒定时 x-x1 严格为 0, 输出精确收敛到 0, 无直流泄漏
    now_output = b0 * (now_input - last_input) + pole * last_output;
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
 * @brief 用给定值填充整个窗口
 * @note 填成"窗口已满且全是 v"的状态: 输出立即为 v, 且后续 N 拍内不会被旧数据影响
 */
void median_filter_class::reset(const float &__value)
{
    for (uint16_t i = 0; i < Median_Filter_Max_Size; i++)
    {
        data_buffer[i] = __value;
        sort_buffer[i] = __value;
    }

    data_count = median_filter_size;
    data_index = 0;
    now_input = __value;
    now_output = __value;
}

/**
 * @brief 中值滤波器处理函数
 * @note 维护升序副本, 每拍只做"删一个最旧值 + 插一个最新值", 均为 O(N) 移位;
 *       取中值直接按下标, O(1)。参见类注释中的实测对比。
 */
void median_filter_class::median_calculate_callback(void)
{
    // 非有限输入(NaN/Inf)时保持上一拍输出: NaN 的比较全为 false, 会卡在升序缓冲里污染后续窗口
    if (!isfinite(now_input))
    {
        return;
    }

    // 1. 满窗: 从升序缓冲里移除即将被覆盖的最旧值(重复值删任意一个即可)
    if (data_count >= median_filter_size)
    {
        const float evicted = data_buffer[data_index];
        uint16_t pos = 0;
        while (pos < data_count && sort_buffer[pos] != evicted)
        {
            ++pos;
        }
        if (pos < data_count)
        {
            for (uint16_t k = pos; (uint16_t)(k + 1u) < data_count; ++k)
            {
                sort_buffer[k] = sort_buffer[k + 1u];
            }
            --data_count;
        }
        else
        {
            // 升序副本失配时从仍有效的环形窗口恢复, 排除即将淘汰的位置。
            data_count = 0;
            for (uint16_t i = 0; i < median_filter_size; ++i)
            {
                if (i == data_index)
                {
                    continue;
                }
                uint16_t insert_pos = data_count;
                while (insert_pos > 0u && sort_buffer[insert_pos - 1u] > data_buffer[i])
                {
                    sort_buffer[insert_pos] = sort_buffer[insert_pos - 1u];
                    --insert_pos;
                }
                sort_buffer[insert_pos] = data_buffer[i];
                ++data_count;
            }
        }
    }

    // 2. 写入环形缓冲, 推进下标
    data_buffer[data_index] = now_input;
    data_index = (uint16_t)((data_index + 1u) % median_filter_size);

    // 3. 插入升序缓冲(从尾部往前挪, 数据量大时也只是一次 O(N) 移位)
    uint16_t pos = data_count;
    while (pos > 0u && sort_buffer[pos - 1u] > now_input)
    {
        sort_buffer[pos] = sort_buffer[pos - 1u];
        --pos;
    }
    sort_buffer[pos] = now_input;
    ++data_count;

    // 4. 取中值(偶数窗口取中间两值平均)
    if ((data_count % 2u) == 0u)
    {
        now_output = (sort_buffer[data_count / 2u - 1u] + sort_buffer[data_count / 2u]) * 0.5f;
    }
    else
    {
        now_output = sort_buffer[(data_count - 1u) / 2u];
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
    resum_countdown = Sliding_Mean_Resum_Interval;
    sum = 0.0f;
    now_output = 0.0f;
}

/**
 * @brief 用给定值填满整个窗口
 * @note 填成"窗口已满且全是 v"的状态: 输出立即为 v, 累加和也一致(无需等待重算)
 */
void sliding_mean_filter_class::reset(const float &__value)
{
    for (uint16_t i = 0; i < Sliding_Mean_Filter_Max_Size; i++)
    {
        data_buffer[i] = __value;
    }

    data_count = sliding_mean_filter_size;
    data_index = 0;
    resum_countdown = Sliding_Mean_Resum_Interval;
    sum = __value * static_cast<float>(sliding_mean_filter_size);
    now_input = __value;
    now_output = __value;
}

/**
 * @brief 从环形缓冲重新求窗口和
 * @note 增量累加(sum += 0.3; sum -= 0.3)的舍入误差会留在累加器里做随机游走, 永不衰减,
 *       定期整窗重算可把该漂移压在常数级(见类注释中的实测数据)
 */
void sliding_mean_filter_class::resum_buffer(void)
{
    const uint16_t count = (data_count < sliding_mean_filter_size) ? data_count : sliding_mean_filter_size;

    float acc = 0.0f;
    for (uint16_t i = 0; i < count; i++)
    {
        acc += data_buffer[i];
    }
    sum = acc;
}

/**
 * @brief 滑动均值滤波器处理函数
 * @note 运行式累加和(O(1)): 满窗后先减去即将被覆盖的最旧值, 再加上新值,
 *       避免每周期 O(N) 重算整个窗口之和
 */
void sliding_mean_filter_class::sliding_mean_calculate_callback(void)
{
    // 非有限输入(NaN/Inf)时保持上一拍输出, 避免污染累加和(一次 NaN 会让 sum 永久为 NaN)
    if (!isfinite(now_input))
    {
        return;
    }

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

    // 4. 定期整窗重算累加和, 清除增量累加留下的舍入漂移
    if (--resum_countdown == 0)
    {
        resum_countdown = Sliding_Mean_Resum_Interval;
        resum_buffer();
    }

    // 5. 输出均值
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

    // 钳到 [0,1]: 超出范围会得到负阻尼或越界插值
    if (!(frict_alpha_min > 0.0f))
    {
        frict_alpha_min = 0.0f;
    }
    else if (frict_alpha_min > 1.0f)
    {
        frict_alpha_min = 1.0f;
    }
    if (!(frict_alpha_max > 0.0f))
    {
        frict_alpha_max = 0.0f;
    }
    else if (frict_alpha_max > 1.0f)
    {
        frict_alpha_max = 1.0f;
    }

    // 检查参数有效性, 避免运行时出现异常
    if(frict_alpha_min >= frict_alpha_max)
    {
        // 无法在 [0, 1] 内拉开时保持为常数系数, 避免生成 >1 的临时参数
        frict_alpha_max = frict_alpha_min;
    }
    if(frict_err_low >= frict_err_high)
    {
        // 退化为在低阈值处切换, 避免除以零或反向插值
        frict_err_high = frict_err_low;
    }

    // 重置滤波状态, 避免运行时重新初始化时输出从旧状态跳变
    now_input = 0.0f;
    now_output = 0.0f;
    last_output = 0.0f;
    abs_err = 0.0f;
    alpha = frict_alpha_min; // 初始滤波系数设置为最小值
}

/**
 * @brief 用给定值播种滤波状态
 */
void adaptive_filter_class::reset(const float &__value)
{
    now_input = __value;
    now_output = __value;
    last_output = __value;
    abs_err = 0.0f;
    alpha = frict_alpha_min;
}

/**
 * @brief 自适应滤波器处理函数
 */
void adaptive_filter_class::adaptive_calculate_callback(void)
{
    // 非有限输入(NaN/Inf)时保持上一拍输出, 否则 abs_err 变 NaN 后 alpha 也会永久为 NaN
    if (!isfinite(now_input))
    {
        now_output = last_output;
        return;
    }

    // 计算当前误差值
    abs_err = fabsf(now_input - last_output);

    // 根据误差值调整滤波系数
    if(abs_err > frict_err_low)
    {
        if(frict_err_high <= frict_err_low || abs_err >= frict_err_high)
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

    // 钳到 [0,1]: alpha>1 会让 y 冲过输入并反向振荡(等效负阻尼), alpha<0 会让 y 朝反方向跑。
    // 只在 init 里做参数防呆不够 —— 调用方可能直接传 1.5/负值, 一次就会把状态打出去。
    if (!(alpha > 0.0f))
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    // 计算当前滤波输出值yn = alpha * xn + (1 - alpha) * yn-1
    now_output = alpha * now_input + (1.0f - alpha) * last_output;
    last_output = now_output;
}