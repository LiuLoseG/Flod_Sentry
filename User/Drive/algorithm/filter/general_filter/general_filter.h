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
 *         递推写成增量式 y[n] = y[n-1] + b0*((x[n] + x[n-1]) - 2*y[n-1]), 每拍 2 乘 2 加
 *
 *       双线性把模拟原型的 s=∞ 零点映射到 z=-1(Nyquist), 带来三点好处:
 *         1. |H(fc)| 严格 -3dB, 全带幅度单调, 与 fc/fs 无关(仿真实测 0.707107, 误差 <1e-6);
 *         2. 通带相位等于弯折后的模拟相位 -atan(tan(pi*f*T)/tan(pi*fc*T)), fc*T 较小时
 *            就是教科书式 -atan(f/fc), 相位预算可以直接按模拟公式算;
 *         3. Nyquist 处增益为 0(匹配Z法有 2*b0 的透传下限), 对编码器量化/开关噪声抑制更强:
 *            fc=50Hz/T=1ms 时 250Hz 处比匹配Z多 2.9dB, 375Hz 处多 8.2dB。
 *
 *       为什么不用 y += alpha*(x-y) 这种单极点半离散形式(匹配Z/冲激响应不变):
 *         它能凑出准确的 -3dB, 但通带相位比模拟原型小约 15%
 *         (fc=50Hz,T=1ms 时 20Hz 处 -18.5° vs 理想 -21.8°), 会让环路相位预算算乐观;
 *         且高频增益有下限 (1-p)/(1+p) ≈ -16dB, Nyquist 附近噪声会透传。
 *
 *       更早的实现 alpha = x/(1+x) (x = 2*pi*fc*T) 只是模拟近似, 连 -3dB 都不准:
 *         fc=65Hz, T=1ms  旧式实际 -3dB 仅 55.0Hz
 *         fc=65Hz, T=5ms  旧式实际 -3dB 仅 39.8Hz
 *       但它的相位恰好接近双线性(fc=50Hz,T=1ms 时 0.4fc 处 -21.3° vs 双线性 -21.7°),
 *       所以换双线性不会推翻现场已经调好的相位裕度, 只是把幅频修正到位。
 *
 *       使用约束:
 *         - fc 应 <= fs/4。fc/fs 超过 0.25 后极点 (1-K)/(1+K) 变负, 阶跃开始过冲
 *           (实测 fc/fs=0.3 时 +6.7%, 0.4 时 +12.5%), 不再是"一阶"行为;
 *         - 阶跃 t63 比模拟公式 1/(2*pi*fc) 晚: fc/fs=0.05 晚 15%, 0.1 晚 30%
 *           (零点在 Nyquist 导致起始段慢半拍; 极点时间常数 -T/ln(极点) 不受影响);
 *         - 直流增益是结构精确的(2*b0+极点=1, 且增量式在输入恒定时括号内为 0),
 *           极低 fc(如 fc/fs=1e-4)也不漂, 这一点优于直接型 biquad。
 */
class low_pass_filter_class
{
public:

    /**
     * @brief 初始化
     * @param __cutoff_frequency 截止频率(Hz), 有效范围 (0, 0.5/T), 超出会被钳位
     * @param __sampling_period  调用周期(s), 必须等于 low_calculate_callback() 的真实调用间隔
     */
    void init(const float &__cutoff_frequency, const float &__sampling_period = 0.001f);

    /**
     * @brief 运行时修改截止频率(不清滤波状态, 无输出跳变), 用于在线调参
     */
    void set_cutoff_frequency(const float &__cutoff_frequency);

    /**
     * @brief 运行时修正调用周期(不清滤波状态), 用于按实测任务周期自适应
     */
    void set_sampling_period(const float &__sampling_period);

    /**
     * @brief 用给定值播种内部状态
     * @note 上电/切模式时用当前测量值播种, 避免输出从 0 爬升造成的力矩冲击;
     *       init() 会清零状态, 运行中不要用 init() 重配参数, 用 set_cutoff_frequency()
     */
    void reset(const float &__value);

    inline float get_out() const;

    /**
     * @brief 获取等效一阶平滑系数(= b0+b1 = 1-极点), 量纲与旧式 alpha 一致
     * @note 仅供遥测/核对, 内部递推只用 b0; 白噪声增益 = sqrt(alpha/2)。
     *       fc/fs > 0.25 时极点变负, 该值会 > 1(这不是错误, 见类注释的使用约束)
     */
    inline float get_alpha() const;

    inline float get_cutoff_frequency() const;

    /**
     * @brief 按 CMSIS-DSP arm_biquad_cascade_df1 的系数顺序导出系数
     * @param __coeff 长度 5 的输出数组, 写入 {b0, b1, b2, a1, a2}
     * @note 其中 a1 = -极点、a2 = 0, 即 CMSIS 的 y[n] = b0*x[n] + b1*x[n-1] + a1*y[n-1] 形式,
     *       可直接喂给 arm_biquad_cascade_df1_init_f32(&inst, 1, __coeff, state)。
     *       用于上位机核对离散化结果, 或需要切到 CMSIS-DSP 时避免重算系数。
     */
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
 *
 *       ⚠ __cutoff_frequency 是分母的自然频率 wc(rad/s 折算成 Hz), **只有 Q = 0.7071 时它才等于 -3dB 点**。
 *         下表是【模拟原型】的 -3dB 落点(fc 标称 50Hz, T=1ms), 数字滤波器经双线性弯折后略有差异,
 *         括号内为对应的真实数字落点(主机侧逐点扫描 |H(e^jw)| 实测):
 *           Q=0.50 → f3dB = 0.644·fc (数字 0.647), |H(fc)| = -6.02dB, 过冲  0.0%(临界阻尼)
 *           Q=0.60 → f3dB = 0.827·fc (数字 0.829), |H(fc)| = -4.44dB, 过冲  0.9%
 *           Q=0.7071→ f3dB = 1.000·fc (数字 1.000), |H(fc)| = -3.01dB, 过冲  4.3%(巴特沃斯)
 *           Q=1.00 → f3dB = 1.272·fc (数字 1.266), |H(fc)| =  0.00dB, 过冲 16.3%, 幅频峰 +1.25dB
 *           Q=2.00 → f3dB = 1.485·fc (数字 1.470), |H(fc)| = +6.02dB, 过冲 44.4%, 幅频峰 +6.30dB
 *         降低 Q 来消过冲会让实际带宽显著收窄(Q=0.5 时带宽只有标称的 64%), 需要 -3dB 精确落点时
 *         用 get_minus3db_frequency() 核对(该接口已含双线性弯折, 直接给真实数字频率),
 *         或反过来按 Q 预补偿标称 fc。
 *         "无过冲"的分界是 Q <= 0.5(临界/过阻尼), 不是 Q < 0.7071。
 *         "幅频无峰"的分界才是 Q <= 0.7071。
 *
 *       与"两级相同一阶低通串联"的区别(两者都是二阶, -40dB/dec):
 *         - 串联两个相同一阶(各自 fc') 是过阻尼(Q=0.5), 要达到 -3dB = fc 需 fc' = 1.554*fc;
 *           结果是低频相位更小, 但远端衰减更弱
 *         - 巴特沃斯在 fc 处严格 -3dB, 幅频单调无峰, 远端多 20dB/dec 的子午线
 *       选择依据: 噪声集中在环路带宽附近(几倍 wc 以内) 用一阶更划算;
 *                 要压制的是远高于 fc 的噪声(编码器量化、开关噪声) 才值得付二阶的相位
 *       实测(T=1ms, -3dB 均为 50Hz): 20Hz 处一阶相位 -21.66°, 二阶 -33.70°(多付 12.04°);
 *                 而 100Hz 处一阶 |H|=0.438, 二阶 |H|=0.231(多 5.55dB 抑制),
 *                 200Hz 处 0.213 vs 0.048(多 13.0dB)。
 *
 *       相比串联两个 low_pass_filter_class, 单个 biquad 只占一份对象、少一次函数调用
 *
 *       数值注意: 直流增益 = (b0+b1+b2)/(1+a1+a2), 而这两个和都等于 4K²/D(约 4K²),
 *         K 很小时它们都是 O(1) 项相消出来的小数, float 会丢掉有效位。
 *         实测(从系数解析, 不跑仿真)的直流增益误差:
 *           fc/fs=1e-2 → -0.001% ; 1e-3 → -0.39% ; 2e-4 → -1.96% ; 1e-4 → -17.2%
 *           fc/fs=5e-5 → 分母 1+a1+a2 已算成 -5.96e-8(符号都反了), 增益无意义
 *         极点半径始终 < 1(不会发散), 但增益会错。
 *         需要极低截止频率(fc/fs < 1e-3)时改用一阶 low_pass_filter_class:
 *         它的直流增益是结构精确的, fc/fs=1e-5 时误差仍 <0.02%。
 */
class biquad_low_pass_filter_class
{
public:

    /**
     * @brief 初始化
     * @param __cutoff_frequency 分母自然频率(Hz), 有效范围 (0, 0.5/T), 超出会被钳位
     * @param __sampling_period  调用周期(s), 必须等于 low_calculate_callback() 的真实调用间隔
     * @param __q                品质因数: 0.7071 巴特沃斯(幅频无峰, 阶跃过冲 4.3%);
     *                           0.5 临界阻尼(无过冲, 但 -3dB 落到 0.644*fc);
     *                           >0.7071 出现幅频峰与大过冲, 控制回路里不要用
     * @note __cutoff_frequency 只有在 Q = 0.7071 时才是 -3dB 点, 详见类注释
     */
    void init(const float &__cutoff_frequency, const float &__sampling_period = 0.001f, const float &__q = 0.70710678f);

    void set_cutoff_frequency(const float &__cutoff_frequency);

    /**
     * @brief 运行时修改品质因数(保留滤波状态, 无输出跳变)
     */
    void set_q(const float &__q);

    void set_sampling_period(const float &__sampling_period);

    void reset(const float &__value);

    inline float get_out() const;

    inline float get_cutoff_frequency() const;

    inline float get_q() const;

    /**
     * @brief 实际 -3dB 频率(Hz)
     * @note Q = 0.7071 时等于 cutoff_frequency; Q < 0.7071 时更低, Q > 0.7071 时更高。
     *       先解模拟原型 |H|^2 = 1/2 得 r = f3dB/fc 满足 r^4 + (1/Q^2 - 2)r^2 - 1 = 0,
     *       再经双线性弯折: f3dB = atan(r*tan(pi*fc*T)) / (pi*T)。
     *       只取模拟比值乘 fc 会在 fc/fs 较大时偏大(fc/fs=0.05, Q=2 时 1.485 vs 真实 1.470),
     *       弯折后该接口给的就是真实数字滤波器 |H|=0.7071 的落点。init 时算一次, 不占运行时间。
     */
    inline float get_minus3db_frequency() const;

    /**
     * @brief 直流增益(从当前系数解析计算)
     * @note 设计值为 1。fc/fs 很小时会因系数相消偏离 1, 可用于 init 后自检:
     *       |get_dc_gain() - 1| > 0.01 说明这个 fc/fs 已超出直接型 biquad 的精度范围
     */
    inline float get_dc_gain() const;

    /**
     * @brief 按 CMSIS-DSP arm_biquad_cascade_df1 的系数顺序导出系数
     * @param __coeff 长度 5 的输出数组, 写入 {b0, b1, b2, a1, a2}
     * @note 导出时已把分母系数取反(a1 = -a1, a2 = -a2), 即 CMSIS 的
     *       y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2] 形式,
     *       可直接喂给 arm_biquad_cascade_df1_init_f32(&inst, 1, __coeff, state)。
     */
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
 *
 *       匹配Z法(alpha = e^(-2*pi*fc*T), y = alpha*(y[n-1] + x[n] - x[n-1])) 的 Nyquist 增益只有
 *       2*alpha/(1+alpha) < 1(fc=50Hz,T=1ms 时 -1.5dB), 且 -3dB 点与相位都偏离模拟原型,
 *       因此与低通统一改用双线性。
 */
class high_pass_filter_class
{
public:

    /**
     * @brief 初始化
     * @param __low_limit_frequency 下限截止频率(Hz), 有效范围 (0, 0.5/T), 超出会被钳位
     * @param __sampling_period     调用周期(s), 必须等于 high_calculate_callback() 的真实调用间隔
     */
    void init(const float &__low_limit_frequency, const float &__sampling_period = 0.001f);

    /**
     * @brief 运行时修改截止频率(不清滤波状态, 无输出跳变)
     */
    void set_cutoff_frequency(const float &__low_limit_frequency);

    /**
     * @brief 运行时修正调用周期(不清滤波状态), 用于按实测任务周期自适应
     * @note 与 low_pass_filter_class::set_sampling_period() 一致; init() 会清零状态, 不要用它重配
     */
    void set_sampling_period(const float &__sampling_period);

    /**
     * @brief 用给定值播种内部状态(用于上电/切模式, 避免直流分量冲击)
     * @note 稳态时 y = 0, 但 last_input 必须与当前输入一致, 否则首拍会出现 b0*(x-x_prev) 的尖峰
     */
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

    /**
     * @brief 用给定值填充整个窗口(用于上电/切模式)
     * @note init() 清计数, 后续按新样本组成的部分窗口计算, 不读取旧窗口数据;
     *       reset(v) 则把整窗填成 v, 输出立即为 v
     */
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
 *
 *       纯增量累加有个隐患: float 下 a-b+b != a, 舍入误差会留在 sum 里做随机游走, 永不衰减。
 *       实测输入幅度 ±100、1kHz 连续跑 2e7 拍(≈5.5h) 后, 输出相对理想均值漂移约 7e-3,
 *       虽然慢但会一直长。因此每 Sliding_Mean_Resum_Interval 拍从环形缓冲重新求和一次,
 *       把漂移限制在常数级(实测 <1e-4), 平均代价 (窗口长度/64) 次加/拍。
 */
class sliding_mean_filter_class
{
public:

    void init(const uint16_t &__sliding_mean_filter_size);

    /**
     * @brief 用给定值填满整个窗口(用于上电/切模式)
     * @note init() 清计数和累加和, 后续按新样本组成的部分窗口计算;
     *       reset(v) 直接得到"窗口内全是 v"的状态, 输出立即为 v
     */
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

    /**
     * @brief 用给定值播种内部状态(上电/切模式时用当前测量值播种, 避免输出从 0 爬升)
     */
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

