#pragma once

#include "main.h"
#include "arm_math.h"

/* =================================================================== */
/*  波形类型枚举                                                        */
/* =================================================================== */

/**
 * @brief 波形发生器支持的波形类型
 */
typedef enum
{
    WAVE_SINE = 0,              /**< 正弦波 — 频率响应测试                 */
    WAVE_SQUARE,                /**< 方波   — 阶跃响应 / 带宽测试          */
    WAVE_TRIANGLE,              /**< 三角波 — 速度跟随测试                 */
    WAVE_SAWTOOTH,              /**< 正向锯齿波                            */
    WAVE_REVERSE_SAWTOOTH,      /**< 反向锯齿波                            */
    WAVE_STEP,                  /**< 阶跃波 — 瞬态响应测试                 */
    WAVE_RAMP,                  /**< 斜坡波 — 线性跟随测试                 */
    WAVE_CUSTOM,                /**< 自定义波形 (查找表)                   */
    WAVE_TYPE_NUM,              /**< 波形种类总数 (内部数组尺寸)           */
} wave_type_e;

/* =================================================================== */
/*  波形发生器对象                                                      */
/* =================================================================== */

/**
 * @brief 波形发生器对象
 *
 * @note  所有字段由 API 函数统一管理, 不建议外部直接读写
 */
typedef struct
{
    /* ======================== 用户配置 ======================== */

    wave_type_e type;               /**< 波形类型                          */
    float frequency;                /**< 频率 (Hz)                         */
    float amplitude;                /**< 幅值                              */
    float offset;                   /**< 偏置                              */
    float phase;                    /**< 初始相位 (弧度)                   */
    float duty_cycle;               /**< 占空比 [0,1] (方波 / 锯齿波有效) */

    /* ======================== 运行时状态 ======================= */

    struct
    {
        float elapsed_time;         /**< 使能后经过的时间 (s)              */
        float phase_accum;          /**< 相位累积 (cycles), 供 calc() 积分
                                         频率平滑时 phase_accum += freq×dt
                                         避免 elapsed_time×freq 的振荡    */
        float current_value;        /**< 当前输出值                        */
        float last_value;           /**< 上一次输出值                      */
        float tick;                 /**< 内部 DWT 时间戳缓存               */
    } state;

    /* ======================== 控制标志 ======================== */

    struct
    {
        bool enable;                /**< 使能标志                          */
        bool single_shot;           /**< 单次触发: 一个周期后自动停止      */
        bool completed;             /**< 已完成 (单次 / 阶跃 / 斜坡完成)   */
        uint32_t cycle_count;       /**< 已完成的周期数                    */
    } flag;

    /* ======================== 自定义波形 ======================= */

    struct
    {
        float *table;               /**< 查找表指针 (外部维护生命周期)     */
        uint16_t size;              /**< 查找表元素个数                    */
    } custom;

    /* ======================== 暂态参数 ======================== */

    struct
    {
        float start_value;          /**< 阶跃 / 斜坡起始值                 */
        float target_value;         /**< 阶跃 / 斜坡目标值                 */
        float ramp_duration;        /**< 斜坡持续时间 (s)                  */
    } transient;

    /* ======================== 参数平滑 (防 Ozone 跳变) ========== */

    struct
    {
        bool enable;                /**< 平滑使能 (默认 true)             */
        float smooth_time;          /**< 过渡时间 (s), 默认 0.3s          */
        float elapsed;              /**< 当前过渡已用时间                 */

        /** @brief 当前正在被平滑的参数 */
        struct
        {
            bool frequency;
            bool amplitude;
            bool offset;
            bool duty_cycle;
        } active;

        /* 影子参数 — calc() 实际使用的值 (平滑过渡中) */
        float cur_frequency;
        float cur_amplitude;
        float cur_offset;
        float cur_duty_cycle;

        /* 目标参数 — 经过平滑后最终要达到的值 */
        float tgt_frequency;
        float tgt_amplitude;
        float tgt_offset;
        float tgt_duty_cycle;

        /* 起始参数 — 本次过渡开始时的值 */
        float src_frequency;
        float src_amplitude;
        float src_offset;
        float src_duty_cycle;

    } smooth;

} wave_generator_t;

/* =================================================================== */
/*  API 函数声明                                                        */
/* =================================================================== */

/* --------------------------- 初始化与基础配置 ------------------------- */

/**
 * @brief 初始化波形发生器
 *
 * @param wg          对象指针
 * @param type        波形类型, 取值 wave_type_e
 * @param frequency   频率 (Hz), 必须 > 0
 * @param amplitude   幅值
 * @param offset      偏置 (输出中心值)
 */
void wave_generator_init(wave_generator_t *wg, wave_type_e type,
                         float frequency, float amplitude, float offset);

/**
 * @brief 切换波形类型
 *
 * @param wg   对象指针
 * @param type 新的波形类型
 */
void wave_generator_set_type(wave_generator_t *wg, wave_type_e type);

/**
 * @brief 设置频率 (Hz)
 * @note  参数通过平滑系统渐变过渡，避免输出跳变
 */
void wave_generator_set_frequency(wave_generator_t *wg, float frequency);

/**
 * @brief 设置幅值
 * @note  参数通过平滑系统渐变过渡
 */
void wave_generator_set_amplitude(wave_generator_t *wg, float amplitude);

/**
 * @brief 设置偏置 (输出中心值)
 * @note  参数通过平滑系统渐变过渡
 */
void wave_generator_set_offset(wave_generator_t *wg, float offset);

/**
 * @brief 设置方波 / 锯齿波占空比
 *
 * @param wg          对象指针
 * @param duty_cycle  占空比, 自动约束到 [0, 1]
 */
void wave_generator_set_duty_cycle(wave_generator_t *wg, float duty_cycle);

/* --------------------------- 阶跃 / 斜坡 ----------------------------- */

/**
 * @brief 设置阶跃信号 — 从当前输出值瞬间跳变到目标值
 *
 * @param wg            对象指针
 * @param target_value  目标值
 *
 * @note  调用后自动使能, 下一个 calc() 输出目标值后自动完成并停用
 */
void wave_generator_set_step(wave_generator_t *wg, float target_value);

/**
 * @brief 设置斜坡信号 — 从当前输出值线性过渡到目标值
 *
 * @param wg            对象指针
 * @param target_value  目标值
 * @param duration      过渡时间 (s), 必须 > 0
 *
 * @note  调用后自动使能, 到达目标值后自动完成并停用
 */
void wave_generator_set_ramp(wave_generator_t *wg,
                             float target_value, float duration);

/* --------------------------- 启停与模式 ------------------------------ */

/**
 * @brief 使能波形发生器, 重置时间基准开始输出
 *
 * @param wg  对象指针
 */
void wave_generator_enable(wave_generator_t *wg);

/**
 * @brief 禁能波形发生器, 输出保持当前值
 *
 * @param wg  对象指针
 */
void wave_generator_disable(wave_generator_t *wg);

/**
 * @brief 重置波形发生器内部状态
 *
 * @param wg  对象指针
 *
 * @note  将 elapsed_time 清零, 输出值还原到 offset
 */
void wave_generator_reset(wave_generator_t *wg);

/**
 * @brief 设置单次触发模式
 *
 * @param wg           对象指针
 * @param single_shot  true = 输出一个完整周期后自动停止; false = 连续输出
 */
void wave_generator_set_single_shot(wave_generator_t *wg, bool single_shot);

/**
 * @brief 设置自定义波形查找表 (WAVE_CUSTOM 模式有效)
 *
 * @param wg    对象指针
 * @param table 查找表 float 数组, 调用方保证生命周期
 * @param size  表大小 (元素个数)
 *
 * @note  表内值 ∈ [-1, +1], 最终映射到 [offset ± amplitude]
 */
void wave_generator_set_custom_table(wave_generator_t *wg,
                                     float *table, uint16_t size);

/* --------------------------- 核心计算 -------------------------------- */

/**
 * @brief 波形计算 — 需在控制循环中周期性调用
 *
 * @param wg  对象指针
 * @return float  当前波形输出值
 *
 * @note  内部使用 DWT 时间戳驱动, 不依赖 HAL 定时器
 *        若 type == WAVE_STEP / WAVE_RAMP, 完成后自动禁能
 */
float wave_generator_calc(wave_generator_t *wg);

/**
 * @brief 波形计算(外部 dt 版) — 允许调用方传入统一的时间步长
 *
 * @param wg  对象指针
 * @param dt  本帧时间步长 (s)
 * @return float  当前波形输出值
 *
 * @note  与 wave_generator_calc() 的区别在于 dt 由调用方提供，
 *        避免在同一个控制循环中多次调用 bsp_dwt_get_timeline_s()。
 *        适用于扫频模块等需要与外部同步 dt 的场景。
 */
float wave_generator_calc_ex(wave_generator_t *wg, float dt);

/* --------------------------- 内联查询 -------------------------------- */

/* --------------------------- 参数平滑配置 --------------------------- */

/**
 * @brief 设置参数平滑过渡时间
 *
 * @param wg              对象指针
 * @param smooth_time     过渡时间 (s), 建议 0.1~1.0s; 传 0 则关闭平滑
 */
void wave_generator_set_smooth_time(wave_generator_t *wg, float smooth_time);

/**
 * @brief 查询参数平滑是否正在进行中
 *
 * @param wg  对象指针
 * @return true  当前有参数正在平滑过渡
 * @return false 所有参数已就绪
 */
bool wave_generator_is_smoothing(const wave_generator_t *wg);

/**
 * @brief 获取当前波形值 (不触发计算)
 */
static inline float wave_generator_get_value(const wave_generator_t *wg)
{
    return wg->state.current_value;
}

/**
 * @brief 查询波形是否已完成 (单次 / 阶跃 / 斜坡)
 */
static inline bool wave_generator_is_completed(const wave_generator_t *wg)
{
    return wg->flag.completed;
}

/**
 * @brief 获取已完成的周期数
 */
static inline uint32_t wave_generator_get_cycle_count(const wave_generator_t *wg)
{
    return wg->flag.cycle_count;
}

