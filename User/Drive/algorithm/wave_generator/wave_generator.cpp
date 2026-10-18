/**
 * @file wave_generator.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief   波形发生器 — 为电机波形跟随测试提供多种周期 / 暂态波形
 * @date 2026-07-16
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 * @details 周期波形 (SINE, SQUARE, TRIANGLE 等) 通过函数指针分发表派发;
 *          暂态波形 (STEP, RAMP) 独立处理, 完成后自动禁能;
 *          时间推进基于 DWT 时间戳, 不依赖 HAL 定时器。
 */

#include "wave_generator.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include <string.h>
#include <math.h>

/* =================================================================== */
/*  内部常量                                                            */
/* =================================================================== */

/** @brief DWT 时间差最大有效值 (s), 超过此值视为异常跳变, 丢弃该帧 */
#define DWT_DT_MAX_S  60.0f

/* =================================================================== */
/*  相位归一化 to [0, 1)                                                  */
/* =================================================================== */

/**
 * @brief 将任意浮点相位映射到 [0, 1) 区间
 *
 * @param phase  任意浮点值 (通常为 elapsed_time * frequency + 偏置)
 * @return float  归一化相位 t ∈ [0, 1)
 *
 * @note  利用 int32 截断取小数, 在 Cortex-M7 上比 fmodf 更高效
 */
static inline float _norm_phase(float phase)
{
    int32_t q = (int32_t)phase;
    float p = phase - (float)q;
    if (p < 0.0f) p += 1.0f;
    return p;
}

/* =================================================================== */
/*  周期波形函数 — t ∈ [0,1) → v ∈ [-1, +1]                               */
/* =================================================================== */

/**
 * @brief 正弦波
 */
static float _wave_sine(float t, float duty, const float *lut, uint16_t n)
{
    (void)duty; (void)lut; (void)n;
    return arm_sin_f32(2.0f * PI * t);
}

/**
 * @brief 方波
 *
 * @param duty  占空比: 高电平 = [0, duty), 低电平 = [duty, 1)
 */
static float _wave_square(float t, float duty, const float *lut, uint16_t n)
{
    (void)lut; (void)n;
    /* duty ∈ [0,1], 夹紧后直接比较 */
    return (t < duty) ? 1.0f : -1.0f;
}

/**
 * @brief 对称三角波: t = [0, 0.5) 上升段, t = [0.5, 1) 下降段
 */
static float _wave_triangle(float t, float duty, const float *lut, uint16_t n)
{
    (void)duty; (void)lut; (void)n;
    float ramp = 4.0f * t;  /* 上升斜率: 4 = 2/0.5 */
    return (t < 0.5f) ? (ramp - 1.0f) : (3.0f - ramp);
}

/**
 * @brief 正向锯齿波: 线性上升, 周期末尾快速回落
 */
static float _wave_sawtooth(float t, float duty, const float *lut, uint16_t n)
{
    (void)duty; (void)lut; (void)n;
    return 2.0f * t - 1.0f;
}

/**
 * @brief 反向锯齿波: 线性下降, 周期末尾快速回升
 */
static float _wave_reverse_sawtooth(float t, float duty, const float *lut, uint16_t n)
{
    (void)duty; (void)lut; (void)n;
    return 1.0f - 2.0f * t;
}

/**
 * @brief 自定义波形 — 线性插值查找表
 *
 * @param lut   查找表, 元素值 ∈ [-1, +1]
 * @param n     表大小 (≥ 2 方可插值)
 */
static float _wave_custom(float t, float duty, const float *lut, uint16_t n)
{
    (void)duty;
    if (lut == NULL || n < 2) return 0.0f;

    float idx_f = t * (float)(n - 1);
    uint16_t i0 = (uint16_t)idx_f;
    uint16_t i1 = (i0 < n - 1) ? (i0 + 1) : i0;
    float frac = idx_f - (float)i0;

    return lut[i0] + frac * (lut[i1] - lut[i0]);
}

/* =================================================================== */
/*  函数指针分发表                                                        */
/* =================================================================== */

/** @brief 周期波形函数指针类型 */
typedef float (*_wave_func_t)(float t, float dc, const float *lut, uint16_t n);

/**
 * @brief 周期波形分发表, 按 wave_type_e 枚举值索引
 *
 * @note  WAVE_STEP 和 WAVE_RAMP 为暂态波形, 表中置 NULL 以区分
 */
static const _wave_func_t s_wave_table[WAVE_TYPE_NUM] = {
    _wave_sine,             /**< WAVE_SINE             */
    _wave_square,           /**< WAVE_SQUARE           */
    _wave_triangle,         /**< WAVE_TRIANGLE         */
    _wave_sawtooth,         /**< WAVE_SAWTOOTH         */
    _wave_reverse_sawtooth, /**< WAVE_REVERSE_SAWTOOTH */
    NULL,                   /**< WAVE_STEP             */
    NULL,                   /**< WAVE_RAMP             */
    _wave_custom,           /**< WAVE_CUSTOM           */
};

/* =================================================================== */
/*  内部: 时间 / 完成态辅助                                               */
/* =================================================================== */

/**
 * @brief 重置计时基准 — 清零经过时间, 锁定当前 DWT 时间戳开始累积
 */
static inline void _time_start(wave_generator_t *wg)
{
    wg->state.elapsed_time = 0.0f;
    wg->state.phase_accum  = 0.0f;
    wg->state.tick         = bsp_dwt_get_timeline_s();
}

/**
 * @brief 标记波形已完成并自动禁能
 */
static inline void _mark_completed(wave_generator_t *wg)
{
    wg->flag.completed = true;
    wg->flag.enable    = false;
}

/* =================================================================== */
/*  暂态波形: 阶跃 / 斜坡                                                 */
/* =================================================================== */

/**
 * @brief 阶跃波形计算 — 下一拍直接输出目标值并自停
 */
static void _step_calc(wave_generator_t *wg)
{
    wg->state.current_value = wg->transient.target_value;
    _mark_completed(wg);
}

/**
 * @brief 斜坡波形计算 — 按经过时间线性插值, 到达目标后自停
 *
 * @note  elapsed_time / ramp_duration ∈ [0, 1] 控制插值进度
 */
static void _ramp_calc(wave_generator_t *wg)
{
    float dur = wg->transient.ramp_duration;
    if (dur <= 0.0f)
    {
        _step_calc(wg);
        return;
    }

    float ratio = wg->state.elapsed_time / dur;
    if (ratio >= 1.0f)
    {
        wg->state.current_value = wg->transient.target_value;
        _mark_completed(wg);
    }
    else
    {
        wg->state.current_value = wg->transient.start_value
            + (wg->transient.target_value - wg->transient.start_value) * ratio;
    }
}

/* =================================================================== */
/*  参数平滑 (防 Ozone Watch 跳变)                                       */
/* =================================================================== */

/**
 * @brief 检测原始参数变化, 启动平滑过渡; 更新平滑参数值
 *
 * @param wg  波形发生器
 * @param dt  当前帧时间差 (s)
 *
 * @note  每次 calc() 中调用, 放在获取 dt 之后、使用参数之前。
 *        平滑后的参数在 wg->smooth.cur_* 中, calc() 使用它们代替
 *        wg->frequency / amplitude / offset / duty_cycle。
 */
static void _smooth_update(wave_generator_t *wg, float dt)
{
    /* 平滑未使能: 直接同步到原始值, 不做过渡 */
    if (!wg->smooth.enable)
    {
        wg->smooth.cur_frequency  = wg->frequency;
        wg->smooth.cur_amplitude  = wg->amplitude;
        wg->smooth.cur_offset     = wg->offset;
        wg->smooth.cur_duty_cycle = wg->duty_cycle;
        wg->smooth.active.frequency  = false;
        wg->smooth.active.amplitude  = false;
        wg->smooth.active.offset     = false;
        wg->smooth.active.duty_cycle = false;
        return;
    }

    /* ---- 检测原始参数 vs 目标参数, 有差异则启动新过渡 ---- */
    bool freq_dirty = (fabsf(wg->frequency  - wg->smooth.tgt_frequency)  > 1e-8f);
    bool amp_dirty  = (fabsf(wg->amplitude  - wg->smooth.tgt_amplitude)  > 1e-8f);
    bool off_dirty  = (fabsf(wg->offset     - wg->smooth.tgt_offset)     > 1e-8f);
    bool duty_dirty = (fabsf(wg->duty_cycle - wg->smooth.tgt_duty_cycle) > 1e-6f);

    if (freq_dirty)
    {
        wg->smooth.src_frequency  = wg->smooth.cur_frequency;
        wg->smooth.tgt_frequency  = wg->frequency;
        wg->smooth.active.frequency = true;
    }
    if (amp_dirty)
    {
        wg->smooth.src_amplitude  = wg->smooth.cur_amplitude;
        wg->smooth.tgt_amplitude  = wg->amplitude;
        wg->smooth.active.amplitude = true;
    }
    if (off_dirty)
    {
        wg->smooth.src_offset     = wg->smooth.cur_offset;
        wg->smooth.tgt_offset     = wg->offset;
        wg->smooth.active.offset = true;
    }
    if (duty_dirty)
    {
        wg->smooth.src_duty_cycle = wg->smooth.cur_duty_cycle;
        wg->smooth.tgt_duty_cycle = wg->duty_cycle;
        wg->smooth.active.duty_cycle = true;
    }

    /* 有任意参数需要平滑过渡, 重置计时器 */
    if (freq_dirty || amp_dirty || off_dirty || duty_dirty)
    {
        wg->smooth.elapsed = 0.0f;
    }

    /* ---- 推进过渡, 线性插值 ---- */
    float sm_time = wg->smooth.smooth_time;
    if (sm_time <= 0.0f) sm_time = 0.3f;  /* 默认保护 */

    wg->smooth.elapsed += dt;

    if (wg->smooth.active.frequency)
    {
        float ratio = wg->smooth.elapsed / sm_time;
        if (ratio >= 1.0f)
        {
            wg->smooth.cur_frequency  = wg->smooth.tgt_frequency;
            wg->smooth.active.frequency = false;
        }
        else
        {
            wg->smooth.cur_frequency = wg->smooth.src_frequency
                + (wg->smooth.tgt_frequency - wg->smooth.src_frequency) * ratio;
        }
    }

    if (wg->smooth.active.amplitude)
    {
        float ratio = wg->smooth.elapsed / sm_time;
        if (ratio >= 1.0f)
        {
            wg->smooth.cur_amplitude  = wg->smooth.tgt_amplitude;
            wg->smooth.active.amplitude = false;
        }
        else
        {
            wg->smooth.cur_amplitude = wg->smooth.src_amplitude
                + (wg->smooth.tgt_amplitude - wg->smooth.src_amplitude) * ratio;
        }
    }

    if (wg->smooth.active.offset)
    {
        float ratio = wg->smooth.elapsed / sm_time;
        if (ratio >= 1.0f)
        {
            wg->smooth.cur_offset  = wg->smooth.tgt_offset;
            wg->smooth.active.offset = false;
        }
        else
        {
            wg->smooth.cur_offset = wg->smooth.src_offset
                + (wg->smooth.tgt_offset - wg->smooth.src_offset) * ratio;
        }
    }

    if (wg->smooth.active.duty_cycle)
    {
        float ratio = wg->smooth.elapsed / sm_time;
        if (ratio >= 1.0f)
        {
            wg->smooth.cur_duty_cycle  = wg->smooth.tgt_duty_cycle;
            wg->smooth.active.duty_cycle = false;
        }
        else
        {
            wg->smooth.cur_duty_cycle = wg->smooth.src_duty_cycle
                + (wg->smooth.tgt_duty_cycle - wg->smooth.src_duty_cycle) * ratio;
        }
    }
}

/* =================================================================== */
/*  公有 API                                                             */
/* =================================================================== */

/* --------------------------- 初始化与基础配置 ------------------------- */

void wave_generator_init(wave_generator_t *wg, wave_type_e type,
                         float frequency, float amplitude, float offset)
{
    /* 清零所有字段, 避免遗漏新字段 */
    memset(wg, 0, sizeof(*wg));

    wg->type       = type;
    wg->frequency  = frequency;
    wg->amplitude  = amplitude;
    wg->offset     = offset;
    wg->duty_cycle = 0.5f;

    wg->state.current_value = offset;
    wg->state.last_value    = offset;

    wg->transient.start_value   = offset;
    wg->transient.target_value  = offset;
    wg->transient.ramp_duration = 1.0f;

    /* 参数平滑默认使能, 过渡时间 0.3s */
    wg->smooth.enable     = true;
    wg->smooth.smooth_time = 0.3f;
    wg->smooth.cur_frequency  = frequency;
    wg->smooth.cur_amplitude  = amplitude;
    wg->smooth.cur_offset     = offset;
    wg->smooth.cur_duty_cycle = 0.5f;
    wg->smooth.tgt_frequency  = frequency;
    wg->smooth.tgt_amplitude  = amplitude;
    wg->smooth.tgt_offset     = offset;
    wg->smooth.tgt_duty_cycle = 0.5f;
    wg->smooth.src_frequency  = frequency;
    wg->smooth.src_amplitude  = amplitude;
    wg->smooth.src_offset     = offset;
    wg->smooth.src_duty_cycle = 0.5f;

    _time_start(wg);
}

void wave_generator_set_type(wave_generator_t *wg, wave_type_e type)
{
    wg->type = type;
}

void wave_generator_set_frequency(wave_generator_t *wg, float frequency)
{
    wg->frequency = frequency;
}

void wave_generator_set_amplitude(wave_generator_t *wg, float amplitude)
{
    wg->amplitude = amplitude;
}

void wave_generator_set_offset(wave_generator_t *wg, float offset)
{
    wg->offset = offset;
}

void wave_generator_set_duty_cycle(wave_generator_t *wg, float duty_cycle)
{
    wg->duty_cycle = float_constrain(duty_cycle, 0.0f, 1.0f);
}

/* --------------------------- 阶跃 / 斜坡 ----------------------------- */

void wave_generator_set_step(wave_generator_t *wg, float target_value)
{
    /* 从当前波形值出发, 瞬间跳变 */
    wg->transient.start_value   = wg->state.current_value;
    wg->transient.target_value  = target_value;
    wg->transient.ramp_duration = 0.0f;
    wg->type                    = WAVE_STEP;
    wg->flag.completed          = false;
    wg->flag.enable             = true;
    _time_start(wg);
}

void wave_generator_set_ramp(wave_generator_t *wg,
                             float target_value, float duration)
{
    /* duration ≤ 0 退化为阶跃 */
    if (duration <= 0.0f)
    {
        wave_generator_set_step(wg, target_value);
        return;
    }
    wg->transient.start_value   = wg->state.current_value;
    wg->transient.target_value  = target_value;
    wg->transient.ramp_duration = duration;
    wg->type                    = WAVE_RAMP;
    wg->flag.completed          = false;
    wg->flag.enable             = true;
    _time_start(wg);
}

/* --------------------------- 启停与模式 ------------------------------ */

void wave_generator_enable(wave_generator_t *wg)
{
    wg->flag.enable      = true;
    wg->flag.completed   = false;
    wg->flag.cycle_count = 0;
    _time_start(wg);
}

void wave_generator_disable(wave_generator_t *wg)
{
    wg->flag.enable = false;
}

void wave_generator_reset(wave_generator_t *wg)
{
    /* 还原输出到 offset, 清零周期计数 */
    wg->state.current_value = wg->offset;
    wg->state.last_value    = wg->offset;
    wg->flag.completed      = false;
    wg->flag.cycle_count    = 0;
    _time_start(wg);
}

void wave_generator_set_single_shot(wave_generator_t *wg, bool single_shot)
{
    wg->flag.single_shot = single_shot;
    if (single_shot)
    {
        wg->flag.cycle_count = 0;
        wg->flag.completed   = false;
    }
}

void wave_generator_set_custom_table(wave_generator_t *wg,
                                     float *table, uint16_t size)
{
    wg->custom.table = table;
    wg->custom.size  = size;
}

void wave_generator_set_smooth_time(wave_generator_t *wg, float smooth_time)
{
    if (smooth_time <= 0.0f)
    {
        wg->smooth.enable = false;
    }
    else
    {
        wg->smooth.enable     = true;
        wg->smooth.smooth_time = smooth_time;
    }
}

bool wave_generator_is_smoothing(const wave_generator_t *wg)
{
    return wg->smooth.enable &&
           (wg->smooth.active.frequency ||
            wg->smooth.active.amplitude  ||
            wg->smooth.active.offset     ||
            wg->smooth.active.duty_cycle);
}

/* --------------------------- 核心计算 -------------------------------- */

/**
 * @brief 内部核心计算 — 接收外部 dt, 供 calc / calc_ex 共用
 */
static float _wave_generator_calc_core(wave_generator_t *wg, float dt)
{
    wg->state.last_value = wg->state.current_value;

    /* ---- 参数平滑: 检测 Ozone 修改, 平滑过渡 (先更新频率, 供相位累积) ---- */
    _smooth_update(wg, dt);

    if (dt > 0.0f && dt < DWT_DT_MAX_S)  /* 过滤异常跳变 */
    {
        wg->state.elapsed_time += dt;
        wg->state.phase_accum  += dt * wg->smooth.cur_frequency;  /* 相位增量累积 */
    }

    /* ---- 暂态波形: 阶跃 / 斜坡 ---- */
    if (wg->type == WAVE_STEP) { _step_calc(wg); return wg->state.current_value; }
    if (wg->type == WAVE_RAMP) { _ramp_calc(wg); return wg->state.current_value; }

    /* ---- 周期波形: 查函数指针表分发 ---- */
    if (wg->type >= WAVE_TYPE_NUM || s_wave_table[wg->type] == NULL)
    {
        wg->state.current_value = wg->smooth.cur_offset;
        return wg->state.current_value;
    }

    /* 使用平滑后的频率 */
    float freq = wg->smooth.cur_frequency;
    if (freq <= 0.0f)
    {
        wg->state.current_value = wg->smooth.cur_offset;
        return wg->state.current_value;
    }

    /* 相位 = phase_accum + phase/(2π) (phase_accum 已由 dt×freq 累积) */
    float phase = wg->state.phase_accum
                + wg->phase * (1.0f / (2.0f * PI));
    phase = _norm_phase(phase);

    /* 查表 → 归一化值 [-1, +1] → 映射到 [offset - amp, offset + amp] */
    float norm = s_wave_table[wg->type](phase, wg->smooth.cur_duty_cycle,
                                        wg->custom.table, wg->custom.size);
    wg->state.current_value = wg->smooth.cur_offset + wg->smooth.cur_amplitude * norm;

    /* 单次模式: 经过一个完整周期后自动停止 */
    if (wg->flag.single_shot)
    {
        uint32_t cyc = (uint32_t)wg->state.phase_accum;
        if (cyc > wg->flag.cycle_count)
        {
            wg->flag.cycle_count = cyc;
            _mark_completed(wg);
        }
    }

    return wg->state.current_value;
}

float wave_generator_calc(wave_generator_t *wg)
{
    /* 未使能或已完成, 返回保持值 */
    if (!wg->flag.enable || wg->flag.completed)
        return wg->state.current_value;

    /* ---- DWT 时间驱动: 累加两次调用之间的时间增量 ---- */
    float now = bsp_dwt_get_timeline_s();
    float dt  = now - wg->state.tick;
    wg->state.tick = now;

    return _wave_generator_calc_core(wg, dt);
}

float wave_generator_calc_ex(wave_generator_t *wg, float dt)
{
    /* 未使能或已完成, 返回保持值 */
    if (!wg->flag.enable || wg->flag.completed)
        return wg->state.current_value;

    return _wave_generator_calc_core(wg, dt);
}

