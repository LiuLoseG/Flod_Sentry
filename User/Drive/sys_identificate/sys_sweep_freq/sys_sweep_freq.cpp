#include "sys_sweep_freq.h"
#include "wave_generator.h"
#include "bsp_dwt.h"

/* =================================================================== */
/*  内部常量                                                           */
/* =================================================================== */

/** @brief dt 最大有效值 — 超过此值视为首次启动或长时间暂停后恢复, 归零防跳变 */
#define SWEEP_DT_MAX_S  0.1f

/** @brief 频点切换时的参数平滑过渡时间 (s) */
#define SWEEP_SMOOTH_TIME_S  0.05f

/* =================================================================== */
/*  内部辅助宏                                                         */
/* =================================================================== */

/** @brief 可移植的 NaN 检测（避免嵌入式工具链对 isnan() 支持不一） */
#ifndef SWEEP_ISNAN
  #define SWEEP_ISNAN(x)  ((x) != (x))
#endif

/** @brief 判断浮点数是否在闭区间 [0,1] 内且非 NaN */
#define SWEEP_DUTY_VALID(d)  ((d) >= 0.0f && (d) <= 1.0f && !SWEEP_ISNAN(d))

/* =================================================================== */
/*  内部辅助函数                                                       */
/* =================================================================== */

/** @brief 根据模式判断扫频方向: true = 正向(索引递增), false = 反向(索引递减) */
static inline bool sweep_is_forward(const sweep_frequency_t *sf)
{
    return sf->mode == SWEEP_MODE_SEQUENTIAL;
}

/** @brief 获取起始端索引 (正向=0, 反向=末尾) */
static inline uint32_t sweep_start_index(const sweep_frequency_t *sf)
{
    return sweep_is_forward(sf) ? 0 : (sf->freq_table_size - 1);
}

/**
 * @brief 触发事件回调（如果已注册）
 * @note  调用者保证 sf->current_index < sf->freq_table_size
 */
static void sweep_fire_event(sweep_frequency_t *sf, sweep_event_e event)
{
    if (sf->callback == NULL)
        return;

    sweep_event_data_t data;
    data.event           = event;
    data.point_index     = sf->current_index;
    data.point_total     = sf->freq_table_size;
    data.current_frequency = sf->freq_table[sf->current_index].frequency;
    data.current_amplitude = sf->freq_table[sf->current_index].amplitude;
    data.current_wave_type = sf->freq_table[sf->current_index].wave_type;
    data.current_value    = wave_generator_get_value(&sf->wave_gen);
    data.elapsed_time     = sf->total_elapsed;

    sf->callback(&data, sf->callback_userdata);
}

/**
 * @brief 加载指定索引的频点配置到波形发生器
 */
static void sweep_load_point(sweep_frequency_t *sf, uint32_t index)
{
    if (index >= sf->freq_table_size)
        return;

    const sweep_freq_point_t *pt = &sf->freq_table[index];

    /* 使用 setter API 平滑切换参数，而非 re-init（避免 memset 清除流畅状态） */
    wave_generator_set_type(&sf->wave_gen, pt->wave_type);
    wave_generator_set_frequency(&sf->wave_gen, pt->frequency);
    wave_generator_set_amplitude(&sf->wave_gen, pt->amplitude);
    wave_generator_set_offset(&sf->wave_gen, pt->offset);

    /* 如果频点指定了有效占空比，则设置 */
    if (SWEEP_DUTY_VALID(pt->duty_cycle))
    {
        wave_generator_set_duty_cycle(&sf->wave_gen, pt->duty_cycle);
    }

    wave_generator_enable(&sf->wave_gen);
    wave_generator_set_smooth_time(&sf->wave_gen, SWEEP_SMOOTH_TIME_S);

    /* 重置频点计时 */
    sf->point_elapsed = 0.0f;
    sf->current_index = index;

    /* 触发频点开始事件 */
    sweep_fire_event(sf, SWEEP_EVENT_POINT_START);
}

/**
 * @brief 检查当前频点的终止条件是否满足
 * @return true = 需要切换到下一个频点
 */
static bool sweep_check_duration_elapsed(sweep_frequency_t *sf)
{
    if (sf->current_index >= sf->freq_table_size)
        return true;

    const sweep_freq_point_t *pt = &sf->freq_table[sf->current_index];

    if (pt->duration_type == SWEEP_DURATION_BY_TIME)
    {
        /* 按时间终止 */
        return (sf->point_elapsed >= pt->duration_s);
    }
    else /* SWEEP_DURATION_BY_CYCLE */
    {
        /* 按周期数终止 */
        return (wave_generator_get_cycle_count(&sf->wave_gen) >= pt->cycle_count);
    }
}

/**
 * @brief 根据扫频方向计算下一个频点索引
 * @return 下一个索引值，越界时返回 UINT32_MAX
 */
static uint32_t sweep_next_index(const sweep_frequency_t *sf)
{
    if (sweep_is_forward(sf))
    {
        uint32_t next = sf->current_index + 1;
        return (next < sf->freq_table_size) ? next : UINT32_MAX;
    }
    else
    {
        if (sf->current_index == 0)
            return UINT32_MAX;
        return sf->current_index - 1;
    }
}

/**
 * @brief 推进到下一个频点（内部使用）
 * @return true = 成功进入下一频点；false = 全部完成
 */
static bool sweep_advance_point(sweep_frequency_t *sf)
{
    /* 触发当前频点结束事件 */
    sweep_fire_event(sf, SWEEP_EVENT_POINT_END);

    uint32_t next_index = sweep_next_index(sf);

    if (next_index != UINT32_MAX)
    {
        /* 还有下一个频点，加载之 */
        sweep_load_point(sf, next_index);
        return true;
    }

    /* 所有频点已遍历完，处理重复逻辑 */
    if (sf->repeat_times == 0 || sf->current_repeat + 1 < sf->repeat_times)
    {
        /* 无限重复 (0) 或 还有剩余重复次数 — 回到起始端 */
        sf->current_repeat++;
        sweep_load_point(sf, sweep_start_index(sf));
        return true;
    }

    /* 全部完成 */
    wave_generator_disable(&sf->wave_gen);
    wave_generator_reset(&sf->wave_gen);
    sf->status = SWEEP_STATUS_COMPLETED;
    sweep_fire_event(sf, SWEEP_EVENT_ALL_COMPLETE);
    return false;
}

/* =================================================================== */
/*  内部：更新 time step                                                */
/* =================================================================== */

/**
 * @brief 获取当前帧的时间步长 (dt)
 * @note  首次调用或用 stop 后首次调用时返回 0，防止 dt 跳变
 */
static float sweep_get_dt(sweep_frequency_t *sf)
{
    float now = bsp_dwt_get_timeline_s();
    float dt  = now - sf->last_timestamp;
    sf->last_timestamp = now;

    /* 防止首次 / 长时间停顿后的 dt 跳变 */
    if (dt < 0.0f || dt > SWEEP_DT_MAX_S)
        dt = 0.0f;

    return dt;
}

/* =================================================================== */
/*  公开 API 实现                                                      */
/* =================================================================== */

void sweep_frequency_init(sweep_frequency_t *sf,
                          const sweep_freq_point_t *freq_table,
                          uint32_t table_size,
                          sweep_mode_e mode,
                          uint32_t repeat)
{
    if (sf == NULL || freq_table == NULL || table_size == 0)
        return;

    sf->freq_table       = freq_table;
    sf->freq_table_size  = table_size;
    sf->mode             = mode;
    sf->repeat_times     = repeat;

    /* 初始化波形发生器 — 使用第一个频点的配置 */
    const sweep_freq_point_t *first = &freq_table[0];
    wave_generator_init(&sf->wave_gen, first->wave_type, first->frequency, first->amplitude, first->offset);

    /* 如果首点指定了有效占空比，则设置 */
    if (SWEEP_DUTY_VALID(first->duty_cycle))
    {
        wave_generator_set_duty_cycle(&sf->wave_gen, first->duty_cycle);
    }

    /* 运行时状态 */
    sf->status          = SWEEP_STATUS_IDLE;
    sf->current_index   = 0;
    sf->current_repeat  = 0;
    sf->point_elapsed   = 0.0f;
    sf->total_elapsed   = 0.0f;
    sf->last_timestamp  = 0.0f;

    /* 回调 */
    sf->callback            = NULL;
    sf->callback_userdata   = NULL;

    sf->initialized = true;
}

void sweep_frequency_set_callback(sweep_frequency_t *sf,
                                  sweep_callback_t callback,
                                  void *user_data)
{
    if (sf == NULL || !sf->initialized)
        return;
    sf->callback            = callback;
    sf->callback_userdata   = user_data;
}

void sweep_frequency_start(sweep_frequency_t *sf)
{
    if (sf == NULL || !sf->initialized)
        return;

    /* 重置状态 */
    sf->current_index  = 0;
    sf->current_repeat = 0;
    sf->point_elapsed  = 0.0f;
    sf->total_elapsed  = 0.0f;

    /* 重置时间戳，加载起始频点 */
    sf->last_timestamp = bsp_dwt_get_timeline_s();
    sweep_load_point(sf, sweep_start_index(sf));

    sf->status = SWEEP_STATUS_RUNNING;
}

void sweep_frequency_stop(sweep_frequency_t *sf)
{
    if (sf == NULL || !sf->initialized)
        return;

    wave_generator_disable(&sf->wave_gen);
    wave_generator_reset(&sf->wave_gen);

    sf->status         = SWEEP_STATUS_IDLE;
    sf->current_index  = 0;
    sf->current_repeat = 0;
    sf->point_elapsed  = 0.0f;
    sf->total_elapsed  = 0.0f;
}

void sweep_frequency_pause(sweep_frequency_t *sf, bool pause)
{
    if (sf == NULL || !sf->initialized)
        return;

    if (pause && sf->status == SWEEP_STATUS_RUNNING)
    {
        sf->status = SWEEP_STATUS_PAUSED;
    }
    else if (!pause && sf->status == SWEEP_STATUS_PAUSED)
    {
        /* 恢复时刷新时间戳，防止 dt 跳变 */
        sf->last_timestamp = bsp_dwt_get_timeline_s();
        sf->status = SWEEP_STATUS_RUNNING;
    }
}

float sweep_frequency_calc(sweep_frequency_t *sf)
{
    if (sf == NULL || !sf->initialized)
        return 0.0f;

    /* 非运行状态：空闲 / 暂停保持当前值，完成时归零 */
    if (sf->status != SWEEP_STATUS_RUNNING)
    {
        if (sf->status == SWEEP_STATUS_COMPLETED)
            return 0.0f;
        return wave_generator_get_value(&sf->wave_gen);
    }

    /* 计算时间步长 */
    float dt = sweep_get_dt(sf);

    /* 更新计时 */
    sf->point_elapsed += dt;
    sf->total_elapsed += dt;

    /* 计算当前波形输出 — 传入统一 dt，避免重复取 DWT 时间戳 */
    float value = wave_generator_calc_ex(&sf->wave_gen, dt);

    /* 检查当前频点是否应该结束 */
    if (sweep_check_duration_elapsed(sf))
    {
        sweep_advance_point(sf);
    }

    return value;
}

void sweep_frequency_next_point(sweep_frequency_t *sf)
{
    if (sf == NULL || !sf->initialized || sf->status != SWEEP_STATUS_RUNNING)
        return;

    sweep_advance_point(sf);
}

void sweep_frequency_jump_to(sweep_frequency_t *sf, uint32_t index)
{
    if (sf == NULL || !sf->initialized || index >= sf->freq_table_size)
        return;

    if (sf->status == SWEEP_STATUS_RUNNING ||
        sf->status == SWEEP_STATUS_PAUSED)
    {
        /* 触发当前点结束事件 */
        sweep_fire_event(sf, SWEEP_EVENT_POINT_END);

        /* 加载目标频点 */
        sweep_load_point(sf, index);

        if (sf->status == SWEEP_STATUS_PAUSED)
        {
            /* 暂停模式下也要更新时间戳 */
            sf->last_timestamp = bsp_dwt_get_timeline_s();
        }
    }
}