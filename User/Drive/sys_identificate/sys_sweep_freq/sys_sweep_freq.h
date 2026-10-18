#pragma once

#include "wave_generator.h"
#include "user_lib.h"

/* =================================================================== */
/*  扫频模式                                                           */
/* =================================================================== */
typedef enum
{
    SWEEP_MODE_SEQUENTIAL = 0,      // 顺序扫频：从低频到高频依次进行
    SWEEP_MODE_REVERSE,             // 反向扫频：从高频到低频依次进行
} sweep_mode_e;

/* =================================================================== */
/*  扫频状态                                                           */
/* =================================================================== */
typedef enum
{
    SWEEP_STATUS_IDLE = 0,          // 空闲
    SWEEP_STATUS_RUNNING,           // 运行中
    SWEEP_STATUS_PAUSED,            // 已暂停
    SWEEP_STATUS_COMPLETED,         // 全部完成
} sweep_status_e;

/* =================================================================== */
/*  频点终止条件类型                                                   */
/* =================================================================== */
typedef enum
{
    SWEEP_DURATION_BY_TIME = 0,     // 按时间（秒）终止
    SWEEP_DURATION_BY_CYCLE,        // 按周期数终止
} sweep_duration_type_e;

/* =================================================================== */
/*  单个扫频点配置                                                     */
/* =================================================================== */
typedef struct
{
    float       frequency;              // 频率 (Hz)
    float       amplitude;              // 幅值
    wave_type_e wave_type;              // 波形类型 @ref wave_type_e
    float       offset;                 // 偏置
    float       duty_cycle;             // 占空比 [0,1]，仅对方波/锯齿波有效（传 NAN 则使用波形发生器默认值 0.5）

    sweep_duration_type_e duration_type; // 终止方式
    union
    {
        float   duration_s;             // 持续时间 (秒), duration_type = BY_TIME
        uint32_t cycle_count;           // 周期数, duration_type = BY_CYCLE
    };
} sweep_freq_point_t;

/* =================================================================== */
/*  扫频事件类型 — 用于回调通知                                        */
/* =================================================================== */
typedef enum
{
    SWEEP_EVENT_POINT_START = 0,    // 一个频点开始
    SWEEP_EVENT_POINT_END,          // 一个频点结束
    SWEEP_EVENT_ALL_COMPLETE,       // 全部频点完成
} sweep_event_e;

/* =================================================================== */
/*  扫频事件数据 — 传递给回调函数                                      */
/* =================================================================== */
typedef struct
{
    sweep_event_e       event;              // 事件类型
    uint32_t            point_index;        // 当前频点索引 (从 0 开始)
    uint32_t            point_total;        // 总频点数
    float               current_frequency;  // 当前频率 (Hz)
    float               current_amplitude;  // 当前幅值
    wave_type_e         current_wave_type;  // 当前波形类型
    float               current_value;      // 当前波形输出值
    float               elapsed_time;       // 扫频已运行总时间 (s)
} sweep_event_data_t;

/** 扫频事件回调函数类型 */
typedef void (*sweep_callback_t)(const sweep_event_data_t *event_data, void *user_data);

/* =================================================================== */
/*  扫频器对象                                                         */
/* =================================================================== */
typedef struct
{
    /* ---------- 扫频配置 ---------- */
    const sweep_freq_point_t    *freq_table;        // 频点表指针（外部静态数组）
    uint32_t                    freq_table_size;    // 频点数
    sweep_mode_e                mode;               // 扫频模式
    uint32_t                    repeat_times;       // 重复次数 (0 = 无限循环)

    /* ---------- 内部波形发生器 ---------- */
    wave_generator_t            wave_gen;

    /* ---------- 运行时状态 ---------- */
    sweep_status_e              status;             // 当前状态
    uint32_t                    current_index;      // 当前频点索引
    uint32_t                    current_repeat;     // 当前第几次重复（从 0 开始）
    float                       point_elapsed;      // 当前频点已运行时间 (s)
    float                       total_elapsed;      // 总运行时间 (s)
    float                       last_timestamp;     // 上次更新时间戳 (s)

    /* ---------- 事件回调 ---------- */
    sweep_callback_t            callback;           // 事件回调函数
    void                       *callback_userdata;  // 回调用户数据

    /* ---------- 内部标志 ---------- */
    bool                        initialized;
} sweep_frequency_t;

/* =================================================================== */
/*  API 函数声明                                                       */
/* =================================================================== */

/**
 * @brief   初始化扫频器
 * @param   sf          扫频器对象指针
 * @param   freq_table  频点表（外部静态数组，调用者保证生命周期）
 * @param   table_size  频点数
 * @param   mode        扫频模式 @ref sweep_mode_e
 * @param   repeat      重复次数（0 = 无限循环）
 */
void sweep_frequency_init(sweep_frequency_t *sf,
                          const sweep_freq_point_t *freq_table,
                          uint32_t table_size,
                          sweep_mode_e mode,
                          uint32_t repeat);

/**
 * @brief   设置扫频事件回调
 * @param   sf          扫频器对象
 * @param   callback    回调函数，在事件发生时调用
 * @param   user_data   用户数据指针，透传给回调
 */
void sweep_frequency_set_callback(sweep_frequency_t *sf,
                                  sweep_callback_t callback,
                                  void *user_data);

/**
 * @brief   启动扫频
 * @param   sf 扫频器对象
 */
void sweep_frequency_start(sweep_frequency_t *sf);

/**
 * @brief   停止扫频（重置到空闲状态）
 * @param   sf 扫频器对象
 */
void sweep_frequency_stop(sweep_frequency_t *sf);

/**
 * @brief   暂停 / 恢复扫频
 * @param   sf      扫频器对象
 * @param   pause   true = 暂停, false = 恢复
 */
void sweep_frequency_pause(sweep_frequency_t *sf, bool pause);

/**
 * @brief   扫频主循环 —— 必须在控制循环中周期性调用
 * @param   sf 扫频器对象
 * @return  当前频点的波形输出值
 */
float sweep_frequency_calc(sweep_frequency_t *sf);

/**
 * @brief   手动跳转到下一个频点
 * @param   sf 扫频器对象
 */
void sweep_frequency_next_point(sweep_frequency_t *sf);

/**
 * @brief   跳转到指定索引的频点
 * @param   sf      扫频器对象
 * @param   index   目标频点索引
 */
void sweep_frequency_jump_to(sweep_frequency_t *sf, uint32_t index);

/* =================================================================== */
/*  内联查询接口                                                       */
/* =================================================================== */

/** @brief 获取当前扫频状态 */
static inline sweep_status_e sweep_frequency_get_status(const sweep_frequency_t *sf)
{
    return sf->status;
}

/** @brief 获取当前频点索引 */
static inline uint32_t sweep_frequency_get_current_index(const sweep_frequency_t *sf)
{
    return sf->current_index;
}

/** @brief 获取当前频点的频率值 */
static inline float sweep_frequency_get_current_frequency(const sweep_frequency_t *sf)
{
    if (sf->current_index >= sf->freq_table_size)
        return 0.0f;
    return sf->freq_table[sf->current_index].frequency;
}

/** @brief 获取当前频点的幅值 */
static inline float sweep_frequency_get_current_amplitude(const sweep_frequency_t *sf)
{
    if (sf->current_index >= sf->freq_table_size)
        return 0.0f;
    return sf->freq_table[sf->current_index].amplitude;
}

/** @brief 获取当前频点的波形类型 */
static inline wave_type_e sweep_frequency_get_current_wave_type(const sweep_frequency_t *sf)
{
    if (sf->current_index >= sf->freq_table_size)
        return WAVE_SINE;
    return sf->freq_table[sf->current_index].wave_type;
}

/** @brief 获取当前波形输出值（不触发计算） */
static inline float sweep_frequency_get_current_value(const sweep_frequency_t *sf)
{
    return wave_generator_get_value(&sf->wave_gen);
}

/** @brief 获取扫频总已运行时间 (s) */
static inline float sweep_frequency_get_total_elapsed(const sweep_frequency_t *sf)
{
    return sf->total_elapsed;
}

/** @brief 获取当前频点已运行时间 (s) */
static inline float sweep_frequency_get_point_elapsed(const sweep_frequency_t *sf)
{
    return sf->point_elapsed;
}

/** @brief 检查扫频是否全部完成 */
static inline bool sweep_frequency_is_completed(const sweep_frequency_t *sf)
{
    return sf->status == SWEEP_STATUS_COMPLETED;
}

/** @brief 获取当前重复次数 */
static inline uint32_t sweep_frequency_get_current_repeat(const sweep_frequency_t *sf)
{
    return sf->current_repeat;
}

/** @brief 获取当前频点配置指针，空闲或无效时返回 NULL */
static inline const sweep_freq_point_t *sweep_frequency_get_current_point(const sweep_frequency_t *sf)
{
    if (sf->current_index >= sf->freq_table_size)
        return NULL;
    return &sf->freq_table[sf->current_index];
}