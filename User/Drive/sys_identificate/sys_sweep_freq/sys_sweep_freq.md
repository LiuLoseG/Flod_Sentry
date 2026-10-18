# 扫频模块 (Sweep Frequency)

## 概述

`sweep_frequency` 模块用于对电机系统进行**扫频测试**（sweep frequency test），自动按预设的频点表依次生成激励波形，驱动电机运行，从而获得被测系统在不同频率下的响应数据。扫频数据可用于：

- **系统辨识**（System Identification）— 拟合被控对象的传递函数 / 频率特性
- **控制器频域设计** — 获取系统的幅频 / 相频特性曲线（Bode 图）
- **谐振点排查** — 找出机械谐振频率，指导滤波器设计
- **带宽验证** — 确认闭环系统的带宽是否满足设计要求

---

## 依赖

| 模块 | 用途 |
|------|------|
| `wave_generator` | 底层波形生成（正弦波、方波、三角波等） |
| `bsp_dwt` | 高精度时间戳（驱动波形相位积累和扫频计时） |
| `user_lib` | 工具宏和数学函数 |

---

## 核心数据结构

### `sweep_freq_point_t` — 单个扫频点

```c
typedef struct
{
    float       frequency;          // 频率 (Hz)
    float       amplitude;          // 幅值
    wave_type_e wave_type;          // 波形类型（正弦、方波等）
    float       offset;             // 偏置
    float       duty_cycle;         // 占空比 [0,1]，仅对方波/锯齿波有效

    sweep_duration_type_e duration_type; // 终止方式
    union
    {
        float   duration_s;         // 持续时间 (秒)
        uint32_t cycle_count;       // 周期数
    };
} sweep_freq_point_t;
```

每个频点可以独立设置：
- **频率** `frequency` — 激励信号频率 (Hz)
- **幅值** `amplitude` — 激励信号幅值
- **波形类型** `wave_type` — 波形选择（建议扫频用 `WAVE_SINE`）
- **偏置** `offset` — 信号直流偏置
- **占空比** `duty_cycle` — 仅对方波/锯齿波有效，传 `NAN` 或不关心时使用波形发生器默认值
- **终止条件** — 按时间 (`SWEEP_DURATION_BY_TIME`) 或按周期数 (`SWEEP_DURATION_BY_CYCLE`)

### `sweep_event_data_t` — 扫频事件数据

事件回调中携带的运行信息：

| 字段 | 类型 | 说明 |
|------|------|------|
| `event` | `sweep_event_e` | 事件类型 (POINT_START / POINT_END / ALL_COMPLETE) |
| `point_index` | `uint32_t` | 当前频点索引 (从 0 开始) |
| `point_total` | `uint32_t` | 总频点数 |
| `current_frequency` | `float` | 频点频率 (Hz) |
| `current_amplitude` | `float` | 频点幅值 |
| `current_wave_type` | `wave_type_e` | 频点波形类型 |
| `current_value` | `float` | 当前波形瞬时输出值 |
| `elapsed_time` | `float` | 扫频已运行总时间 (s) |

### `sweep_frequency_t` — 扫频器对象

```c
typedef struct
{
    /* 配置 */
    const sweep_freq_point_t *freq_table;    // 频点表指针
    uint32_t                  freq_table_size;
    sweep_mode_e              mode;           // 扫频模式
    sweep_dir_e               dir;            // 当前扫频方向
    uint32_t                  repeat_times;   // 0 = 无限循环

    /* 内部波形发生器 */
    wave_generator_t          wave_gen;

    /* 运行时状态 */
    sweep_status_e            status;
    uint32_t                  current_index;
    uint32_t                  current_repeat;
    float                     point_elapsed;
    float                     total_elapsed;
    float                     last_timestamp;

    /* 事件回调 */
    sweep_callback_t          callback;
    void                     *callback_userdata;

    bool                      initialized;
} sweep_frequency_t;
```

---

## API 参考

### 初始化和配置

| 函数 | 说明 |
|------|------|
| `sweep_frequency_init()` | 初始化扫频器，传入频点表、频点数、**模式**（顺序/反向）、重复次数 |
| `sweep_frequency_set_callback()` | 注册事件回调函数 |

### 生命周期控制

| 函数 | 说明 |
|------|------|
| `sweep_frequency_start()` | 启动扫频（从第一个频点开始） |
| `sweep_frequency_stop()` | 停止扫频，重置到空闲状态 |
| `sweep_frequency_pause()` | 暂停 / 恢复扫频 |

### 运行时

| 函数 | 说明 |
|------|------|
| `sweep_frequency_calc()` | **主循环函数** — 需在控制循环中周期性调用，返回当前波形值 |
| `sweep_frequency_next_point()` | 手动跳转到下一个频点 |
| `sweep_frequency_jump_to()` | 跳转到指定索引的频点 |

### 内联查询

| 函数 | 说明 |
|------|------|
| `sweep_frequency_get_status()` | 获取当前状态 (IDLE / RUNNING / PAUSED / COMPLETED) |
| `sweep_frequency_get_current_index()` | 当前频点索引 |
| `sweep_frequency_get_current_frequency()` | 当前频率 (Hz) |
| `sweep_frequency_get_current_amplitude()` | 当前幅值 |
| `sweep_frequency_get_current_wave_type()` | 当前波形类型 |
| `sweep_frequency_get_current_value()` | 当前波形输出值（不触发计算） |
| `sweep_frequency_get_total_elapsed()` | 扫频总运行时间 (s) |
| `sweep_frequency_get_point_elapsed()` | 当前频点已运行时间 (s) |
| `sweep_frequency_is_completed()` | 是否全部完成 |
| `sweep_frequency_get_current_repeat()` | 当前第几次重复 |
| `sweep_frequency_get_current_point()` | 获取当前频点配置指针 |

---

## 扫频状态机

```
         ┌─────────────────────────────────────────────────────┐
         │                                                     │
         v                                                     │
    ┌─────────┐   start()     ┌──────────┐   pause(true)   ┌───────┐
    │  IDLE   │ ────────────> │ RUNNING  │ ──────────────> │PAUSED │
    └─────────┘               └──────────┘                 └───────┘
         ▲                       │    │                        │
         │   stop()              │    │ 全部频点完成           │ pause(false)
         │                       │    v                        │
         │                       │  ┌───────────┐             │
         │                       └─>│COMPLETED  │<─────────────┘
         │                          └───────────┘
         └──────────────────────────────────── stop()
```

---

## 使用示例

> **注意：** 示例中使用了 `NAN` 宏（定义于 `<math.h>`）。如果您的工具链不支持或不想引入 `<math.h>`，可将 `duty_cycle` 设为 `0.0f` 来使用波形发生器默认值（`SWEEP_DUTY_VALID(0.0f)` 会判定为有效，但请注意 0% 占空比可能不是您想要的场景）。

### 示例 1：简单正弦扫频

```c
#include "sweep_frequency.h"

/* 定义频点表：从 1Hz 到 20Hz，每个频点持续 5 秒 */
static const sweep_freq_point_t freq_table[] = {
    { 1.0f,  0.5f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 5.0f },
    { 2.0f,  0.5f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 5.0f },
    { 5.0f,  0.5f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 5.0f },
    { 10.0f, 0.5f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 5.0f },
    { 20.0f, 0.5f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 5.0f },
};

static sweep_frequency_t sweep_freq;

void sweep_task_init(void)
{
    /* 初始化：一次扫频（repeat = 1），顺序模式 */
    sweep_frequency_init(&sweep_freq, freq_table, 5, SWEEP_MODE_SEQUENTIAL, 1);
    sweep_frequency_start(&sweep_freq);
}

void control_loop_1kHz(void)
{
    float output = sweep_frequency_calc(&sweep_freq);

    /* 将波形输出作为电机电流给定（需根据实际电机标定比例） */
    motor_set_current(output);

    /* 搭配示波器 / 上位机记录电机响应，用于系统辨识 */
    log_sweep_data(sweep_frequency_get_current_frequency(&sweep_freq),
                   sweep_frequency_get_total_elapsed(&sweep_freq),
                   output,
                   motor_get_feedback_speed());

    if (sweep_frequency_is_completed(&sweep_freq))
    {
        LOG_INFO("Sweep complete!");
    }
}
```

### 示例 2：按周期数 + 不同波形

```c
static const sweep_freq_point_t freq_table2[] = {
    /* 低频用正弦，持续 10 个周期 */
    { 1.0f,  0.3f, WAVE_SINE,   0.0f, NAN, SWEEP_DURATION_BY_CYCLE, .cycle_count = 10 },
    /* 中频用方波测阶跃响应，50% 占空比 */
    { 5.0f,  0.3f, WAVE_SQUARE, 0.0f, 0.5f, SWEEP_DURATION_BY_CYCLE, .cycle_count = 15 },
    /* 高频用正弦 */
    { 15.0f, 0.3f, WAVE_SINE,   0.0f, NAN, SWEEP_DURATION_BY_TIME,  .duration_s  = 8.0f },
};
```

### 示例 3：使用事件回调 + 完整信息打印

```c
/* 回调函数：每次切频点或扫频完成时打印信息 */
static void sweep_callback_handler(const sweep_event_data_t *ev, void *user)
{
    switch (ev->event)
    {
    case SWEEP_EVENT_POINT_START:
        LOG_INFO("[Sweep] >>> Point %d/%d : f = %.1f Hz, amp = %.3f, type = %d",
                 ev->point_index + 1, ev->point_total,
                 ev->current_frequency,
                 ev->current_amplitude,
                 ev->current_wave_type);
        break;

    case SWEEP_EVENT_POINT_END:
        LOG_INFO("[Sweep] <<< Point %d done, elapsed %.2f s",
                 ev->point_index + 1, ev->elapsed_time);
        break;

    case SWEEP_EVENT_ALL_COMPLETE:
        LOG_INFO("[Sweep] === All complete! ===");
        break;
    }
}

void setup(void)
{
    sweep_frequency_init(&sweep_freq, freq_table, 5, SWEEP_MODE_SEQUENTIAL, 1);
    sweep_frequency_set_callback(&sweep_freq, sweep_callback_handler, NULL);
    sweep_frequency_start(&sweep_freq);
}
```

### 示例 4：无限循环 + 暂停/恢复

```c
/* repeat = 0 表示无限循环扫频 */
sweep_frequency_init(&sweep_freq, freq_table, 5, SWEEP_MODE_SEQUENTIAL, 0);
sweep_frequency_start(&sweep_freq);

/* 遥控器逻辑中 */
if (rc_switch_pressed())
{
    sweep_frequency_pause(&sweep_freq, true);   // 暂停
}

if (rc_switch_released())
{
    sweep_frequency_pause(&sweep_freq, false);  // 恢复
}

/* 手动切频点 */
if (rc_button_clicked())
{
    sweep_frequency_next_point(&sweep_freq);
}
```

### 示例 5：反向扫频（从高频到低频）

```c
static const sweep_freq_point_t freq_table_sweep[] = {
    /* 从高频往低频扫 — SWEEP_MODE_REVERSE 会自动从末尾开始 */
    { 20.0f, 0.3f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 4.0f },
    { 10.0f, 0.3f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 4.0f },
    {  5.0f, 0.3f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 4.0f },
    {  2.0f, 0.3f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 4.0f },
    {  1.0f, 0.3f, WAVE_SINE, 0.0f, NAN, SWEEP_DURATION_BY_TIME, .duration_s = 4.0f },
};

sweep_frequency_init(&sweep_freq, freq_table_sweep, 5, SWEEP_MODE_REVERSE, 1);
sweep_frequency_start(&sweep_freq);
/* 执行顺序: 20Hz → 10Hz → 5Hz → 2Hz → 1Hz */
```

---

## 与上位机配合的数据采集方案

扫频时建议同时记录以下数据：

| 数据 | 获取方式 |
|------|----------|
| 时间戳 | `sweep_frequency_get_total_elapsed()` |
| 当前频率 | `sweep_frequency_get_current_frequency()` |
| 激励值（给定） | `sweep_frequency_get_current_value()` |
| 电机响应（实际） | 编码器反馈速度 / 位置 |

### 推荐的扫频参数设置

| 参数 | 建议值 | 说明 |
|------|--------|------|
| 幅值 | 额定电流/转速的 5% ~ 20% | 太小信噪比差，太大非线性明显 |
| 最低频率 | 0.5 ~ 2 Hz | 覆盖系统稳态增益区 |
| 最高频率 | 期望带宽的 2~5 倍 | 覆盖高频衰减区 |
| 每个频点持续时间 | 3~10 个周期 | 确保瞬态消退，进入稳态 |
| 频点分布 | 对数均匀分布 | 低频密高频疏，Bode 图更均匀 |

---

## 数据记录格式示例（CSV）

```
time_s, frequency_Hz, setpoint, feedback
0.000,  1.0,          0.000,    0.000
0.001,  1.0,          0.031,    0.028
0.002,  1.0,          0.063,    0.059
...
5.001,  2.0,          0.000,    0.001    // 切到 2Hz
5.002,  2.0,          0.031,    0.027
...
```

采集完成后，可用 MATLAB / Python 对数据做 FFT 分析，提取每个频率点的增益和相位：

```
频率(Hz)    增益(dB)     相位(°)
1.0         -0.12        -2.3
2.0         -0.25        -5.1
5.0         -0.87        -12.8
10.0        -2.34        -28.5
20.0        -5.67        -53.2
```

---

## 注意事项

1. **安全幅值**：扫频时电机可能会在某些频率点发生共振或啸叫，初始幅值应设小，观察系统无异常后再逐步增大。
2. **低频保护**：极低频（< 0.5 Hz）时电机长时间单向运动可能打到机械限位，建议配合位置保护或使用小幅值偏置归中。
3. **平稳过渡**：模块内部波形发生器自带 50ms 平滑过渡，在频点切换时避免输出跳变对电机的冲击。
4. **实时性**：`sweep_frequency_calc()` 应放在固定频率的控制循环中调用（如 1kHz），以确保波形精度。
5. **内存**：`sweep_freq_point_t` 频点表由用户定义（通常为 `static const` 数组），扫频器只持有指针，不拷贝数据。