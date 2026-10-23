# PID 模块（pid_class）

## 概述

`pid_class`（`User/Drive/algorithm/pid/pid.{h,cpp}`）是本仓库通用 PID 控制器，支持：

| 特性 | 相关字段 / 参数 |
|---|---|
| 位置式 PID（P/I/D + 前馈） | `k_p` / `k_i` / `k_d` / `k_f` |
| 死区 | `dead_zone` |
| 变速积分 | `i_variable_speed_a/b` |
| 积分分离 | `i_separate_threshold` |
| 积分项限幅 | `i_out_max` |
| 微分先行 + 微分低通 | `d_first` / `d_filter_n` |
| **抗积分饱和（anti-windup）** | `anti_windup` / `anti_windup_gain` / `anti_windup_out_max` |

> 本文重点说明**抗积分饱和**；其余特性沿用原有实现，不在此重复。

---

## 0. 快速上手（TL;DR）

```cpp
// 老代码最小补丁：init 之后加 3 行即可
p.pid_omega.set_anti_windup(PID_Anti_Windup_BACK_CALC);        // 力矩/速度环用反算
p.pid_omega.set_anti_windup_gain(12.0f);                       // Kb，1/s，10~15 起
p.pid_omega.set_anti_windup_out_max(YAW_SMALL_TORQUE_LIMIT);   // ★ 必须=下游真实限幅

p.pid_angle.set_anti_windup(PID_Anti_Windup_CLAMP);            // 角度环用条件积分，零参数
```

- 默认 `PID_Anti_Windup_DISABLE`，**不开则行为与原逻辑逐位一致**，可安全合入。
- 唯一容易配错、且一错就失效的点：`anti_windup_out_max` 必须等于 **PID 出口后最后一级真正生效的限幅**（详见 §5.1）。

---

## 1. 为什么需要抗积分饱和（windup）

位置式 PID 的执行顺序：

```
error → P → I(每帧 integral_error += d_t*error) → D → F → out = P+I+D+F → 限幅
```

问题：**积分器不知道输出被限幅了**。当执行器已到顶（力矩/速度到上限），误差仍在，`integral_error` 继续累加。等系统追上、误差反号时，这段"超额积分"要很久才退得回去，退的过程还在朝原方向猛推，于是：

- 明显**过冲**；
- **退饱和慢**、调节时间长；
- 严重时**来回振荡**（近极限环）。

核心矛盾一句话：**限幅作用在 `out` 上，却没有作用在 `integral_error` 上**。抗积分饱和就是补上这条反馈。

---

## 2. 作用原理

通过枚举 `pid_anti_windup_e` 选择方案：

| 枚举值 | 方案 | 一句话 |
|---|---|---|
| `PID_Anti_Windup_DISABLE` | 关闭（默认） | 行为与原逻辑逐位一致 |
| `PID_Anti_Windup_CLAMP` | 条件积分 | 饱和且误差仍朝限幅方向时，撤销本步积分 |
| `PID_Anti_Windup_BACK_CALC` | 反算 | 按输出超出量，把积分主动往回拉 |

两个关键中间量（`pid.cpp` 内每帧计算）：

| 变量 | 含义 |
|---|---|
| `out_unsaturated` | 限幅**前**的原始输出 `P+I+D+F`（PID 的"意愿"） |
| `out` | 限幅**后**的实际输出（真正下发） |
| `saturated` | 本帧是否被削顶：`out != out_unsaturated` |
| `out_unsaturated - out` | "意愿被削掉的那部分"，**符号 = 需求被堵住的方向** |
| `out - out_unsaturated` | 同上**反号**（反算公式使用） |

### 2.1 条件积分（CLAMP）

```c
if (saturated && anti_windup != PID_Anti_Windup_DISABLE && k_i != 0.0f)
{
    if (term.error * (out_unsaturated - out) > 0.0f)   // 误差与"被堵方向"同号
    {
        integral_error -= integ_delta;                 // 撤销本帧积分增量
        term.i_out = k_i * integral_error;
    }
}
```

判据 `error * (out_unsaturated - out) > 0` 的四象限含义：

| 误差方向 | 饱和方向 | 乘积 | 动作 |
|---|---|---|---|
| 正 | 顶正上限 | >0 | **冻结**本帧积分（继续积累只会更堵） |
| 负 | 顶负下限 | >0 | **冻结**本帧积分 |
| 负 | 顶正上限 | <0 | 允许积分（误差在回救，帮助退饱和） |
| 正 | 顶负下限 | <0 | 允许积分（同上） |

要点：**只撤"本帧增量"，不回滚历史积分** → 稳态所需积分不动，故不改稳态精度；误差一反向立即恢复积分，帮助快速脱离饱和。

### 2.2 反算（BACK_CALC）

```c
integral_error += (anti_windup_gain / k_i) * (out - out_unsaturated) * d_t;
term.i_out = k_i * integral_error;
```

$$\Delta(\text{integral}) = \frac{K_b}{k_i}\,(u_{\text{sat}}-u_{\text{unsat}})\,\Delta t$$

- `u_sat-u_unsat = out - out_unsaturated`：**被削掉的量（带符号，顶正上限为负）**。
- 顶正上限 → 该量为负 → 积分被往下拽；顶负下限 → 为正 → 往上拽。方向永远指向"把积分拉回到让输出刚好不饱和"。
- 物理意义：这是对积分状态的**一阶跟踪**，等效时间常数 `Tt = 1/Kb`（见 §5 表）。它是一个**自趋近负反馈**，因此退饱和非常利落。

> ⚠️ 符号坑：CLAMP 用 `out_unsaturated - out`（顶正上限为正），BACK_CALC 用 `out - out_unsaturated`（顶正上限为负）。两者是同一物理量的相反符号约定，改代码时别混。

### 2.3 两者差异与选型

| | CLAMP（条件积分） | BACK_CALC（反算） |
|---|---|---|
| 动作力度 | 一帧一帧"不加深" | 由 `Kb` 决定，可快可慢、连续 |
| 能否回滚历史积分 | ❌ 只挡新增 | ✅ 可主动拉回 |
| 需整定参数 | 无 | 需 `Kb` |
| 主要风险 | 长期饱和仍残留 → 有过冲 | `Kb` 过大 / 持续饱和过久 → **过度回退**，退饱和后欠补（见 §6 实例 B） |
| 适用 | 角度环、带持续负载的环 | 速度/力矩环、瞬态饱和 |

---

## 3. 实现说明（改了哪些地方）

仅改 `pid.h` / `pid.cpp`，**不改变现有调用方式**，新增参数全部带默认值。

### 3.1 `pid.h`

1. 新增枚举 `pid_anti_windup_e`：`PID_Anti_Windup_DISABLE / CLAMP / BACK_CALC`。
2. `init()` 末尾追加 3 个带默认值形参（旧调用不受影响）：
   - `const pid_anti_windup_e &__anti_windup = PID_Anti_Windup_DISABLE`
   - `const float &__anti_windup_gain = 10.0f`（反算增益 `Kb`，1/s）
   - `const float &__anti_windup_out_max = 0.0f`（抗饱和判定用输出上限，0=沿用 `out_max`）
3. 新增成员（就地初始化）：`anti_windup`、`anti_windup_gain`、`anti_windup_out_max`、`out_unsaturated`。
4. 新增 `get_out_unsaturated()` 与 3 个 `set_anti_windup*()`（`inline`）。

### 3.2 `pid.cpp`

1. 积分累加处把"每步增量"记到 `integ_delta`（`integral_error += ...` → 先记 `integ_delta` 再加）。
2. 积分项限幅加 `k_i != 0.0f` 守卫（顺带消除原 `i_out_max/k_i` 在 `k_i==0` 时的除零隐患）。
3. 输出段：先算 `out_unsaturated`，再用 `out_limit = (anti_windup_out_max != 0 ? anti_windup_out_max : out_max)` 限幅，并记录 `saturated`。
4. 限幅后按方案执行 CLAMP / BACK_CALC 回退，并同步刷新 `term.i_out`。

> `anti_windup == PID_Anti_Windup_DISABLE` 时抗饱和分支不执行，`out` 限幅逻辑与原实现一致，**输出逐位相同**。

---

## 4. 使用方法

### 4.1 方式一：`init()` 直接带参（推荐新代码）

```cpp
// 速度环，反算抗饱和，下游真实限幅 1.0
pid.init(0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f, 0.001f, 0.0f, 0.2f, 0.65f, 1.0f,
         PID_D_First_ENABLE,
         PID_Anti_Windup_BACK_CALC, 12.0f, 1.0f);
//                                    ↑Kb    ↑aw_out_max
```

### 4.2 方式二：`init()` 之后用 setter（改动最小，推荐老代码补丁）

```cpp
p.pid_omega.init(0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f);        // 原样，不动
p.pid_omega.set_anti_windup(PID_Anti_Windup_BACK_CALC);
p.pid_omega.set_anti_windup_gain(12.0f);
p.pid_omega.set_anti_windup_out_max(YAW_SMALL_TORQUE_LIMIT);   // ★ 对齐下游
```

### 4.3 选型建议

| 环路 | 建议 | 理由 |
|---|---|---|
| 角度环（`k_i > 0`，输出为速度指令） | `CLAMP` | 零参数、最稳；一般不常饱和 |
| 速度 / 力矩环（直接驱动力矩） | `BACK_CALC` | 退饱和快、不顶力矩 |
| 带持续恒定负载、需长期保持力矩 | `CLAMP`（或 `i_out_max` 兜底） | BACK_CALC 可能过度回退致稳态误差（§6-B） |
| `k_i == 0` 的环 | 无需设置 | 本无 windup，代码有守卫 |

---

## 5. 参数说明与整定

| 参数 | 单位 | 作用 | 建议 |
|---|---|---|---|
| `anti_windup` | — | 方案选择 | 先 `DISABLE` 观察，再按 §4.3 开启 |
| `anti_windup_gain`（`Kb`） | 1/s | 仅 BACK_CALC：积分回退强度，等效跟踪时间常数 `Tt=1/Kb` | **10 ~ 15** 起 |
| `anti_windup_out_max` | 同 `out` | 抗饱和判定用的真实输出上限 | **必须 = 下游最后一级硬限幅**；0 表示沿用 `out_max` |

`Kb` ↔ 跟踪时间常数：

| `Kb` (1/s) | 5 | 10 | 12 | 20 | 30 |
|---|---|---|---|---|---|
| `Tt=1/Kb` | 200 ms | 100 ms | 83 ms | 50 ms | 33 ms |

`Kb` 越大 → 回退越快 → 退饱和越利落，但过大易与积分/P 形成新动态（抖动）；过小则退得慢。

### 5.1 ★ `anti_windup_out_max` 是整定成败的关键

`pid_class` 内部按 `out_max` 限幅；但业务层在 PID 出口之后往往还有一次硬限幅，例如：

```cpp
yaw_small_motor.set_target_torque(basic_math_constrain(
    pid_omega.get_out() + ff, -YAW_SMALL_TORQUE_LIMIT, YAW_SMALL_TORQUE_LIMIT));
```

若 PID 内部按 `out_max = 1.2` 判饱和、而下游实际按 `1.0` 削，则 PID 在 `1.0 ~ 1.2` 区间会**误以为没饱和**，抗饱和失效。

> **规则**：`anti_windup_out_max` 一律填**真正生效的最后一级限幅**（上例 `YAW_SMALL_TORQUE_LIMIT`）。
> 若确实只有 `out_max` 一道限幅，则无需设 `anti_windup_out_max`（填 0）。

### 5.2 与现有特性的交互

| 现有特性 | 交互说明 |
|---|---|
| `i_out_max`（积分项限幅） | 与 AW 互补，**建议保留**：AW 管"动态回退"，`i_out_max` 管"积分绝对幅值上限" |
| `i_separate_threshold`（积分分离） | 误差大时清零积分，此时 AW 无增量可撤；**建议二选一，倾向保留 AW**（按"输出是否真被削"判断，比"按误差大小猜"更准） |
| `i_variable_speed_a/b`（变速积分） | 正常工作；`integ_delta` 已按 `speed_ratio` 缩放 |
| `dead_zone`（死区） | 正常工作，不影响 AW |
| `d_first` / `d_filter_n` | 正常工作，不影响 AW |

### 5.3 整定流程

1. 全部 `DISABLE`；Ozone 观察各环 `get_out()` 与 `get_out_unsaturated()` 的差值，找出**长期饱和**的环。
2. 角度环设 `CLAMP`（无需调参）。
3. 力矩/速度环设 `BACK_CALC`，`anti_windup_out_max` 填下游真实限幅，`Kb` 从 10 起。
4. 复现饱和工况（大阶跃 / 大负载 / 扫频大振幅），观察：
   - 过冲仍大 → 加大 `Kb`；
   - 抖动 / 来回切 → 减小 `Kb`；
   - 退饱和后**欠补 / 稳态误差** → 减小 `Kb` 或改用 `CLAMP`（持续负载场景）；
   - 仍无改善 → 检查 `anti_windup_out_max` 是否与下游一致。

---

## 6. 效果实例（离线仿真，供选型参考）

### 实例 A：干净对象 —— 看 CLAMP

对象 `y' = -2y + 2u`（直流增益 1）；阶跃 `ref 0→1.0`；`out_max=2.5` 瞬态饱和；`Kp=5, Ki=20, Kd=0`，关积分分离：

| 方案 | 超调 | 2% 调节时间 | 峰值 | 最大 \|I 输出\| | 饱和时长 |
|---|---|---|---|---|---|
| 无抗饱和 | +16.2% | 0.954 s | 1.162 | 2.35 | 244 ms |
| **CLAMP** | **+0.77%** | **0.416 s** | 1.008 | **1.06** | **111 ms** |

饱和期（0.2–0.35 s）积分被冻结，退饱和后才继续积累；**稳态终值两者完全一致** → 只削多余的动态累积，不改稳态精度。

### 实例 B：带恒定重力负载的角度环 —— CLAMP vs BACK_CALC

`kp=6, ki=4, kd=0.15`，力矩限幅 1.0 Nm，恒定负载 0.4 Nm，阶跃 0→2 rad：

| 方案 | 超调 | 峰值 | 最大 \|I 输出\| | 说明 |
|---|---|---|---|---|
| 无抗饱和 | +26.5% | 2.530 | 1.51 | 明显过冲 |
| CLAMP | +13.6% | 2.271 | 0.25 | 过冲减半，无副作用 |
| BACK_CALC (Kb=12) | **+0.2%** | 2.004 | 3.70 | 几乎无超调 |

**重要结论（选型依据）**：本例存在**持续**重力负载、饱和持续时间长。BACK_CALC 在持续饱和期会把积分一路往回拉（甚至拉到负值），退饱和后需重新补回，表现为**小的稳态欠补 / 稳态误差**，且长时间大 `(out-uns)` 会暂时把 \|I 输出\| 推到较大值。因此：

- **瞬态饱和**（大阶跃、快速机动）→ BACK_CALC 最优（超调 26.5%→0.2%）；
- **持续饱和 + 持续负载**（如重力补偿场合）→ 优先 CLAMP，或用 `i_out_max` 兜底并适当降低 `Kb`。

---

## 7. 观测与调试

| 接口 | 含义 | 用途 |
|---|---|---|
| `get_out()` | 限幅后实际输出 | 下发的真实值 |
| `get_out_unsaturated()` | 限幅前原始输出 | 与 `get_out()` **持续不等 = 该环长期饱和**，据此决定是否开 AW 并核查 `anti_windup_out_max` |
| `get_integral_error()` | 积分状态 | 确认饱和期积分不再无界增长 |

建议三个量一起加入 Ozone 观察：`out`、`out_unsaturated`、`integral_error`。开 AW 后应看到：饱和期 `integral_error` 不再单调增长、`out_unsaturated` 的峰值明显下降。

---

## 8. 版本记录

| 版本 | 日期 | 说明 |
|---|---|---|
| 0.1 | 2026-10-23 | 新增抗积分饱和（CLAMP / BACK_CALC + `get_out_unsaturated` 观测）；`init` 参数为带默认值的追加，向后兼容 |
| 0.2 | 2026-10-23 | 优化文档：补充 `Kb↔Tt` 换算、符号约定、持续负载下 BACK_CALC 过度回退的选型依据、快速上手与观测表 |
