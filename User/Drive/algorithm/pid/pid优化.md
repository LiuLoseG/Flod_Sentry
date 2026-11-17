# PID 模块（pid_class）优化详解

> 本文按“每个优化环节”逐一展开：**要解决什么问题 → 原理与数学 → 代码实现 → 参数与整定 → 注意事项**。
> 说明：此代码是在**麻神代码和玺佬**的基础上进行优化和更新。

> **2026-11-17 迭代说明（优先于下文历史推导片段）**：修复死区边界、目标被改写、积分当帧越限、负 `k_i` 限幅、CLAMP 回退与输出不一致、BACK_CALC 回退后越限以及下游限幅放宽内部限幅的问题。接口及默认参数保持兼容，但这些边界行为有意修正，不再保证与旧版逐位一致。复现方法与验证结果见文末。

---

## 目录

| 章节 | 主题 |
|---|---|
| 0 | 总览（优化全景图 + 速览表） |
| 1 | 基础计算流程（读代码前必看） |
| 2 | 优化① 死区处理（Dead Zone） |
| 3 | 优化② 变速积分（Variable-Speed Integral） |
| 4 | 优化③ 积分分离（Integral Separation） |
| 5 | 优化④ 积分项限幅（Integral Clamp） |
| 6 | 优化⑤ 微分先行 + 微分一阶低通 |
| 7 | 优化⑥ 前馈补偿（Feedforward） |
| 8 | 优化⑦ 抗积分饱和（Anti-Windup） |
| 9 | 优化⑧ 数值安全与向下兼容（Robustness） |
| 10 | 关键参数速查表 |
| 11 | 使用示例 |
| 12 | 整定流程（推荐顺序） |
| 13 | 观测与调试 |
| 14 | 版本记录 |
| 15 | 迭代回归与闭环仿真（位于使用示例之前） |
| 16 | 第二轮迭代：采样周期一致性、积分精度与抗饱和语义 |
| 17 | 最小修复落地与历史建议修正 |

---

## 0. 总览

`pid_class` 是一个**位置式（Positional）PID** 控制器，在基础 `P + I + D` 之上叠加了前馈，并针对实车常见问题做了多项工程化优化。整体结构如下：

```
error = target - now
      │
      ├─ ① 死区（dead_zone）─────────────► 抑制微小抖动 / 抑制积分累积
      │
      ├─ ② P 项  = k_p * error
      │
      ├─ ③ I 项  = k_i * integral_error
      │     ├─ 变速积分（i_variable_speed_a/b）── 大误差时减速积分
      │     ├─ 积分分离（i_separate_threshold）─ 大误差时闭锁积分
      │     ├─ 积分项限幅（i_out_max）────────── 积分绝对幅值上限
      │     └─（抗饱和在输出段回退，见 ⑦）
      │
      ├─ ④ D 项  = k_d * d(error)/dt
      │     └─ 微分先行（d_first）+ 一阶低通（d_filter_n）── 抑制微分噪声
      │
      ├─ ⑤ 前馈  = k_f * (target - pre_target)
      │
      └─ ⑥ out = P + I + D + F ──► 硬输出限幅（out_max）
                                   └─ ⑦ 抗积分饱和（anti_windup）── 反馈回积分器
```

| 编号 | 优化环节 | 主要参数 | 一句话作用 |
|---|---|---|---|
| ① | 死区处理 | `dead_zone` | 极小幅误差不动作，抗抖振、防积分累积 |
| ② | 变速积分 | `i_variable_speed_a/b` | 误差越大积分越慢，减小超调 |
| ③ | 积分分离 | `i_separate_threshold` | 大误差直接闭锁积分，防大超调 |
| ④ | 积分项限幅 | `i_out_max` | 限制积分项绝对幅值 |
| ⑤ | 微分先行 + 微分低通 | `d_first` / `d_filter_n` | 消除“设定突变”造成的微分冲击、滤除微分噪声 |
| ⑥ | 前馈补偿 | `k_f` | 提升跟随性、减小相位滞后 |
| ⑦ | 抗积分饱和 | `anti_windup` 等 | 饱和时回退积分，抑制积分过冲 |
| ⑧ | 数值安全 / 鲁棒性 | `k_i != 0` 守卫等 | 防除零、参数缺省、向下兼容 |

> 说明：新增参数带默认值，既有调用（如 `init(0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f)`）无需修改即可继续编译。边界行为修正需结合实际对象重新验证。

---

## 1. 基础计算流程（读代码前必看）

`pid_calculate_callback()` 每 `d_t` 调用一次，核心步骤如下（摘自 `pid.cpp`）：

```cpp
term.error     = target - now;                 // 当前误差
term.abs_error = basic_math_abs(term.error);   // 误差绝对值

// ① 死区处理后得到“处理误差”
...

term.p_out = k_p * term.error;                 // ② P
// ③ I（含变速/分离/限幅）
// ④ D（含微分先行/低通）
term.f_out = k_f * (target - pre_target);      // ⑤ 前馈

out_unsaturated = term.p_out + term.i_out + term.d_out + term.f_out;
// ⑥ 输出限幅 → out，并记录 saturated
// ⑦ 抗饱和回退 → 修正 integral_error / term.i_out

pre_now = now; pre_target = target; pre_out = out; pre_error = term.error;
```

**⚙ 计算顺序（每帧固定执行次序，务必记牢）**

```
① 死区整形 error        → P 和标准 D 使用整形误差；F 使用原始目标变化量
② P = k_p·error
③ I = k_i·integral      → 判变速/分离，累加前后限幅，再算 i_out
④ D = 微分先行+低通     → 用 now - pre_now（上一帧真实测量）
⑤ F = k_f·(target-pre_target)
⑥ out = P+I+D+F → 限幅（记录 saturated）
⑦ 抗饱和：saturated 时按 CLAMP/BACK_CALC 回退 integral
⑧ 帧尾统一更新 pre_*    → pre_now / pre_target / pre_out / pre_error
```

> ⚠️ 顺序敏感点：**⑦ 抗饱和在 ⑥ 限幅之后、⑧ 状态更新之前**执行——所以回退用的是“本帧刚算出的 `integ_delta` 与 `saturated`”，而 `pre_error/pre_now` 记录的是**本帧最终值**，下一帧求导/判据才连续。

几个贯穿全文的关键量：

| 变量 | 含义 |
|---|---|
| `term.error` | 经死区“整形”后的误差（死区内为 0，死区外被削去一个 `dead_zone`） |
| `term.abs_error` | 整形后误差绝对值（供变速/分离判断） |
| `integral_error` | 积分器状态（误差对时间的累积，未乘 `k_i`） |
| `term.i_out` | 实际积分项输出 `= k_i * integral_error` |
| `out_unsaturated` | 限幅**前**的原始输出 `P+I+D+F` |
| `out` | 限幅**后**真正下发的输出 |

---

## 2. 优化①：死区处理（Dead Zone）

### 2.1 要解决什么问题

执行器/传感器存在静态摩擦、齿隙、量化噪声、电机最小起动死区。若误差在零点附近有几十个 LSB 的抖动，普通 PID 会让输出来回正负跳变：

- 电机“嗡嗡”抖动、发热、加速磨损；
- 每次微小反向都在给引入微分噪声
- 稳态时输出在正负之间高频抖振（chattering）。

死区的作用：**当误差足够小时认为“已到位”，不再动作**。

### 2.2 代码实现

```cpp
// 判断死区
if (term.abs_error <= dead_zone)
{
    term.error = 0.0f;
}
// 死区外：把误差“平移”掉一个死区宽度（对称削去）
else if (term.error > 0.0f && term.abs_error > dead_zone)
{
    term.error -= dead_zone;
}
else if (term.error < 0.0f && term.abs_error > dead_zone)
{
    term.error += dead_zone;
}
term.abs_error = basic_math_abs(term.error);
```

### 2.3 原理说明（两个动作）

1. **死区内及边界（`|error| <= dead_zone`）**：仅将本帧误差置 0，保留调用方目标。P 项归零，积分不再新增；历史积分、D 项和目标差分前馈仍可能输出，因此死区并不保证总输出为 0。
2. **死区外（`|error| > dead_zone`）**：不是简单“保留原误差”，而是**削去一个死区宽度**（正误差减、负误差加）。这是一个**连续化**处理：
   - 误差刚好跨过死区边界时，整形后误差从 0 平滑开始增长，避免在边界处输出“突跳一个死区宽度”造成的不连续；
   - 对称处理正负误差，保证正反向行为一致。

> 直观理解：这相当于把误差曲线在 ±dead_zone 上“开一个平底凹槽”，槽底为 0、槽外斜率仍是 1。既消除了零附近抖动，又不影响远处增益。

### 2.4 保留目标值

死区不改写 `target`，避免测量噪声污染目标历史，也避免调用方只设置一次目标时丢失指令。前馈始终使用调用方目标的帧间变化。

### 2.5 参数与整定

| 参数 | 单位 | 建议 |
|---|---|---|
| `dead_zone` | 与被控量同单位（rad、m/s…） | 取“传感器噪声峰峰值 + 机械静态误差”略大一点；过大→稳态误差变大、定位变“软”，过小→抖动抑制不明显 |

**权衡**：死区会引入**稳态误差带**（误差落在死区内不修正）。一般由外层位置/角度环或更高精度需求决定是否使用。速度环通常设很小或为 0，角度环可按编码器分辨率与机械间隙设定。

---

## 3. 优化②：变速积分（Variable-Speed Integral）

### 3.1 要解决什么问题

标准积分 `integral += error * d_t` 在误差很大时会**全速累积**，导致：

- 大阶跃时积分“冲过头” → 大超调、长调节时间；
- 退饱和慢（积分已经很大，要靠反号误差慢慢退）。

变速积分的思路：**误差大时让积分“踩刹车”，误差小时才全速积分**——越接近目标，积分越积极；越远离目标，积分越克制。

### 3.2 代码实现

```cpp
if (i_variable_speed_a == 0.0f && i_variable_speed_b == 0.0f)
{
    term.speed_ratio = 1.0f;                       // 未启用：始终全速
}
else
{
    if (term.abs_error <= i_variable_speed_a)      // 小误差：全速
        term.speed_ratio = 1.0f;
    else if (i_variable_speed_a < term.abs_error && term.abs_error < i_variable_speed_b)
        term.speed_ratio = (i_variable_speed_b - term.abs_error)     // 中间区：线性减速
                         / (i_variable_speed_b - i_variable_speed_a);
    else if (term.abs_error >= i_variable_speed_b) // 大误差：停止积分
        term.speed_ratio = 0.0f;
}
// 积分累加时乘以 speed_ratio
integ_delta      = term.speed_ratio * d_t * term.error;
integral_error  += integ_delta;
```

### 3.3 原理说明（分段线性权重）

定义区间 `[A, B] = [i_variable_speed_a, i_variable_speed_b]`，积分权重 `speed_ratio = f(|error|)`：

```
speed_ratio
  1.0 ┤────────●
      │         ╲
      │          ╲
      │           ╲
  0.0 ┤            ●────────
      └──────┬─────┬────────► |error|
             A     B
```

| 误差区间 | `speed_ratio` | 效果 |
|---|---|---|
| `|e| ≤ A` | 1.0 | 全速积分（正常收敛段，保证无稳态误差） |
| `A < |e| < B` | `(B-|e|)/(B-A)` | 线性递减（过渡段，平滑降速） |
| `|e| ≥ B` | 0 | 停止积分（大误差段，禁止盲目累积） |

数学上等价于把积分增量乘以一个**误差的连续分段函数权重**，比“全速/停止”的硬切换更平滑，不会引入突变。

### 3.4 参数与整定

| 参数 | 含义 | 建议 |
|---|---|---|
| `i_variable_speed_a` | 全速积分上界（内段阈值） | 设为“正常跟踪时的典型误差”，如接近稳态误差的 2~3 倍 |
| `i_variable_speed_b` | 停止积分下界（外段阈值） | 设为“可接受的大误差门限”，应 `B > A` |

- `B=0` 表示关闭变速积分，与 `A` 的值无关。
- 启用时必须满足 `0 <= A < B`；`A=B` 时中间段不可达，不会除零，但会退化成不连续门限，不属于有效配置。

### 3.5 与其它环节的关系

变速积分只缩放**积分增量**，`speed_ratio` 在 `integ_delta` 里生效，因此与抗饱和（后期回退 `integ_delta`）天然兼容——抗饱和回退的正是“已按 `speed_ratio` 缩放过的实际增量”。

---

## 4. 优化③：积分分离（Integral Separation）

### 4.1 要解决什么问题

当误差很大（如刚收到大阶跃目标、被外力推开很远）时，积分不应参与，否则会在“还没追上”的阶段疯狂累积，造成巨量超调。积分分离就是：**误差超过阈值时，直接把积分闭锁（清零）**，只用 P/D 快速把系统拉近目标；进入阈值内再恢复积分消除稳态误差。

> 与变速积分的区别：变速积分是“减速”，连续柔和；积分分离是“闭锁”，简单粗暴但更彻底。二者可二选一，也可叠加（下面说明）。

### 4.2 代码实现

```cpp
if (i_separate_threshold == 0.0f)
{
    // 未启用积分分离：正常积分
    integ_delta    = term.speed_ratio * d_t * term.error;
    integral_error += integ_delta;
    term.i_out     = k_i * integral_error;
}
else
{
    if (term.abs_error < i_separate_threshold)
    {
        // 阈值内：正常积分
        integ_delta    = term.speed_ratio * d_t * term.error;
        integral_error += integ_delta;
        term.i_out     = k_i * integral_error;
    }
    else
    {
        // 阈值外：闭锁并清零积分
        integral_error = 0.0f;
        integ_delta    = 0.0f;
        term.i_out     = 0.0f;
    }
}
```

### 4.3 原理说明

- `|error| < i_separate_threshold`：正常积分（与变速积分叠加时仍乘 `speed_ratio`）。
- `|error| ≥ i_separate_threshold`：`integral_error` **清零**、本步增量 `integ_delta = 0`、`term.i_out = 0`。等于“积分器被旁路”。

这样在大误差阶段输出只有 P（+D+F），系统快速逼近；一旦进入阈值带，积分重新从 0 开始积累，平稳消除静差。

### 4.4 参数与整定

| 参数 | 含义 | 建议 |
|---|---|---|
| `i_separate_threshold` | 积分分离阈值（正数），0 表示不启用 | 取“希望开始用积分消除静差时的误差量级”，通常为期望稳态精度的若干倍 |

**副作用**：阈值选择不当会有“积分突入”——误差刚跌破阈值时积分从 0 开始，可能产生小的二次爬升。可通过**变速积分**（在阈值附近让 `speed_ratio` 平滑过渡）来缓解，所以工程上常见“**积分分离 + 变速积分**”组合使用。

---

## 5. 优化④：积分项限幅（Integral Clamp）

### 5.1 要解决什么问题

即使有变速/分离，特殊工况（持续单向扰动、模型失配、长时间堵转）仍可能让 `integral_error` 越积越大。积分项限幅给积分**设一个绝对幅值上限**，是最基础的一道“兜底保险”。

### 5.2 代码实现

```cpp
// 积分限幅 (k_i==0 时积分项恒为0, 不做除法, 避免除零)
if (i_out_max != 0.0f && k_i != 0.0f)
{
    const float limit = basic_math_abs(i_out_max / k_i);
    basic_math_constrain(&integral_error, -limit, limit);
}
```

### 5.3 原理说明

限幅对象是**积分项输出 `term.i_out = k_i * integral_error`**，所以对 `integral_error` 的限制边界是 `± abs(i_out_max / k_i)`。累加前、累加后以及 BACK_CALC 修正后均应用限幅，保证当前帧不越限：

$$
-\left|\frac{i\_out\_max}{k_i}\right| \le integral\_error \le \left|\frac{i\_out\_max}{k_i}\right|
\quad\Longrightarrow\quad
|term.i\_out| = |k_i \cdot integral\_error| \le i\_out\_max
$$

`basic_math_constrain(&x, min, max)` 为**指针版限幅**，直接就地修改 `integral_error`（见 `basic_math.h`）。

### 5.4 参数与整定

| 参数 | 含义 | 建议 |
|---|---|---|
| `i_out_max` | 积分项输出绝对上限，0 表示不限制 | 一般取输出限幅 `out_max` 的 30%~70%；太小→静差消不掉，太大→起不到兜底作用 |

### 5.5 与抗饱和的分工

| | 积分项限幅 `i_out_max` | 抗积分饱和 `anti_windup` |
|---|---|---|
| 触发条件 | 无条件（每帧都对积分幅值裁剪） | 仅在**输出被限幅**时 |
| 作用 | 限制积分**绝对幅值**（静态上限） | 抑制积分**动态过冲**（按饱和反馈回退） |
| 关系 | **互补，建议同时保留** | 二者叠加更稳 |

---

## 6. 优化⑤：微分先行 + 微分一阶低通（Derivative on Measurement + Low-Pass）

这是本模块**最有价值**的优化之一。它实际上包含两个子优化，且**绑定在同一个开关 `d_first` 上**：

1. **微分先行（Derivative on Measurement）**：对**测量值 `now`** 求导，而不是对误差 `error` 求导。
2. **微分一阶低通滤波**：对微分结果做一阶低通，抑制噪声放大。

### 6.1 要解决什么问题：高频噪声放大与“微分爆炸”

**(a) 理想微分会无限放大高频噪声**

理想微分 `D(s) = K_d·s` 的幅值随频率**无限线性上升**（+20 dB/dec，永不封顶）——频率越高，微分项对噪声的放大越猛。而实际系统里噪声无处不在：

- **传感器噪声**：编码器量化、IMU/陀螺仪高频噪声、ADC 抖动；
- **控制链路噪声**：目标下发的抖动、通信抖动、上位机指令噪声。

而误差 `error = target - now` 里同时叠加了**当前值与目标值两路噪声**。对误差做微分时，这些高频噪声被 `K_d·s` 成倍放大，于是输出出现高频毛刺，电机/机构发出“**神秘的吱吱声**”，甚至**高频振动**；严重时整个闭环被噪声“顶”得抖振甚至失稳——这就是所谓的**微分爆炸（Derivative Explosion）**。

**(b) 目标突变带来的微分冲击（Derivative Kick）**

即便不考虑噪声，标准 D 项是 `d(error)/dt = d(target - now)/dt`。当**目标值突变**（如遥控器瞬间打杆、上层下发阶跃目标）时，误差瞬间跳变，微分会算出一个**巨大尖峰**，造成输出冲击、机械振动甚至打坏机构。

**解决思路分两步**：先**微分先行**消除目标跳变引起的尖峰，再**一阶低通**压住高频噪声放大。

### 6.2 代码实现

```cpp
if (d_first == PID_D_First_DISABLE)
{
    // 未启用：标准微分（对误差求导，无滤波）
    term.d_out = k_d * (term.error - pre_error) / d_t;
}
else
{
    // 微分先行 + 一阶低通
    float d_raw   = -(now - pre_now) / d_t;                       // 对测量值求导（注意负号：误差=目标-测量）
    float alpha_d = d_t * d_filter_n / (1.0f + d_t * d_filter_n); // 一阶低通系数
    d_filter_out  = alpha_d * d_raw + (1.0f - alpha_d) * d_filter_out;
    term.d_out    = k_d * d_filter_out;
}
```

### 6.3 原理说明

**微分先行**：误差 = `target - now`，其导数 = `d(target)/dt - d(now)/dt`。微分先行**丢掉 `d(target)/dt`**，只保留 `-d(now)/dt`：

$$
\text{d\_raw} = -\frac{now - pre\_now}{d_t} \approx -\dot{y}
$$

于是：
- 目标阶跃时 `d(target)` 为无穷大——标准微分会产生尖峰；微分先行的 `d_raw` 只反映**被控量的真实变化率**，目标突变不会引起微分跳变；
- 被控量平滑变化时，`d_raw` 提供正常的阻尼作用。

**一阶低通**：这是一个离散一阶 IIR 低通（指数加权滑动）。记 `T = d_t`、`N = d_filter_n`：

$$
y_k = \alpha\,x_k + (1-\alpha)\,y_{k-1},
\qquad
\alpha = \frac{T N}{1 + T N},\qquad 1-\alpha = \frac{1}{1 + T N}
$$

其中 `N = d_filter_n` 为截止角频率（rad/s），换算为 Hz 时使用 `f_c = N/(2π)`。它与连续域一阶低通 `N/(s+N)` 经后向差分离散化的结果一致：`α` 越大跟随越快、滤波越弱；`α` 越小滤波越强、相位滞后越大。

**等效传递函数：Kd 变成有界的一阶高通**

连续域一阶低通 `L(s) = N/(s+N)`（截止角频率 `ω_c = N`）串在微分后面，微分项的等效传递函数变为：

$$
D(s) = K_d \cdot s \cdot \frac{N}{s+N} = \frac{K_d N\, s}{s+N}
$$

这带来一个 **Amazing** 的结论：**加入一阶低通后，Kd 控制器从“理想微分 `K_d s`”变成了 `K_d N s / (s+N)`**——即一个**截止角频率为 `ω = N rad/s`、高频增益被钳在 `K_d N` 的一阶高通滤波器**。也就是说，此时的 Kd 不再是“频率越高放大越猛”的理想微分，而是一个**带限的一阶高通**。

**伯德图视角：噪声放大从“无界”变“有界”**

```
幅值 |dB|
        │        理想 Kd·s（+20dB/dec，永远上升）
        │      ╱
 Kd·N   │────●──────────────  滤波后 KdNs/(s+N)：高频趋平
        │   ╱
        │  ╱
        └──┴──────┬───────────► ω（对数）
                  N
```

| 频段 | 幅频特性 | 结论 |
|---|---|---|
| `ω < N` | 与理想微分重合（+20 dB/dec，相位 +90°） | **滤波 PID 与理想 PID 完全相同，性能一致** |
| `ω = N` | 转折点 | 截止角频率 `N` |
| `ω > N` | 幅值**趋于平坦**（≈ `K_d·N`），不再上升 | **最大微分增益为 `K_d·N`，噪声放大有界** |

一句话：**低频段保留理想微分的全部好处（阻尼、+90° 超前相位），高频段则“封顶”在 `K_d·N`，把噪声放大彻底限制住。**

### 6.4 参数与整定

| 参数 | 含义 | 建议 |
|---|---|---|
| `d_first` | `PID_D_First_ENABLE` 开启微分先行（同时启用低通）；`DISABLE` 为标准微分 | 建议**默认开启**（本仓库绝大多数环已使用 `PID_D_First_ENABLE`） |
| `d_filter_n` | 微分低通截止角频率 `N`（rad/s），默认 10 | 噪声大时降低，响应要求高时提高；若按 Hz 整定，先乘 `2π` |

- `N` 越大 → 转折点右移 → 越接近理想微分（阻尼强，但噪声放大更明显）；
- `N` 越小 → 越早“封顶” → 抗噪更好，但 D 的有效作用频段变窄、相位滞后变大。

### 6.5 注意点

- 开关 `d_first` **同时控制“微分先行”和“微分低通”**——本实现里微分低通只在微分先行分支里存在。若关闭微分先行，得到的是**无滤波的标准微分**（`k_d*(error-pre_error)/d_t`）。如需“标准微分 + 滤波”，需自行扩展。
- `d_filter_n` 只在 `d_first == ENABLE` 时生效；`k_d == 0` 时 D 项恒为 0。
- 微分先行用 `now - pre_now`，注意 `pre_now` 在帧尾才更新为 `now`，保证求导用的是“上一帧的真实测量值”。
- `d_filter_n` 的单位是 rad/s；本次仅纠正文档单位，不改变滤波公式与既有参数效果。

---

## 7. 优化⑥：前馈补偿（Feedforward）

### 7.1 要解决什么问题

纯反馈 PID 只能在“误差出现后”才动作，天生有相位滞后、跟随性不足。前馈利用**目标值的变化趋势**提前给出控制量，提升跟随性能、减小动态误差。

### 7.2 代码实现

```cpp
term.f_out = k_f * (target - pre_target);   // 目标变化量 * 前馈系数
```

### 7.3 原理说明

前馈项是**目标值的差分**（近似目标速度），乘系数 `k_f` 后直接叠加到输出：

$$
f_{out} = k_f \cdot (target_k - target_{k-1})
$$

其本质是“**一阶前馈 / 速度前馈**”：
- 目标静止时 `target - pre_target = 0`，前馈不输出，不影响稳态；
- 目标快速变化时，前馈提前给出一个与目标变化率成比例的控制量，帮助系统更快跟上。

### 7.4 参数与整定

| 参数 | 含义 | 建议 |
|---|---|---|
| `k_f` | 前馈系数，默认 0（关闭） | 从 0 逐步加大，直到跟随误差明显减小但不产生过冲；它是“开环补偿”，过大易过冲，通常配合目标斜坡（slope）使用 |

> 注：本实现的前馈是**目标差分前馈**（未除以 `d_t`），量纲与输出一致、直接叠加；若目标经斜坡发生器平滑过，效果最明显。

---

## 8. 优化⑦：抗积分饱和（Anti-Windup）

> 这是本模块相对原版**新增的核心优化**，下面详细展开。

### 8.1 要解决什么问题（windup 本质）

位置式 PID 的执行顺序是：

```
error → P → I(每帧 integral_error += d_t*error) → D → F → out = P+I+D+F → 限幅
```

问题：**积分器不知道输出已被限幅**。当执行器已到顶（力矩/速度到上限），误差仍在，`integral_error` 继续累加。等系统追上、误差反号时，这段“超额积分”要很久才退得回去，退的过程还在朝原方向猛推，于是：

- 明显**过冲**；
- **退饱和慢**、调节时间长；
- 严重时**来回振荡**（近极限环）。

一句话：**限幅作用在 `out` 上，却没有作用在 `integral_error` 上**。抗饱和就是补上这条反馈。

### 8.2 两个关键中间量

| 变量 | 含义 |
|---|---|
| `out_unsaturated` | 限幅**前**的原始输出 `P+I+D+F`（PID 的“意愿”） |
| `out` | 限幅**后**的实际输出（真正下发） |
| `saturated` | 本帧是否被削顶：`out != out_unsaturated` |
| `out_unsaturated - out` | “意愿被削掉的部分”，**符号 = 需求被堵住的方向** |
| `out - out_unsaturated` | 同上**反号**（反算公式使用） |

代码里先记录每步真正加进积分的增量 `integ_delta`，供回退使用：

```cpp
float integ_delta = 0.0f;       // 本步实际累加进积分的增量
...
integ_delta    = term.speed_ratio * d_t * term.error;
integral_error += integ_delta;   // 先记增量，再累加
```

历史输出段（当前上限取两者非零绝对值较小者，见 §15）：

```cpp
out_unsaturated = term.p_out + term.i_out + term.d_out + term.f_out;

float out_limit = basic_math_abs(out_max); // 实际输出硬限幅
float anti_windup_limit = (anti_windup_out_max != 0.0f)
    ? basic_math_abs(anti_windup_out_max) : out_limit; // 仅供抗饱和判定
out = out_unsaturated;

bool saturated = false;
if (out_limit != 0.0f)
{
    out = basic_math_constrain(out_unsaturated, -out_limit, out_limit);
    saturated = (out != out_unsaturated);          // 是否被削顶
}

if (saturated && anti_windup != PID_Anti_Windup_DISABLE && k_i != 0.0f)
{
    // CLAMP 或 BACK_CALC
}
```

### 8.3 方案一：条件积分（CLAMP）

以下为历史判据示意；当前实现改用 `k_i * integ_delta` 判断方向，并在恢复积分后重算输出，见 §15。

```cpp
if (anti_windup == PID_Anti_Windup_CLAMP)
{
    // 这一帧输出被限幅，且误差方向还在往限幅方向推 → 撤销本帧刚加进积分的量
    if (term.error * (out_unsaturated - out) > 0.0f)
    {
        integral_error -= integ_delta;
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

要点：**只撤“本帧增量”，不回滚历史积分** → 稳态所需积分不动，故不改稳态精度；误差一反向立即恢复积分，帮助快速脱离饱和。**零参数、最稳**。

### 8.4 方案二：反算（BACK_CALC）

以下展示反算公式；当前实现还会在修正后再次应用积分限幅，见 §15。

```cpp
else if (anti_windup == PID_Anti_Windup_BACK_CALC)
{
    // 反算: integral += (Kb/k_i) * (u_sat - u_unsat) * d_t
    integral_error += (anti_windup_gain / k_i) * (out - out_unsaturated) * d_t;
    term.i_out = k_i * integral_error;
}
```

$$
\Delta(\text{integral}) = \frac{K_b}{k_i}\,(u_{\text{sat}}-u_{\text{unsat}})\,\Delta t
$$

- `u_sat - u_unsat = out - out_unsaturated`：**被削掉的量（带符号，顶正上限为负）**。
- 顶正上限 → 该量为负 → 积分被往下拽；顶负下限 → 为正 → 往上拽。方向永远指向“把积分拉回到让输出刚好不饱和”。
- 物理意义：这是对积分状态的**一阶跟踪**，等效时间常数 `Tt = 1/Kb`。它是一个**自趋近负反馈**，因此退饱和非常利落、不顶力矩。

> ⚠️ **符号坑**：CLAMP 用 `out_unsaturated - out`（顶正上限为正），BACK_CALC 用 `out - out_unsaturated`（顶正上限为负）。两者是同一物理量的相反符号约定，改代码时别混。

### 8.5 CLAMP vs BACK_CALC 选型

| | CLAMP（条件积分） | BACK_CALC（反算） |
|---|---|---|
| 动作力度 | 一帧一帧“不加深” | 由 `Kb` 决定，可快可慢、连续 |
| 能否回滚历史积分 | ❌ 只挡新增 | ✅ 可主动拉回 |
| 需整定参数 | 无 | 需 `Kb` |
| 主要风险 | 长期饱和仍残留 → 有过冲 | `Kb` 过大/持续饱和过久 → **过度回退**，退饱和后欠补 |
| 适用 | 角度环、带持续负载的环 | 速度/力矩环、瞬态饱和 |

| 环路场景 | 建议 | 理由 |
|---|---|---|
| 角度环（`k_i > 0`，输出为速度指令） | `CLAMP` | 零参数、最稳；一般不常饱和 |
| 速度 / 力矩环（直接驱动力矩） | `BACK_CALC` | 退饱和快、不顶力矩 |
| 带持续恒定负载、需长期保持力矩 | `CLAMP`（或 `i_out_max` 兜底） | BACK_CALC 可能过度回退致稳态误差 |
| `k_i == 0` 的环 | 无需设置 | 本无 windup，代码有守卫 |

### 8.6 `Kb` 与跟踪时间常数

| `Kb` (1/s) | 5 | 10 | 12 | 20 | 30 |
|---|---|---|---|---|---|
| `Tt = 1/Kb` | 200 ms | 100 ms | 83 ms | 50 ms | 33 ms |

`Kb` 越大 → 回退越快 → 退饱和越利落，但过大易与积分/P 形成新动态（抖动）；过小则退得慢。建议 **10~15** 起步。

---

## 9. 优化⑧：数值安全与向下兼容（Robustness）

这些是“看不见但重要”的工程细节：

1. **积分限幅防除零**：`if (i_out_max != 0.0f && k_i != 0.0f)` —— 当 `k_i == 0`（纯 PD 环，如本仓库多个 `pid_omega` 的 `k_i=0`）时不做 `i_out_max / k_i`，避免除零；同时 `k_i=0` 时积分项本就恒为 0。
2. **抗饱和也守 `k_i != 0`**：`if (saturated && anti_windup != DISABLE && k_i != 0.0f)` —— 纯 PD 环无积分可回退，直接跳过。
3. **输出限幅可选**：`out_limit == 0` 视为“不限制”，`basic_math_constrain` 不会被调用（避免把输出强行拉到 0）。
4. **所有新增参数带默认值**：`init()` 末尾新增的 `anti_windup` 等 3 个形参均有默认值，**老调用无需修改**；`anti_windup` 默认 `DISABLE`，但本次修复的死区与积分限幅边界行为同样适用于此模式。
5. **成员就地初始化**：`pre_now / pre_target / pre_out / pre_error / out / out_unsaturated` 等均 `= 0.0f` 初始化，避免上电首帧读到垃圾值。
6. **量纲统一**：`f_out`、`i_out`、`p_out`、`d_out` 单位一致，直接相加得到 `out`；`i_out_max`、`out_max`、`anti_windup_out_max` 单位均与输出一致。
7. **接口完备**：提供 `get_out()`、`get_out_unsaturated()`、`get_integral_error()` 供观测，以及全套 `set_*()` 供在线整定。

---

## 10. 关键参数速查表

| 参数 | 单位 | 默认 | 作用 | 建议起手 |
|---|---|---|---|---|
| `k_p` | — | — | 比例增益 | 按对象调 |
| `k_i` | — | — | 积分增益 | 纯 PD 环设 0 |
| `k_d` | — | — | 微分增益 | 配合 `d_filter_n` |
| `k_f` | — | 0 | 前馈系数 | 0 起，逐步加 |
| `i_out_max` | 同输出 | 0（不限） | 积分项绝对幅值上限 | `out_max` 的 30%~70% |
| `out_max` | 同输出 | 0（不限） | 输出限幅 | 按执行器能力 |
| `d_t` | s | 0.001 | 控制周期 | 与控制回调周期一致 |
| `dead_zone` | 同被控量 | 0（关闭） | 死区宽度 | 噪声峰峰值略大 |
| `i_variable_speed_a` | 同误差 | 0（关闭） | 变速积分全速上界 | 正常误差的 2~3 倍 |
| `i_variable_speed_b` | 同误差 | 0（关闭） | 变速积分停止下界 | `> A`，大误差门限 |
| `i_separate_threshold` | 同误差 | 0（关闭） | 积分分离阈值 | 稳态精度的若干倍 |
| `d_first` | 枚举 | DISABLE | 微分先行 + 微分低通 | 建议 `ENABLE` |
| `d_filter_n` | rad/s | 10 | 微分低通截止角频率 | 噪声大调低，响应强调高 |
| `anti_windup` | 枚举 | DISABLE | 抗饱和方案 | 见 §8.6 |
| `anti_windup_gain`（`Kb`） | 1/s | 10 | 反算回退强度（仅 BACK_CALC） | 10~15 |
| `anti_windup_out_max` | 同输出 | 0（沿用 `out_max`） | 抗饱和跟踪阈值，不限制实际输出 | 设为积分器开始回算的输出边界 |

---

## 15. 迭代回归与闭环仿真（2026-11-17）

当前实现的顺序与约定：

- 死区使用连续的 `sign(e) * max(abs(e) - dead_zone, 0)`，不改写目标；变速积分及积分分离使用整形后的绝对误差。
- 积分累加前后都限幅，边界使用 `abs(i_out_max / k_i)`；BACK_CALC 更新后再次限幅。
- CLAMP 根据实际接纳的积分输出增量 `k_i * integ_delta` 判断是否加深饱和，恢复累加前状态后重新计算当前帧输出。积分分离清零不属于可回退的积分增量。
- BACK_CALC 的 `out_unsaturated` 与 `out` 保留本帧修正前的输出决策，修正后的积分用于下一帧；因此它们不一定等于修正后各项之和。
- 实际输出只由 `out_max` 硬限幅；`anti_windup_out_max` 仅在启用抗积分饱和时用于判定是否开始回算/条件积分，不改变下发输出。两项均为 0 时，输出不受 PID 内部限幅。
- `pid_clear()` 将误差、测量和目标历史对齐当前输入，避免输入不变时下一帧出现复位引起的 D/F 冲击。`init()` 仍只更新参数，不自动复位状态。
- 参数约定：输入与增益须有限，`d_t > 0`，死区和阈值非负，启用变速积分时 `B > A`，反算增益与滤波角频率非负。本次未新增运行时参数校验接口。

主机测试编译真实的 `pid.cpp`，使用仓库 CMSIS 头文件，不替换 PID 或数学辅助函数。从仓库根目录运行（需本机 `g++`，先确保 `build\tests` 存在）：

```powershell
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I User\Drive\algorithm\pid -I User\Drive\algorithm\basic_math -I Middlewares\Third_Party\DSP\Include -I Drivers\CMSIS\Include tests\pid_tests.cpp User\Drive\algorithm\pid\pid.cpp -o build\tests\pid_tests.exe
if ($LASTEXITCODE -eq 0) { & .\build\tests\pid_tests.exe }
```

测试覆盖死区正负边界、目标保持、整形误差、正负积分增益、当帧积分限幅、CLAMP 精确恢复及输出重算、积分分离与抗饱和交互、BACK_CALC 限幅、内外输出上限、复位以及微分先行。

闭环对象为一阶模型 `dy/dt = (u-y)/0.2`，周期 1 ms、执行器上限 ±1；先施加不可达目标 ±3 持续 3 s，再恢复至可达目标 ±0.4 持续 7 s。采用 `Kp=2, Ki=4, Kd=0.05, Kb=10`，开启微分先行，正负方向分别验证：

| 模式 | 恢复阶段累计绝对误差 IAE | 相对 DISABLE 降低 | 最终绝对误差 |
|---|---|---|---|
| DISABLE | 4.200226 | — | 作为对照，不要求收敛 |
| CLAMP | 0.147578 | 96.5% | 0.00000653 |
| BACK_CALC | 0.478668 | 88.6% | 0.00002101 |

结果：120025 项断言通过；固件通过 VS Code CMake Tools 构建和链接。仿真断言要求输出全程有限且不越限，启用抗饱和后恢复 IAE 至少降低 50%、最终误差小于 0.001。以上为软件闭环仿真，不代表实际云台对象；本次未烧录、未进行硬件闭环测试。

## 17. 最小修复落地（2026-11-18）

本轮只修复两项，接口和默认参数不变：

- `B=0` 关闭变速积分，即使 `A!=0` 也全速积分；启用时参数仍要求 `0<=A<B`。`A=B` 不会执行中间段除法，但不属于有效配置。
- CLAMP/BACK_CALC 使用 `tracking_out=clamp(out_unsaturated, ±anti_windup_limit)` 计算饱和反馈，不再误用硬限幅后的 `out`。实际输出仍只受 `out_max` 限制；`out_max=0` 时也可通过独立跟踪阈值进行抗饱和。

`tests/pid_tests.cpp` 为本轮聚焦回归测试，可按 §15 的主机命令编译运行；历史轮次的断言数量与仿真结果不是当前测试文件的输出。

本轮验证：588 项检查通过，覆盖正负误差、正负积分增益、独立/硬限幅组合、抗饱和开关、CLAMP 允许反向退积分以及变速积分分段；主机编译启用 `-Wall -Wextra -Werror -pedantic`，固件通过 CMake Tools 编译和链接。未烧录或做硬件闭环验证。

对前述建议的修正（优先于历史建议）：标准 BACK_CALC 允许积分抵消 P/D 引起的饱和，这是跟踪反馈的正常行为，不应仅凭产生反向积分而增加方向门控。CLAMP 保持条件积分语义，不改为投影历史积分；不强制 `i_out_max<=out_max`，以免破坏各项抵消时的可用范围。前馈量纲、首帧/复位语义、异常输入策略、采样调度和积分精度均未在本轮变更。非法参数仍须由调用方保证，不能静默替换为默认值。

### 17.1 闭环回归补充

本次只扩展测试与文档，没有继续修改 PID 实现。对象使用精确离散的一阶模型 `y_next=a*y+(1-a)*(u_applied+load)`，`a=exp(-dt/0.2)`；执行器实际将命令裁剪到 ±1，不能用 PID 的输出命令代替实际施加的控制量。参数 `Kp=2, Ki=4, Kd=0.05, Kb=10`，微分先行开启；不可达目标 ±3 持续 3 s，随后切换到 ±0.4 并恢复 7 s。负载用例在恢复阶段施加与目标方向相反的 0.2 恒定扰动，恢复目标仍可达。

覆盖 CLAMP/BACK_CALC、1/2 ms 周期、正负方向、有无负载，以及三种限幅配置：内部硬限幅 1；内部硬限幅 10 + 跟踪阈值 1；内部无限幅 + 跟踪阈值 1。后两种均保留下游真实执行器限幅。

| 模式 | 周期 | 无负载恢复 IAE | 有负载恢复 IAE |
|---|---|---|---|
| CLAMP | 1 ms | 0.147698 | 0.190980 |
| BACK_CALC | 1 ms | 0.478435 | 0.538755 |
| CLAMP | 2 ms | 0.146936 | 0.190698 |
| BACK_CALC | 2 ms | 0.477096 | 0.537388 |

总计 812 项检查全部通过，新增断言要求：输出和状态全程有限、硬限幅不越界、峰值积分状态不超过 2、恢复 IAE 比同场景 DISABLE 降低至少 50%、最终误差小于 0.001；内部/下游限幅的 IAE 差及正负方向差均不超过 1e-5；1/2 ms 周期的 IAE 差不超过 2%；`A>0,B=0` 与关闭变速积分的恢复 IAE 一致。所有闭环结果均为软件仿真，不能代替实际电机闭环验证。

## 16. 第二轮迭代：采样周期一致性、积分精度与抗饱和语义（2026-11-17）

在 §15 的基础上又追加了 8 组主机测试（`tests/pid_tests.cpp` 的 `iteration: ...` 章节），编译/运行命令与 §15 相同。本轮**未修改 `pid.cpp`**，只做黑盒测试与量化，结论如下。

### 16.1 验证结论：P/I/D/AW 与周期一致，前馈不一致

| 项目 | d_t=1 ms | d_t=2 ms | 结论 |
|---|---|---|---|
| P 项（阶跃 1） | 2.000000 | 2.000000 | 一致 |
| I 项（1 s 单位误差） | 0.999991 | 0.999994 | 一致（差 3e-6，仅 float 累加误差） |
| 标准 D（1 rad/s 测量斜坡） | -0.500000 | -0.500000 | 一致 |
| 微分先行（同上，滤波稳态） | -0.499976 | -0.499975 | 一致 |
| 饱和恢复 IAE（CLAMP） | 0.104445 | 0.103647 | 差 0.8% |
| 饱和恢复终值 | 0.389095 | 0.389130 | 一致 |
| **前馈（1 rad/s 目标斜坡）** | **0.002500** | **0.005000** | **随 d_t 线性缩放（2×）** |

闭环仿真使用**精确离散**的一阶对象 `y ← a·y + (1-a)·u, a = exp(-d_t/0.05)`，避免把对象积分误差误算到 PID 头上（用显式欧拉对象时 2 ms 采样会多出约 6% 的 IAE 偏差，那属于仿真误差）。

结论：除 `f_out` 外，`pid_class` 的所有通道都是**按秒**标定的，换采样率只需保持 `d_t` 与实际回调周期一致。`f_out = k_f·Δtarget` 是**逐帧**标定，换周期后物理意义改变。

### 16.2 新增问题（按严重度）

1. **变速积分 `i_variable_speed_b == 0` 与头文件契约冲突（真缺陷）**
   头文件写的是“变速区间, `0` 为不限制”，代码却把 `B == 0` 当成硬门限。
   闭环实测（`k_p=2, k_i=4, d_t=1 ms`，一阶对象 `τ=0.05`）：`A=0.05, B=0` 终值误差 **0.3333**（植物停在 0.667），`B=10` 时误差 3.6e-4。原因：`|e| > A` 时 `speed_ratio = 0`，积分被永久关断，退化为纯 P。
2. **`anti_windup_out_max` 曾在 `anti_windup == DISABLE` 时削输出**
   已修复：实际输出只由 `out_max` 硬限幅；`anti_windup_out_max` 仅在启用抗积分饱和时作为跟踪阈值，不再改变下发输出。测试覆盖抗饱和开/关及两种限幅值不同时的情形。
3. **积分状态是 `float`，大积分时有精度死区**
   `i_out_max=10, k_i=0.1`（积分上限 100）时，积分在 99 处 `ULP=7.63e-6`；误差 1e-3 的单帧增量 `d_t·e = 1e-6` 低于半个 ULP，1 s 内积分**完全不增加**（实测 delta=0）。等效误差死区 `0.5·ULP/d_t ≈ 3.8e-3`（约 0.22°），成为稳态误差下限。误差 0.05 时仍能正常累积（99 → 99.0534）。
4. **CLAMP 只能“阻止新增”，不能“回退已有”**
   预置积分 5、`out_max=1`、误差反向后：输出仍在**错误方向**保持饱和 **400 帧（0.4 s）**；同样条件下 BACK_CALC（`Kb=10`）为 161 帧。积分限幅不是万能解药：只要积分在开环/未饱和阶段已经涨过 `out_max/k_i`，CLAMP 就没有手段把它拉回来。
5. **微分先行对单帧跳变没有上界**
   首帧响应约为 `k_d·N·Δ`（与 `d_t` 无关）：`k_d=0.38, N=10` 时，0.05 rad 跳变产生 `d_out=-0.188`（out_max 的 6.3%），0.2 rad 跳变产生 `-0.752`（25.1%）。编码器单点跳变/通信丢帧会被直接放大成力矩尖峰。
6. **`i_out_max > out_max` 会成倍拉长退饱和时间**
   `i_out_max=5, out_max=1` 充 10 s 后积分到 5，误差反向后输出还要 5000 帧（5 s）才换向；`i_out_max=1` 时为 1000 帧（1 s）。积分上限应 ≤ 输出上限。
7. **量化反馈本身不会自激；死区的代价是 1:1 的稳态偏移**
   1 LSB=0.5 mrad、`k_p=1, k_i=2` 的环在 20 s 后收敛到**静止点**（输出峰峰值 0），残差 = `dead_zone + 0.25 mrad`：`dz=0 → 0.000249`、`3 LSB → 0.001750`、`20 LSB → 0.010249`、`40 LSB → 0.020249`。死区应按**噪声峰峰值**设，而不是按编码器 LSB 设。
8. **固件侧 `d_t` 是标称值，不是实测值**
   云台任务用 `osDelay(1)` 轮询（见 [gimbal.cpp](../../../APP/gimbal/gimbal.cpp:1341)，tick=1 kHz），实际周期 = 1 tick + 本拍执行时间，恒 > 1 ms，且随负载抖动。若本拍耗时 100 µs，实际约 909 Hz，等效 `k_i` 偏低约 9%、`k_d` 偏高约 10%。所有 PID 都以 `d_t=0.001` 初始化，因此整定值与实测周期绑定。仓库里规划器已经在测 `plan_dt_meas_s`，PID 没有对应通道。

### 16.3 优化建议

**P0（建议本轮修）**

1. 明确变速积分契约（二选一，并同步头文件注释）：
   ```cpp
   if (i_variable_speed_a == 0.0f && i_variable_speed_b == 0.0f) { ratio = 1.0f; }
   else if (i_variable_speed_b <= i_variable_speed_a)            { ratio = 1.0f; } // 非法配置=关闭
   else { /* 现有分段线性 */ }
   ```
   如果希望保留“B=0 表示不限制”，则 `B==0` 时应改成 `ratio = 1`（即从 A 开始就全速），不能是 `ratio = 0`。
2. `anti_windup_out_max` 仅用于抗饱和跟踪判定，不能代替执行器硬限幅；实际执行器上限应配置在 `out_max`。若下游还会施加更严格的限幅，可通过 `anti_windup_out_max` 让抗饱和按实际可用输出范围工作。
3. `init()`/`set_i_out_max()` 里把 `i_out_max` 约束到 `out_max`：`if (out_max != 0 && (i_out_max > out_max || i_out_max < 0)) i_out_max = out_max;`

**P1（整定前建议做）**

4. **积分状态升为 `double`**，或改为直接累积积分项输出：
   ```cpp
   double integral_error = 0.0;          // 内部累加用 double, 输出时转 float
   ```
   `get_integral_error()`/`set_integral_error()` 仍用 `float` 接口即可。H7 每拍多几十个周期，1 kHz 下可忽略。
5. **CLAMP 改为“边界钳位”**：饱和时不要整帧丢弃增量，而是把积分钳到“输出正好等于限幅”的边界：
   ```cpp
   // 仅在积分项本身把输出往限幅方向推时
   if (k_i * integral_error * (out_unsaturated - out) > 0.0f)
   {
       integral_error = (out_limit - (term.p_out + term.d_out + term.f_out)) / k_i;
       integral_error = basic_math_constrain(integral_error, -integral_limit, integral_limit);
   }
   ```
   这样既不会 stall（纯 I + 大 Ki），也能在误差反向后**立刻**退出饱和（解决 0.4 s 反向输出）。
6. **BACK_CALC 加方向门控 + 幅值限制**：只有积分项在推动饱和时才回算，避免 P/D/前馈造成的瞬时饱和把积分打穿（对应当前 `separation + BACK_CALC` 终值误差 0.276 与 `k_f` 尖峰问题）：
   ```cpp
   if (k_i * integral_error * (out - out_unsaturated) > 0.0f)
   {
       float delta = (anti_windup_gain / k_i) * (out - out_unsaturated) * d_t;
       delta = basic_math_constrain(delta, -0.5f * integral_limit, 0.5f * integral_limit); // Kb*d_t<=1
       integral_error += delta;
   }
   ```
7. **给微分项加上限** `d_out_max`（0=不限），直接把 `term.d_out` 钳位；配合调用方做单点跳变剔除。这比调低 `d_filter_n` 更直接，也不会牺牲正常频段的阻尼。
8. **补 `set_d_t()` 并用实测周期驱动**：用 `bsp_dwt` 计算上一拍周期（与 `plan_dt_meas_s` 同源），或把 `gimbal_motor_calculate()` 放进固定频率定时器回调。至少增加：
   ```cpp
   inline void set_d_t(const float &__d_t);      // d_t > 0
   inline void set_dead_zone(const float &__dz); // >= 0
   inline void set_d_first(const pid_d_first_e &__d_first);
   ```

**P2（清理与文档）**

9. `k_f` 建议删除或重定义为速率前馈 `k_f·(target-pre_target)/d_t`。当前参与编译的调用点 `k_f` 全为 0（前馈已由独立 `feedforward` 模块承担），但未参与编译的 [steering_wheel.cpp](../../../APP/steering_wheel/steering_wheel.cpp) 仍在用 `k_f=1.2/3.5`，说明这条通道仍有人依赖；保留逐帧差分形式只会在换周期、重新启用时留下隐患。
10. `pre_out` 只写不读，可直接删除；若要保留，用它实现输出变化率限制。
11. `init()` 末尾调用 `pid_clear()`（或增加 `clear` 参数），避免“重新 init 后旧积分/旧滤波状态继续生效”。
12. 积分分离当前是“清零”，跨阈值时输出会阶跃；如需平滑，改为“保持（冻结）”或按 `k` 衰减，并作为可选项暴露。

### 16.4 本轮结果

```
checks: 49166, failures: 0, findings: 30
RESULT: PASS (documented behavior and invariants hold)
```

30 条 FINDING 中，§16.2 的 8 条为本轮新增；其余为 §15 已记录的参数/用法风险。所有结论均为**主机软件仿真**，未烧录、未做硬件闭环；实车验证前请先按 §16.3 的 P0 修完并重新整定。

---

## 11. 使用示例

`init()` 形参顺序固定为：

```cpp
init(k_p, k_i, k_d, k_f, i_out_max, out_max, d_t,
     dead_zone, i_variable_speed_a, i_variable_speed_b, i_separate_threshold,
     d_first, anti_windup, anti_windup_gain, anti_windup_out_max);
```

### 11.1 角度环（带变速积分 + 积分分离 + 微分先行）——取自 `gimbal.cpp`

```cpp
// k_p=11.5, k_i=0.45, k_d=0.38, k_f=2.5, i_out_max=1.0, out_max=3.0,
// d_t=0.001, dead_zone=0, 变速积分 A/B=0.045/0.1, 积分分离=0.15, 微分先行
gimbal.rc_yaw_small_pid.pid_angle.init(
    11.5f, 0.45f, 0.38f, 2.5f, 1.0f, 3.0f, 0.001f,
    0.0f, 0.045f, 0.1f, 0.15f, PID_D_First_ENABLE);

// 角度环一般建议条件积分（零参数），按需在 init 后追加：
gimbal.rc_yaw_small_pid.pid_angle.set_anti_windup(PID_Anti_Windup_CLAMP);
```

### 11.2 速度环（纯 PD，`k_i=0`）——取自 `gimbal.cpp`

```cpp
// k_p=0.7, k_i=0（纯 PD，无 windup）, k_d=0.023, i_out_max=0.5, out_max=1.2,
// 变速积分 A/B=0.2/0.65, 积分分离=1.0（k_i=0 时积分恒 0，此处门限无实际作用）
gimbal.pc_yaw_small_pid.pid_omega.init(
    0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f, 0.001f,
    0.0f, 0.2f, 0.65f, 1.0f, PID_D_First_ENABLE);
// 纯 PD 环无需设置 anti_windup
```

### 11.3 力矩/速度环（带积分，需反算抗饱和）

```cpp
// init 后按老代码补丁方式开启 BACK_CALC
pid_omega.init(0.7f, 0.02f, 0.023f, 0.0f, 0.5f, 1.2f);
pid_omega.set_anti_windup(PID_Anti_Windup_BACK_CALC);
pid_omega.set_anti_windup_gain(12.0f);
pid_omega.set_anti_windup_out_max(YAW_SMALL_TORQUE_LIMIT);  // ★ 对齐下游真实限幅
```

> 规律：**`k_i!=0` 且可能长期饱和的环**才需要抗饱和；`k_i==0` 的纯 PD 环无需开启（代码有 `k_i!=0` 守卫）。

---

## 12. 整定流程（推荐顺序）

1. **基础环**：先关全部高级项（`dead_zone=0`、变速/分离=0、`anti_windup=DISABLE`），把 `k_p/k_i/k_d` 调到基本稳定。
2. **微分先行**：开启 `d_first=ENABLE`，调 `d_filter_n` 至无高频尖叫且阻尼足够。
3. **前馈**：若跟随滞后明显，加 `k_f`（配合目标斜坡）。
4. **积分整形**：
   - 大阶跃超调大 → 加**变速积分**（设 `A/B`）或**积分分离**（设阈值）；
   - 持续扰动下积分发散 → 加**积分项限幅** `i_out_max`。
5. **抗饱和**：
   - 观察 `get_out()` 与 `get_out_unsaturated()`，找长期饱和的环；
   - 角度环设 `CLAMP`；力矩/速度环设 `BACK_CALC` 且 `anti_windup_out_max` 填下游真实限幅，`Kb` 从 10 起；
   - 复现饱和工况，过冲大→加大 `Kb`；抖动→减小 `Kb`；退饱和后欠补→减小 `Kb` 或改 `CLAMP`。

---

## 13. 观测与调试

| 接口 | 含义 | 用途 |
|---|---|---|
| `get_out()` | 限幅后实际输出 | 下发的真实值 |
| `get_out_unsaturated()` | 限幅前原始输出 | 与 `get_out()` **持续不等 = 该环长期饱和**，据此决定是否开 AW 并核查 `anti_windup_out_max` |
| `get_integral_error()` | 积分状态 | 确认饱和期积分不再无界增长 |

建议把 `out`、`out_unsaturated`、`integral_error` 三者一起加入 Ozone 观察。开 AW 后应看到：饱和期 `integral_error` 不再单调增长、`out_unsaturated` 峰值明显下降。

---

## 14. 参考文献

- [PID库与PID基本优化（一） - WangHongxi - 博客园](https://www.cnblogs.com/WangHongxi/p/12404424.html)
- [知乎专栏 · PID 优化相关（p/226304120）](https://zhuanlan.zhihu.com/p/226304120)
- [知乎专栏 · PID 优化相关（p/562593474）](https://zhuanlan.zhihu.com/p/562593474)
- [博客园 · PID 相关（foxclever）](https://www.cnblogs.com/foxclever/p/9159677.html)

---

