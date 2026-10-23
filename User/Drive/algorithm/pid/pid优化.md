# PID 模块（pid_class）优化详解

> 本文按“每个优化环节”逐一展开：**要解决什么问题 → 原理与数学 → 代码实现 → 参数与整定 → 注意事项**。
> 说明：此代码是在**麻神代码和玺佬**的基础上进行优化和更新。

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
      └─ ⑥ out = P + I + D + F ──► 输出限幅（out_max / anti_windup_out_max）
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

> 说明：所有新增优化均**带默认值且关时不改变原有行为**，可安全合入既有工程。既有调用（如 `init(0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f)`）无需修改即可继续编译运行。

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
① 死区整形 error        → 让 P/D/F 都基于“整形后误差”
② P = k_p·error
③ I = k_i·integral      → 先判变速/分离/限幅，再累加，再算 i_out
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
if (term.abs_error < dead_zone)
{
    target = now;            // 把目标直接对齐到当前值
    term.error = 0.0f;       // 误差清零 → P/D/F 全为 0
    term.abs_error = 0.0f;
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
```

### 2.3 原理说明（两个动作）

1. **死区内（`|error| < dead_zone`）**：误差直接置 0，`target = now` 把目标值拉平到当前值。这样后续所有环节（P、D、前馈）都看到“误差=0”，输出自然为 0，彻底停止动作。
2. **死区外（`|error| > dead_zone`）**：不是简单“保留原误差”，而是**削去一个死区宽度**（正误差减、负误差加）。这是一个**连续化**处理：
   - 误差刚好跨过死区边界时，整形后误差从 0 平滑开始增长，避免在边界处输出“突跳一个死区宽度”造成的不连续；
   - 对称处理正负误差，保证正反向行为一致。

> 直观理解：这相当于把误差曲线在 ±dead_zone 上“开一个平底凹槽”，槽底为 0、槽外斜率仍是 1。既消除了零附近抖动，又不影响远处增益。

### 2.4 `target = now` 的深意

把 `target` 拉到 `now`，会让 `pre_target` 在帧尾也被更新为 `now`。这样**前馈项 `k_f*(target-pre_target)` 在死区内恒为 0**，不会因目标与实际的小偏差持续输出，进一步抑制抖振。

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

- 两者**同时为 0** 表示关闭变速积分（代码第一条分支）。
- 必须满足 `0 < A < B`；若 `A = B` 会导致中间段除零（`B-A=0`）。

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
    basic_math_constrain(&integral_error, -i_out_max / k_i, i_out_max / k_i);
}
```

### 5.3 原理说明

限幅对象是**积分项输出 `term.i_out = k_i * integral_error`**，所以对 `integral_error` 的限制边界是 `± i_out_max / k_i`：

$$
-\frac{i\_out\_max}{k_i} \le integral\_error \le \frac{i\_out\_max}{k_i}
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

其中 `N = d_filter_n` 为截止频率（Hz）。它与连续域一阶低通 `N/(s+N)` 经后向差分离散化的结果一致：`α` 越大（`N` 越高）跟随越快、滤波越弱；`α` 越小滤波越强、相位滞后越大。

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
| `d_filter_n` | 微分低通截止频率 `N`（Hz），默认 10 | 按噪声水平调：噪声大→降低（如 5~15Hz）；响应要求高→提高（如 30~50Hz）；过低会削弱 D 的阻尼作用 |

- `N` 越大 → 转折点右移 → 越接近理想微分（阻尼强，但噪声放大更明显）；
- `N` 越小 → 越早“封顶” → 抗噪更好，但 D 的有效作用频段变窄、相位滞后变大。

### 6.5 注意点

- 开关 `d_first` **同时控制“微分先行”和“微分低通”**——本实现里微分低通只在微分先行分支里存在。若关闭微分先行，得到的是**无滤波的标准微分**（`k_d*(error-pre_error)/d_t`）。如需“标准微分 + 滤波”，需自行扩展。
- `d_filter_n` 只在 `d_first == ENABLE` 时生效；`k_d == 0` 时 D 项恒为 0。
- 微分先行用 `now - pre_now`，注意 `pre_now` 在帧尾才更新为 `now`，保证求导用的是“上一帧的真实测量值”。
- 严格来说，代码把 `d_filter_n` 直接当作截止**角频率 `N`** 参与 `α = T·N/(1+T·N)` 计算；文档标注 Hz 属工程习惯，若需严格以 Hz 计，可按 `N = 2π·f_c` 换算理解。

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

输出段：

```cpp
out_unsaturated = term.p_out + term.i_out + term.d_out + term.f_out;

float out_limit = (anti_windup_out_max != 0.0f) ? anti_windup_out_max : out_max;
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
4. **所有新增参数带默认值**：`init()` 末尾新增的 `anti_windup` 等 3 个形参均有默认值，**老调用无需修改**；且 `anti_windup` 默认 `DISABLE`，不开启时新分支不执行，**输出与原实现逐位一致**。
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
| `d_filter_n` | Hz | 10 | 微分低通截止频率 | 噪声大调低，响应强调高 |
| `anti_windup` | 枚举 | DISABLE | 抗饱和方案 | 见 §8.6 |
| `anti_windup_gain`（`Kb`） | 1/s | 10 | 反算回退强度（仅 BACK_CALC） | 10~15 |
| `anti_windup_out_max` | 同输出 | 0（沿用 `out_max`） | 抗饱和判定用真实限幅 | ★=下游最后一级硬限幅 |

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

