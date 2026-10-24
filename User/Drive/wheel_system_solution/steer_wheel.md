# 舵轮轮系 (Steer Wheel)

> 对应类：`steer_wheel_class`（[wheel_system_solution.h](./wheel_system_solution.h) / [wheel_system_solution.cpp](./wheel_system_solution.cpp)）
> 通用定义：参考坐标系见 [总览 §3](./wheel_system_solution.md)、轮系定义见 [总览 §4](./wheel_system_solution.md)、执行流程见 [总览 §5](./wheel_system_solution.md)、接口设计原则见 [总览 §7](./wheel_system_solution.md)。

舵轮是三种轮系里唯一**不能解耦**的一种：逆解不仅要算出轮子该转多快，还要算出舵向该指哪个方向，并且方向的选择直接影响轮速的正负与有效性。因此本类在逆解之后串了一条**三级优化流水线**（就近转位 → 斜坡限幅 → 余弦优化），再叠加静止死区与全局限速。

输出分三组，用途不同：

| 输出 | 含义 | 谁用 |
|------|------|------|
| `solution_angle[]` | 纯几何解算的舵向角，单圈 `[-π,π)`，**不含零位** | 诊断、日志 |
| `solution_speed[]` | 未优化的轮向角速度（恒为正） | 诊断、日志 |
| `optimize_angle[]` | 最终舵向**多圈**目标角（含零位，与 `now_angle` 同框） | 舵向位置环 |
| `optimize_speed[]` | 最终轮向目标角速度（含方向符号） | 轮向速度环 |

---

## 1. 构型与几何

4 组"舵向电机 + 轮向电机"，轮心均匀分布在方位角

```
φ = [45°, 135°, 225°, 315°]     对应 [0]=FL [1]=RL [2]=RR [3]=FR
```

```
                     +X (车头 / 舵向零位)
                          ↑
        FL [0] φ=45°       │       FR [3] φ=315°
      ( d·cos45, d·sin45) │      ( d·cos(−45), d·sin(−45))
               ╲          │          ╱
                ╲         │         ╱
   +Y (左) ←──────────────┼──────────────→ −Y (右)
                ╱         │         ╲
               ╱          │          ╲
        RL [1] φ=135°     │      RR [2] φ=225°
```

| 下标 | 轮位 | φᵢ | sinφᵢ | cosφᵢ |
|------|------|-----|-------|-------|
| 0 | FL | 45° | +√2/2 | +√2/2 |
| 1 | RL | 135° | +√2/2 | −√2/2 |
| 2 | RR | 225° | −√2/2 | −√2/2 |
| 3 | FR | 315° | −√2/2 | +√2/2 |

两组常量必须同步：

```cpp
static constexpr float Wheel_Azimuth[4]     = { PI/4, 3PI/4, 5PI/4, 7PI/4 };
static constexpr float Wheel_Azimuth_Sin[4] = { SQRT_2_2,  SQRT_2_2, -SQRT_2_2, -SQRT_2_2 };
static constexpr float Wheel_Azimuth_Cos[4] = { SQRT_2_2, -SQRT_2_2, -SQRT_2_2,  SQRT_2_2 };
```

改了方位角（例如不是标准正方形布局）就要按新角度重算这两个 sin/cos 表，否则逆解的旋转分量和正解的力矩分量会一起错。

**编码器零位** `Enc_Zero_Angle[4]`（当前全 0）：舵向电机读数为 0 时，轮子实际指向与 +X 的偏差。逆解在把 `solution_angle` 变成下发目标时加它，正解/里程计在把 `now_angle`/`steer_angle` 变成几何角时减它。它是**编译期常量**：标定后改这个数组并重新编译，运行期不修改（因此同固件的所有实例共用同一组零位）。

---

## 2. 参数

| 参数 | 符号 | 默认值 | 说明 |
|------|------|--------|------|
| 驱动轮半径 | `R` | 0.0761 m（编译期） | 轮子线速度 = `w·R` |
| 轮心到底盘中心距离 | `dᵢ` | 0.239 m | 旋转力臂，**每轮可单独配置** |
| 舵向目标单拍最大变化量 | `target_max_step` | 0.20 rad/拍 | 斜坡限幅斜率，与调用频率耦合 |
| 轮向最大角速度 | `max_wheel_speed` | 25 rad/s | 超限时四轮等比缩放（≈1.90 m/s 线速度） |
| 静止判定阈值 | `min_limit` | 0.1 | `vx`/`vy`(m/s) 与 `vw`(rad/s) 都 ≤ 该值视为零指令 |
| 舵向误差死区 | `steer_angle_deadband` | 0.05 rad (≈2.9°) | 非小陀螺模式下真实未对齐角小于该值不再修舵 |
| 就近转位翻转迟滞 | `flip_hysteresis` | 0.1 rad (≈5.7°) | 已反转时需 \|误差\| < 90°−迟滞 才转回，正向时需 > 90°+迟滞 才反转 |
| 控制周期 | `T` | 0.001 s | 仅速度里程计使用 |
| 余弦优化开关 | `cosine_optimize_enable` | `true` | 关闭后见 §4.4 的联动 |
| 就近转位开关 | `nearest_optimize_enable` | `true` | 允许目标角加/减 π 并反转轮向 |
| 斜坡限幅开关 | `target_step_limit_enable` | `true` | 关闭后直接下发解算角 |
| 编码器里程计位移累积死区 | — | 1e-4 m | 累积到该值才积分（抖动互抵，慢速不丢） |
| 编码器里程计角度累积死区 | — | 1e-3 rad | 同上 |
| 速度里程计线/角死区 | — | 0.01 m/s / 0.05 rad/s | 速度口径，与调用频率无关 |

---

## 3. 逆运动学

### 3.1 推导

轮心位置 `rᵢ = dᵢ·(cosφᵢ, sinφᵢ)`，底盘角速度 `vw`，则轮心线速度

```
v_c,i = v_body + ω × rᵢ = ( vx − vw·dᵢ·sinφᵢ ,  vy + vw·dᵢ·cosφᵢ )
```

舵轮可以指向任意方向，所以不需要像全向轮/麦轮那样做投影——解算结果就是这个速度矢量的**极坐标**：

```
vel_x = vx − vw·dᵢ·sinφᵢ        (实现里是 target_speed.vw * Wheel_Azimuth_Sin[i] * dᵢ)
vel_y = vy + vw·dᵢ·cosφᵢ

|v|   = √(vel_x² + vel_y²)        (arm_sqrt_f32)
ωᵢ    = |v| / R                   → solution_speed[i]（恒 ≥ 0）
θᵢ*   = atan2f(vel_y, vel_x)      → solution_angle[i]（单圈 [-π,π)）
```

`|v| = 0` 时 `atan2f(0,0)` 无意义，实现里保持当前舵向角（不是清零），避免舵向在零速瞬间乱指；写入的是折到单圈、并扣除零位后的**几何角** `wrap(now_angle − 零位)`，与 `atan2f` 分支同框。

### 3.2 纯旋转校核

`vx = vy = 0, vw ≠ 0` 时：

```
|v| = |vw|·dᵢ ,   θᵢ* = φᵢ + 90°·sign(vw)
```

即四轮各自指向自己圆周的切线方向，速度大小与到中心距离成正比。用这个工况可以一次性校核方位角表与力臂是否正确。

---

## 4. 优化流水线

逆解只给出"几何上该指向哪、转多快"，下发前还要过 5 个处理环节（静止判定 → 就近转位 → 斜坡限幅 → 死区 → 余弦优化）与 2 个全局约束（方向合成、全局限速）。环节总览见 [总览 §5.4](./wheel_system_solution.md)，本节只讲各组参数的行为。

### 4.1 静止判定（`cmd_zero`）

```cpp
cmd_zero = (|vx| <= min_limit) && (|vy| <= min_limit) && (|vw| <= min_limit);
```

命中时逐轮锁定当前舵向（`dir = +1`、`error = 0`、`k = 1`），随后把 `solution_speed` 与 `optimize_speed` 一并清零。

因此"停车"的行为是**舵向原地保持、不做回正**。若希望停车后舵向回到某姿态，需上层继续发一个小的 `vw`/`vx`，或绕开本模块直接给舵向环目标。

### 4.2 就近转位（`nearest_optimize_enable`）

```cpp
e = wrap(target − now_angle, 2π);
flip_th = π/2 + (上一拍 dir < 0 ? −h : +h);            // h = flip_hysteresis
if (|e| > flip_th) { e = wrap(e + π, 2π); dir = −1; }  // 反向轮速的等效姿态
optimize_target = now_angle + e;
```

舵向**永远只转 ≤ 90°**：超过 90° 时改用"姿态差 180° + 轮子反转"的等效解，舵向调整时间减半，是所有舵轮底盘必开的功能。

**翻转迟滞**：判据不是硬性的 90°，而是在 90° 两侧各让开 `flip_hysteresis`。正向时超过 90°+h 才反转、已反转时小于 90°−h 才转回，因此目标角在 90° 附近抖动时不会逐拍翻转。实测（目标在 90° 附近摆动 ±0.05 rad + 编码器噪声 ±0.004 rad，2000 拍）：无迟滞翻转 1999 次（逐拍翻转），h = 0.05 rad 降到 623 次，h = 0.1 rad 时 0 次。无迟滞时的表现为轮速符号抖动、舵向小幅来回摆。

迟滞上限必须小于 90°：`h ≥ 90°` 时正向阈值 `≥ 180°`（永远触发不到）而反向阈值 `≤ 0°`（永远成立），方向被永久锁在初始分支，就近转位退化为"只认一侧"，因此 setter 直接拒绝。迟滞的代价上界只有 h 的转角——迟滞区间内舵向离解算方向 ≤ 90°+h，余弦系数 `|cos e|³` 已接近 0，本来就没有推力可丢。

`dir = −1` 会带到最终的 `optimize_speed`，故**下游必须原样使用 `optimize_speed` 的符号**；再取绝对值会导致反向行驶。关闭该开关时 `dir` 恒为 `+1`，大角度换向会退化为"大角度倒车"（见 §4.5 边界行为）。

### 4.3 斜坡限幅（`target_step_limit_enable`）

```cpp
prev = 上周期 optimize_angle；prev_valid = optimize_angle_valid[i];
if (!prev_valid || !isfinite(prev) || |prev − now_angle| > π) prev = now_angle;  // 首拍 / 失步后从反馈接管
target = clamp(desired, prev − target_max_step, prev + target_max_step);
```

把阶跃目标变成斜坡，抑制舵向电流尖峰与四轮合力突变。三个细节：

1. `prev` 取自上周期输出，故**逆解必须逐帧调用**，跳帧会让斜坡停顿一拍。
2. `optimize_angle_valid[]` 在 `init()` 后为 `false`：首拍直接从当前反馈接管，**不从 0 起跳**。早期实现首拍误把 `init()` 写入的 0 当作"上周期目标"，只要当前舵向离 0 不到 180° 就会整段跳过去（实测 `now = −1.0 rad`、`max_step = 0.02` 时首拍直接跳 1.0 rad，且引发 `dir` 翻转），这也是"上电猛一冲"的典型成因。
3. `|prev − now| > π` 视为失步（舵向被掰动、编码器回绕），直接从当前反馈接管，避免绕大圈追旧目标。

`target_max_step` 是**每拍**量，与调用频率强耦合：1 kHz 下 `0.20 rad/拍` 相当于 200 rad/s，几乎不限制；若舵向电机最大角速度约 20 rad/s，应设 `0.02`。

### 4.4 舵向死区（`steer_angle_deadband`）

命中条件：`cosine_optimize_enable` 为真 **且** `|真实未对齐角| < 死区` **且** `control_mode != 2`。命中时锁当前角、`error = 0`、`k = 1`（不修舵、不削减轮速）。

判据必须用**真实未对齐角**（解算目标 − 当前舵向，见 §4.6），不能用斜坡后的目标误差：斜坡每拍只放行一小步，用斜坡误差会让"正在大角度换向"被误判成"已对齐"，从而放行满轮速。

- **只在余弦开关为真时生效**：关掉余弦优化会连带失去防抖。若要"满轮速 + 防抖"，保留余弦开关并调小死区即可。
- `control_mode == 2`（小陀螺）跳过死区，否则舵向会被锁死无法跟随旋转。取值沿用 `chassis_mode_e`：`0` 无力、`1` 跟随、`2` 小陀螺。

### 4.5 余弦优化（`cosine_optimize_enable`）

```cpp
e_align = wrap(solution_angle[i] + 零位 − now_angle, 2π);        // 真实未对齐角
e_eff   = (dir < 0) ? wrap(e_align + π, 2π) : e_align;           // 折到所选方向分支
cos_use = nearest_optimize_enable ? |cos(e_eff)| : cos(e_eff);   // 就近转位后 |e_eff| ≤ 90°
k       = constrain(cos_use³, 0, 1);
```

物理含义：舵向未对齐时有效推力按 `cos` 衰减，`cos³` 进一步压低轮速，给舵向留出对齐时间，避免"边转边冲"。

**逐项含义**（`vx/vy/vw` 全零或 `solution_speed ≤ 0` 的分支不会走到这里，见 §4.1）：

| 量 | 表达式 | 含义 |
|----|--------|------|
| `solution_target` | `solution_angle[i] + Enc_Zero_Angle[i]` | 逆解舵向角折回编码器多圈坐标系，与 `now_angle[i]` 同域 |
| `e_align` | `wrap(solution_target − now_angle[i], 2π)` | **真实未对齐角**：当前舵向到解算目标的最短有符号角差，0 = 已对齐，±π = 正好反向 |
| `e_eff` | `dir < 0 ? wrap(e_align + π, 2π) : e_align` | **折算到所选方向分支的等效角**，即从当前舵向到该分支真实目标的未对齐量（未受斜坡限幅） |

其中 `wrap(x, 2π)` 即 `basic_math_modulus_normalization`，把 `x` 折到 `[−π, π)`。

**为什么 `dir = −1` 必须 `+π`**：舵轮的等效解不是"原地掉头 180°"，而是"舵向转小角度 + 轮子反转"。§4.2 返回的 `optimize_target = now + wrap(e_align + π)` 正是 `dir = −1` 时的真实舵向目标，减去 `now_angle` 即 `wrap(e_align + π)`。不折算的话，姿态差 170° 会被原样当成 `e_eff`：死区（§4.4）在反转分支里永远命中不了，遥测的 `error_angle[]` 也会偏 180°。

数值走查（`now_angle = 0`、`flip_hysteresis = 5°`）：

| `solution_target` | `e_align` | `dir` | 舵向实际目标 | `e_eff` | `k` |
|---|---|---|---|---|---|
| +30° | +30° | +1 | +30° | +30° | 0.650 |
| +170° | +170° | −1 | −10° | −10° | 0.955 |

第二行即"舵向只转 −10°，轮子反转以 95.5% 速度输出"，对应 `|e_align| > 90°+h` 触发的翻转。

**`+π` 折算对 `k` 是恒等的，但仍不能省**：`cos(e ± π) = −cos(e)`，而 `nearest = true` 时又对 `cos(e_eff)` 取了 `fabsf`，所以折与不折算出的 `k` 完全相同。保留折算的意义在别处——`error_angle[]` 的观测值与死区判据。反过来说，一旦去掉 `fabsf`，这段折算就成了 `k` 正确性的必需品。

**区间端点**：`wrap` 返回 `[−π, π)`，恰好 `+π` 会折成 `−π`，与 `basic_math_loop_constrain` 的闭区间 `[−π, π]` 语义不同。本处翻转阈值是 90°（±h），不受影响；若将来把该函数用于"恰好半个周期"的判据，需留意这个符号翻转。

**关闭就近转位时**（`nearest = false`）：`dir` 恒为 `+1`，`e_eff = e_align` 且不取绝对值，`|e_align| > 90°` 时 `cos³ < 0` 被 clamp 成 `0`，即"原地摆轮期间轮速为 0"（见下方边界行为）。

**误差源必须是真实未对齐角**：早期实现取的是 `planned_target − now_angle`（斜坡之后），斜坡每拍只推进 `target_max_step`，该误差恒很小，`k` 恒接近 1，削减形同虚设。实测（指令突变 90°、舵向仍停在 0、`target_max_step = 0.02`）：旧实现前 30 拍 `k = 0.999 → 0.539`，新实现全程 `k = 0.000`。若要让车辆"边转边走"，应调大 `target_max_step` 或直接关掉余弦开关，而不是靠误差源取巧。

| `e_eff` | 0° | 15° | 30° | 45° | 60° | 75° | 90° |
|-----|----|-----|-----|-----|-----|-----|-----|
| `k` | 1.000 | 0.902 | 0.650 | 0.354 | 0.125 | 0.017 | 0.000 |

**边界行为**：

| 开关组合 | `\|e\| > 90°` 时的 `k` | 结果 |
|----------|----------------------|------|
| `nearest = true` | `\|cos\|³ > 0` | 小角度摆轮，轮速平滑 |
| `nearest = false` | `cos³ < 0` → clamp 为 **0** | 先摆轮再走（原地转向期间轮速为 0） |

即"关闭就近转位"是用效率换舵向姿态稳定。若大角度换向时"卡住不动"，先看这个开关。

### 4.6 合成、限速与对齐判据

合成：`optimize_speed = solution_speed · k · dir`；`cmd_zero` 时在上述计算后统一清零，保证纯几何解也为 0。

全局限速：`max|optimize_speed| > max_wheel_speed` 时四轮按同一比例缩放（`solution_speed` 同步缩放，避免下游改读它绕过限速），不改变运动形态。

`error_angle[]` 记录的是**真实未对齐角** `e_eff`（公式与逐项含义见 §4.5；已折到所选方向分支，0 表示对齐；落在死区内时被清零），可以直接用来判断"是否已对齐"：

```
|error_angle[i]| < 阈值   ⇔   |wrap(solution_angle[i] + 零位 − now_angle[i], 2π)| < 阈值      (dir = +1)
```

`dir = −1` 时 `error_angle` 已经是折算到"反向轮速分支"的等效角，上式的右侧需同样加 π 折算。斜坡的剩余行程可用 `optimize_angle − now_angle` 观察，它才是"还要转多远"。

---

## 5. 正运动学

```cpp
steer_i = wrap(now_angle[i] − Enc_Zero_Angle[i], 2π);
u_i     = now_speed[i] · R;                        // 轮子线速度

vx = (1/4)·Σ u_i·cos(steer_i)
vy = (1/4)·Σ u_i·sin(steer_i)
vw = (1/4)·Σ u_i·sin(steer_i − φ_i) / d_i
```

推导：轮心速度 = 底盘速度 + 旋转速度，取轮子方向上的分量得 `u_i`，其中切向分量 `u_i·sin(θ_i−φ_i)` 全部来自底盘旋转（`= vw·d_i`），法向分量来自平移。因为 `Σsinφᵢ = Σcosφᵢ = 0`，四个轮子的平移项求和后旋转项恰好抵消，所以这组"1/4 平均"在**四轮数据相互一致**时是精确解，不需要矩阵求逆。

由此得到两个实用性结论：

1. 正常行驶（舵向已对齐）时 `robot_speed` 可信，可以直接用于速度里程计。
2. 舵向正在大幅调整的过渡期，四轮数据不自洽，`robot_speed` 是加权平均的估计值，此时速度里程计会有偏差——这也是编码器里程计更可靠的原因。

---

## 6. 里程计

### 6.1 编码器里程计

舵轮里程计比另外两种多一步：轮位移必须投影到**实测舵向角**上（不能用目标角）。

```
Δs_i   = (θᵢ − θᵢ_prev)·R                                // 轮向编码器差分
steer_i = wrap(steer_angle[i] − Enc_Zero_Angle[i], 2π)    // 舵向编码器实测角
dx  = (1/4)·Σ Δs_i·cos(steer_i)
dy  = (1/4)·Σ Δs_i·sin(steer_i)
dψ  = (1/4)·Σ Δs_i·sin(steer_i − φ_i)/d_i
```

死区是**累积量**而非单帧量：三轴累积不足线 1e-4 m / 角 1e-3 rad 时本帧不积分，位移留在 `odom_residual_*` 里，等后续帧攒够再连同历史位移一次积分。慢速运动因此不会被整帧丢掉（最多滞后 1 个死区量），零位噪声的 `+n / −n` 抖动则互相抵消、不会触发。任一舵向角或轮位移非有限时整帧跳过（`calc_body_displacement()` 会先把 `body_dx/dy/dyaw` 清零再返回）。

**两个舵向角来源不要混**：

| 字段 | 来源 | 用途 |
|------|------|------|
| `status.now_angle[]` | `update_now_state()` | 逆解/正解/死区判断（控制回路） |
| `steer_angle[]` | `update_odom_state()` | 里程计 |

两者理论上同源（同一个编码器），但入口不同、更新时机可以不同。若里程计方向正确而控制方向反了（或反之），第一件事就是核对这两路是否接了同一个量、符号是否一致。

### 6.2 两条航向路线与速度里程计

```cpp
wheel.encoder_odometry_callback();   // A：航向由 dψ 自持积分
wheel.imu_odometry_callback();       // B：位移由编码器算，航向直接取 IMU
wheel.speed_odometry_callback();     // 必须在 forward_calculate_callback() 之后
```

---

## 7. 调参指南

| 参数 | 调大 | 调小 | 建议起手值（1 kHz） |
|------|------|------|---------------------|
| `target_max_step` | 舵向响应快、电流尖峰大 | 舵向平滑、换向变慢 | 按舵向电机最大角速度 × `T` 取，如 20 rad/s × 1 ms = 0.02 |
| `max_wheel_speed` | 允许更高车速 | 更保守，容易触发等比缩放 | 按轮向电机额定角速度，如 25 rad/s ≈ 1.9 m/s |
| `min_limit` | 更容易判为"静止"，小指令被吞 | 静止时可能仍有细微爬行 | 0.05 ~ 0.1 |
| `steer_angle_deadband` | 抖动少、对准精度差 | 对准精度高、易抖 | 0.03 ~ 0.06 rad |
| `flip_hysteresis` | 边界更稳定、±90° 附近略偏"不反转" | 换向更灵、易在 90° 边界抖动 | 0.05 ~ 0.15 rad（须 < 90°） |
| `cosine_optimize_enable` | — | — | 保持 `true`（同时启用死区） |
| `nearest_optimize_enable` | — | — | 保持 `true`（换向更快） |
| `target_step_limit_enable` | — | — | 保持 `true`（保护舵向电机） |

联调顺序建议：

1. **纯旋转**（`vw` 单独给）：确认四轮方位角与力臂正确、`optimize_direction` 全为 `+1`。
2. **纯平移**（`vx` 或 `vy` 单独给）：确认 `optimize_angle` 收敛到 `φᵢ` 或 `φᵢ+180°`，`error_angle` 收敛到 0。
3. **对角线平移**：确认就近转位不会引起方向来回跳（若跳，减小 `steer_angle_deadband` 或检查 `min_limit`）。
4. **大角度换向**：观察 `cosine_optimize` 曲线是否平滑，`optimize_speed` 是否过零时无突变。
5. **全速工况**：确认全局限速没有被频繁触发（若频繁触发，`max_wheel_speed` 偏小）。

---

## 8. API 速查

| 分类 | 接口 | 说明 |
|------|------|------|
| 初始化 | `init(max_step, max_speed, min_limit, deadband, flip_hyst, T, cos_en, nearest_en, step_en)` | 非法值回退默认 |
| 配置 | `set_target_max_step()` / `set_max_wheel_speed()` | 必须 > 0 |
| 配置 | `set_min_limit()` / `set_steer_angle_deadband()` | 允许 0（关闭判定/死区） |
| 配置 | `set_flip_hysteresis(h)` | 就近转位翻转迟滞，须在 `[0, π/2)`，否则拒绝 |
| 配置 | `set_distance_to_core(i, d)` | 逐轮力臂，`d ≤ 0` 被拒绝 |
| 配置 | `set_cosine_optimize_enable()` / `set_nearest_optimize_enable()` / `set_target_step_limit_enable()` | 无条件写入 |
| 配置 | `set_control_period(T)` | 仅速度里程计 |
| 输入 | `update_robot_state({vx,vy,vw}, control_mode)` | 速度非法时**仍然**更新 `control_mode` |
| 输入 | `update_now_state({now_angle}, {now_speed})` | 逐通道独立校验 |
| 输入 | `update_odom_state({θ}, {舵向角}, imu_yaw)` | 轮向角 + 舵向角整帧有效才提交；IMU 航向独立校验 |
| 解算 | `inverse_calculate_callback()` | 逆解 + 三级优化 + 限速 |
| 解算 | `forward_calculate_callback()` | 正解 → `robot_speed` |
| 解算 | `encoder_odometry_callback()` / `imu_odometry_callback()` | 编码器里程计 |
| 解算 | `speed_odometry_callback()` | 速度里程计 |
| 输出 | `get_solution_angle(i)` / `get_solution_speed(i)` | 纯几何解，诊断用 |
| 输出 | `get_optimize_angle(i)` / `get_optimize_speed(i)` | **下发电机用** |
| 输出 | `get_optimize_direction(i)` | ±1，已包含在 `optimize_speed` 里 |
| 输出 | `get_error_angle(i)` / `get_cosine_optimize(i)` | 真实未对齐角 / 削减系数，诊断用 |
| 输出 | `get_robot_speed_vx/vy/vw()` | 正解速度 |
| 输出 | `get_position_x/y/yaw()` / `get_speed_position_x/y/yaw()` | 两套里程计位姿 |
| 复位 | `reset_position()` / `set_position()` / `reset_speed_position()` / `set_speed_position()` | 同上 |

---

## 9. 调用示例

```cpp
static steer_wheel_class chassis_wheel;

void chassis_init(void)
{
    chassis_wheel.init(/*max_step*/ 0.02f,
                       /*max_speed*/ 25.0f,
                       /*min_limit*/ 0.1f,
                       /*deadband*/ 0.05f,
                       /*flip_hyst*/ 0.1f,
                       /*period*/ 0.001f,
                       /*cosine*/ true,
                       /*nearest*/ true,
                       /*step_limit*/ true);

    // 非等距布局时逐轮设定力臂
    chassis_wheel.set_distance_to_core(0, 0.239f);
    chassis_wheel.set_distance_to_core(1, 0.239f);
    chassis_wheel.set_distance_to_core(2, 0.239f);
    chassis_wheel.set_distance_to_core(3, 0.239f);
}

void chassis_task(void)                       // 1 kHz
{
    // ① 反馈
    steer_angle_t now_angle{};
    wheel_speed_t now_speed{};
    for (uint8_t i = 0; i < 4; i++)
    {
        now_angle.angle[i]     = steer_motor[i].get_total_angle();
        now_speed.wheel_speed[i] = drive_motor[i].get_speed();
    }
    chassis_wheel.update_now_state(now_angle, now_speed);

    wheel_angle_t wheel_angle{};
    steer_angle_t steer_angle{};
    for (uint8_t i = 0; i < 4; i++)
    {
        wheel_angle.angle[i] = drive_motor[i].get_total_angle();
        steer_angle.angle[i] = steer_motor[i].get_total_angle();
    }
    chassis_wheel.update_odom_state(wheel_angle, steer_angle, imu.get_yaw());

    // ② 正解 + 里程计
    chassis_wheel.forward_calculate_callback();
    chassis_wheel.speed_odometry_callback();
    chassis_wheel.imu_odometry_callback();

    // ③ 逆解（control_mode 取 chassis_mode_e）
    chassis_wheel.update_robot_state({vx_target, vy_target, vw_target}, now_mode);
    chassis_wheel.inverse_calculate_callback();

    // ④ 下发：舵向位置环用 optimize_angle，轮向速度环用 optimize_speed
    for (uint8_t i = 0; i < 4; i++)
    {
        steer_motor[i].set_angle_ref(chassis_wheel.get_optimize_angle(i));
        drive_motor[i].set_speed_ref(chassis_wheel.get_optimize_speed(i));
    }
}
```

### 9.1 下发映射

| 下游 | 取哪个接口 | 说明 |
|------|------------|------|
| 舵向位置环 | `get_optimize_angle(i)` | **多圈**、含零位、已做就近转位 + 斜坡限幅 |
| 轮向速度环 | `get_optimize_speed(i)` | 已含余弦削减与方向符号（±1） |
| 对齐判定 | `get_error_angle(i)` | `abs(error_angle) < 阈值` 即已对齐（已折到所选方向分支、死区内清零） |
| 诊断 / 日志 | `get_solution_*`、`get_error_angle`、`get_cosine_optimize`、`get_optimize_direction` | 只读，不下发 |
| 底盘速度反馈 | `get_robot_speed_vx/vy/vw()` | 舵向大幅调整的过渡期只是估计值 |

> 不要用 `solution_angle` / `solution_speed` 下发：前者未加零位、未做就近转位与斜坡限幅，后者未做余弦削减、方向翻转与全局限速。
>
> 与无舵向轮系不同，舵轮的反馈与解算字段是**两组独立成员**，不存在"反馈被指令覆盖"的问题；但仍建议按 [总览 §5.2](./wheel_system_solution.md) 的编排顺序调用，便于三种轮系共用同一份 chassis 代码。

---

## 10. 标定与排查

### 10.1 标定顺序

1. **编码器零位**：手动把四轮摆到指向 `+X`，读舵向编码器值写入 `Enc_Zero_Angle[4]` 常量并重新编译（约定"编码器角 = 几何角 + 零位"，逆解加、正解减）。
2. **力臂**：原地 `vw = 1 rad/s` 转 2π，航向超转 → 力臂偏小。
3. **方位角表**：单独给 `vx`，四轮目标角应收敛到 45°/135°/225°/315°。都偏同一角度 → 零位问题；两两偏不同角度 → 方位角表问题。
4. **轮速上限**：给一个明显超限的车速，确认四轮等比缩小且比例一致。
5. **斜坡 / 死区**：从 §7 起手值出发，按"抖动 ↔ 响应"折中微调。

### 10.2 常见问题（舵轮特有）

| 现象 | 原因 | 处理 |
|------|------|------|
| 四轮指向成 90° 交叉（本该对边平行） | 方位角表与 sin/cos 表不一致，或某轮电机方向反 | 先核对常量，再逐轮点动 |
| 转向呈"风车"状 | 方位角符号约定反（φ 应按 CCW 正） | 检查方位角递增顺序 |
| 停车后舵向自己摆动 | 未命中静止判定（`min_limit` 太小） | 提高 `min_limit`，或检查上位是否在停车时发噪声 |
| 小速度时舵向原地抖 | 死区太小，或余弦开关被关 | 提高死区；保持余弦开关为真 |
| 慢速爬行（应停未停） | `min_limit` 与有效小指令量级重叠 | 在 0.05 ~ 0.1 之间二分 |
| 大角度换向"卡住不走" | 就近转位关闭且 `\|e\| > 90°`，余弦项把轮速压到 0 | 打开就近转位，或接受"先摆轮后走" |
| 换向时车尾甩 | 斜坡步长太大，四轮不同步 | 减小 `target_max_step` |
| 舵向在 ±90° 附近逐拍换向、轮速符号跟着抖 | 翻转迟滞为 0 | `set_flip_hysteresis(0.05 ~ 0.15)` |
| 上电后舵向猛冲一下、`dir` 翻一次 | 斜坡首拍把 `init()` 的 0 当基准（旧实现） | code 1.9 起首拍从当前反馈接管，确认 `optimize_angle_valid` 已复位 |
| 直线好、一转向就偏 | 里程计用了目标舵向角而非实测角 | 核对里程计输入的舵向角是否为真实反馈 |
| 里程计方向对但控制反（或反之） | 控制路与里程计路的舵向角不同源 | 两路必须来自同一编码器、同一符号 |
| 转速上不去 | 全局限速被触发 | 观察最大轮速目标，必要时提高上限 |
| 高速时车辆抖动 | 舵向无法跟随（斜坡 / 死区 / 电机带宽） | 提高舵向电机带宽，减小死区 |

通用现象见 [总览 §8.2](./wheel_system_solution.md) 联调检查表。
