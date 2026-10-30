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
| `force_direction[]` | 轮端出力方向 ±1（力域，只在力控/混控时有效） | 诊断、日志 |

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

| 下标 | 轮位 | 轮心坐标 (xᵢ, yᵢ) | 手性 |
|------|------|------------------|------|
| 0 | FL | (+Lx, +Ly) | 前左 |
| 1 | RL | (−Lx, +Ly) | 后左 |
| 2 | RR | (−Lx, −Ly) | 后右 |
| 3 | FR | (+Lx, −Ly) | 前右 |

单一事实来源是**半轴距/半轮距** `Lx`/`Ly`（`Wheel_Projection_X/Y`），轮心坐标与符号由它们给出：

```cpp
static constexpr float Wheel_Projection_X = 0.12f;   // Lx: 前后半轴距
static constexpr float Wheel_Projection_Y = 0.12f;   // Ly: 左右半轮距
static constexpr float Wheel_Sign_X[4] = { 1.0f, -1.0f, -1.0f,  1.0f};
static constexpr float Wheel_Sign_Y[4] = { 1.0f,  1.0f, -1.0f, -1.0f};
static constexpr float Wheel_D2 = Lx² + Ly²;         // d², 四轮相同
```

不再单独维护 `Wheel_Azimuth[4]` 角度表，也不再维护 `d` 与单位方向向量表：轮心坐标直接就是 `(±Lx, ±Ly)`，角速度公式只用到 `d²`，**正方形与长方形布局同一套公式**。逆解耦合项、正解与里程计的切向投影只认这两组数，手性由符号表直接表达。

> ⚠️ **历史坑（v1.2 及以前）**：旧实现用 `d·(+sinφ, +cosφ)` 作耦合项，理由是"舵向编码器正向与几何 CCW 相反"。这个推理不成立——`v + ω×r` 由底盘刚体几何决定，与编码器读数方向无关；编码器反向只该影响"编码器角 ↔ 几何角"的换算（`几何 = −(读数 − 零位)`、`目标读数 = 零位 − 解算角`），不会把旋转场变成镜像场。`(ωsinφ, ωcosφ)` 是散度与旋度都为 0 的剪切场，纯自旋时四轮推力两两对消，表现为**小陀螺憋死不转**。详见 §3.1。

**编码器零位与方向** `Enc_Zero_Angle[4]`（已按实车逐轮标定，见头文件当前值）：舵向电机读数为 `Enc_Zero_Angle[i]` 时，轮子指向 `+X`。它是**编译期常量**：标定后改这个数组并重新编译，运行期不修改（因此同固件的所有实例共用同一组零位）。

方向由 `steer_encoder_direction`（`set_steer_encoder_direction(±1)`）单独表达，两者合成一条换算：

```
几何角 = dir · (读数 − Enc_Zero_Angle)        下发读数 = Enc_Zero_Angle + dir · 几何角
```

`dir = +1` 表示电机读数正向与几何 CCW 同向，`-1` 表示反向。**编码器反向时只改这一个量**——逆解、正解、里程计和舵向目标会自动一起保持一致。绝不要去翻逆解的耦合项：那会把旋转场变成镜像剪切场，小陀螺会原地憋死不转（见 §3.1）。

> **本车配置**：`chassis_init()` 里通过 `chassis.steer_wheel.init(Wheel_Speed_Control_Method, -1.0f)` 的**第二参**指定极性为 −1（实车舵向编码器反向），首参是控制方式。**上层不要再对 `vy`/`vw` 取反**——那是旧镜像逆解时代的补丁，与正确的运动学叠加会变成双重镜像。
>
> **上机首检**：只给 `vw > 0`（`vx = vy = 0`）。四轮应为风车姿态**且车体原地逆时针转**。若车体顺时针转、或平移左右反了，就是 `dir` 与实际极性不符，改这一个量即可。`dir` 错误不会报任何错——逆解/正解/里程计三者依然自洽，只是整台车镜像；而**里程计的 `vy` 与 `yaw` 会随之反向**，一旦接入闭环就会发散。

---

## 2. 参数

| 参数 | 符号 | 默认值 | 说明 |
|------|------|--------|------|
| 驱动轮半径 | `R` | 0.0761 m（编译期） | 轮子线速度 = `w·R` |
| 前后半轴距 | `Lx` | 0.12 m（编译期） | `Wheel_Projection_X`；轮心到中心线的**前半**距离，不是整轴距。正方形/长方形布局只改这里与 `Ly` |
| 左右半轮距 | `Ly` | 0.12 m（编译期） | `Wheel_Projection_Y`；同上，左右方向的半距 |
| 轮心径向距离 | `d` | √(Lx²+Ly²)（推导） | 不单独维护：代码里只用 `Wheel_D2 = Lx²+Ly²`，角速度公式无需开方 |
| 底盘等效质量 | `m` | 10.0 kg（编译期） | 改 `Chassis_Mass`，换配重后重编（仅力控/混控用） |
| 绕 Z 轴等效转动惯量 | `J` | 0.35 kg·m²（编译期） | 改 `Chassis_Inertia`（仅力控/混控用） |
| 舵向目标单拍最大变化量 | `target_max_step` | 0.20 rad/拍 | 斜坡限幅斜率，与调用频率耦合 |
| 轮向最大角速度 | `max_wheel_speed` | 25 rad/s | 超限时四轮等比缩放（≈1.90 m/s 线速度） |
| 静止判定阈值 | `min_limit` | 0.1 | `vx`/`vy`(m/s) 与 `vw`(rad/s) 都 ≤ 该值视为零指令 |
| 舵向误差死区 | `steer_angle_deadband` | 0.05 rad (≈2.9°) | 非小陀螺模式下真实未对齐角小于该值不再修舵 |
| 就近转位翻转迟滞 | `flip_hysteresis` | 0.1 rad (≈5.7°) | 已反转时需 \|误差\| < 90°−迟滞 才转回，正向时需 > 90°+迟滞 才反转 |
| 控制周期 | `T` | 0.001 s | 仅速度里程计使用 |
| 余弦优化开关 | `cosine_optimize_enable` | `true` | **速控与力控共用**；关闭后两条路径都不按 `cos³e` 削减出力，见 §4.4 的联动 |
| 就近转位开关 | `nearest_optimize_enable` | `true` | **速控与力控共用**；打开时目标角可加/减 π 并反转方向，关闭时只转解算（力）方向本体、`dir` 恒 +1 |
| 斜坡限幅开关 | `target_step_limit_enable` | `true` | **速控与力控共用**；关闭后两条路径的舵向目标都直接跟随
| 舵向编码器方向 | `steer_encoder_direction` | +1 | +1 = 读数正向与几何 CCW 同向，-1 = 反向；由 `init()` 第二参指定（非法值按 +1），运行期可用 setter 覆盖 |
| 编码器里程计位移累积死区 | — | 1e-4 m | 累积到该值才积分（抖动互抵，慢速不丢） |
| 编码器里程计角度累积死区 | — | 1e-3 rad | 同上 |
| 速度里程计线/角死区 | — | 0.01 m/s / 0.05 rad/s | 速度口径，与调用频率无关 |

---

## 3. 逆运动学

### 3.1 推导

轮心位置 `rᵢ = (xᵢ, yᵢ) = (±Lx, ±Ly)`（`[0]=FL(+,+) [1]=RL(−,+) [2]=RR(−,−) [3]=FR(+,−)`），底盘角速度 `vw`，右手系下轮心线速度为 `v_body + ω × rᵢ`：

```
vel_x = vx − vw·yᵢ        // 实现: target_speed.vx - target_speed.vw * Wheel_Sign_Y[i] * Wheel_Projection_Y
vel_y = vy + vw·xᵢ        // 实现: target_speed.vy + target_speed.vw * Wheel_Sign_X[i] * Wheel_Projection_X
```

坐标直接是半轴距/半轮距，**正方形与长方形布局同一式**；径向距离 `d = √(Lx²+Ly²)` 只在正解/力分配里以 `d²` 形式出现，不必开方。四轮对称布置保证 `Σxᵢ = Σyᵢ = 0`。

> ⚠️ **`vel_x` 的 `−` 不能改成 `+`。** 这是刚体运动学的固有形式，与舵向编码器读数方向无关。若舵向编码器正向与几何 CCW 相反，正确做法是在"编码器角 ↔ 几何角"换算处取负（`几何 = −(读数 − 零位)`、`目标读数 = 零位 − 解算角`），而不是把耦合项翻成 `d·(+sinφ, +cosφ)`——后者是散度与旋度都为 0 的**镜像剪切场**，不是旋转场，纯自旋时四轮推力两两对消，车原地不动而电流顶满。

舵轮可以指向任意方向，所以不需要像全向轮/麦轮那样做投影——解算结果就是这个速度矢量的**极坐标**：

```
|v|   = √(vel_x² + vel_y²)        (arm_sqrt_f32)
ωᵢ    = |v| / R                   → solution_speed[i]（恒 ≥ 0）
θᵢ*   = atan2f(vel_y, vel_x)      → solution_angle[i]（单圈 [-π,π)）
```

> **耦合项符号是配套约定，不能单改一侧。** 正解与编码器里程计对应写 `uᵢ·(sinθᵢ·xᵢ − cosθᵢ·yᵢ)/d²`（见 §5、§6）；若逆解用 `(vx − vw·yᵢ, vy + vw·xᵢ)`、正解却把 `−yᵢ` 的符号翻掉，纯自旋时四轮贡献正负抵消，实测 `vw` 与里程计 `dyaw` 会恒为 0。

`|v| = 0` 时 `atan2f(0,0)` 无意义，实现里保持当前舵向角（不是清零），避免舵向在零速瞬间乱指；写入的是折到单圈、并扣除零位后的**几何角** `wrap(now_angle − 零位)`，与 `atan2f` 分支同框。

### 3.2 纯旋转校核

`vx = vy = 0, vw ≠ 0` 时，代入 `vel_x = −vw·yᵢ`、`vel_y = vw·xᵢ` 得：

```
|v| = |vw|·d ,   θᵢ* = atan2(xᵢ, −yᵢ)   (+180° when vw < 0)
```

即四轮各自指向**自己圆周的切线方向**（切向单位向量 `tᵢ = (−yᵢ, xᵢ)/d`），速度大小与到中心距离成正比，四轮合力矩全部同号，车体绕中心 CCW 旋转。

正方形布局（`Lx = Ly`）时四轮方位角为 `45°/135°/225°/315°`，`vw > 0` 下解算角为 `{135°, 225°, 315°, 45°}`（等价写作 `{135°, −135°, −45°, 45°}`），即**标准“风车”姿态**：四轮严格垂直于各自半径、沿逆时针环绕。

> **台架最快判定**：只给 `vw > 0`（`vx = vy = 0`）。正确实现四轮应呈上述风车形且车体原地逆时针转；若看到 `{45°, −45°, −135°, 135°}`（每轮沿对角线朝外/朝内）且车体不动，就是耦合项镜像了。**只测直行（`vx > 0`）查不出这个问题**——纯平移时两种写法给出的解完全相同。

### 3.3 力控（目标加速度 → 舵向目标 + 轮端力矩）

输入是**底盘目标加速度**（`ax/ay` 单位 m/s²，`aw` 单位 rad/s²），模块内部按 `F = m·a`、`M = J·α` 换算成等效力/力矩，因此质量与惯量是本模块的配置量（编译期常量 `Chassis_Mass` / `Chassis_Inertia`），换配重改常量后重新编译，不动上层。

舵轮的力控比定角轮系多一步：力是矢量，得先决定"往哪个方向出力"，再决定"出多大"。分两步：

**① 期望轮端力矢量**（机体系，N）。平移力四轮均分，偏航力沿轮心切向分配：

```
F_i,x = fx/4 − Y·yᵢ        F_i,y = fy/4 + Y·xᵢ        Y = mz/Σdᵢ² = mz/(4d²)
```

其中 `(xᵢ, yᵢ) = (±Lx, ±Ly)`（半轴距/半轮距），`d² = Lx²+Ly²`。代码用**循环 + 符号表现算**轮心坐标：

```cpp
const float yaw_force = mz / (4.0f * Wheel_D2);
for (i = 0; i < 4; i++) {
    F[i].x = fx * 0.25f - yaw_force * Wheel_Sign_Y[i] * Wheel_Projection_Y;  // −Y·yᵢ
    F[i].y = fy * 0.25f + yaw_force * Wheel_Sign_X[i] * Wheel_Projection_X;  // +Y·xᵢ
}
```

正方形与长方形布局同一式，`4d²` 用常量 `Wheel_D2` 在循环外算一次除法。

> **不要把符号展开成字面量写四遍。** 展开写法等于把 `Wheel_Sign_X/Y` 表抄了第二份，改符号表时这里就静默失配——与此前硬编码 `±√2/2` 是同一类错误。轮心坐标必须由符号表现算，与逆解/正解/位移解算四处同源。

四轮对齐时满足 `ΣFᵢ = F` 与 `Σ(rᵢ × Fᵢ) = fw`（数值验证：正方形 / 长方形 / 窄长 / 高窄四组布局，守恒误差 ≤ 7e-15；循环版与展开版 float32 逐位零差异）。

> **`d` 不要手填，由 `Lx`/`Ly` 推导。** 代码里没有独立的 `d` 常量：角速度公式只用到 `Wheel_D2 = Lx²+Ly²`，逆解与力分配直接用轮心坐标 `(±Lx, ±Ly)`，全程无需开方。换构型只改 `Wheel_Projection_X` / `Wheel_Projection_Y` 两个数。
>
> 历史教训：1.37 之前 `0.12` 被当成**径向距离**用（等价于设 `Lx = Ly = 0.0849`，底盘仅 17 cm 见方）。若把半轴距误当径向距离，会让偏航轮速偏小 √2 倍、里程计航向偏大 √2 倍，而正逆解自身仍自洽，数据上看不出矛盾——只能对实测（编码器航向 vs IMU 航向）。

> **中心对称是数学前提。** 上述解耦依赖 `Σxᵢ = Σyᵢ = 0`。四轮一旦不对称（比如某一侧半轴距不同），均分平移会在合力矩里多出寄生项 `(fy·Σx − fx·Σy)/4`，切向偏航项也会在合力里多出 `fw(−Σy, Σx)/Σd²`，两者互相耦合，**单给力分配补一项只会丢掉另一个**（实测：只补力矩后 `ΣF=F` 从对变错，误差约输入的 2%）。逆解、正解、位移解算同样依赖该前提，真要支持非对称布局需把四处矩阵一并换成完整伪逆。

**② 出力幅值**（与逆解同一口径）：

```
fᵢ = |Fᵢ| · |cos eᵢ|³ · force_directionᵢ
```

`force_directionᵢ` 是**力域**的出力方向（`status.force_direction[i]`，±1）：

| 路径 | `force_directionᵢ` | 理由 |
|------|-------------------|------|
| 纯力控 | = 就近转位选定的分支 `optimize_directionᵢ` | 舵向本就由力方向决定，“指向力方向”与“反向出力”两个分支等价且都已对齐 |
| 混控 | = `sign(Fᵢ·û(θᵢ))`（不带 `dir`） | 舵向由**速度**逆解给定，电机域正力矩恒把底盘推向 `+û(θ)`、与轮子正/反转无关；制动时力与轮向恰好相反 |

②③ 全部收在 `force_calculate_core()` **一个无参函数一次循环**内（纯力控与力速混控共用），与 `forward_calculate_callback()` 同形——读 `target_accel`、写 `status`：

```cpp
void force_calculate_core()
{
    // 读 target_accel -> (robot_force_x, robot_force_y, robot_torque_z)，非法则关断输出
    const float yaw_force    = robot_torque_z / (4.0f * Wheel_D2);
    const float cos_deadband = arm_cos_f32(steer_angle_deadband);
    for (i = 0; i < 4; i++) {
        // ① 期望轮端力（与逆解同形的一行）
        const float force_x = robot_force_x * 0.25f - yaw_force * Wheel_Sign_Y[i] * Wheel_Projection_Y;
        const float force_y = robot_force_y * 0.25f + yaw_force * Wheel_Sign_X[i] * Wheel_Projection_X;
        // 力幅（矢量模长，不是力矩），与上游 vel_modulus / 下游 force_modulus 同一口径
        float force_modulus = 0.0f;
        arm_sqrt_f32(force_x * force_x + force_y * force_y, &force_modulus);
        if (force_modulus <= force_deadband) { /* 锁舵向，该轮不出力 */ continue; }
        status.cosine_optimize[i] = 1.0f;   // 共用字段，缺省不削减（1 = 已对齐）
        if (control_method != Wheel_Mixed_Control_Method) {
            // ② 力方向先换成与逆解同域的电机目标角，之后整段与 inverse_calculate_callback() 逐行同构
            const float force_target = steer_encoder_direction * atan2f(force_y, force_x) + Enc_Zero_Angle[i];
            // 就近转位（方向直接写 status.optimize_direction[i]）/ 单圈折算 /
            // align_error / effective_error / 死区 / 斜坡限幅 ...；需要削减时覆盖 status.cosine_optimize[i]
            status.force_direction[i] = status.optimize_direction[i];
        } else {
            // ②' 带符号投影直取 cos e（舵向与轮向方向已由逆解给出）
            const float cos_angle = (force_x * cos(θ) + force_y * sin(θ)) / force_modulus;
            status.force_direction[i] = (cos_angle >= 0) ? 1.0f : -1.0f;   // 符号：力在哪一侧
            if (fabsf(cos_angle) <= cos_deadband) {                        // 幅值：|cos e|³
                status.cosine_optimize[i] = fabsf(cos_angle)³;
            }
        }
        // ③ 出力：力幅 × 共用余弦削减（幅值）× 共用出力方向
        wheel_force[i] = force_modulus * status.cosine_optimize[i] * status.force_direction[i];
    }
    // 力 -> 力矩（扣除式死区 + 力矩上限）
}
```

**舵向规划整段跑在电机域**（与逆解同一个坐标系、同一套处理步骤）：力方向是几何角，先换成
`force_target = steer_encoder_direction·atan2f(force_y, force_x) + Enc_Zero_Angle[i]`，之后
`steer_select_nearest_target()` / `align_error` / `effective_error` / 斜坡限幅与
`inverse_calculate_callback()` 一一对应，只把 `solution_target` 换成 `force_target`：

| 环节 | 速控 `inverse_calculate_callback()` | 力控 `force_calculate_core()` |
|------|------|------|
| 解算方向 | `solution_angle[i] = atan2f(vel_y, vel_x)` → `solution_target` | `atan2f(force_y, force_x)` → `force_target` |
| 就近转位 | `target = steer_select_nearest_target(solution_target, now_angle, dir, hyst, &optimize_direction)` → 写回 `status.optimize_direction[i]` | 同左，只换 `force_target`；方向直接写 `status.optimize_direction[i]`（无局部量） |
| 未对齐角 | `align_error = wrap(solution_target − now_angle)` | `align_error = wrap(force_target − now_angle)` |
| 余弦 | `cos_angle_raw = arm_cos_f32(effective_error)` → `cos_angle` → `cos³` | 纯力控同左；混控用带符号投影定 `force_direction`，幅值比 `cos_deadband` |
| 斜坡 | `optimize_angle = 限幅(target)`；`optimize_angle_valid[i] = true` | 同左 |
| 出力 | `optimize_speed = solution_speed·cosine_optimize·optimize_direction` | `wheel_force = force_modulus·cosine_optimize·force_direction` |

局部量名逐一对齐：`force_x/force_y`↔`vel_x/vel_y`、`force_modulus`↔`vel_modulus`、`target`↔`target`。
`optimize_direction` **不再有局部量**：本拍方向输出与下一拍的迟滞记忆是 `status.optimize_direction[i]`
同一个元素（帮助函数先读 `__last_direction` 再写 `*__direction`，同一对象同进出安全），与逆解
“读 `status` 定方向、写回 `status`”同构；`cosine_optimize` / `error_angle` 同样只存在 `status` 里，
出力方向则统一落在 `status.force_direction`——两条流水线都只写 `status`，没有一套平行的局部副本。
好处不只是好读：`error_angle` 在两条流水线上从此是**同一个坐标域**，`steer_encoder_direction = −1`
时不再一边报几何角、一边报电机角（换算后两者恰好互为相反数，|error| 与 cos 完全相同）。

余弦削减也已内联进 core（原独立的 `force_cosine_optimize()` 已删除，它只服务 core 一处调用）：

```cpp
const float align_error = wrap(force_target - status.now_angle[i]);
const float effective_error = (status.optimize_direction[i] < 0) ? wrap(align_error + PI) : align_error;
status.error_angle[i] = effective_error;

if (cosine_optimize_enable && fabsf(effective_error) >= steer_angle_deadband) {
    const float c = arm_cos_f32(effective_error);
    const float ca = nearest_optimize_enable ? fabsf(c) : c;
    status.cosine_optimize[i] = constrain(ca * ca * ca, 0.0f, 1.0f);
}
// 否则保持循环开头置的 1（开关关闭 / 已在死区内）
```

两条公开入口因此各是一行：

```cpp
force_calculate_callback()  →  force_calculate_core();                    // 力方向定舵向
mixed_calculate_callback()  →  inverse_calculate_callback(); core();       // 舵向已由逆解给出
```

- **力分配公式与出力公式各只出现一次**，两条流水线不会因为各写一遍而在日后漂移；中间力矢量只是循环内的局部量，不需要 `wheel_force_x[4]` / `y[4]` 两个数组，也不用分三趟遍历。
- 加速度合法性校验与非法输入分支也合并在 core 内（`control_method != Wheel_Mixed_Control_Method` 决定是否清全部目标字段）。
- `cos e` 的两种取法等价但开销不同：纯力控本就要用未对齐角写 `status.cosine_optimize`（诊断需要），顺带复用；混控不需要角度值，用点积 `dir·F·û(θ)/|F|` 直达（`cos_angle_raw`），一次点积代替 `atan2f` + `arm_cos_f32`。两条路径写的都是**同一个** `status.cosine_optimize[i]`。
- 死区判据：力死区直接比力幅 `force_modulus <= force_deadband`，与下游 `wheel_force_force_to_torque()` 的 `force_modulus <= deadband` **完全同形**（上游门限与扣除式死区同一表达式，不会出现“一个用平方、一个用模长”的口径差）。代价是死区轮也多做一次开方：4 次 `VSQRT` 在 1 kHz 下约 56 周期/拍，可忽略。余弦死区换算到余弦域 `|eᵢ| < deadband ⇔ cos eᵢ > cos(deadband)`（循环外算一次，即 `cos_deadband`）。
- `control_method == Wheel_Mixed_Control_Method` 时**不重新规划舵向、轮向方向与未对齐角**（这三个沿用逆解的结果），只写力域自己的两个量：`force_direction`（= 带符号投影的符号）与 `cosine_optimize`（= `|cos e|³`）。`cosine_optimize` 因此在本拍被覆盖成力域幅值——不影响任何输出（逆解已在末尾用它算完 `optimize_speed`，力矩本就要按力方向削减），代价只是 `get_cosine_optimize()` 在混控下与同拍的 `get_error_angle()`（速度域残差）不同域。

> ⚠️ **混控的出力方向不能用 `optimize_direction`。** `optimize_direction` 是**轮向方向**（轮子往哪边转，由速度逆解给出）；电机域正力矩恒把底盘推向 `+û(θ)`，与轮子正/反转无关。减速时力与轮向恰好相反，若用轮向方向定符号，制动会变成加速（实测：期望 `−0.2425 N·m` 却输出 `+0.2425`，四轮全中）。因此出力方向单独存在力域字段 `force_direction`，**且力控不写 `optimize_direction`**——否则会踩坏逆解下一拍的翻转迟滞记忆。

> ⚠️ **契约：`init()` 首参须与实际调用的入口一致**（`Force` ↔ `force_calculate_callback`，`Mixed` ↔ `mixed_calculate_callback`）。core 是否规划舵向直接由 `control_method != Wheel_Mixed_Control_Method` 判定，若 `init` 成 `Speed`/`Force` 却调 `mixed_calculate_callback`，core 会按 Force 语义重新规划舵向，把逆解刚写好的舵向目标覆盖掉（实测：逆解写 `0.9`，被改成 `0.09 / −1.35 / −0.68 / 0.02`）。

> **死区不能用 `|cos eᵢ|` 判（纯力控）。** 纯力控的 `cos e` 必须先用 `dirᵢ` 折回成带符号值再比，因为 `|cos|` 在 `[0, π]` 上非单调（π/2 处最小），单看模长会把“舵向反转后的接近对齐”与“正向的大失配”混为一谈。混控则相反：符号由 `force_direction` 单独承担，死区只需拿 **幅值** 比 `|cos e| > cos(deadband)`（与纯力控的 `|e| >= deadband` 严格等价）。
三件事和速控完全一致：

| 环节 | 速控 | 力控 |
|------|------|------|
| 就近转位 | `steer_select_nearest_target()` + 翻转迟滞 | 同一个函数、同一个 `flip_hysteresis` |
| 余弦削减 | `轮速 = |v|·|cos e|³·dir` | `轮端力 = |Fᵢ|·|cos eᵢ|³·force_direction` |
| 舵向死区 | 余弦开关打开且 `|e| < steer_angle_deadband` 时不修舵、不削减 | 同左 |

为什么用 `cos³e` 而不是"把力投影到当前轮向"（`|F|·cos e`）：投影只抵消方向偏差带来的**必然损失**，不额外给舵向留对齐时间；`cos³` 是"先摆轮后出力"的既定策略，与速控一致，否则同一条指令在力控下落差更大。`cos³` 的代价是未对齐期间出力更小，但这段窗口很短（斜坡每拍只能转 `target_max_step`）。

**③ 与速控共用同一份状态**：力控写的就是速控那五个字段（`optimize_angle` / `optimize_speed` / `optimize_direction` / `cosine_optimize` / `error_angle`），另加一个**力域专属**的 `force_direction`（速控不碰），所以：

- 两条路径切换时**斜坡基准天然连续**，不存在"两套目标互相踩基准"或切换瞬间跳变；
- 下游只认这一组 getter（`get_optimize_angle` / `get_optimize_speed` / `get_cosine_optimize`），**不需要知道本拍走的是哪条路径**；
- 纯力控把 `optimize_speed` 置 0（速度环不参与），力矩走 `get_wheel_torque`；混控则速度、力矩同时有效。

| 路径 | 舵向目标来源 | 轮速目标 | 力矩 |
|------|--------------|----------|------|
| `speed_calculate_callback()` | 速度逆解 | 速度逆解 | 逆解入口清零 |
| `force_calculate_callback()` | 期望轮端力方向 + 就近转位（受 `nearest_optimize_enable` 控制） | **0** | `|Fᵢ|·cos³e·force_direction` |
| `mixed_calculate_callback()` | 速度逆解 | 速度逆解 | `|Fᵢ|·cos³(|e|)`·`sign(Fᵢ·û(θᵢ))`（不覆盖舵向与轮向方向） |

限制与自检：

- **必须先 `update_now_state()`**：出力幅值依赖实测 `now_angle`，缺反馈时舵向角停在上电值，四轮会朝错误方向出力。
- **力矩前馈必须排在逆解之后**：`inverse_calculate_callback()` / `speed_calculate_callback()` 入口会整体清零 `status.wheel_torque`（速控不做力矩前馈，清零是为了不让上一拍力矩残留），顺序颠倒会让前馈静默失效。这也是 `mixed_calculate_callback()` 不能调 `speed_calculate_callback()` 的原因：后者会把 `target_accel` 关断掉，前馈就永远算不出力。
- 力矩上限 `max_wheel_torque` 与死区 `force_deadband` 直接作为本类成员（无结构体），由 `init()` 整份校验后两项一次写齐（质量/惯量为编译期常量，不进 init）；模块**不做加速度限幅**（`ax/ay/aw` 由上层裁剪）；死区以内的小力按 0 输出，且**不出力时舵向锁定当前角**（不做无意义摆动）。
- **无隐式超时**：力控与速控同构，不另配超时门限。使能边界就是**目标加速度是否为零**：`set_robot_accel_x/y/w()` 写入，`speed_calculate_callback()` 入口整体清零。切模式 / 停车时切回速控流水线即可，这是显式契约——比靠 `control_period` 软件累加的隐式超时更可靠（调用周期一变门限就失真），也不会把"上层忘了关"掩盖成"到点自动归零"。
- **无饱和标志位**：力矩输出已限幅在 ±`max_wheel_torque` 内，上层拿 `get_wheel_torque(i)` 与该上限一比即知是否触顶，不另存一份可能与实际值不同步的标志。
- **三个优化开关是速控与力控共用的**：`set_cosine_optimize_enable(false)` 让两条路径同时"不按 `cos³e` 削减出力"；`set_nearest_optimize_enable(false)` 让两条路径都"只转解算/力方向本体、`dir` 恒 +1"；`set_target_step_limit_enable(false)` 让两条路径的舵向目标都直接跟随。同一个开关不会只关掉一边——否则同一条指令在两套流水线下的舵向行为会不一致。
- **自检**：只给 `fw > 0`（`fx = fy = 0`）时应四轮切向出力、原地 CCW 转；只给 `fx > 0` 时应四轮同向指向前方。同时给 `vx` 与 `fx` 时混控的舵向必须跟**速度方向**（不是力的方向），力矩才是速度方向上的加/减速。大角度换向时观察 `get_cosine_optimize(i)`：应先接近 0 再随舵向对齐回升到 1。

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

**力控路径同口径**：`force_calculate_callback()` 里的"期望轮端力方向"也走同一个开关——打开时与速控一样用 `steer_select_nearest_target()`（含翻转迟滞）取等效姿态并反转出力符号；关闭时舵向只转力方向本体、`dir` 恒 `+1`。开关是同一份，两条流水线不会出现"一边就近转位、一边不倒车"的差异。

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

力控路径用同一个开关与同一个 `target_max_step`：`force_calculate_callback()` 算出的力方向目标角也过 `steer_limit_target_step()`，关闭后同样直接跟随。

### 4.4 舵向死区（`steer_angle_deadband`）

命中条件：`cosine_optimize_enable` 为真 **且** `|真实未对齐角| < 死区` **且** `control_mode != 2`。命中时 **只把 `k` 置 1（不削减轮速）**，舵向目标**照常朝解算角收敛**。

> ⚠️ **死区绝不能把目标改写成 `now_angle`。** 早期实现这么写（"命中时锁当前角"），等于把残余误差**永久放弃**：四轮各自锁在距目标至多 `deadband` 的任意位置，机械摩擦/齿隙/刚度不同 → 锁定位置不同 → 形成静态的指向不一致。
>
> 失效方向高度不对称，这是"**vx 走直线、vy 走圆弧**"的根因：
>
> | 指令 | 四轮目标 | 死区锁定的后果 |
> |------|----------|----------------|
> | `vx`（前进） | 都在零位 0° | 本来就不用转舵，残余误差≈0，四轮一致 → **直线** |
> | `vy`（横移） | 都要转到 90° | 四轮在不同时刻进入死区、锁在不同位置 → 前后不对称 → 持续净力矩 → **圆弧** |
>
> 仿真复现（`deadband = 0.05 rad`，四轮起步参差）：旧实现 vy 时三轮收敛到 90.01°、FR 锁在 87.44°，**单轮差 2.6° 就产生 `ω = 0.032 rad/s`**，1 m 行程横漂约 16 mm 且持续弯曲。修复后四轮最大偏差 0.01°，`ω = 0`。
>
> 敏感度：前后反向静差 1° → `ω ≈ 0.059 rad/s`；3° → `ω ≈ 0.177 rad/s`（1 m 行程漂 88 mm）。
>
> **现状（code 1.17 起）**：两条流水线的死区分支都已改为"只把 `k` 置 1、不碰 `optimize_angle`"，`error_angle` 也不再被清零。

判据必须用**真实未对齐角**（解算目标 − 当前舵向，见 §4.6），不能用斜坡后的目标误差：斜坡每拍只放行一小步，用斜坡误差会让"正在大角度换向"被误判成"已对齐"，从而放行满轮速。

- **只在余弦开关为真时生效**：关掉余弦优化会连带失去防抖。若要"满轮速 + 防抖"，保留余弦开关并调小死区即可。
- `control_mode == 2`（小陀螺）跳过死区，否则舵向会被锁死无法跟随旋转。取值沿用 `chassis_mode_e`：`0` 无力、`1` 跟随、`2` 小陀螺。
- 死区只影响"是否削减轮速"，**不影响舵向精度**：目标永远是解算角，位置环（纯 P）+ 速度环（PI）会把误差消到只剩各轮物理静摩擦（实测 < 0.4°）。
- **`error_angle[]` 保留真实残差，不因命中死区被清零**：否则遥测会把"还差 0.04 rad"伪装成"完全对齐"，看不到静态指向偏差的来源。
- **力控路径同口径**：`force_calculate_callback()` 的死区分支同样只在 `|e_eff| < deadband` 时把 `k` 置 1，舵向目标照常朝力方向收敛；它不会把目标改写成 `now_angle`。
- 真正"锁当前角"的只有两类**无输出**分支：速控的零指令 / 零轮速分支、力控的力死区（`|F| < force_deadband`）分支。这两处没有推力可丢，保持姿态不动是对的。

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

`error_angle[]` 记录的是**真实未对齐角** `e_eff`（公式与逐项含义见 §4.5；已折到所选方向分支，0 表示对齐）。它**恒为真实值，不会因落在死区内被清零**——这样遥测能直接看到残余误差，而不是被伪装成 0：

```
|error_angle[i]| < 阈值   ⇔   |wrap(solution_angle[i] + 零位 − now_angle[i], 2π)| < 阈值      (dir = +1)
```

`dir = −1` 时 `error_angle` 已经是折算到"反向轮速分支"的等效角，上式的右侧需同样加 π 折算。斜坡的剩余行程可用 `optimize_angle − now_angle` 观察，它才是"还要转多远"。

---

## 5. 正运动学

```cpp
steer_i = wrap(dir·(now_angle[i] − Enc_Zero_Angle[i]), 2π);   // dir = steer_encoder_direction
u_i     = now_speed[i] · R;                                    // 轮子线速度

vx = (1/4)·Σ u_i·cos(steer_i)
vy = (1/4)·Σ u_i·sin(steer_i)
vw = (1/(4d²))·Σ u_i·(sin(steer_i)·x_i − cos(steer_i)·y_i)
```

推导：轮心速度 = 底盘速度 + 旋转速度。取轮子方向上的分量得 `u_i`，再取该速度在"绕中心的切向" `t_i = (−y_i, x_i)/d` 上的分量即得该轮的 `vw`：

```
u_i·(p_i · t_i) = u_i·[cosθ_i·(−y_i/d) + sinθ_i·(x_i/d)] = u_i·(sinθ_i·x_i − cosθ_i·y_i)/d = vw·d
```

与 §3.1 逆解的耦合项 `(vx − vw·y_i, vy + vw·x_i)` 严格互为逆运算。实现上直接乘轮心坐标 `(±Lx, ±Ly)`，`d²` 提到循环外除一次。

精度条件：`vx/vy` 的等权平均要求四轮轮心对称（`Σr_i = 0`）；`vw` 的等权平均要求四轮径向距离相同——`Lx/Ly` 对称布置下四轮 `d = √(Lx²+Ly²)` 恒相等，两个条件都满足，因此上式是**精确解**（数值验证：正方形/长方形/窄长三组布局，逆解→正解互逆误差 ≤ 4e-15）。

由此得到两个实用性结论：

1. 正常行驶（舵向已对齐）时 `robot_speed` 可信，可以直接用于速度里程计。
2. 舵向正在大幅调整的过渡期，四轮数据不自洽，`robot_speed` 是加权平均的估计值，此时速度里程计会有偏差——这也是编码器里程计更可靠的原因。

---

## 6. 里程计

### 6.1 编码器里程计

舵轮里程计比另外两种多一步：轮位移必须投影到**实测舵向角**上（不能用目标角）。

```
Δs_i    = (θᵢ − θᵢ_prev)·R                                    // 轮向编码器差分
steer_i = wrap(dir·(steer_angle[i] − Enc_Zero_Angle[i]), 2π)   // 舵向编码器实测角(几何化)

dx  = (1/4)·Σ Δs_i·cos(steer_i)
dy  = (1/4)·Σ Δs_i·sin(steer_i)
dψ  = (1/(4d²))·Σ Δs_i·(sin(steer_i)·x_i − cos(steer_i)·y_i)
```

三轴合成与 §5 形式完全一致，只把 `u_i` 从"轮子线速度"换成"本帧线位移"。

死区是**累积量**而非单帧量：三轴累积不足线 1e-4 m / 角 1e-3 rad 时本帧不积分，位移留在 `odom_residual_*` 里，等后续帧攒够再连同历史位移一次积分。慢速运动因此不会被整帧丢掉（最多滞后 1 个死区量），零位噪声的 `+n / −n` 抖动则互相抵消、不会触发。任一舵向角或轮位移非有限时整帧跳过（`calc_body_displacement()` 返回 false，本帧不进死区累积器）。

**编码器跳变保护**：`wheel_odom_calc_ds()` 对每轮做 `|Δθ| > π`（半圈）判定。100 Hz 下半圈已相当于 314 rad/s 以上，任何真实轮速都达不到，出现即说明编码器丢帧 / 上电复位 / 多圈计数跳变（C620 的 `total_round` 是帧间差分统计的，一帧丢包就会跳整圈，约 `2πR/19 ≈ 16 mm`）。命中时**整帧重建基准并丢弃本帧**，而不是只丢坏的那一轮——只丢单轮会让其余三轮照常积分，算出一个方向错误却"看起来合理"的位移。

**两个舵向角快照用途不同，不要混用**：

| 字段 | 来源 | 用途 |
|------|------|------|
| `status.now_angle[]` | `update_now_state()`（逐通道校验） | 逆解/正解/死区判断（控制回路） |
| `steer_angle[]` | `update_now_state()`（整帧校验，与轮向角同批提交） | 里程计 |

两者同源（同一个编码器、同一次调用），差别只在**校验粒度**：坏通道只跳过控制回路的那一拍，而里程计快照要求轮/舵向角整帧有效才推进，避免用新旧混合快照算出虚假位移。若里程计方向正确而控制方向反了（或反之），第一件事就是核对这两路是否接了同一个量、符号是否一致。

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
| `steer_angle_deadband` | 轮速削减更晚结束（对舵向精度无影响） | 换向过程中更早按 `cos³` 削减轮速 | 0.03 ~ 0.06 rad（**不影响指向精度**，见 §4.4） |
| `flip_hysteresis` | 边界更稳定、±90° 附近略偏"不反转" | 换向更灵、易在 90° 边界抖动 | 0.05 ~ 0.15 rad（须 < 90°） |
| `cosine_optimize_enable` | — | — | 保持 `true`（同时启用死区） |
| `nearest_optimize_enable` | — | — | 保持 `true`（换向更快） |
| `target_step_limit_enable` | — | — | 保持 `true`（保护舵向电机） |

联调顺序建议：

1. **纯旋转**（`vw` 单独给）：确认四轮方位角与力臂正确、`optimize_direction` 全为 `+1`。
2. **纯平移**（`vx` 或 `vy` 单独给）：四轮应全部平行指向平移方向（`vx>0` 时为几何角 `0`，或就近转位后的 `180°`，此时 `optimize_direction = −1`），`error_angle` 收敛到 0。
3. **对角线平移**：确认就近转位不会引起方向来回跳（若跳，减小 `steer_angle_deadband` 或检查 `min_limit`）。
4. **大角度换向**：观察 `cosine_optimize` 曲线是否平滑，`optimize_speed` 是否过零时无突变。
5. **全速工况**：确认全局限速没有被频繁触发（若频繁触发，`max_wheel_speed` 偏小）。

---

## 8. API 速查

| 分类 | 接口 | 说明 |
|------|------|------|
| 初始化 | `init(method, enc_dir, period, max_step, max_speed, min_limit, deadband, flip_hyst, max_torque, force_deadband, cos_en, nearest_en, step_en)` | **一次性写齐全部可配参数**（全部带默认值）：控制模式与控制参数 → 速控参数 → 力控死区和最大力矩限制 → 优化开关；速控参数逐项校验，力控两项整份校验 |
| 初始化·力控 | `init()` 的 `max_torque` / `force_deadband` 两形参 + 常量 `Chassis_Mass` / `Chassis_Inertia` | 力矩上限与死区，**整份校验**：任一项非法则两项全不改；质量/惯量是编译期常量，改配重就改常量重编 |
| 配置 | `set_target_max_step()` / `set_max_wheel_speed()` | 必须 > 0 |
| 配置 | `set_min_limit()` / `set_steer_angle_deadband()` | 允许 0（关闭判定/死区） |
| 配置 | `set_flip_hysteresis(h)` | 就近转位翻转迟滞，须在 `[0, π/2)`，否则拒绝 |
| 配置 | — | 半轴距/半轮距 `Wheel_Projection_X/Y` 是编译期常量（换构型改这两个数），无运行期修改入口 |
| 配置 | `set_cosine_optimize_enable()` / `set_nearest_optimize_enable()` / `set_target_step_limit_enable()` | 无条件写入 |
| 配置 | `set_steer_encoder_direction(±1)` | 舵向编码器极性，非 ±1 / 非有限被拒绝；`init()` 第二参已可指定，此接口用于运行期改（会覆盖 init 的设置） |
| 配置 | `set_control_period(T)` | 仅速度里程计 |
| 输入 | `update_robot_state({vx,vy,vw}, control_mode)` | 速度非法时**仍然**更新 `control_mode` |
| 输入 | `update_now_state(steer_state_t, wheel_state_t, imu_yaw = NAN)` | 唯一反馈入口：取 `steer.angle` / `wheel.speed` 写控制回路（逐通道校验），取两者 `angle` 提交里程计快照（整帧校验）+ IMU 航向；`imu_yaw` 默认 `NAN` = 不更新，未接 IMU 时航向保持原值而非被钉成 0 |
| 解算 | `speed_calculate_callback()` | **速控入口**：关断力控输出 + 逆解 + 三级优化 + 限速 |
| 解算 | `inverse_calculate_callback()`（protected） | 纯逆运动学（同时清力矩，但**不改**目标加速度）；仅供 `speed_`/`mixed_calculate_callback()` 内部调用 |
| 解算 | `forward_calculate_callback()` | 正解 → `robot_speed` |
| 解算 | `encoder_odometry_callback()` / `imu_odometry_callback()` | 编码器里程计 |
| 解算 | `speed_odometry_callback()` | 速度里程计 |
| 输出 | `get_solution_angle(i)` / `get_solution_speed(i)` | 速控纯几何解，诊断用 |
| 输出 | `get_optimize_angle(i)` / `get_optimize_speed(i)` | **下发电机用**：本拍生效的舵向/轮速目标（速控与力控写同一组） |
| 输出 | `get_optimize_direction(i)` | ±1，已包含在 `optimize_speed` 里（**速度域**，力控不写） |
| 输出 | `get_error_angle(i)` / `get_cosine_optimize(i)` | 真实未对齐角（速度域）/ 余弦削减系数幅值 `[0,1]`（混控下为力域），诊断用 |
| 输出 | `get_force_direction(i)` | ±1 轮端出力方向（力域）：纯力控 = 就近转位分支，混控 = `sign(F·û(θ))`（速控下为上次值） |
| 输出 | `get_robot_speed_vx/vy/vw()` | 正解速度 |
| 输出 | `get_position_x/y/yaw()` / `get_speed_position_x/y/yaw()` | 两套里程计位姿 |
| 复位 | `reset_position()` / `set_position()` / `reset_speed_position()` / `set_speed_position()` | 同上 |
| 力控·读取 | 无（无读回接口） | 配置量由上层自己掌握（质量/惯量是常量）；要看本拍是否出力用 `get_wheel_torque(i)` |
| 力控·输入 | `set_robot_accel_x/y/w(v)` | 目标加速度逐轴写入 (m/s² / m/s² / rad/s²)；单轴非有限值只跳过该轴。**零加速度 = 不出力**，无另设使能位 |
| 力控·解算 | `force_calculate_callback()` | **纯力控**：加速度 → 舵向对齐目标 + 轮端力矩（`optimize_speed` 置 0） |
| 力控·解算 | `mixed_calculate_callback()` | 力速混控：速度逆解给出舵向/轮速目标，再按当前实际舵向叠加加速度前馈力矩（须排在逆解之后，逆解入口清力矩）。**力矩符号按力投影定**，减速时与轮速方向相反 |
| 力控·输出 | `get_wheel_torque(i)` | 轮端力矩 N·m（速控路径恒 0）；是否被 `max_wheel_torque` 截断，拿它与该上限比就知道，不另设标志位 |

---

## 9. 调用示例

```cpp
static steer_wheel_class chassis_wheel;

void chassis_init(void)
{
    // 一次性写齐: 控制模式和控制参数 → 速控参数 → 力控死区和最大力矩限制 → 优化开关
    chassis_wheel.init(/*method*/ Wheel_Speed_Control_Method,   // 控制方式首个参数选定
                       /*enc_dir*/ -1.0f,     // 舵向编码器极性, 本车反向; 同向填 +1
                       /*period*/ 0.001f,
                       /*max_step*/ 0.02f,
                       /*max_speed*/ 25.0f,
                       /*min_limit*/ 0.1f,
                       /*deadband*/ 0.05f,
                       /*flip_hyst*/ 0.1f,
                       /*max_torque*/ 6.0f,  // 以下两项只在力控/混控时生效
                       /*force_deadband*/ 0.1f,
                       /*cosine*/ true,
                       /*nearest*/ true,
                       /*step_limit*/ true);

    // 轮径、逐轮力臂、方位角表、编码器零位、质量/惯量都是编译期常量,
    // 正方形/长方形布局只改 Wheel_Projection_X / Wheel_Projection_Y 后重编
}

void chassis_task(void)                       // 1 kHz
{
    // ① 反馈
    steer_state_t now_steer{};   // 舵向：angle 喂正/逆解，speed 供舵向速度环
    wheel_state_t now_wheel{};   // 轮向：speed 喂正/逆解，angle 喂里程计
    for (uint8_t i = 0; i < 4; i++)
    {
        now_steer.angle[i] = steer_motor[i].get_total_angle();
        now_steer.speed[i] = steer_motor[i].get_speed();
        now_wheel.angle[i] = drive_motor[i].get_total_angle();
        now_wheel.speed[i] = drive_motor[i].get_speed();
    }
    chassis_wheel.update_now_state(now_steer, now_wheel, imu.get_yaw());

    // ② 正解 + 里程计
    chassis_wheel.forward_calculate_callback();
    chassis_wheel.speed_odometry_callback();
    chassis_wheel.imu_odometry_callback();

    // ③ 解算（control_mode 取 chassis_mode_e）
    chassis_wheel.update_robot_state({vx_target, vy_target, vw_target}, now_mode);
    chassis_wheel.speed_calculate_callback();   // 速控流水线（当前选定的方式）

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
| 对齐判定 | `get_error_angle(i)` | `abs(error_angle) < 阈值` 即已对齐（已折到所选方向分支，恒为真实值） |
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
3. **方位角 / 手性**：单独给 `vx`，四轮目标角应收敛到 0°（都朝前）。再单独给 `vw > 0`，四轮应为风车姿态 135°/225°/315°/45° 且**车体原地逆时针转**——这一项才能查出耦合项镜像，只测直行查不出来。
4. **编码器极性**：上一步若车体转向相反，把 `init()` 第二参改为 `+1`（或运行期调 `set_steer_encoder_direction(+1)`）重测，不要改耦合项。
5. **轮速上限**：给一个明显超限的车速，确认四轮等比缩小且比例一致。
6. **斜坡 / 死区**：从 §7 起手值出发，按"抖动 ↔ 响应"折中微调。

### 10.2 常见问题（舵轮特有）

| 现象 | 原因 | 处理 |
|------|------|------|
| 四轮指向成 90° 交叉（本该对边平行） | 轮心符号表或正逆解不配套，或某轮电机方向反 | 先核对 `Wheel_Sign_X/Y`，再逐轮点动 |
| 纯自旋（小陀螺）电流顶满、车原地不转 | 旋转耦合项写成了镜像场 `d·(+sinφ, +cosφ)` | 逆解必须是 `vx − ω·d·sinφ`；正解必须是 `sin(θ−φ)` |
| 只给 `vw` 时四轮呈"朝外/朝内的叉状"（45°/−45°/−135°/135°） | 同上，耦合项镜像 | 正确姿态是风车形 135°/225°/315°/45° |
| 原地转的方向与指令相反、或平移 `vy` 反向 | `steer_encoder_direction` 与实际极性不符 | 改 `init()` 第二参为 `+1`（或 `set_steer_encoder_direction(+1)`），别动耦合项 |
| **`vx` 走直线、`vy` 走圆弧** | 死区把目标改写成 `now_angle`，四轮锁在各自不同的残余误差上（见 §4.4） | 死区只控制轮速削减，目标必须继续朝解算角收敛 |
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
