# 全向轮轮系 (Omni Wheel)

> 对应类：`omni_wheel_class`（[wheel_system_solution.h](./wheel_system_solution.h) / [wheel_system_solution.cpp](./wheel_system_solution.cpp)）
> 通用定义：参考坐标系见 [总览 §3](./wheel_system_solution.md)、轮系定义见 [总览 §4](./wheel_system_solution.md)、执行流程见 [总览 §5](./wheel_system_solution.md)、接口设计原则见 [总览 §7](./wheel_system_solution.md)。

全向轮是最"干净"的一种轮系：逆解是纯线性组合，没有舵向角、没有优化环节，解算结果直接就是四个轮向电机的目标角速度。代价是**每帧输出只对当帧有效**，不存在舵向对齐的过渡过程。

---

## 1. 适用构型与几何

4 个 45° 全向轮，轮心均匀分布在以底盘中心为圆心、半径 `d` 的圆上，方位角 φ = 45° / 135° / 225° / 315°（X 型布局）。

每个轮子的**驱动方向与轮心径向垂直**（切向），辊子提供径向的自由滚动，因此轮子只能感知自己切向的速度分量。

```
                     +X (车头)
                          ↑
        FL [0] φ=45°      │      FR [3] φ=315°
         (+x, +y)         │         (+x, −y)
               ╲          │          ╱
                ╲         │         ╱
   +Y (左) ←──────────────┼──────────────→ −Y (右)
                ╱         │         ╲
               ╱          │          ╲
        RL [1] φ=135°     │      RR [2] φ=225°
         (−x, +y)         │         (−x, −y)
```

轮子的驱动方向沿切向 `t̂ᵢ = (−sinφᵢ, cosφᵢ)`，辊子吸收径向速度分量。

| 下标 | 轮位 | 方位角 φᵢ | 切向 t̂ᵢ = (−sinφᵢ, cosφᵢ) | sinφᵢ | cosφᵢ |
|------|------|-----------|--------------------------|-------|-------|
| 0 | FL | 45° | (−√2/2, +√2/2) | +√2/2 | +√2/2 |
| 1 | RL | 135° | (−√2/2, −√2/2) | +√2/2 | −√2/2 |
| 2 | RR | 225° | (+√2/2, −√2/2) | −√2/2 | −√2/2 |
| 3 | FR | 315° | (+√2/2, +√2/2) | −√2/2 | +√2/2 |

> 方位角不参与本类代码中的显式计算（逆/正解已展开成 `SQRT_2_2` / `SQRT_2_4` 常量），列出来是为了和舵轮分册的 `Wheel_Unit_Cos/Sin` 对照——两者用的是同一套轮位方位。

---

## 2. 参数

| 参数 | 符号 | 默认值 | 类型 | 修改方式 |
|------|------|--------|------|----------|
| 驱动轮半径 | `R` | 0.0761 m | 编译期常量 | 改 `Wheel_Radius`，需重新编译 |
| 轮心到底盘中心距离 | `d` | 0.25075 m | 编译期常量 | 改 `Wheel_Distance_To_Core`，需重新编译（无 setter） |
| 底盘等效质量 | `m` | 10.0 kg | 编译期常量 | 改 `Chassis_Mass`，换配重后重编（仅力控/混控用） |
| 绕 Z 轴等效转动惯量 | `J` | 0.35 kg·m² | 编译期常量 | 改 `Chassis_Inertia`（仅力控/混控用） |
| 轮向电机方向 | `wheel_encoder_direction` | **-1** | 运行时可改 | +1 = 电机读数正向与轮心切向同向，-1 = 反向（本车）；由 `init()` **第二参**指定，四个轮向电机共用一个极性 |
| 控制周期 | `T` | 0.001 s | 运行时可改 | `init()` 或 `set_control_period()`（仅速度里程计用） |
| 编码器里程计位移累积死区 | — | 1e-4 m | 常量 | 累积到该值才积分（抖动互抵，慢速不丢） |
| 编码器里程计角度累积死区 | — | 1e-3 rad | 常量 | 同上 |
| 速度里程计线速度死区 | — | 0.01 m/s | 常量 | 速度口径，与调用频率无关 |
| 速度里程计角速度死区 | — | 0.05 rad/s | 常量 | 同上 |

`d` 必须是**中心到轮心的真实距离**（√(半轴距² + 半轮距²)），不是半轴距，也不是半轮距。这个值错 1 cm，纯旋转时里程计航向就会系统性偏差。

---

## 3. 逆运动学（底盘速度 → 四轮角速度）

### 3.1 推导

轮心位置 `rᵢ = d·(cosφᵢ, sinφᵢ)`，底盘角速度 `ω = vw·ẑ`，则轮心线速度：

```
vᵢ = v_body + ω × rᵢ = (vx − vw·d·sinφᵢ,  vy + vw·d·cosφᵢ)
```

轮子只能沿切向滚动，取切向投影得到轮子线速度：

```
uᵢ = t̂ᵢ · vᵢ = −sinφᵢ·vx + cosφᵢ·vy + vw·d
```

四个轮子代入 φ 后：

```
u₀ = √2/2·(−vx + vy) + d·vw
u₁ = √2/2·(−vx − vy) + d·vw
u₂ = √2/2·( vx − vy) + d·vw
u₃ = √2/2·( vx + vy) + d·vw
```

本实现中轮向电机角速度取 `wᵢ = σ·uᵢ / R`，几何公式本身不带负号，极性全部由轮向方向系数 `σ` 承担：

### 3.2 实现公式

```
w₀ = σ·( √2/2·(−vx + vy) + d·vw ) / R
w₁ = σ·( √2/2·(−vx − vy) + d·vw ) / R
w₂ = σ·( √2/2·( vx − vy) + d·vw ) / R
w₃ = σ·( √2/2·( vx + vy) + d·vw ) / R
```

`wᵢ` 单位为 rad/s，可直接作为轮向电机速度环目标。`σ` 即 `init()` 的 `__wheel_encoder_direction`：电机/编码器正方向与切向 `t̂ᵢ` 同向取 `+1`，反向取 `−1`（本车取 `−1`，见 §8）。

### 3.3 力控（目标加速度 → 四轮轮端力矩）

输入是**底盘目标加速度**（`ax/ay` 单位 m/s²，`aw` 单位 rad/s²），模块内部按 `F = m·a`、`M = J·α` 换算成等效力/力矩，因此质量与惯量是本模块的配置量（编译期常量 `Chassis_Mass` / `Chassis_Inertia`），换配重改常量后重新编译，不动上层。

力分配矩阵就是逆解矩阵的**转置**（功率对偶：`Σ fᵢ·uᵢ ≡ fx·vx + fy·vy + fw·vw`）。逆解里每轮驱动方向 `t̂ᵢ = (−sinφᵢ, cosφᵢ)` 是单位向量，且 `Σ t̂ᵢ·t̂ᵢᵀ = 2I`，故平动项系数为 `1/2`；旋转项由 `Σ fᵢ = fw/d`（逆解 `vw` 系数恒为 `+d`）定出：

```
f₀ (FL) =  (√2/4)·(−fx + fy) + fw/(4d)
f₁ (RL) =  (√2/4)·(−fx − fy) + fw/(4d)
f₂ (RR) =  (√2/4)·( fx − fy) + fw/(4d)
f₃ (FR) =  (√2/4)·( fx + fy) + fw/(4d)
```

`fᵢ` 为轮端力 N，乘轮半径 `R` 得轮端力矩 N·m。输出已乘方向系数 `σ`，所以与 `status.speed` 同属**电机域**，可直接乘轮径下发。`Σfᵢ` 与 `Σ(rᵢ×fᵢ)` 分别精确等于输入的 `(fx, fy)` 与 `fw`。

调用与限幅：

```cpp
// 力控两项在 init() 里一次写齐 (质量/惯量为编译期常量, 不参与 init; 完整签名见 §6)
omni.init(Wheel_Force_Control_Method, /*enc_dir*/ -1.0f, 0.001f,
          /*max_torque*/ 4.9f, /*force_deadband*/ 0.1f);
omni.set_robot_accel_x(ax);                     // 目标加速度, 三轴分别写
omni.set_robot_accel_y(ay);
omni.set_robot_accel_w(aw);
omni.force_calculate_callback();                // 加速度 → 轮端力矩
for (i) drive_motor[i].set_target_torque(omni.get_wheel_torque(i));
```
- **不做加速度限幅**：`ax/ay/aw` 由上层按整车能力裁剪，模块内直接乘质量/惯量成等效力，最后过**单轮力矩上限**（`max_wheel_torque`）；后者是逐轮截断——不像舵轮限速那样等比缩放，因为力控只关心"不出力矩上限"，形态退化的风险小于过流。
- `force_deadband` 以下的小力按 0 输出，抑制静止时的力矩抖动；全向轮无反拖力矩需求时保持默认 0.1 N。
- **无隐式超时**：力控与速控同构，不喂参数就不该解算，不另配超时门限。使能边界就是**目标加速度是否为零**：`set_robot_accel_x/y/w()` 写入，`speed_calculate_callback()` 入口整体清零；不另存一份使能位（两份状态只会同时写、不会分开读，存了反而多一条可能不同步的路径）。
- **自检**：把 `ax = fx/m`、`aw = fw/J` 反代回本式，应还原逆解给出的 `(fx, fy, fw)`；`fx` 单项工况下四轮力应与 §3.2 的轮速符号族一致。

---

## 4. 正运动学（四轮角速度 → 底盘速度）

代码先乘一次方向系数 `wsᵢ = σ·status.speed[i]`（把电机角速度还原成切向速度 `uᵢ` 的符号族），再做加权求和。化简后为：

```
vx =  (√2/4)·R·(−w₀ − w₁ + w₂ + w₃)
vy =  (√2/4)·R·( w₀ − w₁ − w₂ + w₃)
vw =  (R / (4·d))·( w₀ + w₁ + w₂ + w₃)
```

其中 `wᵢ` 即 `status.speed[i]`。

**逆/正解互逆性自检**：把 §3.2 的 `wᵢ` 代入上式应精确还原 `vx/vy/vw`。这是修改任何一处符号后必须跑的验证（本模块的逆解、正解、里程计三处符号是配套的）：

| 速度项 | 用到的轮速组合 | 对应的里程计组合 |
|--------|----------------|------------------|
| `vx` | `−w₀ − w₁ + w₂ + w₃` | `−Δs₀ − Δs₁ + Δs₂ + Δs₃` |
| `vy` | ` w₀ − w₁ − w₂ + w₃` | ` Δs₀ − Δs₁ − Δs₂ + Δs₃` |
| `vw` | ` w₀ + w₁ + w₂ + w₃` | ` Δs₀ + Δs₁ + Δs₂ + Δs₃` |

**为什么系数是 √2/4**：把逆解写成矩阵形式，以四个轮速为行、以 `(vx, vy, vw)` 为列，其 3 列两两正交：

```
c₁ = √2/2·( +1, +1, −1, −1)   ‖c₁‖² = 2
c₂ = √2/2·( −1, +1, +1, −1)   ‖c₂‖² = 2
c₃ =        −d·(  1,  1,  1,  1)   ‖c₃‖² = 4d²
```

所以 `MᵀM = diag(2, 2, 4d²)/R²` 是对角阵，伪逆即"逐列除以列平方和"：`vx = (R/2)·Σc₁ᵢ·wᵢ`，把 `√2/2` 提出后正好是 `√2/4`。正解不需要解线性方程组，这也是它能一帧算完的原因。

---

## 5. 里程计

### 5.1 编码器里程计

先把轮向累计转角差分成本帧线位移（`Δsᵢ = (θᵢ − θᵢ_prev)·R`，首帧只建基准），再用与正解完全同构的组合：

```
dx = (√2/4)·(−Δs₀ − Δs₁ + Δs₂ + Δs₃)
dy = (√2/4)·( Δs₀ − Δs₁ − Δs₂ + Δs₃)
dψ =  (Δs₀ + Δs₁ + Δs₂ + Δs₃) / (4·d)
```

死区是**累积量**而非单帧量：三轴累积不足线 1e-4 m / 角 1e-3 rad 时本帧不积分，位移留在 `odom_residual_*` 里，等后续帧攒够再连同历史位移一次积分。所以慢速运动不会被整帧丢掉（只是最多滞后 1 个死区量），零位噪声的 `+n / −n` 抖动则互相抵消、不会触发——两者都不依赖调用频率。

### 5.2 调用与选择

```cpp
wheel.encoder_odometry_callback();   // 航向由 dψ 自持积分
wheel.imu_odometry_callback();       // 位移由编码器算，航向直接取 IMU
wheel.speed_odometry_callback();     // 必须在 forward_calculate_callback() 之后
```

| 场景 | 用法 |
|------|------|
| 有可靠 IMU | `imu_odometry` + `get_position_xxx`（航向抗打滑） |
| 无 IMU / 短期估计 | `encoder_odometry` + `get_position_xxx` |
| 预测 / 前馈（对打滑不敏感） | `speed_odometry` + `get_speed_position_xxx` |

速度里程计的死区是**瞬时速度**（线 0.01 m/s、角 0.05 rad/s），编码器里程计的死区是**累积位移**（线 1e-4 m / 角 1e-3 rad）；两者都与调用频率无关，低速段编码器里程计同样能输出，只是位姿最多滞后 1 个死区量。两套位姿独立，可同时运行并相减判断打滑程度。

---

## 6. API 速查

| 分类 | 接口 | 说明 |
|------|------|------|
| 初始化 | `init(method, enc_dir, T, max_torque, force_deadband)` | 首参选定控制方式（速控/力控/混控），第二参为轮向电机极性（默认 `-1`，非法值按 +1）；周期非法值回退默认；力控两项**整份校验**；清零全部状态。**几何量与质量/惯量都是编译期常量，不进 init** |
| 配置 | — | 轮心距离 `d` 是编译期常量（`Wheel_Distance_To_Core`），不提供运行期修改入口 |
| 配置 | `set_wheel_encoder_direction(±1)` | 轮向电机极性，非 ±1 / 非有限被拒绝；`init()` 第二参已可指定，此接口用于运行期改 |
| 配置 | `set_control_period(T)` | `T ≤ 0` 或非有限值被拒绝 |
| 输入 | `update_robot_state({vx,vy,vw})` | 逆解目标速度 |
| 输入 | `update_now_state(wheel_state_t, imu_yaw = 0.0f)` | 唯一反馈入口：取 `speed[]` 写 `status.speed`（逐通道校验），取 `angle[]` 提交里程计快照（整帧校验）+ IMU 航向；**须在正解之前写入**，逆解会覆盖 `status.speed`；不传 `imu_yaw` 时按 0 处理，此时应走 `encoder_odometry_callback()` |
| 输入 | `set_wheel_speed(i, w)` | 单轮写入 `status.speed[i]`，`i ≥ 4` 被丢弃 |
| 解算 | `speed_calculate_callback()` | **速控入口**：关断力控输出 + 逆解，输出四轮目标角速度 |
| 解算 | `inverse_calculate_callback()`（protected） | 纯逆运动学（不碰力控状态），仅供 `speed_`/`mixed_calculate_callback()` 内部调用 |
| 解算 | `forward_calculate_callback()` | 输出 `robot_speed` |
| 解算 | `encoder_odometry_callback()` | 编码器里程计（dyaw 自持） |
| 解算 | `imu_odometry_callback()` | 编码器里程计（IMU 航向） |
| 解算 | `speed_odometry_callback()` | 速度里程计 |
| 输出 | `get_wheel_speed(i)` | 单轮角速度，`i ≥ 4` 返回 0 |
| 输出 | `get_robot_speed_vx/vy/vw()` | 正解速度 |
| 输出 | `get_position_x/y/yaw()` | 编码器里程计位姿 |
| 输出 | `get_speed_position_x/y/yaw()` | 速度里程计位姿 |
| 复位 | `reset_position()` / `set_position()` | 编码器里程计：清零（重建基准）/ 重定位 |
| 复位 | `reset_speed_position()` / `set_speed_position()` | 速度里程计：清零 / 重定位 |
| 力控·配置 | `init()` 的 `max_torque` / `force_deadband` 两形参 + 常量 `Chassis_Mass` / `Chassis_Inertia` | 单轮力矩上限与力死区**整份校验**，任一项非法则两项全不改；质量/惯量是编译期常量，改配重就改常量重编，无运行期改参接口 |
| 力控·读取 | 无（无读回接口） | 配置量由上层自己掌握（质量/惯量是常量）；要看本拍是否出力用 `get_wheel_torque(i)` |
| 力控·输入 | `set_robot_accel_x/y/w(v)` | 目标加速度逐轴写入 (m/s² / m/s² / rad/s²)；单轴非有限值只跳过该轴。**零加速度 = 不出力**，无另设使能位 |
| 力控·解算 | `force_calculate_callback()` | 纯力控：加速度 → 轮端力矩（速度目标不参与） |
| 力控·解算 | `mixed_calculate_callback()` | 力速混控：速度逆解 → `get_wheel_speed(i)` + 加速度 → 力矩前馈 |

| 力控·输出 | `get_wheel_torque(i)` | 轮端力矩 N·m；是否被 `max_wheel_torque` 截断，拿它与该上限比就知道，不另设标志位 |

---

## 7. 调用示例

```cpp
static omni_wheel_class chassis_wheel;

void chassis_init(void)
{
    // 一次性写齐: 控制方式 + 轮向电机方向 + 周期 + 力矩上限/死区 (后两项只在力控/混控时生效)
    // 轮向极性 -1 = 本车电机/编码器正方向与轮心切向相反 (不传参也是这个默认值)
    chassis_wheel.init(Wheel_Speed_Control_Method, /*enc_dir*/ -1.0f, 0.001f,
                       /*max_torque*/ 4.9f, /*force_deadband*/ 0.1f);

    // 几何量 (轮径/轮心距离) 与质量/惯量都是编译期常量, 非标准布局直接改常量后重编
}

void chassis_task(void)                     // 1 kHz
{
    // ① 反馈
    wheel_state_t now{};   // 一份快照同时喂运动学(speed)与里程计(angle)
    for (uint8_t i = 0; i < 4; i++)
    {
        now.speed[i] = motor[i].get_speed();        // rad/s，电机侧符号
        now.angle[i] = motor[i].get_total_angle();  // rad，多圈累计
    }
    chassis_wheel.update_now_state(now, imu.get_yaw());

    // ② 正解 + 速度里程计
    chassis_wheel.forward_calculate_callback();
    chassis_wheel.speed_odometry_callback();

    // ③ 编码器里程计（IMU 航向）
    chassis_wheel.imu_odometry_callback();

    // ④ 逆解 + 下发
    chassis_wheel.update_robot_state({vx_target, vy_target, vw_target});
    chassis_wheel.speed_calculate_callback();

    for (uint8_t i = 0; i < 4; i++)
    {
        motor[i].set_speed_ref(chassis_wheel.get_wheel_speed(i));
    }
}
```

### 7.1 上层接驳点

| 上层需求 | 用哪个接口 | 说明 |
|----------|------------|------|
| 轮向速度环目标 | `get_wheel_speed(i)` | 逆解输出，rad/s，已是电机侧符号 |
| 底盘速度反馈 | `get_robot_speed_vx/vy/vw()` | 要求本周期已调 `forward` 且未调 `inverse`（顺序见总览 §5.2 / §5.3） |
| 定位 | `get_position_xxx` / `get_speed_position_xxx` | 两套独立，差值可评估打滑 |
| 复位 / 重定位 | `reset_position` / `set_position` | 复位后下一帧才重建基准 |

---

## 8. 电机方向约定

公式本身**不含硬件极性**：`wᵢ = σ·uᵢ/R`，`σ` 由 `init()` 的 `__wheel_encoder_direction` 给定（本车为 `−1`，麦轮为 `+1`）。`σ` 同时作用于逆解、正解、里程计、力分配四处，所以改一个参数就能整车翻转，且逆解结果喂回正解仍能还原原速度。处理原则：

- 整车方向**全反** → 改 `__wheel_encoder_direction` 的符号（等价于整体取负 `vx/vy/vw`），不改变运动形态。
- **单个轮**方向反 → 只改那一路电机，不要动公式。
- **绝不**只改逆解或只改正解 / 里程计其中一处——四处必须同源同号（自检口径见总览 §3.6(d)）。

---

## 9. 标定与排查

### 9.1 标定顺序

1. **轮径**：发 `vx = 0.3 m/s` 直行 3 s，实测位移 0.9 m；偏差比例直接修正 `Wheel_Radius`。
2. **力臂 `d`**：原地 `vw = 1 rad/s` 转 2π，实测应转 1 圈；航向超转说明 `d` 偏小，欠转说明 `d` 偏大。
3. **轮位顺序**：单独给 `w₀ = +5 rad/s`，观察 FL 轮是否正向转、底盘是否朝对应方向平移。

### 9.2 常见问题（全向轮特有）

| 现象 | 原因 | 处理 |
|------|------|------|
| 只走 `vx` 时里程计 `y` 也涨 | 轮径不一致 / 某轮打滑 | 对比四轮单帧位移，应几乎相等 |
| 直线走 1 m 里程计只有 0.95 m | `R` 偏小 | 按比例修正轮半径常量 |
| 原地旋转航向差 5% | `d` 不准 | 用实测中心到轮心距离 |
| 45° 方向速度偏大 / 偏小 | `√2/2` 假设不成立（轮子实际不是 45°） | 结构问题，需按实际方位角重推 |
| 停转后位姿缓慢爬行 | 正解输入混入了目标轮速 | 检查是否先逆解后正解（总览 §5.3 约束 2） |

通用现象（转向反、整车全反、慢速不动、位姿翻倍等）见 [总览 §8.2](./wheel_system_solution.md) 联调检查表。
