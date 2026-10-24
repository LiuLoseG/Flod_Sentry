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

> 方位角不参与本类代码中的显式计算（逆/正解已展开成 `SQRT_2_2` / `SQRT_2_4` 常量），列出来是为了和舵轮分册的 `Wheel_Azimuth` 对照——两者用的是同一套轮位方位。

---

## 2. 参数

| 参数 | 符号 | 默认值 | 类型 | 修改方式 |
|------|------|--------|------|----------|
| 驱动轮半径 | `R` | 0.0761 m | 编译期常量 | 改 `Wheel_Radius`，需重新编译 |
| 轮心到底盘中心距离 | `d` | 0.25075 m | 运行时可改 | `init()` 或 `set_distance_to_core()` |
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

本实现中轮向电机角速度取 `wᵢ = −uᵢ / R`：

### 3.2 实现公式

```
w₀ = −( √2/2·(−vx + vy) + d·vw ) / R
w₁ = −( √2/2·(−vx − vy) + d·vw ) / R
w₂ = −( √2/2·( vx − vy) + d·vw ) / R
w₃ = −( √2/2·( vx + vy) + d·vw ) / R
```

`wᵢ` 单位为 rad/s，可直接作为轮向电机速度环目标（取负号是因为该轮系电机/编码器安装正方向与麦轮相反，见 §8）。

---

## 4. 正运动学（四轮角速度 → 底盘速度）

代码先做一次整体取负 `wsᵢ = −wheel_speed[i]`（把电机角速度还原成 `uᵢ` 的符号族），再做加权求和。化简后为：

```
vx =  (√2/4)·R·( w₀ + w₁ − w₂ − w₃)
vy =  (√2/4)·R·(−w₀ + w₁ + w₂ − w₃)
vw = −(R / (4·d))·( w₀ + w₁ + w₂ + w₃)
```

其中 `wᵢ` 即 `status.wheel_speed[i]`。

**逆/正解互逆性自检**：把 §3.2 的 `wᵢ` 代入上式应精确还原 `vx/vy/vw`。这是修改任何一处符号后必须跑的验证（本模块的逆解、正解、里程计三处符号是配套的）：

| 速度项 | 用到的轮速组合 | 对应的里程计组合 |
|--------|----------------|------------------|
| `vx` | `w₀ + w₁ − w₂ − w₃` | `Δs₀ + Δs₁ − Δs₂ − Δs₃` |
| `vy` | `−w₀ + w₁ + w₂ − w₃` | `−Δs₀ + Δs₁ + Δs₂ − Δs₃` |
| `vw` | `−(w₀ + w₁ + w₂ + w₃)` | `−(Δs₀ + Δs₁ + Δs₂ + Δs₃)` |

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
dx = (√2/4)·( Δs₀ + Δs₁ − Δs₂ − Δs₃)
dy = (√2/4)·(−Δs₀ + Δs₁ + Δs₂ − Δs₃)
dψ = −( Δs₀ + Δs₁ + Δs₂ + Δs₃) / (4·d)
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
| 初始化 | `init(d, T)` | 非法 `d`/`T` 回退默认值；清零全部状态 |
| 配置 | `set_distance_to_core(d)` | `d ≤ 0` 或非有限值被拒绝 |
| 配置 | `set_control_period(T)` | `T ≤ 0` 或非有限值被拒绝 |
| 输入 | `update_robot_state({vx,vy,vw})` | 逆解目标速度 |
| 输入 | `update_now_state({w0..w3})` | 轮速反馈（写 `status.wheel_speed`）；**须在正解之前写入**，逆解会覆盖同一字段 |
| 输入 | `set_wheel_speed(i, w)` | 单轮写入 `status.wheel_speed`，`i ≥ 4` 被丢弃 |
| 输入 | `update_odom_state({θ0..θ3}, imu_yaw)` | 编码器累计角 + IMU 航向 |
| 解算 | `inverse_calculate_callback()` | 输出四轮目标角速度 |
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

---

## 7. 调用示例

```cpp
static omni_wheel_class chassis_wheel;

void chassis_init(void)
{
    chassis_wheel.init(0.25075f, 0.001f);   // d = 0.25075 m, T = 1 ms
}

void chassis_task(void)                     // 1 kHz
{
    // ① 反馈
    wheel_speed_t now{};
    wheel_angle_t ang{};
    for (uint8_t i = 0; i < 4; i++)
    {
        now.wheel_speed[i] = motor[i].get_speed();        // rad/s，电机侧符号
        ang.angle[i]       = motor[i].get_total_angle();  // rad，多圈累计
    }
    chassis_wheel.update_now_state(now);
    chassis_wheel.update_odom_state(ang, imu.get_yaw());

    // ② 正解 + 速度里程计
    chassis_wheel.forward_calculate_callback();
    chassis_wheel.speed_odometry_callback();

    // ③ 编码器里程计（IMU 航向）
    chassis_wheel.imu_odometry_callback();

    // ④ 逆解 + 下发
    chassis_wheel.update_robot_state({vx_target, vy_target, vw_target});
    chassis_wheel.inverse_calculate_callback();

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

逆解四个式子整体取负、正解再整体取负还原，等价于 `wᵢ = σ·uᵢ/R`，`σ = −1`。它反映的是**该车电机 / 编码器的安装正方向**（麦轮为 `σ = +1`）。处理原则：

- 整车方向**全反** → 改上层电机反馈 / 指令符号，等价于整体取负 `vx/vy/vw`，不改变运动形态。
- **单个轮**方向反 → 只改那一路电机，不要动公式。
- **绝不**只改逆解或只改正解 / 里程计其中一处——三处必须同源同号（自检口径见总览 §3.6(d)）。

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
