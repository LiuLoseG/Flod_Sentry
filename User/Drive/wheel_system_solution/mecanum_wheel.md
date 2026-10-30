# 麦轮轮系 (Mecanum Wheel)

> 对应类：`mecanum_wheel_class`（[wheel_system_solution.h](./wheel_system_solution.h) / [wheel_system_solution.cpp](./wheel_system_solution.cpp)）
> 通用定义：参考坐标系见 [总览 §3](./wheel_system_solution.md)、轮系定义见 [总览 §4](./wheel_system_solution.md)、执行流程见 [总览 §5](./wheel_system_solution.md)、接口设计原则见 [总览 §7](./wheel_system_solution.md)。

麦轮与全向轮同属"无舵向"轮系：逆解是线性组合，输出直接就是四个轮向电机的目标角速度。区别在于麦轮的等效驱动方向由辊子角度决定，因此横向分量 `vy` 以 **±1** 的权重进入每个轮子（全向轮是 ±√2/2），旋转项系数是 `(a+b)` 而不是 `d`。

---

## 1. 适用构型与几何

4 个麦轮，轮轴与机体系 X 轴平行，辊子与轮面成 45°。本模块按头文件注释的**外八辊子布局**实现：

```
辊子走向（俯视）：      FL "\"      FR "/"
                        RL "/"      RR "\"
小口指向车内，大口指向车外
```

```
                     +X (车头)
                          ↑
        FL [0]            │            FR [3]
      ( +a, +b)           │           ( +a, −b)
               ╲          │          ╱
                ╲         │         ╱
   +Y (左) ←──────────────┼──────────────→ −Y (右)
                ╱         │         ╲
               ╱          │          ╲
        RL [1]            │            RR [2]
      ( −a, +b)           │           ( −a, −b)
```

| 下标 | 轮位 | xᵢ | yᵢ | σᵢₓ | σᵢᵧ |
|------|------|----|----|-----|-----|
| 0 | FL | +a | +b | +1 | −1 |
| 1 | RL | −a | +b | +1 | +1 |
| 2 | RR | −a | −b | −1 | +1 |
| 3 | FR | +a | −b | −1 | −1 |

`a = Wheel_Projection_X`（半轴距），`b = Wheel_Projection_Y`（半轮距），`S = Wheel_Sum_XY = a + b`。

---

## 2. 参数

| 参数 | 符号 | 默认值 | 类型 | 修改方式 |
|------|------|--------|------|----------|
| 驱动轮半径 | `R` | 0.0761 m | 编译期常量 | 改 `Wheel_Radius`，需重新编译 |
| 轮心 X 向投影（半轴距） | `a` | 0.12 m | 编译期常量 | 改 `Wheel_Projection_X`，需重新编译（无 setter） |
| 轮心 Y 向投影（半轮距） | `b` | 0.11 m | 编译期常量 | 改 `Wheel_Projection_Y`，需重新编译（无 setter） |
| 轮向电机方向 | `wheel_encoder_direction` | +1 | 运行时可改 | +1 = 电机读数正向与辊子驱动方向同向，-1 = 反向；由 `init()` **第二参**指定，四个轮向电机共用一个极性 |
| 投影和 | `S = a+b` | 0.23 m | 编译期常量 | `Wheel_Sum_XY`，由 `a`/`b` 自动推导 |
| 底盘等效质量 | `m` | 10.0 kg | 编译期常量 | 改 `Chassis_Mass`，换配重后重编（仅力控/混控用） |
| 绕 Z 轴等效转动惯量 | `J` | 0.35 kg·m² | 编译期常量 | 改 `Chassis_Inertia`（仅力控/混控用） |
| 控制周期 | `T` | 0.001 s | 运行时可改 | `init()` 或 `set_control_period()`（仅速度里程计用） |
| 编码器里程计位移累积死区 | — | 1e-4 m | 常量 | 累积到该值才积分（抖动互抵，慢速不丢） |
| 编码器里程计角度累积死区 | — | 1e-3 rad | 常量 | 同上 |
| 速度里程计线速度死区 | — | 0.01 m/s | 常量 | 速度口径，与调用频率无关 |
| 速度里程计角速度死区 | — | 0.05 rad/s | 常量 | 同上 |

`a` / `b` 是**轮心在机体轴上的投影**，不是轮子外缘尺寸。两者都允许为 0，但 `a + b` 必须 > 0，否则逆/正解退化（除数为 0）。改这两个常量后需重新编译并重做标定。

---

## 3. 逆运动学（底盘速度 → 四轮角速度）

### 3.1 推导

轮心速度（`ω = vw·ẑ`，轮心坐标 `(xᵢ, yᵢ)`）：

```
v_cx = vx − vw·yᵢ
v_cy = vy + vw·xᵢ
```

麦轮靠 45° 辊子把机体系的两个速度分量都耦合到轮面线速度上，因此

```
uᵢ = σᵢₓ·v_cx + σᵢᵧ·v_cy
wᵢ = uᵢ / R
```

代入 §1 的 σ 表：

```
u₀ = ( vx − vw·b) − ( vy + vw·a) =  vx − vy − S·vw
u₁ = ( vx − vw·b) + ( vy − vw·a) =  vx + vy − S·vw
u₂ = −(vx + vw·b) + ( vy − vw·a) = −vx + vy − S·vw
u₃ = −(vx + vw·b) − ( vy + vw·a) = −vx − vy − S·vw
```

### 3.2 实现公式

```
w₀ (FL) = ( vx − vy − S·vw ) / R
w₁ (RL) = ( vx + vy − S·vw ) / R
w₂ (RR) = (−vx + vy − S·vw ) / R
w₃ (FR) = (−vx − vy − S·vw ) / R
```

- `vy` 权重为 (−1, +1, +1, −1)（电机侧口径，含 §1 的 σ 表）：纯左移时**前轴两轮同向、后轴两轮同向，两轴之间反向**——不是"对角轮同向"。
- 容易混淆的是**有效驱动方向**按对角线成对（FL 与 RR 共用一条辊子线，RL 与 FR 共用另一条），而轮子**滚动方向**按前后轴成对；两者相差一个随轮而变的符号，标定时不要拿其中一个去核对另一个。
- 旋转项四个轮子相同（都是 `−S·vw`）：纯旋转时四轮同向，这与麦轮"斜向合力产生力矩"的物理事实一致。
- 若实测某个方向变成斜移（例如发 `vy` 却斜着走），说明实际辊子布局与 §1 的 σ 表不符，需按实际布局重推，**不能**靠单轮取负硬凑。

### 3.3 力控（目标加速度 → 四轮轮端力矩）

输入是**底盘目标加速度**（`ax/ay` 单位 m/s²，`aw` 单位 rad/s²），模块内部按 `F = m·a`、`M = J·α` 换算成等效力/力矩，因此质量与惯量是本模块的配置量（编译期常量 `Chassis_Mass` / `Chassis_Inertia`），换配重改常量后重新编译，不动上层。

力分配矩阵是逆解矩阵的**转置**（功率对偶：`Σ fᵢ·uᵢ ≡ fx·vx + fy·vy + fw·vw`）。逆解每轮驱动方向 `dᵢ` 长度为 `√2` 且 `Σ dᵢ·dᵢᵀ = 4I`，故平动项系数为 `1/4`；旋转项由 `Σ fᵢ = −fw/S`（`S = a + b`，逆解 `vw` 系数恒为 `−S`）定出：

```
f₀ (FL) = ( fx − fy)/4 − fw/(4S)
f₁ (RL) = ( fx + fy)/4 − fw/(4S)
f₂ (RR) = (−fx + fy)/4 − fw/(4S)
f₃ (FR) = (−fx − fy)/4 − fw/(4S)
```

平动项符号与 §3.2 的 `vx/vy` 权重逐轮一一对应（例如 `fx` 项 `(+,+,−,−)` 与轮速 `vx` 权重同族），所以**逆解结果喂回本式可还原 `(fx, fy, fw)`**——这是改任何符号后必过的自检。

调用与限幅同全向轮（`init()` 写入力矩上限/死区 → `set_robot_accel_x/y/w` → `force_calculate_callback` → `get_wheel_torque`）；`S ≤ 0` 时旋转项强制为 0，避免除零。

---

## 4. 正运动学（四轮角速度 → 底盘速度）

```
vx =  (R/4)·( w₀ + w₁ − w₂ − w₃)
vy =  (R/4)·(−w₀ + w₁ + w₂ − w₃)
vw = −(R/(4·S))·( w₀ + w₁ + w₂ + w₃)
```

**为什么是简单的 1/4 平均**：逆解矩阵

```
        ⎡ σ₀ₓ  σ₀ᵧ  −S ⎤
M = (1/R)·⎢ σ₁ₓ  σ₁ᵧ  −S ⎥      σₓ = (+1,+1,−1,−1), σᵧ = (−1,+1,+1,−1)
        ⎢ σ₂ₓ  σ₂ᵧ  −S ⎥
        ⎣ σ₃ₓ  σ₃ᵧ  −S ⎦
```

的三**列**两两正交：

```
Σσₓσᵧ = 0,  Σσₓ = 0,  Σσᵧ = 0   ⇒  MᵀM 的非对角元素全为 0
‖列₁‖² = 4,  ‖列₂‖² = 4,  ‖列₃‖² = 4S²
```

因此 `MᵀM = diag(4, 4, 4S²)/R²` 是对角阵，伪逆 `(MᵀM)⁻¹Mᵀ` 退化成"逐列除以列平方和 + `Mᵀ`"，不需要解方程组。这就是正解能写成三个独立线性组合（且系数恰好是 1/4、1/4、1/(4S)）的原因。

> 注意是**列**正交而不是行正交：两个轮子的行内积等于 `σᵢₓσⱼₓ + σᵢᵧσⱼᵧ + S² ≠ 0`（对角轮之间恒为 `S²`）。这也是"单个轮子单独转，车不会只沿一个方向走"的数学解释。

**逆/正解互逆性自检**：

| 速度项 | 轮速组合 | 里程计组合 |
|--------|----------|------------|
| `vx` | `w₀ + w₁ − w₂ − w₃` | `Δs₀ + Δs₁ − Δs₂ − Δs₃` |
| `vy` | `−w₀ + w₁ + w₂ − w₃` | `−Δs₀ + Δs₁ + Δs₂ − Δs₃` |
| `vw` | `−(w₀ + w₁ + w₂ + w₃)` | `−(Δs₀ + Δs₁ + Δs₂ + Δs₃)` |

---

## 5. 里程计

### 5.1 编码器里程计

```
dx = ( Δs₀ + Δs₁ − Δs₂ − Δs₃) / 4
dy = (−Δs₀ + Δs₁ + Δs₂ − Δs₃) / 4
dψ = −( Δs₀ + Δs₁ + Δs₂ + Δs₃) / (4·S)        Δsᵢ = (θᵢ − θᵢ_prev)·R
```

里程计的 `S = Wheel_Sum_XY` 与逆解同源（同一个编译期常量），改 `a`/`b` 后旋转增益会同步变化，需重编并重做一次纯旋转标定。

死区是**累积量**而非单帧量：三轴累积不足线 1e-4 m / 角 1e-3 rad 时本帧不积分，位移留在 `odom_residual_*` 里，等后续帧攒够再连同历史位移一次积分。所以慢速运动不会被整帧丢掉（最多滞后 1 个死区量），零位噪声的 `+n / −n` 抖动则互相抵消、不会触发——两者都不依赖调用频率。

### 5.2 调用与精度特点

```cpp
wheel.encoder_odometry_callback();   // 航向由 dψ 自持积分
wheel.imu_odometry_callback();       // 位移由编码器算，航向直接取 IMU
wheel.speed_odometry_callback();     // 必须在 forward_calculate_callback() 之后
```

辊子存在不可避免的**打滑与侧向滑移**，编码器里程计误差大于全向轮 / 舵轮，尤其横移与原地旋转时：

- 长距离定位优先 `imu_odometry`（航向交给 IMU）。
- 用两套里程计的差值判断打滑，持续增大说明辊子接触不良（离地、磨损、地面太滑）。
- 横移时 `dx`（前进方向）明显串扰，优先怀疑 `a` / `b` 写反。

---

## 6. API 速查

| 分类 | 接口 | 说明 |
|------|------|------|
| 初始化 | `init(method, enc_dir, T, max_torque, force_deadband)` | 首参选定控制方式（速控/力控/混控），第二参为轮向电机极性（非法值按 +1）；周期非法值回退默认；力控两项**整份校验**；清零全部状态。**几何量与质量/惯量都是编译期常量，不进 init** |
| 配置 | — | `a`/`b`/`S` 分别是 `Wheel_Projection_X` / `Wheel_Projection_Y` / `Wheel_Sum_XY` 编译期常量，无运行期修改入口 |
| 配置 | `set_wheel_encoder_direction(±1)` | 轮向电机极性，非 ±1 / 非有限被拒绝；`init()` 第二参已可指定，此接口用于运行期改 |
| 配置 | `set_control_period(T)` | 仅影响速度里程计 |
| 输入 | `update_robot_state({vx,vy,vw})` | 逆解目标速度 |
| 输入 | `update_now_state(wheel_state_t, imu_yaw = 0.0f)` | 唯一反馈入口：取 `speed[]` 写 `status.speed`（逐通道校验），取 `angle[]` 提交里程计快照（整帧校验）+ IMU 航向；**须在正解之前写入**，逆解会覆盖 `status.speed`；不传 `imu_yaw` 时按 0 处理，此时应走 `encoder_odometry_callback()` |
| 输入 | `set_wheel_speed(i, w)` | 单轮写入 `status.speed[i]` |
| 解算 | `speed_calculate_callback()` | **速控入口**：关断力控输出 + 逆解，输出四轮目标角速度 |
| 解算 | `inverse_calculate_callback()`（protected） | 纯逆运动学（不碰力控状态），仅供 `speed_`/`mixed_calculate_callback()` 内部调用 |
| 解算 | `forward_calculate_callback()` | 输出 `robot_speed` |
| 解算 | `encoder_odometry_callback()` / `imu_odometry_callback()` | 编码器里程计两种航向来源 |
| 解算 | `speed_odometry_callback()` | 速度里程计 |
| 输出 | `get_wheel_speed(i)` | 单轮角速度，`i ≥ 4` 返回 0 |
| 输出 | `get_robot_speed_vx/vy/vw()` | 正解速度 |
| 输出 | `get_position_x/y/yaw()` | 编码器里程计位姿 |
| 输出 | `get_speed_position_x/y/yaw()` | 速度里程计位姿 |
| 复位 | `reset_position()` / `set_position()` | 编码器里程计清零（重建基准）/ 重定位 |
| 复位 | `reset_speed_position()` / `set_speed_position()` | 速度里程计清零 / 重定位 |
| 力控·配置 | `init()` 的 `max_torque` / `force_deadband` 两形参 + 常量 `Chassis_Mass` / `Chassis_Inertia` | 单轮力矩上限与力死区**整份校验**，任一项非法则两项全不改；质量/惯量是编译期常量，改配重就改常量重编，无运行期改参接口 |
| 力控·读取 | 无（无读回接口） | 配置量由上层自己掌握（质量/惯量是常量）；要看本拍是否出力用 `get_wheel_torque(i)` |
| 力控·输入 | `set_robot_accel_x/y/w(v)` | 目标加速度逐轴写入 (m/s² / m/s² / rad/s²)；单轴非有限值只跳过该轴。**零加速度 = 不出力**，无另设使能位 |
| 力控·解算 | `force_calculate_callback()` | 纯力控：加速度 → 轮端力矩（速度目标不参与） |
| 力控·解算 | `mixed_calculate_callback()` | 力速混控：速度逆解 → `get_wheel_speed(i)` + 加速度 → 力矩前馈 |
| 力控·输出 | `get_wheel_torque(i)` | 轮端力矩 N·m；是否被 `max_wheel_torque` 截断，拿它与该上限比就知道，不另设标志位 |

---

## 7. 调用示例

```cpp
static mecanum_wheel_class chassis_wheel;

void chassis_init(void)
{
    // 一次性写齐: 控制方式 + 轮向电机方向 + 周期 + 力矩上限/死区 (后两项只在力控/混控时生效)
    chassis_wheel.init(Wheel_Speed_Control_Method, /*enc_dir*/ 1.0f, 0.001f,
                       /*max_torque*/ 4.9f, /*force_deadband*/ 0.1f);

    // 几何量 (轮径/轮心投影) 与质量/惯量都是编译期常量, 非标准布局直接改常量后重编
}

void chassis_task(void)                         // 1 kHz
{
    wheel_state_t now{};   // 一份快照同时喂运动学(speed)与里程计(angle)
    for (uint8_t i = 0; i < 4; i++)
    {
        now.speed[i] = motor[i].get_speed();
        now.angle[i] = motor[i].get_total_angle();
    }
    chassis_wheel.update_now_state(now, imu.get_yaw());

    chassis_wheel.forward_calculate_callback();     // 先正解
    chassis_wheel.speed_odometry_callback();
    chassis_wheel.imu_odometry_callback();

    chassis_wheel.update_robot_state({vx_target, vy_target, vw_target});
    chassis_wheel.speed_calculate_callback();     // 速控流水线：关断力控输出 + 逆解

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
| 定位 | `get_position_xxx` / `get_speed_position_xxx` | 两套独立，差值可评估辊子打滑 |
| 复位 / 重定位 | `reset_position` / `set_position` | 复位后下一帧才重建基准 |
| 几何变更 | 无运行期接口 | 改 `Wheel_Projection_X/Y` 常量后重新编译 |

> 麦轮与全向轮字段共用规则相同（轮速字段先写反馈、后写指令），但麦轮逆解**不含整体负号**（`σ = +1`），正解也不做还原。

---

## 8. 标定与排查

### 8.1 标定顺序

1. **轮径 `R`**：`vx = 0.3 m/s` 直行 3 s，按实测位移比例修正。
2. **投影和 `S`**：原地 `vw = 1 rad/s` 转 2π，航向超转说明 `S` 偏小。
3. **`a` / `b` 分配**：分别只发 `vx`、只发 `vy`，看是否严格正交。
4. **单轮点动**：给 `w₀ = +1 rad/s`（其余 0），用正解验证符号组合是否符合 §1 的 σ 表；四轮逐一验证，不符先查电机反馈符号。

### 8.2 常见问题（麦轮特有）

| 现象 | 原因 | 处理 |
|------|------|------|
| 前进正常，横移变斜移 | 辊子布局与 σ 表不符（内 / 外八搞反） | 按实际布局重推，或核对辊子朝向 |
| 纯平移时 `vw` 不为 0 | `S` 与几何不符，或某轮打滑 | 对比四轮单帧位移 |
| 原地旋转时四轮速度不等 | `a`、`b` 与实车不符（轮子不在矩形角上） | 用实测轮心坐标 |
| 横移速度仅为设定值的 70~80% | 辊子效率损耗（物理事实） | 前馈补偿或接受；不影响里程计一致性 |
| 长时间运行航向漂移大 | 麦轮打滑 | 改用 IMU 航向路线 |

通用现象（转向反、整车全反、慢速不动、位姿翻倍等）见 [总览 §8.2](./wheel_system_solution.md) 联调检查表。
