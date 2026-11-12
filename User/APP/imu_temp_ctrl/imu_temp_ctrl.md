# imu_temp_ctrl 模块说明（板载 BMI088 IMU 采样 / 恒温 / 姿态解算）

> 代码：[imu_temp_ctrl.cpp](./imu_temp_ctrl.cpp)、[imu_temp_ctrl.h](./imu_temp_ctrl.h)
> 驱动：[BMI088driver.cpp](../../Drive/bmi088/BMI088driver.cpp)、[BMI088Middleware.cpp](../../Drive/bmi088/BMI088Middleware.cpp)
> 算法：[QuaternionEKF.cpp](../../Drive/algorithm/QuaternionEKF/QuaternionEKF.cpp)
> 规范依据：[RoboMaster电控C++开发规范](../../Docs/RoboMaster电控C++开发规范.md) §5.1 单位、§6 RTOS、§8 数学与算法、§10.2 文档结构

## 1. 系统架构

### 1.1 文件结构

| 文件                  | 职责                                                                                                        |
| --------------------- | ----------------------------------------------------------------------------------------------------------- |
| `BMI088Middleware.cpp` | 片选、SPI 全双工突发收发（`BMI088_transmit_receive`）、延时                                                   |
| `BMI088driver.cpp`     | 寄存器读写、初始化自检、加速度/陀螺突发读取（`BMI088_read_imu`）、温度读取（`BMI088_read_temperature`）      |
| `QuaternionEKF.cpp`    | 六维状态四元数 EKF（四元数 + 陀螺零偏估计 + 卡方检验）                                                       |
| `imu_temp_ctrl.cpp`    | DRDY 同步采样、恒温 PID、零漂/刻度/杆臂补偿、时间账本、姿态解算与发布                                        |

`imu_temp_ctrl.cpp` 内部按「参数 → 状态 → 状态访问与工具 → 各步骤 → 单步 → 任务 → 对外接口 → ISR」分区，
函数只依赖定义在它之前的部分，**源文件内没有任何前置声明**；`imu_task` 的声明放在头文件里，
与 CubeMX 生成的弱定义形成编译期一致性检查。

### 1.2 数据流

```
BMI088 --SPI2(6.25MHz)--> BMI088driver --2 次突发/帧--> imu_temp_ctrl
                                                   |-- DRDY(1kHz) --> EXTI --> imu_task
                                                   |                              |
                                                   |                              +--> 野值抑制
                                                   |                              +--> 温控节拍(100Hz) --> PID --> TIM3_CH4
                                                   |                              +--> 零偏/刻度 --> 杆臂 --> EKF --> imu_frame
                                                   |-- 温度 1 次突发(100Hz) ------+
```

单帧处理顺序固定为：**读传感器 → 野值抑制 → 温控节拍 → 状态机（时间账本 + 姿态）**。

### 1.3 硬件与采样配置

- 通信：SPI2，`CLKPolarity = HIGH` / `CLKPhase = 2EDGE`，软件 NSS；
  CubeMX 配 3.125 MHz，`BMI088_com_init()` 运行时改为 **6.25 MHz**（BMI088 上限 10 MHz）。
- 采样：BMI088 陀螺仪 1kHz DRDY（INT3，低有效）触发 PE12 的 `EXTI15_10`。
  ISR 只记录 DWT 间隔与序号并通知任务，**SPI 读取与浮点解算全部在任务上下文**执行。
- 加热：TIM3_CH4，`ARR + 1 = 10000`、`PSC + 1 = 1` ⇒ 载波 275 MHz / 10000 = **27.5 kHz**；
  该 `pwm_class` 实例在初始化时限幅 `15 %`（1500/10000），最终写 CCR 前还会再夹一次。
- 任务：`imu_task`，优先级 `High`(40)，静态分配，栈 ≥ 1024 words。
- 温度：每 10 帧读一次（100 Hz），与温控节拍同拍。

> 载波频率的来由：1.1 kHz 载波（旧值 `PSC + 1 = 24`）会激发 41 Hz 结构共振，加热一开
> gy 的 20~60 Hz 分量就到 1.2 °/s rms；27.5 kHz 远高于 BMI088 的 116 Hz 模拟带宽，
> 且电流纹波降为 1/24。详见 `gimbal.cpp` 的陷波注释与 §14 变更记录 1.0。

### 1.4 命名与设计约定（对照 [开发规范](../../Docs/RoboMaster电控C++开发规范.md)）

| 规范条目 | 本模块落点 |
| -------- | ---------- |
| §2.2 类型命名 | `imu_status_e`、`imu_data_resolve_t`、`imu_time_base_t`、`imu_gyro_glitch_t`、`imu_accel_lever_t` |
| §2.3 枚举成员 | `Imu_Status_Uninit / Init_Fail / Wait_Temp / Calibrating / Running`（`模块_类别_取值`） |
| §2.4 语义前缀/后缀 | `pre[] / pre_valid_flag / pre_drdy_dwt / pre_elapsed_dwt`（上一拍）、`last_heater_duty`（上一次下发）、`ekf_ready_flag / attitude_valid_flag / trim_seeded_flag / *_started_flag`（布尔） |
| §2.5 函数命名 | 初始化 `imu_init`；测量更新 `imu_update_measure / _temperature / _gyro_calib / _wait_temp`；周期计算 `imu_calculate_running / _attitude / _yaw_trim`；任务 `imu_task` |
| §2.6 参数集中定义 | 全部模块参数在源文件「1. 参数」分区；整机级物理参数在 `robot_def.h`；源码无裸魔法值 |
| §5.1 SI 单位 | rad / rad/s / m/s² / ℃ / s，变量名与注释均带单位 |
| §5.2 宏后缀 | `_S`、`_MS`、`_C`、`_RAD_S`、`_RAD_S2`、`_M_S2`、`_FRAMES`、`_THRESHOLD`、`_LIMIT`、`_MAX`、`_MIN` |
| §8.1 复用现成工具 | `basic_math_constrain()`（杆臂补偿量、加热输出限幅）、`basic_math_is_invalid_float()`（NaN/Inf 前置判定） |
| §9.4 可观测性 | 内部状态全部是文件作用域 `static` 符号，Ozone 可按名字直接观察（如 `yaw_bias_trim`、`ekf_ready_flag`） |
| §4.3 模块三件套 | 状态结构体 `imu_data_resolve_t` + 全局唯一实例 `imu_frame` + `imu_task` |

两处**有意偏离**，原因记录在此，后续不要"顺手改回去"：

1. **参数宏放在 `.cpp` 而不是头文件宏区**：`IMU_TIME_BASE` / `IMU_GYRO_GLITCH` / `IMU_GYRO_CALIB` /
   `IMU_ACCEL_LEVER` / `IMU_HEATER_POLICY` 五组算法块按 `BEGIN/END` 标记整段抽取做主机侧测试，
   宏、结构体、函数必须留在同一块内自包含；`gimbal` / `chassis` / `shoot` 的参数同样在 `.cpp`。
2. **不使用 `imu_temp_ctrl_t` 汇总状态**：同上，`imu_time_base_t` 等类型位于标记块内，若移入头文件会破坏抽取。
   `imu_frame` 已承担对外唯一状态结构体的角色，内部状态用文件作用域 `static` 保证 Ozone 可搜索。

## 2. 数据类型定义

```cpp
typedef enum
{
    Imu_Status_Uninit = 0,  // 未初始化
    Imu_Status_Init_Fail,   // BMI088 初始化失败, 任务内周期重试
    Imu_Status_Wait_Temp,   // 等待加热到目标温度(仅自标定模式)
    Imu_Status_Calibrating, // 陀螺零漂标定中(仅自标定模式)
    Imu_Status_Running,     // 正常解算
} imu_status_e;

typedef struct
{
    float gyro[3];              // 已扣零漂角速度, rad/s
    float accel[3];             // 加速度, m/s^2; 解算态已做杆臂补偿
    float temperature;          // 芯片温度, ℃
    euler_angle_t euler_angle;  // 欧拉角, rad
    float yaw_total;            // yaw 连续多圈角, rad
    float dt;                   // 本拍姿态积分步长, s
    float ref_temperature;      // 目标温度, ℃
    float heater_out;           // 加热输出, 与 TIM3 ARR 同量纲(0 ~ 1500)
    imu_status_e status;        // 工作状态(唯一真源)
    uint8_t data_valid;         // 1 = 本周期传感器数据有效
    uint32_t sample_count;      // 采样帧计数(温控分频基准)
} imu_data_resolve_t;
```

单位约定：角度 `rad`、角速度 `rad/s`、加速度 `m/s^2`、温度 `℃`、时间 `s`。
坐标系：机体 X=前、Y=左、Z=上（右手系），yaw 绕 Z 逆时针为正，角度自 +X 起算；yaw ∈ (-π, π]。

`Imu_Status_Wait_Temp` / `Imu_Status_Calibrating` 只在 `IMU_GYRO_AUTO_CALIB_ENABLE = 1` 时可达，
默认配置（= 0）下状态机只有 `Uninit / Init_Fail / Running` 三态。

## 3. 接口说明

| 接口                                                    | 说明                                                                 |
| ------------------------------------------------------- | -------------------------------------------------------------------- |
| `void imu_task(void *argument)`                         | 任务入口；名称必须与 CubeMX Entry Function 完全一致（区分大小写）     |
| `float imu_get_roll_rad()` / `imu_get_pitch_rad()` / `imu_get_yaw_rad()` | 欧拉角，rad                                          |
| `float imu_get_yaw_total_rad(void)`                     | 连续多圈 yaw，rad（小陀螺/大 yaw 用）                                |
| `uint8_t imu_is_ready(void)`                            | 1 = 已进入解算 且 数据有效 且 EKF 正常产出                           |
| `extern volatile imu_data_resolve_t imu_frame`          | 解算结果结构体（姿态、角速度、温度、加热输出、状态）                  |

`imu_frame` 由 IMU 任务写、其余任务读：单个 32 位 float 字段读写原子；多字段严格一致时请自行关中断拷贝。
**`imu_frame.status` 是工作状态的唯一真源**，模块内不再保留第二份副本。

姿态可用性已与传感器数据有效性解耦：EKF 异常时 `imu_is_ready()` 返回 0，但 `data_valid` 仍为 1、
加热照常闭环。温控只依赖 `temperature`，与姿态无关。

## 4. 初始化流程

1. `heater_pwm` 绑定 `htim3 / TIM_CHANNEL_4`，配置 15 % 实例上限后以 0 % 启动（上电安全态）。
2. `IMU_QuaternionEKF_Init(10, 0.001, 1e7, 1.0, 0.001, 0.001)`；
   随后 `ekf_ready_flag = IMU_QuaternionEKF_IsReady()` 校验 16 个矩阵指针。
3. 温度 PID 装载：`pid_set_config_pos(Kp, Ki, Kd, integral_limit, max_out)`，`ref_temperature = 50 ℃`。
4. `BMI088_init()`：加速度计/陀螺仪各做「读 chip id → 软复位 → 读 chip id → 写配置并回读校验」，
   单次尝试 3 遍、间隔 20 ms；失败置 `Imu_Status_Init_Fail`，任务内每 500 ms 重试
   （旧版是 `while (BMI088_init());` 死等，传感器缺失会永久阻塞）。
5. 初始化成功 → `imu_runtime_reset()` 清空全部运行时状态（四元数、账本、野值/差分基准、
   静止判定、trim 定标、姿态有效标志、DWT 基准），再进入 `Running`（或自标定模式的 `Wait_Temp`）。

## 5. 采样与调度

### 5.1 DRDY 同步

`imu_task` 等待线程标志 `IMU_DRDY_EVENT_FLAG`：

| 情形                | 处理                                                                 |
| ------------------- | -------------------------------------------------------------------- |
| 正常唤醒            | 用 DWT 实测总间隔 `elapsed_dt` 与单帧周期 `drdy_dt` 调用一次单步       |
| 多次 DRDY 合并      | 只读一次最新帧，但 `elapsed_dt` 是真实经过时间，不丢转角               |
| 假唤醒（无新帧）    | 只在 `Running` 态记账，等下一个有效帧补上                             |
| 等待超时（10 ms）   | 断加热输出（保留积分器），照常读一次传感器；连续 3 次 → `Init_Fail`    |
| `Init_Fail`         | 1 ms 维护重试计时器，每 500 ms 重试 `BMI088_init()`；断加热并清积分    |

### 5.2 时间账本（yaw 精度的核心）

yaw 在 EKF 中无绝对观测，只能靠 z 轴角速度积分，**丢时间即丢转角**。

```
每拍: pending_s += elapsed_dt              (上限 20 ms)
      取 dt = min(pending_s, 9 ms), pending_s -= dt
      dt <= 0 → 本拍不推进 EKF
```

- 单次积分步长上限 9 ms，必须小于 `QuaternionEKF` 内部 0.01 s 的 dt 复位阈值；
- 余额留在账本里由后续几拍排空，因此偶发的一两次通知延迟不会丢掉时间；
- 账本为空时**绝不返回最小步长**（那等于凭空造时间）；
- 姿态解算失败时清空账本（EKF 已复位，旧时间不再有意义）。

### 5.3 两个时间量必须分开

| 时间量       | 含义                            | 用途                                   |
| ------------ | ------------------------------- | -------------------------------------- |
| `frame_dt`   | 相邻 DRDY 的实测单帧周期，限幅 0.5~9 ms | 野值变化率判据、杆臂差分、未解算时的 `dt` 发布 |
| `elapsed_dt` | 距上一次进入单步的真实经过时间  | 时间账本、长间隔判据、自标定时长       |

合成一个数会同时放宽野值阈值并把真实时间截短。

## 6. 解算链路

### 6.1 陀螺野值抑制（`imu_gyro_glitch_filter`）

单帧 SPI 位错 / DRDY 竞争会给出几百~上千 °/s 的假角速度，而 EKF 对 z 轴没有任何绝对观测，
**一次野值就把 yaw 永久推偏且永不恢复**（实测抓到过 −1420 °/s 的注入，yaw 当场跳 −2.15°）。

| 判据       | 阈值                                                       | 行为                                             |
| ---------- | ---------------------------------------------------------- | ------------------------------------------------ |
| 逐轴超量程 | \|ω\| > 36 rad/s（±2000 dps = 34.9）                       | **永不放行**，保持上一拍                       |
| 逐轴变化率 | \|ω(k) − ω(k−1)\| > 10000 rad/s² × dt（1 ms 时 = 10 rad/s） | 先保持上一拍；下一帧仍偏离则认定为真实机动并放行 |
| 长间隔     | `elapsed_dt > 4 × frame_dt`                                | 上一拍作废，只保留量程判据                       |

- 孤立野值：下一帧即回到正常值，被完全剔除；真实阶跃：进、出各滞后 1 帧，误差互相抵消；
- 该函数必须在**任何积分之前**调用，且只用 `frame_dt`（用累计积分步长会把阈值放宽 9 倍）。

### 6.2 零偏与刻度（`imu_gyro_calibrate`）

```
最终角速度 = (原始角速度 − gyro_bias[]) × gyro_scale[] (− yaw_bias_trim, 仅 z 轴)
```

- `gyro_bias[]`、`gyro_scale[]` 都是**原始域标定量**，顺序不能反（否则残留 `(scale−1)·bias`）；
- 默认值来自实车实测（`gyro_scale[2] = 0.9936`，其余为 1.0）；
- `IMU_GYRO_AUTO_CALIB_ENABLE = 1` 时改为「温度稳定 3 s 后采集 1.0 s（丢弃 |ω| > 0.15 rad/s 的运动帧）
  按时间加权求均值」，超时 5 s 按已采样本结算，被扰动累计 2 s 则放弃；
- x/y 轴由 EKF 在线估计并叠加修正；z 轴 EKF 不强估计（`GyroBias[2]` 恒 0），
  精度完全取决于这里的初值 + 运行期静止残差定标。

### 6.3 静止残差零漂修正（`imu_calculate_yaw_trim`）

| 判据       | 处理            | 阈值                            |
| ---------- | --------------- | ------------------------------- |
| 角速度模长 | 0.2 s 一阶低通  | < 0.020 rad/s（≈1.15 °/s）     |
| 加速度模长 | 0.2 s 一阶低通  | \|‖a‖ − 9.80665\| < 0.5 m/s² |

- 两路输入**都必须低通**：瞬时角速度噪声峰值可达阈值本身，瞬时加速度模长也会偶发跳到 9.25 m/s²，
  都会持续打断静止确认（历史故障：静止确认永不成立、yaw 以 0.19 °/s 漂移）。
- 连续静止 0.5 s 后：
  1. **首次定标**：再平均 0.5 s（时间加权）一次性写入 `yaw_bias_trim`，避免从 0 慢积分导致漂移数分钟；
  2. **慢积分跟踪**：增益 `0.05 /s`（时间常数 ≈20 s），限幅 `±0.02 rad/s`。
- 两个阶段都叠加**瞬时角速度门控**（`|ω| > 0.15 rad/s` 的帧不计入），
  因为低通判定比真实运动滞后 3~4 拍，机动起始的几帧会被误判为静止而污染 trim
  （实测单次机动就能留下 0.0115 °/s 的固定偏置）。
- 该修正只作用于 z 轴；EKF 输出的漂移与 1 kHz 陀螺积分仅差 2 %，说明漂移源 100 % 在 z 轴零偏。

### 6.4 加速度计杆臂补偿（`imu_accel_lever_compensate`）

IMU 不在刚体旋转中心时：`a_meas = a_ref + α × r + ω × (ω × r)`。

- 角加速度用「陀螺先 4 ms 低通再差分」估计，再经 8 ms 低通与 ±200 rad/s² 限幅
  （1 kHz 直接差分会把 0.004 rad/s 噪声放大成 4 rad/s²，乘 0.1 m 偏心就是 0.4 m/s² 假加速度）；
- 向心项用瞬时角速度（ω 的二次量，信噪比更好）；补偿量限幅 ±30 m/s²，NaN/Inf 归 0；
- 差分基准与野值抑制同口径：`elapsed_dt > 4 × frame_dt` 时作废；
- **当前 `imu_lever_arm_m[]` 全 0，补偿退化为直通**；装车实测偏心后填入芯片自身坐标系下的位置矢量即可启用。

### 6.5 EKF 与欧拉角发布

- 六维状态：四元数 (4) + 陀螺零偏 (2，z 轴零偏不可观测，固定为 0)；
- 参数：`Q1 = 10`、`Q2 = 0.001`、`R = 1e7`、渐消因子 1.0、加速度低通 1 ms；
- 每拍写入 `QEKF_INS.dt = integrate_dt` 再调用 `IMU_QuaternionEKF_Update()`；
- `Get_Roll/Pitch/Yaw()` 返回**度**，发布前统一乘 `BASIC_MATH_DEG_TO_RAD` 转 rad；
  `QEKF_INS.YawTotalAngle`（度）同样转 rad 后发布到 `yaw_total`；
- EKF 入口拒绝 NaN/Inf 输入与非法 dt；输出非有限时复位 EKF 并作废姿态；
- `ekf_ready_flag == 0`（FreeRTOS 堆不足导致矩阵指针为 NULL）时**禁止调用 Update / Reset**，直接判定姿态不可用。

### 6.6 姿态作废（`imu_attitude_invalidate`）

任何输入/输出非法都走同一路径：复位 EKF + 清静止判定与低通历史 + `attitude_valid_flag = 0`。
低通历史必须一并清掉，否则复位后首帧会拿旧状态续算。

## 7. 温度控制

### 7.1 节拍与执行机构

- 温控节拍 **100 Hz**（`IMU_TEMP_CTRL_DIVIDER = 10`）：先读温度，成功才跑 PID；
- 执行机构：TIM3_CH4，27.5 kHz，输出量与 `ARR` 同量纲（0~1500 ⇒ 0~15 % 占空比）；
- 断加热路径会跳过重复写入：占空比已经是 0 时不再写 CCR（1 kHz 故障路径上避免每帧重复写寄存器）。

### 7.2 PID 参数与整定依据

```cpp
#define IMU_TEMP_PID_KP             75.0f
#define IMU_TEMP_PID_KI             0.008f
#define IMU_TEMP_PID_KD             0.0f
#define IMU_TEMP_PID_OUT_MAX        1500.0f      // = 15 % 占空比
#define IMU_TEMP_PID_INTEGRAL_LIMIT 400000.0f    // 积分权限 = Ki × 本值 = 3200 counts
```

⚠ `pid_calc()` 的 `PID_POSITION` 分支是 `error_sum += error; output = Kp·e + Ki·Σe + ...`，
即 **`error_sum` 累加的是误差本身**，积分项最大贡献 = `Ki × integral_limit`，
积分时间 `Ti = Kp / (Ki · fs)`（fs = 100 Hz）。

| 量                | 取值                              | 依据                                                                 |
| ----------------- | --------------------------------- | -------------------------------------------------------------------- |
| 静态环路增益      | `Kp × 0.005 = 0.375`              | 温度读数与占空比之间存在约 0.005 ℃/count 的瞬时（<50 ms）电气耦合，构成正反馈；Kp=180 时环路增益 0.86，实测产生 ±2.2 ℃ 极限环 |
| 积分时间 Ti       | `75 / (100 × 0.008) ≈ 94 s`       | 与被控对象极点（单容热系统，热时间常数 ≈257 s）同量级                  |
| 积分权限          | `0.008 × 400000 = 3200 counts`    | 覆盖环境温度大幅变化所需稳态功率；上限只作安全钳位                     |
| 微分 Kd           | 0                                 | 温度是 0.125 ℃ 量化值，一阶微分只会把量化台阶放大成占空比抖动          |

**事故记录（必须记住）**：曾按 `error_sum += Ki·error` 的常见写法整定成 `Ki = 0.0045 / limit = 500`，
积分权限只有 `0.0045 × 500 = 2.25 counts`（0.02 % 占空比），积分实际被禁用，
实测温度长期停在 49.63 ℃ 补不上 50 ℃ 的稳态误差。

该电气耦合无法用软件消除（系数跨板不一致，不做补偿），根治要在硬件上给加热回路独立回流地 + 就近储能电容。

### 7.3 抗饱和与安全阀

- **条件积分**（`imu_heater_keep_integral_step`）：输出顶到上限时，只有「误差仍为正（还要更热）」才撤销本拍积分；
  输出被压到 0 时，只有「误差为负（还要更冷）」才撤销。
  误差反向时保留积分，让积分能自然回退——若越限就一律冻结，积分一旦攒到较大权限，
  输出就永远回不到 0，降目标/环境升温时会造成长期过冲。
- 输出限幅在本函数内做：`pid_calc()` 不做输出限幅，`basic_math_constrain` 也拦不住 NaN，
  必须先判 `isfinite` 再夹到 `[0, IMU_TEMP_PID_OUT_MAX]`。
- 两种断加热策略：

| 函数                       | 触发场景                                     | 积分器   |
| -------------------------- | -------------------------------------------- | -------- |
| `imu_heater_output_off()` | 单帧通信毛刺 / DRDY 单次超时 / 单帧数据无效   | **保留** |
| `imu_heater_safe_off()`   | 传感器掉线、初始化失败、温度越界/过温、PID 输出 NaN | **清零** |

  区分二者是因为清积分器会让温控退回冷启动（稳态输出重建需 10 s 量级）。

## 8. 输出限幅汇总

| 位置              | 限幅                                                                        |
| ----------------- | --------------------------------------------------------------------------- |
| 温度 PID 输出     | `[0, IMU_TEMP_PID_OUT_MAX]`（模块内显式限幅 + NaN/Inf 保护）                |
| 温度积分项        | `±IMU_TEMP_PID_INTEGRAL_LIMIT`（= 400000）+ 条件积分抗饱和                  |
| 加热 PWM 占空比   | `pwm_class` 实例级 15 % 上限（`set_max_duty_ratio`，单调收紧，不可再放宽）   |
| 静止残差零漂      | `±IMU_YAW_TRIM_LIMIT = ±0.02 rad/s`                                         |
| 角加速度估计      | `±IMU_LEVER_ALPHA_MAX_RAD_S2 = ±200 rad/s²`                                 |
| 杆臂补偿量        | `±IMU_LEVER_CORR_MAX_M_S2 = ±30 m/s²`                                       |
| 单次积分步长      | 9 ms（< EKF 内部 0.01 s 复位阈值）                                          |
| 待积分时间账本    | 20 ms                                                                       |
| 单帧 DRDY 周期    | 0.5~9 ms（仅单帧语义）                                                      |

`pwm_class` 默认允许通用设备使用 `[0, 100%]`，每个实例/通道可配置自己的上限；
限制是单调收紧的，蜂鸣器、电机、舵机等其他 PWM 实例不会被 IMU 的 15 % 策略影响。
即使 PID 参数、调用方输入或浮点值异常，最终写入定时器的比较值也不会超过 15 %。

> 该软件限制不能替代硬件验证：上车前仍必须确认 PB1 的电气连接、驱动器有效电平、
> MOSFET 栅极保护、24 V 加热回路与 H7 地之间不存在反灌电压。

## 9. 离线检测与状态机

| 判定                | 依据                                                          | 对外表现                                     |
| ------------------- | ------------------------------------------------------------- | -------------------------------------------- |
| 陀螺/加速度读取失败 | `BMI088_read_imu() != BMI088_NO_ERROR`                        | `data_valid = 0`、断加热输出                 |
| 数据非有限值        | 任一通道 NaN/Inf                                              | 同上，且**同样计入离线判据**                 |
| 温度读取失败/越界   | 读取失败或超出 -20~100 ℃ / 超过 60 ℃ 过温                     | 断加热（清积分），姿态不受影响               |
| 传感器离线          | 连续 100 帧数据无效，或连续 3 次 DRDY 超时                     | `Init_Fail` + 周期重试 + 断加热清积分        |
| 姿态不可用          | `imu_is_ready() == 0`                                         | 上层降级（云台保持当前角/乏力）              |

`read_fail_count` 在任意一次有效帧后清零；读失败与 NaN/Inf 帧共用同一个计数器，
**传感器卡在非法值上时必须能走到重连**，否则会永久停在 `data_valid = 0`。

状态机：

```
Uninit ──init ok──► Running ──连续故障──► Init_Fail ──重试 ok──► Running
   │                   ▲                                          │
   └──init fail──► Init_Fail ◄────────────────────────────────────┘
                       │  (IMU_GYRO_AUTO_CALIB_ENABLE = 1 时另有 Wait_Temp / Calibrating)
```

健康值速查（固件只保留实现必需的字段，排查时用调试器看内部静态变量）：

| 字段                          | 健康值                     | 说明                           |
| ----------------------------- | -------------------------- | ------------------------------ |
| `status`                      | 4（`Imu_Status_Running`） | 工作状态                       |
| `data_valid`                  | 1                          | 本帧传感器数据有效             |
| `sample_count`                | 持续增长                   | 采样帧计数（温控分频基准）     |
| `temperature`                 | 目标值附近 ±0.5 ℃          | 芯片温度                       |
| `heater_out`                  | 0 ~ 1500                   | 加热输出（与 TIM3 ARR 同量纲） |
| `dt`                          | ≈ 0.001 s                  | 本拍积分步长                   |
| `gyro[3]` / `accel[3]`        | —                          | 已扣零漂角速度 / 加速度        |
| `euler_angle` / `yaw_total`   | —                          | 欧拉角 / 连续多圈 yaw，rad     |

## 10. 使用示例

```cpp
// 初始化由 imu_task 内部完成, 上层只需要读取
if (imu_is_ready())
{
    const float yaw = imu_get_yaw_total_rad();   // rad, 连续多圈
    const float wz  = imu_frame.gyro[2];         // rad/s, 已扣零漂
}
```

## 11. 常见错误与排查

| 现象                                            | 可能原因                                             | 处理                                                                                     |
| ----------------------------------------------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `status` 长期停在 `Imu_Status_Init_Fail`      | SPI 无响应、片选错误、供电缺失                       | 调试器看 `BMI088_init()` 返回值枚举，或断点看 `read_fail_count`                          |
| `data_valid` 频繁为 0                         | 陀螺 chip id 校验失败（SPI 时序/时钟过高）           | 把 `BMI088_SPI_PRESCALER` 改回 `SPI_BAUDRATEPRESCALER_32`，检查 CPOL/CPHA 与走线          |
| yaw 静止缓慢漂移                                | `gyro_bias[2]` 与实际零偏不符，或静止修正未生效    | 看 `yaw_bias_trim`（应已非 0 且稳定）与 `trim_seeded_flag`；仍漂则开 `IMU_GYRO_AUTO_CALIB_ENABLE` |
| 机动后 yaw 出现固定偏置                         | 机动过渡帧被误判为静止，污染了 `yaw_bias_trim`     | 确认瞬时角速度门控 `IMU_TRIM_SEED_RATE_MAX` 生效；`yaw_bias_trim` 在机动时刻不应跳变       |
| yaw 突然阶跃后永不回正                          | 陀螺单帧野值被直接积分                               | 已由 `imu_gyro_glitch_filter()` 拦截；如再现需在 `gyro_glitch.pre` 处断点确认野值幅值       |
| 姿态恒为 0 / 不更新                             | FreeRTOS 堆不足，Kalman 矩阵指针为 NULL              | 看 `ekf_ready_flag`；把任务改静态分配或增大 `configTOTAL_HEAP_SIZE`                      |
| 采样率明显低于 1000 Hz                          | 任务被更高优先级任务长期占用、DRDY 丢中断            | 确认 IMU 任务优先级为 `High`；用逻辑分析仪测 `GYRO_INT` 频率                              |
| 温度长时间上不去                                | 加热占空比太小 / 目标温度过高 / 积分被写成注释里的错误形式 | 先查硬件驱动与温度反馈；确认 `IMU_TEMP_PID_KI` 的物理含义（§7.2），不得绕过 15 % 上限      |
| 温度超调大                                      | 积分权限过大或条件积分被破坏                         | 降 `IMU_TEMP_PID_KI`；`IMU_TEMP_PID_INTEGRAL_LIMIT` 按 §7.2 推导，不要单独调小              |
| 加热时陀螺出现 20~60 Hz 噪声                     | 加热载波落入结构共振带                               | 确认 TIM3 的 `PSC + 1 = 1`（27.5 kHz），不要回退到 1.1 kHz                                |

## 12. CubeMX 配置与上车前检查

本模块已登记到 `CMakeLists.txt`，但不直接修改 CubeMX 生成的
`Core/Src/freertos.c`、`Core/Src/gpio.c` 或 `Core/Src/stm32h7xx_it.c`。

### 12.1 GPIO/EXTI

1. 打开 `infantry.ioc`，确认 `PE12` 标签为 `GYRO_INT`，信号 `GPIO_EXTI12`，触发边沿 **Falling Edge**；
   PE10 若只作 DRDY 观测，建议改为普通 `GPIO_Input`（加速度计 INT1 为 800 Hz DRDY 推挽输出，
   配成 EXTI 会产生每秒 800 次无效中断）。
2. 上下拉按硬件原理图设置；当前驱动配置为推挽低有效，默认 `No Pull`。
3. NVIC 中勾选 **EXTI line[15:10] interrupts**，优先级 `5`、子优先级 `0`
   （必须数值 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`，
   否则 ISR 中的 `osThreadFlagsSet` 会触发 FreeRTOS 断言）。
4. 点击 Generate Code。不要手动编辑生成的 `gpio.c` 或 `stm32h7xx_it.c`。

### 12.2 FreeRTOS 线程

1. Middleware > FREERTOS > Tasks and Queues 新增任务：

   - Task Name：`ins_task_rtos`
   - Entry Function：`imu_task`（**必须与头文件声明逐字符一致，含大小写**；
     写错会退化成 CubeMX 生成的空壳弱函数，编译不报错但 IMU 完全不工作）
   - Priority：`High`(40)；`Realtime`(48) 会每秒 1000 次抢占 `gimbal`/`shoot`/`safe_daemon`(24)
   - Stack Size：至少 `1024` words（若界面以字节计则填 ≥ `4096` bytes）
   - Allocation：`Static`（**必须**。Kalman 需要约 1.9 KB 堆，动态任务会把堆吃光）
   - Code Generation Option：`As weak`（由本模块的强定义覆盖）

2. 生成后确认链接结果是强符号：

   ```powershell
   arm-none-eabi-nm build\Debug\infantry.elf | Select-String imu_task
   ```

   应看到 `T imu_task`；不要在用户代码里再次调用 `osThreadNew(imu_task, ...)`。

### 12.3 对其它实时外设的影响（WS2812 等）

IMU 任务优先级 `High`、周期 1 kHz，单次连续占用 CPU 实测约 **130 µs（峰值约 190 µs）**。
这个时间就是**所有低优先级任务能被一次性阻塞的上限**。

**已踩过的坑：WS2812 偶发闪停。** WS2812 用 SPI6 位流模拟单总线时序，一帧 25 字节 ≈ 46.5 µs；
灯珠在数据线持续低电平约 50 µs 后就会锁存当前已收到的位。SPI6 只能走 HAL 阻塞发送，
CPU 一旦被抢占超过 1.9 µs 移位时钟就停顿——停顿超过 50 µs 就会把半帧当一帧锁存，
表现为灯短暂变暗/熄灭（实测 **5.6 % 的帧被截断**，约每 1.8 s 闪一次）。
修法：`ws2812_write_color()` 在发送期间 `osKernelLock()` 挂起调度器。

后续新增「用阻塞方式按时序翻转引脚」的外设（软件 SPI/I2C 从机、单总线 DHT11、红外编码等）
必须同样处理，不能靠「任务优先级低所以慢」来保证时序。

### 12.4 上车前检查

- TIM3_CH4 确实接加热电阻（初始化会配置 15 % 上限并以 0 % 启动）；
- 不得通过其他代码直接写 `TIM3->CCR4` 或绕过 `pwm_class` 启动加热；
- SPI2 的 `ACC_CS / GYRO_CS` 与 BMI088 安装方向一致；
- 示波器确认 `GYRO_INT` 为约 1 kHz 的下降沿，且没有长时间低电平/毛刺；
- 若硬件实际输出为高有效，必须同时修改 BMI088 INT3 电平配置和 EXTI 边沿，不得只改一侧；
- 先关加热（把 `IMU_TEMP_PID_OUT_MAX` 置 0）验证姿态解算，再逐步放开温控。

## 13. 实测记录与已知限制

### 13.1 实测记录

> 下列数据均为历史归档，**条件不同不可横向比较**；50 ℃ 目标 + 现 PID 参数的组合标注为「待复测」。

| 项目                            | 条件 / 时间          | 实测值                                                        |
| ------------------------------- | -------------------- | ------------------------------------------------------------- |
| 采样率（外部观测）              | 2026-10-26, 40 ℃     | 998.7 Hz，长期抖动 < 0.1 Hz                                   |
| 一帧 SPI 读取耗时               | 2026-10-26           | ≈30 µs（优化前 ≈43 µs）                                       |
| 上电 4.8 s 时 yaw 累积          | 2026-10-26           | −0.034°（野值抑制前同一时刻 −8.06°）                          |
| 静止 yaw 漂移                   | 2026-10-26, 8 min    | −0.020 °/min，累计 −0.085°，无跳变                            |
| 静止 yaw 漂移（另一轮）         | 2026-10-26, 69 s     | −0.08 °/min（−2.3e-5 rad/s），与 1 kHz 陀螺积分 −0.081 °/min 一致 |
| yaw 估计短期抖动                | 2026-10-26           | ±0.1°（69 s 内 0.21° 峰峰值），已是估值噪声底                  |
| 温控稳态                        | 2026-10-26, 目标 40 ℃ | 39.75~40.5 ℃，超调 0.125 ℃（1 LSB）                          |
| 热时间常数                      | 2026-11-12           | ≈257 s（断电 54 s 内 50.6 → 45.75 ℃）                         |
| 温度-占空比电气耦合             | 2026-11-12           | ≈0.005 ℃/count，瞬时 <50 ms                                   |
| Kp=180 / Ki=1.2 表现            | 2026-11-12, 目标 50 ℃ | ±2.2 ℃ 极限环                                                |
| 现参数（Kp=75 / Ki=0.008）      | 目标 50 ℃            | **待复测**                                                    |

> **读数注意**：`QEKF_INS.YawTotalAngle` 的单位是**度**（`imu_frame.yaw_total` 才是 rad）。
> 换算：1e-5 rad/s = 0.00057 °/s = 0.034 °/min ≈ 2 °/h。

上电首帧的 `dt` 约 0.082 s 已被定位：**不是丢中断**，而是 `BMI088_LONG_DELAY_TIME`
的陀螺软复位死区（这段时间不产生 DRDY），只在开机出现一次，且被 9 ms 上限截断，属良性。

### 13.2 已知限制

1. **yaw 无绝对观测**：一次未被拦住的野值或一段未被记账的时间都会永久留在 yaw 上，只能靠复位恢复。
2. **`yaw_bias_trim` 的噪声底 ≈2e-5 rad/s（0.0011 °/s）**：结构上是时间常数 20 s 的一阶低通，
   继续加大增益只会同步放大噪声；要再降一个数量级需要偏置-温度模型。
3. **温度-占空比电气耦合**：无法用软件消除，只能靠限制环路增益；根治需硬件改地线与储能电容。
4. **杆臂补偿当前未启用**（`imu_lever_arm_m[]` 全 0），装车偏心未标定。
5. **温度目标 50 ℃ 的稳态/超调未复测**，现参数只在 2026-11-12 的极限环排查中验证过稳定性方向。

## 14. 变更记录

| 版本 | 日期       | 变更                                                                                                                                                                                                                                                                                                                                 | 原因                                                                                                                                                                                 |
| ---- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 0.1  | —          | 初版：SPI 逐字节读、Mahony + EKF 双解算、`CHEAT` 小值截断、`while` 死等初始化                                                                                                                                                                                                                                                        | —                                                                                                                                                                                    |
| 0.2  | 2026-10-26 | ① 驱动改突发传输并返回错误码；② 零漂标定不再污染补偿常量；③ 移除 `CHEAT`；④ 移除周期内 Mahony；⑤ 温控改 `pwm_class` + 单向抗饱和；⑥ 发布单位统一 rad；⑦ 修 EKF `accLPFcoef` 未赋值与除零 NaN；⑧ PWM 实例级占空比上限                                                                                                                | 旧实现存在零漂错误累积、低速信息被截断、约一倍冗余解算与潜在 NaN 污染                                                                                                                 |
| 0.3  | 2026-10-26 | ① 修 FreeRTOS 堆耗尽导致 Kalman 矩阵指针为 NULL；② 姿态可用性与传感器有效性解耦；③ 静止判定改用低通角速度；④ 四元数模长异常改为归一化救回；⑤ 掉线恢复时重新对齐 DRDY 序号与 DWT 基准                                                                                                                                                | 实机联调发现堆不足使 EKF 全部矩阵运算落在地址 0；静止阈值低于噪声底使零漂修正永不触发                                                                                                 |
| 0.4  | 2026-10-26 | ① 拆出 `BMI088_read_imu`（2 次事务）与 `BMI088_read_temperature`（1 次），温度降到 100 Hz；② SPI2 运行时提到 6.25 MHz；③ 驱动改用局部缓冲区消除半帧；④ 温度与姿态故障各自降级；⑤ 补温度 PID 输出限幅与 NaN 保护；⑥ 加热安全关断跳过重复 CCR 写入                                                                                       | 1 kHz 采样下每帧多读一次温度纯属浪费带宽；`pid_calc` 不做输出限幅使观测值失真                                                                                                          |
| 0.5  | 2026-10-26 | ① 温控积分限幅改为按 `Ki × limit` 推导 + 条件积分；② 静止判定的加速度模长也做 0.2 s 低通                                                                                                                                                                                                                                             | 温控积分贴着限幅导致恒温点长期偏低；加速度瞬时跳变反复打断静止确认，使 yaw 漂移恶化近 10 倍                                                                                           |
| 0.6  | 2026-10-26 | ① 新增任务耗时观测；② 据此定位并修复 WS2812 偶发闪停（修复落在 `ws2812.cpp`）                                                                                                                                                                                                                                                      | 1 kHz 高优先级任务把低优先级任务的阻塞时长拉到 127~163 µs，超过 WS2812 的 50 µs 锁存门限                                                                                              |
| 0.7  | 2026-10-26 | ① 新增陀螺野值抑制；② 修正 z 轴 `gyro_bias[]` 初值；③ `yaw_bias_trim` 首次静止后一次定标；④ `IMU_YAW_TRIM_LIMIT` 0.01 → 0.02 rad/s；⑤ 软复位后重置 trim 定标与野值基准                                                                                                                                                              | 归档数据抓到 24.8 rad/s 单帧野值使 yaw 跳 −2.15° 并永久保留；硬编码 z 轴零偏比实测小 38 倍导致上电后 yaw 以 0.19 °/s 漂移                                                               |
| 0.8  | 2026-10-26 | ① 静止残差积分加逐帧瞬时角速度门控；② 文档补 yaw 零漂传递路径与实测对照                                                                                                                                                                                                                                                            | trim 的阶跃与机动帧一一对应，单次机动留下 0.0115 °/s 固定偏置                                                                                                                         |
| 0.9  | 2026-10-26 | 移除全部诊断观测变量：`imu_frame` 精简为姿态/角速度/温度/温控/状态；删除 `gyro_raw`、`still_*`、`spi_time_us`、`task_time_us` 等；`QEKF_INS` 删除诊断计数                                                                                                                                                                          | 观测字段上车后持续占 RAM 且增加每帧计算，对控制与解算无贡献；移除后 FLASH −1.7 KB                                                                                                    |
| 1.0  | 2026-11-12 | ① 温控重新整定 `Kp=75 / Ki=0.008 / Kd=0`、积分限幅 400000；② 条件积分按误差方向判决；③ 加热载波 1.1 kHz → 27.5 kHz（TIM3 `PSC+1 = 1`）；④ 新增 `Wait_Temp` 超时（15 s）；⑤ 更新 `gyro_bias[]`、杆臂归零                                                                                                                            | 加热电流经公共地干扰温度传感构成正反馈，旧 Kp=180 在 50 ℃ 附近产生 ±2.2 ℃ 极限环；1.1 kHz 载波激发 41 Hz 结构共振使陀螺噪声达 1.2 °/s rms                                             |
| 2.1  | 2026-11-12 | ① 源文件按「参数 → 状态 → 工具 → 步骤 → 单步 → 任务」重新分区，删除全部前置声明；② `INS_Init`/`INS_Task` 更名为 `imu_init`/`imu_update_step`；③ 删除重复的 `imu_status` 副本，`imu_frame.status` 成为唯一真源；④ 合并「读失败 / 非有限值」两段无效帧处理并抽出 `imu_frame_invalid()`；⑤ **修复**：NaN/Inf 帧计入 `read_fail_count`（此前传感器卡在非法值上永不重连）；⑥ **修复**：假唤醒只在解算态记账（避免等待/标定阶段的时间在进入 Running 后被一次性积分）；⑦ DRDY 单次超时改为只断输出保留积分器；⑧ 抽出 `imu_update_temperature()`、`imu_calculate_yaw_trim()`、`imu_heater_keep_integral_step()`、`imu_runtime_reset()`、`imu_attitude_invalidate()`；⑨ 重连复位补 `attitude_valid`，`ekf_ready == 0` 时不再调用 EKF Reset；⑩ 文档去重（原文件内容整份重复）并按规范 §10.2 重排校准 | 原文件 1308 行中参数区与函数区交错、状态双真源、两处安全逻辑不一致；文档与代码参数（Kp/Ki/Kd、目标温度、过温阈值、dt 上限、PWM 载波、函数名）全面脱节 |
| 2.2  | 2026-11-12 | **按开发规范对齐命名与设计**（对照表见 §1.4）：① 宏单位后缀统一（`IMU_DRDY_TIMEOUT_MS`、`IMU_INIT_RETRY_PERIOD_MS`、`IMU_GRAVITY_ACCEL_M_S2`、`IMU_GYRO_CALIB_SPAN_S`、`IMU_TRIM_SEED_SPAN_S`、`IMU_TEMP_STABLE_SPAN_S`、`IMU_STILL_CONFIRM_SPAN_S`）并重排宏区对齐；② 成员语义前缀（`prev`→`pre`、`pre_valid_flag`、`pre_drdy_dwt`、`pre_elapsed_dwt`、`heater_duty_cmd`→`last_heater_duty`）；③ 布尔量统一 `_flag` 后缀（`ekf_ready_flag`、`attitude_valid_flag`、`trim_seeded_flag`、`still_*_started_flag`、`*_dwt_started_flag`）；④ 函数按 §2.5 分类改名：测量更新 `imu_update_measure / _temperature / _gyro_calib / _wait_temp`，周期计算 `imu_calculate_running / _attitude / _yaw_trim`；⑤ 限幅复用 `basic_math_constrain()`，NaN/Inf 判定复用 `basic_math_is_invalid_float()`（§8.1）；⑥ 标定量补充来源与日期留痕（§3.3/§3.4）；⑦ 状态区补单位注释并说明内部状态由文件作用域 `static` 暴露给 Ozone（§9.4） | 上一版虽已完成结构整理，但仍存在 `_TICKS` 等非单位后缀、`prev`/无后缀布尔量与规范 §2.4/§5.2 不符、限幅与 NaN 判定重复造轮子、标定参数无来源记录（违反 §3.4） |
