# imu_temp_ctrl 模块说明（板载 BMI088 IMU 采样 / 恒温 / 姿态解算）

> 代码：[imu_temp_ctrl.cpp](./imu_temp_ctrl.cpp)、[imu_temp_ctrl.h](./imu_temp_ctrl.h)
> 驱动：[BMI088driver.cpp](../../Drive/bmi088/BMI088driver.cpp)、[BMI088Middleware.cpp](../../Drive/bmi088/BMI088Middleware.cpp)
> 算法：[QuaternionEKF.cpp](../../Drive/algorithm/QuaternionEKF/QuaternionEKF.cpp)
> 规范依据：[RoboMaster电控C++开发规范](../../Docs/RoboMaster电控C++开发规范.md) §5.1 单位、§8 数学与算法、§10.2 本文档结构

## 1. 系统架构

| 文件 | 职责 |
|---|---|
| `BMI088Middleware.cpp` | 片选、SPI 全双工突发收发（`BMI088_transmit_receive`）、延时 |
| `BMI088driver.cpp` | 寄存器读写、初始化自检、加速度/陀螺突发读取（`BMI088_read_imu`）、温度读取（`BMI088_read_temperature`） |
| `QuaternionEKF.cpp` | 六维状态四元数 EKF（姿态角 + 陀螺零偏估计 + 卡方检验） |
| `imu_temp_ctrl.cpp` | DRDY 同步采样、恒温 PID、零漂补偿、姿态解算与发布 |

数据流：

```
BMI088 --SPI2(hspi2, 6.25MHz)--> BMI088driver --2 次突发/帧--> imu_temp_ctrl
                                                     |-- 100Hz --> 温度 PID --> TIM3_CH4 加热 PWM
                                                     |-- 温度 1 次突发 --> 温度 PID
                                                     |-- DRDY --> EXTI --> imu_task
                                                     |             |-- 实测 dt 限幅
                                                     |             |-- 零漂补偿 --> QuaternionEKF --> imu_frame
```

- 通信：SPI2，`CLKPolarity = HIGH` / `CLKPhase = 2EDGE`，预分频 32，软件 NSS。
- 采样：BMI088 陀螺仪 DRDY（INT3，低有效）触发 PE12 的 EXTI15_10；
  ISR 只记录 DWT 时间间隔并通知 `imu_task`，SPI 读取和 EKF 均在任务上下文执行。
- EKF：使用相邻 DRDY 的实测 `dt`，限制在 `0.5ms ~ 3ms`；通知合并时按合并帧数推进
  一次，并累计丢帧诊断。
- 超时：连续 3 次、每次 10ms 未收到 GYRO_INT 时进入 `Imu_Status_Init_Fail`，
  关闭加热并由任务周期性重试 BMI088。
- 加热：TIM3_CH4，ARR = 10000，20kHz；IMU 初始化时为该 `pwm_class` 实例显式配置
  `15%` 最大占空比，最终 CCR 写入仍在 PWM 底层执行限制。

## 2. 数据类型定义

```cpp
typedef enum
{
    Imu_Status_Uninit = 0,  // 未初始化
    Imu_Status_Init_Fail,   // BMI088 初始化失败, 任务内周期重试
    Imu_Status_Wait_Temp,   // 等待加热到目标温度（仅自标定模式）
    Imu_Status_Calibrating, // 陀螺零漂标定中
    Imu_Status_Running,     // 正常解算
} imu_status_e;

typedef struct
{
    float gyro[3];              // 已扣零漂角速度, rad/s
    float accel[3];             // 加速度, m/s^2
    float temperature;          // 芯片温度, ℃
    euler_angle_t euler_angle;  // 欧拉角, rad
    float yaw_total;            // yaw 连续多圈角, rad
    float dt;                   // 实测采样周期, s
    float ref_temperature;      // 目标温度, ℃
    float heater_out;           // 加热输出, 与 TIM3 ARR 同量纲
    uint8_t status;             // imu_status_e
    uint8_t data_valid;         // 1 = 本周期数据有效
    uint32_t sample_count;      // 采样帧计数（温控分频基准）
} imu_data_resolve_t;
```

单位约定：角度 `rad`、角速度 `rad/s`、加速度 `m/s^2`、温度 `℃`。
坐标系：机体 X=前、Y=左、Z=上（右手系），yaw 绕 Z 逆时针为正，角度自 +X 起算；yaw ∈ (-π, π]。

## 3. 接口说明

| 接口 | 说明 |
|---|---|
| `void imu_task(void *argument)` | 任务入口；名称必须与 CubeMX Entry Function 完全一致（区分大小写） |
| `float imu_get_roll_rad(void)` / `imu_get_pitch_rad()` / `imu_get_yaw_rad()` | 欧拉角，rad |
| `float imu_get_yaw_total_rad(void)` | 连续多圈 yaw，rad（小陀螺/大 yaw 用） |
| `uint8_t imu_is_ready(void)` | 1 = 传感器在线且已进入解算状态 |
| `extern volatile imu_data_resolve_t imu_frame` | 解算结果结构体（含姿态、角速度、温度、加热输出） |

`imu_frame` 由 IMU 任务写、其余任务读：单个 32 位 float 字段读写原子；多字段严格一致时请自行关中断拷贝。

## 4. 初始化流程

1. `heater_pwm` 绑定 `htim3 / TIM_CHANNEL_4` 并启动，占空比立即置 0（上电安全态）。
2. `IMU_QuaternionEKF_Init(10, 0.001, 1e7, 1.0, 0.001, 0.001)`；
   后续每帧将 `QEKF_INS.dt` 更新为受限的 DRDY 实测周期。
3. 温度 PID 参数装载（`pid_set_config_pos`），`ref_temperature = 40 ℃`。
4. `BMI088_init()`：加速度计/陀螺仪各做「读 chip id → 软复位 → 读 chip id → 写配置并回读校验」，单次尝试 3 遍、间隔 20ms；失败则 `Imu_Status_Init_Fail`，任务内每 500ms 重试（旧版是 `while (BMI088_init());` 死等，传感器缺失会永久阻塞）。
5. 成功后按配置进入 `Imu_Status_Wait_Temp`（自标定）或 `Imu_Status_Running`；
   `imu_task` 由 CubeMX 配置的 FreeRTOS 线程入口调用。

## 5. 采样与解算逻辑

### 5.1 DRDY 同步与一帧数据读取

BMI088 陀螺仪配置为 1kHz DRDY、INT3 低电平有效。STM32 端必须在 CubeMX
中将 `GYRO_INT` 配置为下降沿 EXTI，并启用对应的 `EXTI15_10_IRQn`。
CubeMX 重新生成 `gpio.c` 和 `stm32h7xx_it.c` 后，HAL 会回调
`HAL_GPIO_EXTI_Callback`；本模块只处理 `GYRO_INT_Pin`，不在 ISR 中执行 SPI、
阻塞延时或浮点姿态计算。

任务等待 DRDY 线程标志；收到事件后读取最新一帧。若多个 DRDY 在任务唤醒前合并，
任务将合并的时间间隔传给 EKF，避免无提示地丢失采样节拍。
`dt` 超出 0.5~3 ms 会被限幅，防止一次异常节拍污染 EKF。
如果没有 DRDY，任务不会继续轮询传感器或维持加热。

| 段 | 寄存器 | 长度 | 说明 | 采样率 |
|---|---|---|---|---|
| 加速度 | 0x12 ~ 0x17 | 6B | ±3g，0.0008974358974 (m/s²)/LSB | 每帧 |
| 陀螺仪 | 0x00 ~ 0x07 | 8B | 首字节为 chip id（0x0F）自检，±2000dps | 每帧 |
| 温度 | 0x22 ~ 0x23 | 2B | 11bit 有符号，0.125℃/LSB，+23℃ 偏置 | 每 10 帧 |

三处均用**单次片选内突发传输**（加速度计读时序为「地址 + 1 个哑字节 + 数据」，陀螺仪为「地址 + 数据」）。

驱动把这段拆成三个接口：

| 接口 | SPI 事务数 | 用途 |
|---|---|---|
| `BMI088_read_imu(gyro, accel)` | 2 | 1kHz 姿态主路径 |
| `BMI088_read_temperature(&t)` | 1 | 温控节拍（100Hz） |
| `BMI088_read_status(gyro, accel, &t)` | 3 | 兼容旧用法 / 调试 |

**旧实现每帧 3 次事务（温度也按 1kHz 读）**。温度的热时间常数是秒级，1kHz 采样纯属浪费带宽：
降到 100Hz 后，每帧 SPI 事务从 3 次降到 2 次，一帧读取耗时从约 43 µs 降到约 30 µs。

`BMI088_read_imu` 先在局部缓冲区解析完两段，全部成功后才一次性写回调用者缓冲区，
避免「加速度已更新、陀螺仪没更新」的半帧数据进入 EKF。

读失败（SPI 超时 / 陀螺 chip id 不符）时返回非 0，本拍不参与解算，`data_valid = 0`。
错误码区分 `BMI088_COM_ERROR`（总线）与 `BMI088_NO_SENSOR`（chip id 不符），便于定位是时序还是走线问题。

SPI 时钟：CubeMX 里 SPI2 为 PLL2P(100MHz)/32 = 3.125 MHz；`BMI088_com_init()` 在运行时改成
`/16 = 6.25 MHz`（BMI088 上限 10 MHz）。若个别批次在 6.25 MHz 下出现 chip id 校验失败，
把 `BMI088_SPI_PRESCALER` 改回 `SPI_BAUDRATEPRESCALER_32` 即可，不需要动 `.ioc`。

### 5.2 零漂

```
最终角速度 = 原始角速度 - gyro_bias[] - yaw_bias_trim
```

- `gyro_bias[]`：默认值来自 40℃ 恒温静止实测；`IMU_GYRO_AUTO_CALIB_ENABLE = 1` 时改为「温度稳定 3s 后采集 1000 帧（丢弃 |ω| > 0.15 rad/s 的运动帧）求均值」。
  - **x/y 由 EKF 在线估计并叠加修正**（总修正量 ≈ `gyro_bias[i] + QEKF_INS.GyroBias[i]`），
    z 轴 EKF 不强估计（`GyroBias[2]` 恒为 0），因此 z 轴精度完全取决于这里的初值 + 运行期静止残差定标。
  - **旧版 z 轴初值 0.0000852 rad/s 比实测 0.0033 rad/s 小 38 倍**，是"上电后 yaw 以 0.19°/s 漂移数分钟"的根因；已按实测更正。
- `yaw_bias_trim`：运行中判定静止后修正 z 轴残差，限幅 `±IMU_YAW_TRIM_LIMIT = ±0.02 rad/s`
  （BMI088 零偏 spec 可达 ±1°/s，旧值 ±0.01 只剩 3 倍余量）。
  分两阶段：
  1. **首次静止确认后直接定标**：再平均 500 帧（0.5 s）求残差均值一次写入，而不是从 0 慢积分。
     旧版仅靠 20 s 时间常数积分，实测需 2~5 分钟才收敛，期间 yaw 以最高 0.19°/s 累积。
  2. 定标完成后改用 `IMU_STILL_TRIM_GAIN` 慢积分跟踪温漂。
  **两个阶段都叠加同一层逐帧门控**：单帧角速度模长 `|ω| > IMU_TRIM_SEED_RATE_MAX (0.15 rad/s)` 时
  本帧不计入零偏。原因见 §5.2.1——静止判定用的是低通值，
  它比真实运动滞后 3~4 拍，机动起始的那几帧会被误判为静止。
  **静止判定的两路输入都必须低通**，否则噪声/瞬态会持续打断确认过程：
  | 判据 | 处理 | 阈值 |
  |---|---|---|
  | 角速度模长 | 0.2 s 一阶低通 | < 0.020 rad/s（≈1.15°/s） |
  | 加速度模长 | 0.2 s 一阶低通 | \|‖a‖ − 9.80665\| < 0.5 m/s² |

  - 角速度若用瞬时值：本机实测瞬时 `|ω|` 噪声峰值可达 0.020 rad/s，噪声会持续把 `still_count` 清零，
    修正永不触发（实测 `still_count` 恒为 0~2、`yaw_bias_trim` 恒为 0，yaw 漂移 0.19°/s）。
  - 加速度若用瞬时值：瞬时模长会偶发跳到 9.25 m/s²（约 0.55 m/s² 偏差，实测每几秒一次），
    同样打断确认（这是实机验证时定位到的第二个根因）。
    低通后单帧跳变被压缩到 1/200，而「被抬起/搬运」这类持续加速度仍会在 0.2 s 内被识别。
  - 两路都低通后，静止确认被打断的次数由 5 次/25 s 降到 0~1 次/30 s，
    静止 yaw 漂移从 0.0095 °/s 回到 **0.0001~0.0018 °/s**。
- **旧版 `CHEAT`（|ωz| < 0.003 rad/s 直接置 0）已移除**：该非线性会吃掉正常的低速转动（0.003 rad/s ≈ 0.17°/s），造成云台低速爬行/丢帧式跟踪；零漂问题改由上面的标定 + 静止残差修正解决。

### 5.2.1 陀螺野值抑制（yaw 跳变的根因防线）

**问题**：单帧 SPI 位错 / DRDY 竞争会给出几百~上千 °/s 的假角速度。EKF 对 z 轴没有任何绝对观测
（`GyroBias[2]` 强制 0、`xhatUpdate` 里 `temp_vector.pData[3] = 0`），所以**一次野值就把 yaw 永久推偏且永不恢复**。
实测捕捉到一次：板子完全静止、`accel` 模长 9.71 m/s²、chip id 校验全程通过，
静止判定用的低通角速度却在单帧从 0.0047 跳到 0.1269，随后严格按 0.2 s 一阶指数衰减
（比值 0.577 / 0.639 / 0.588 ≈ e^(−0.5)）——反推注入量 ≈ 24.8 rad/s（1420 °/s，满量程的 71%），
yaw 当场跳变 **−2.15°** 并永久保留。

**实现**（`imu_gyro_reject_glitch()`，在 `INS_Task` 里于任何积分之前调用，就地修正 `gyro[]`）：

| 判据 | 阈值 | 行为 |
|---|---|---|
| 逐轴超量程 | \|ω\| > 36 rad/s（±2000 dps = 34.9） | **永不放行**，一直保持上一拍 |
| 逐轴变化率 | \|ω(k) − ω(k−1)\| > 10000 rad/s² × dt（1 ms 时 = 10 rad/s） | 先保持上一拍；下一帧仍偏离则认定为真实机动并放行 |

- 孤立野值：下一帧即回到正常值，被**完全剔除**（验证：污染由 1.4° 降到 0.0009°）。
- 真实阶跃：进、出各滞后 1 帧，两者误差**互相抵消**（验证：净积分误差 0）。
- 噪声底：高斯噪声 sd = 0.004 rad/s 连续 20000 帧，**零误判**。
- 该函数只做数值修正，不保留任何统计量，因此不占用额外 RAM、不参与解算。

### 5.2.2 yaw 零漂的传递路径（为什么要单独给 trim 加门控）

EKF 对 z 轴没有任何绝对观测（`GyroBias[2]` 强制 0、融合时 `temp_vector[3] = 0`），
所以 **yaw 的唯一信息源就是 z 轴角速度的积分**。整条链路只有三个会引入固定偏置的地方：

```
原始陀螺 z 轴 --(− gyro_bias[2])--(− yaw_bias_trim)--> gz --> EKF 四元数 --> Yaw
                    ↑ 静态标定      ↑ 静止残差积分
```

实测验证（静止台，40 ℃）：

| 量 | 60 s 内变化 | 折算速率 |
|---|---|---|
| `QEKF_INS.YawTotalAngle`（EKF 输出，度） | +0.0333 ° | **+9.7e-6 rad/s** |
| 1 kHz 陀螺积分 ∫gz·dt | +5.93e-4 rad | **+9.9e-6 rad/s** |

两者相差 2 %，说明 **EKF 没有额外引入漂移，yaw 漂移 100 % 来自 z 轴残余零偏**。
这条结论是后续所有调参的前提：不要在 EKF 侧找零漂，要盯 `gyro_bias[2]` 与 `yaw_bias_trim`。

`yaw_bias_trim` 的积分器是一个时间常数 20 s 的一阶低通，它的噪声底可算出来：
逐帧噪声 σ ≈ 4e-3 rad/s、增益 5e-5、1 kHz，稳态抖动 ≈ σ·√(1/(2·T·fs)) ≈ **2e-5 rad/s**。
也就是说**任何小于 2e-5 rad/s（0.0011 °/s）的零偏，在这个结构下都调不掉**，加大增益只会同步放大噪声。
实测稳态就在这个量级（见 §7 实测表），继续调增益收益很小；
若要再降一个数量级，需要偏置-温度模型（降温/升温各测一条曲线），不是改增益能解决的。

**机动会把 trim 污染掉（v0.8 修复）**：静止判定读的是 0.2 s 低通角速度，比真实运动滞后 3~4 拍。
归档数据里能直接看到这类帧被计入 trim：

| 时刻 | 该拍 z 轴角速度 | 静止判定用低通值 | Δ`yaw_bias_trim` |
|---|---|---|---|
| 120.22 s | −0.163 rad/s | 0.0165（仍判静止） | **−1.9e-4 rad/s** |
| 140.34 s | −0.315 rad/s | 0.1437（仍判静止） | **−2.1e-4 rad/s** |

单次机动就能留下 2e-4 rad/s ≈ **0.0115 °/s** 的固定偏置，要 20 s 才衰减掉。
v0.8 起 trim 的积分再叠加一层瞬时角速度门控（§5.2），这几帧直接被丢弃。

### 5.3 EKF

- 六维状态：四元数 (4) + 陀螺零偏 (2，z 轴零偏不可观测，固定为 0)。
- 加速度模长异常（< 1 或 > 100 m/s²）时，量测方向取上一拍姿态推算的重力方向，使新息为 0、本拍不修正，避免 1/0 产生 NaN 污染状态链。
- EKF 入口拒绝 NaN/Inf 输入和非法 `dt`；滤波后检查四元数模长及欧拉角有限性，
  异常时恢复为单位四元数并让本帧姿态不可用。
- 加速度低通由 `accLPFcoef` 控制（旧版漏赋值导致 `lpf` 形参失效，滤波退化为直通，已修复）。
- **Mahony 已不在周期内调用**：其输出在本模块无消费者，原来每拍白跑一遍姿态解算（CPU 占用约翻倍），现仅保留 EKF 一路。

### 5.4 欧拉角发布

`Get_Roll/Pitch/Yaw()` 返回**度**，发布前统一乘 `BASIC_MATH_DEG_TO_RAD` 转 rad；`QEKF_INS.YawTotalAngle`（连续多圈）同样转 rad 后发布到 `yaw_total`。

## 6. 温度控制

- 周期 100Hz（采样分频 10），位置式 PID：`Kp=180, Ki=5, Kd=120`，输出限幅 1500（= 15% 占空比）。
- **积分限幅 = `max_out / Ki`（= 300）**，含义是「I 项单独就能顶满输出」。旧值 32.2 太小：
  `Ki × 32.2 = 161`，加上最大 P 项 180 也只有 341 counts（3.4% 占空比），
  实测 40 ℃ 目标只能稳定在 **39.1 ℃（稳态误差 0.875 ℃）**，而硬件上限是 1500 counts（15%）。
  实机确认：`error_sum` 长期贴着 32.2 的限幅，`heater_out` 只有 318 counts。
- **条件积分抗饱和**：输出顶到上限时撤销本拍积分（升温阶段误差很大，若继续累积，
  Ki 会在一两秒内把积分推到限幅值，之后必然大超调）。输出被压到 0（偏热）时同样撤销本拍积分。
  两者合起来才既能顶到 15% 满功率升温、又不超调。
- **温度与姿态解耦**：温度只在温控节拍读取（100Hz）。温度读失败时立即断开加热，但本拍的陀螺/加速度仍然正常送入 EKF，
  姿态不中断。反之姿态异常也不关加热。两侧各自独立降级，避免一处故障掩盖另一处。
- **输出限幅在模块内做**：`pid_calc()` 的 `PID_POSITION` 分支本身不做输出限幅，
  必须在 `imu_temperature_ctrl_step()` 里显式夹到 `[0, IMU_TEMP_PID_OUT_MAX]`，
  否则 `imu_frame.heater_out` 观测值会远超上限（只有 PWM 底层兜底，Ozone 上看到的数是错的）。
  NaN/Inf 无法被 `basic_math_constrain` 拦住（NaN 与任何值比较都为假），所以先单独判掉。
- 安全阀：
  | 条件 | 动作 |
  |---|---|
  | 温度 < -20 ℃ 或 > 100 ℃（读数异常） | 加热置 0，清积分 |
  | 温度 > 55 ℃（过温） | 加热置 0，清积分 |
  | 连续读不到 GYRO_INT 或 SPI 数据 | 加热置 0，进入初始化失败并周期重试 |
- 上电/初始化失败期间占空比恒为 0。安全关断会记住上次下发的占空比，
  已经是 0 时跳过 CCR 写入，避免 1kHz 故障路径上每帧重复写寄存器。

实机验证（静止台上，环境 25 ℃ 左右）：约 40 s 由 39.1 ℃ 升到 40 ℃，最大超调 0.125 ℃（1 LSB），
稳态在 39.75 ~ 40.25 ℃ 之间来回 1 个 LSB，收敛期占空比约 9 ~ 13%。

## 7. 离线检测

| 判定 | 依据 | 对外表现 |
|---|---|---|
| 陀螺/加速度读取失败 | `BMI088_read_imu() != BMI088_NO_ERROR` | `data_valid = 0`、关加热 |
| 温度读取失败/越界 | `BMI088_read_temperature()` 失败或超出 -20~100 ℃ | 关加热，姿态不受影响 |
| 传感器离线 | 连续 3 次 DRDY 超时，或连续 100 帧陀螺/加速度读取失败 | 断开加热 + 周期重试 |
| 采样节拍异常 | `dt` 超出 0.5~3 ms 被限幅 | 本帧不推进 EKF，防止一次异常节拍污染状态 |
| 姿态不可用 | `imu_is_ready() == 0` | 上层应降级（云台保持当前角/乏力） |

状态字段（固件只保留实现必需的项，不保留诊断统计量；需要排查时用调试器直接观察内部静态变量）：

| 字段 | 健康值 | 说明 |
|---|---|---|
| `status` | 4 (`Imu_Status_Running`) | 工作状态 |
| `data_valid` | 1 | 本帧传感器数据有效 |
| `sample_count` | 持续增长 | 采样帧计数（温控分频基准） |
| `temperature` | 39.75 ~ 40.5 ℃ | 芯片温度 |
| `heater_out` | 0 ~ 1500 | 加热输出（与 TIM3 ARR 同量纲） |
| `dt` | ≈ 0.001 s | 实测采样周期 |
| `gyro[3]` / `accel[3]` | — | 已扣零漂角速度 / 加速度 |
| `euler_angle` / `yaw_total` | — | 欧拉角 / 连续多圈 yaw，rad |

实测（2026-10-26，静止台上，40 ℃ 恒温）：

| 指标 | 实测值 |
|---|---|
| 采样率（外部观测） | 998.7 Hz（长期稳定，抖动 < 0.1 Hz） |
| 一帧 SPI 读取耗时（外部观测） | 约 30 µs（优化前约 43 µs） |
| 上电 4.8 s 时 yaw 累积 | **−0.034°**（修复前同一时刻 −8.06°） |
| `yaw_bias_trim` 收敛 | 上电 ~1 s 内定标完成，稳态 2e-5 ~ 6e-5 rad/s |
| 静止 yaw 漂移（归档 8 min 窗口） | −0.020 °/min，累计 −0.085°，无跳变 |
| 静止 yaw 漂移（另一轮 69 s 窗口） | **−0.08 °/min（−2.3e-5 rad/s）**，与 1 kHz 陀螺积分 −0.081 °/min 一致 |
| yaw 估计短期抖动 | ±0.1 °（69 s 内 0.21 ° 峰峰值）—— 这已是估值的噪声底，比它更小的"漂移"测不出来 |
| 温控稳态 | 39.75 ~ 40.5 ℃（超调 0.125 ℃ = 1 LSB） |

> **读数注意**：`QEKF_INS.YawTotalAngle` 的单位是**度**（`imu_frame.yaw_total` 才是 rad）。
> 换算：1e-5 rad/s = 0.00057 °/s = 0.034 °/min ≈ 2 °/h。

上电首帧的 `dt` 约 0.082 s 已被定位：**不是丢中断**，而是 `BMI088_LONG_DELAY_TIME`
的陀螺软复位死区（这段时间不产生 DRDY），只在开机出现一次，且被 `IMU_DT_MAX_S` 限幅到 3 ms，属良性。

姿态可用性已与传感器数据有效性**解耦**：EKF 异常时 `imu_is_ready()` 返回 0，
但 `data_valid` 仍保持 1、加热照常闭环。温控只依赖 `temperature`，与姿态无关，
把整链一起关掉会掩盖真正的温控故障。

## 7.1 EKF 内存分配约束（重要）

`Kalman_Filter_Init()` 通过 `pvPortMalloc()` 分配 25 块矩阵缓冲区（约 1.9 KB）。
FreeRTOS 堆不足时返回 `NULL` 且**不报错**，此后矩阵运算会写到地址 0，
四元数变成 NaN，姿态恒为 0 且每帧复位。

本机曾因堆不足（`configTOTAL_HEAP_SIZE = 15360` 被 7 个动态任务占掉 13824）
导致 `Q/FT/K/S/temp_*` 全部为 `NULL`。现已把所有 FreeRTOS 任务改为**静态分配**，
堆完整留给 EKF。

- `IMU_QuaternionEKF_IsReady()` 在初始化后校验全部 16 个关键指针；
- 任一为空则 `ekf_ready = 0`，`imu_attitude_step()` 直接返回，不调用 Update；
- 健康判据：姿态角持续更新且 `imu_is_ready() == 1`；若姿态恒为 0，用调试器看 `ekf_ready`。

## 8. 输出限幅汇总

| 位置 | 限幅 |
|---|---|
| 温度 PID 输出 | `[0, IMU_TEMP_PID_OUT_MAX]`（模块内显式限幅 + NaN/Inf 保护） |
| 积分项 | `±IMU_TEMP_PID_INTEGRAL_LIMIT`（= `max_out / Ki`）+ 输出饱和时冻结积分 |
| 静止残差零漂 | `±IMU_YAW_TRIM_LIMIT` |
| 加热 PWM 占空比 | IMU 初始化时配置的 `pwm_class` 实例限制到 `[0, 15%]`，最终写 CCR 前再次执行 |

`pwm_class` 默认允许通用设备使用 `[0, 100%]`，每个实例/通道可以在初始化时调用
`set_max_duty_ratio()` 配置自己的最大值。限制是单调收紧的：一旦配置为 15%，
后续业务代码不能通过再次调用该接口提高上限。因此蜂鸣器、电机、舵机等其他 PWM
实例不会被 IMU 的 15% 策略影响。
对于 IMU 加热器，即使 PID 参数、调用方输入或浮点值异常，最终写入定时器的比较值也不会超过 15%。
NaN/Inf、传感器无效、过温、初始化失败、PWM 启动失败和停止路径均写入 `CCR4 = 0`。
该软件限制不能替代硬件验证：上车前仍必须确认 PB1 的电气连接、驱动器有效电平、
MOSFET 栅极保护、24V 加热回路与 H7 地之间不存在反灌电压。

## 9. CubeMX 配置与上车前检查

本模块已登记到 `CMakeLists.txt`，但不直接修改 CubeMX 生成的
`Core/Src/freertos.c`、`Core/Src/gpio.c` 或 `Core/Src/stm32h7xx_it.c`。
请按下面步骤在 CubeMX 图形界面配置，之后由 CubeMX 负责生成代码。

### 9.1 GPIO/EXTI 配置

1. 打开 `infantry.ioc`，确认 `PE12` 的标签为 `GYRO_INT`，信号选择
   `GPIO_EXTI12`，触发边沿为 **Falling Edge**；PE10 若只作 DRDY 观测，
   建议改为普通 `GPIO_Input`（加速度计 INT1 为 800Hz DRDY 推挽输出，
   配成 EXTI 会产生每秒 800 次无效中断）。
2. 上下拉按硬件原理图设置；当前 BMI088 驱动配置为推挽低有效，默认使用
   `No Pull`。
3. 在 NVIC Settings 中勾选 **EXTI line[15:10] interrupts**，优先级 `5`、子优先级 `0`
   （必须数值 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`，
   否则 ISR 中的 `osThreadFlagsSet` 会触发 FreeRTOS 断言）。
4. 点击 Generate Code。不要手动编辑生成的 `gpio.c` 或 `stm32h7xx_it.c`。

### 9.2 FreeRTOS 线程配置

1. 在 Middleware > FREERTOS > Tasks and Queues 中新增任务：
   - Task Name：`ins_task_rtos`
   - Entry Function：`imu_task`（**必须与 `imu_temp_ctrl.h` 中的声明逐字符一致，
     含大小写**；C 语言区分大小写，写错会退化成 CubeMX 生成的空壳弱函数，
     编译不报错但 IMU 完全不工作）
   - Priority：`High`(40)。当前实车配置即此项；`Realtime`(48) 会每秒 1000 次抢占
     `gimbal`/`shoot`/`safe_daemon`(24)，不建议
   - Stack Size：至少 `1024` words；若当前版本显示字节，则填写至少 `4096` bytes
   - Allocation：`Static`（**必须**。Kalman 需要约 1.9 KB 堆，动态任务会把堆吃光，
     导致 EKF 矩阵指针为 NULL、姿态恒为 0；静态分配的栈不占用 `configTOTAL_HEAP_SIZE`）
   - Code Generation Option：`As weak`（由本模块的强定义覆盖）
2. 生成代码后确认 `freertos.c` 中只有一个 `imu_task` 线程创建项，
   且其定义在链接结果中不是 `W`（weak）符号：
   ```powershell
   arm-none-eabi-nm build\Debug\infantry.elf | Select-String imu_task
   ```
   应看到 `T imu_task`，并可用 `arm-none-eabi-addr2line` 确认它落在
   `imu_temp_ctrl.cpp`。
3. 不要在其他用户代码中再次调用 `osThreadNew(imu_task, ...)`。

### 9.4 IMU 任务对其它实时外设的影响（WS2812 等）

IMU 任务优先级 `High`、周期 1 kHz，单次连续占用 CPU 实测约 **130 µs（峰值约 190 µs）**。
这个时间就是**所有低优先级任务能被一次性阻塞的上限**，对靠软件/阻塞 SPI 抖动时序的外设是硬约束。

**已踩过的坑：WS2812 偶发闪停。** WS2812 用 SPI6 位流模拟单总线时序，
一帧 25 字节 ≈ 46.5 µs；灯珠在数据线持续低电平约 50 µs 后就会锁存当前已收到的位。
而 SPI6 只能走 HAL 阻塞发送（BDMA 访问不到 DTCM 里的缓冲区），HAL 在
`FifoThreshold = 01DATA` 下每次只往 FIFO 塞 1 字节，CPU 一旦被抢占超过 1.9 µs
移位时钟就停顿——停顿超过 50 µs 就会把半帧当一帧锁存，表现为灯短暂变暗/熄灭。

- 开启 IMU 前，能打断发送的任务都慢得多、也轻得多，所以不复现；
- 开启 IMU 后，实测 **5.6% 的帧被截断**（约每 1.8 s 闪一次）；
- 修法：`ws2812_write_color()` 在发送期间 `osKernelLock()` 挂起调度器。
注意它只禁止**任务切换**，中断照常响应（本工程 IRQ 全为优先级 5，单次只有几微秒，
远小于 50 µs 门限）。

后续新增"用阻塞方式按时序翻转引脚"的外设（软件 SPI/I2C 从机、单总线 DHT11、
红外编码等）必须同样处理，不能靠"任务优先级低所以慢"来保证时序。

### 9.5 上车前检查

   - TIM3_CH4 确实接加热电阻（IMU 初始化会配置该实例 15% 上限并立即以 0% 启动）；
   - 不得通过其他代码直接写 `TIM3->CCR4` 或绕过 `pwm_class` 启动加热；
   - SPI2 的 `ACC_CS / GYRO_CS` 与 BMI088 安装方向一致；
   - 示波器确认 `GYRO_INT` 为约 1kHz 的下降沿，且没有长时间低电平/毛刺；
   - 若硬件实际输出为高有效，必须同时修改 BMI088 INT3 电平配置和 EXTI 边沿，
     不得只改其中一侧；
   - 先关加热（把 `IMU_TEMP_PID_OUT_MAX` 置 0）验证姿态解算，再逐步放开温控。

## 10. 常见错误与排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `status` 长期停在 `Imu_Status_Init_Fail` | SPI 无响应、片选错误、供电缺失 | 用调试器看 `BMI088_init()` 返回值枚举，或断点看 `read_fail_count` |
| `data_valid` 频繁为 0 | 陀螺 chip id 校验失败（SPI 时序/时钟过高） | 把 `BMI088_SPI_PRESCALER` 改回 `SPI_BAUDRATEPRESCALER_32`，检查 CPOL/CPHA 与走线 |
| yaw 静止缓慢漂移 | `gyro_bias[2]` 与实际零偏不符，或静止修正未生效 | 用调试器看 `yaw_bias_trim`（应已非 0 且稳定，稳态 < 1e-4）与 `still_count`（应停在 500）；仍漂则开 `IMU_GYRO_AUTO_CALIB_ENABLE` 重新标定 |
| **机动后 yaw 出现固定偏置（回位后角度差 0.1~1°）** | 机动过渡帧被误判为静止，污染了 `yaw_bias_trim` | v0.8 已用瞬时角速度门控修复；如再现，用调试器确认门控阈值 `IMU_TRIM_SEED_RATE_MAX` 与 `yaw_bias_trim` 的阶跃时刻 |
| **yaw 突然阶跃几十度/几度后永不回正** | **陀螺单帧野值（SPI 位错/DRDY 竞争）被直接积分** | 已由 `imu_gyro_reject_glitch()` 拦截；如再现需在 `gyro_prev` 处断点确认野值幅值。注意 yaw 无绝对基准，一旦被污染只能靠复位恢复 |
| 姿态恒为 0 / 不更新 | FreeRTOS 堆不足，Kalman 矩阵指针为 NULL | 用 `IMU_QuaternionEKF_IsReady()` 确认；把任务改静态分配或增大 `configTOTAL_HEAP_SIZE` |
| yaw 低速跟踪抖动/爬行 | 曾开启 `CHEAT` 小值截断 | 本版已移除，改查机械摩擦与云台前馈 |
| 采样率明显低于 1000 | 任务被更高优先级任务长期占用、DRDY 丢中断 | 确认 IMU 任务优先级为 `High` 且 `GYRO_INT` 是唯一的 1kHz 中断源；用逻辑分析仪测 `GYRO_INT` 频率 |
| 温度长时间上不去 | 加热占空比太小 / 目标温度过高 | 先检查硬件驱动、有效电平和传感器反馈；不得仅为升温而绕过 `TIM3_CH4` 的底层 15% 上限 |
| 温度超调大 | 积分过大 | 降 `IMU_TEMP_PID_KI`；`IMU_TEMP_PID_INTEGRAL_LIMIT` 已按 `max_out / Ki` 推导，不要单独调小，否则会重新引入稳态误差 |

## 11. 使用示例

```cpp
// 初始化由 imu_task 内部完成, 上层只需要读取
if (imu_is_ready())
{
    float yaw = imu_get_yaw_total_rad();       // rad, 连续多圈
    float wz  = imu_frame.gyro[2];             // rad/s, 已扣零漂
}
```

## 12. 变更记录

| 版本 | 日期 | 变更 | 原因 |
|---|---|---|---|
| 0.1 | — | 初版：SPI 逐字节读、Mahony + EKF 双解算、`CHEAT` 小值截断、`while` 死等初始化 | — |
| 0.2 | 2026-10-26 | ① 驱动改突发传输（3 次/帧）并返回错误码；② 零漂标定不再累加进补偿常量，增加运动帧剔除；③ 移除 `CHEAT`，改静止残差修正；④ 移除周期内 Mahony；⑤ 温控改 `pwm_class` + 单向抗饱和 + 过温/离线保护；⑥ 发布单位统一 rad；⑦ 修 EKF `accLPFcoef` 未赋值与除零 NaN；⑧ 改为 PWM 实例级占空比上限 | 旧实现存在零漂错误累积、低速信息被截断、约一倍冗余解算与潜在 NaN 污染 |
| 0.3 | 2026-10-26 | ① 修 FreeRTOS 堆耗尽导致 Kalman 矩阵指针为 NULL（姿态恒 0、每帧复位），增加 `IMU_QuaternionEKF_IsReady()` 与安全降级；② 姿态可用性与传感器有效性解耦；③ 静止判定改用低通角速度，修 yaw 漂移 0.19°/s → 0.0013°/s；④ 四元数模长异常改为归一化救回，避免复位死锁；⑤ `BMI088_delay_ms` 在调度器运行时让出 CPU；⑥ 修 `BMI088_delay_us` 时钟常量 480→`SystemCoreClock`；⑦ 掉线恢复时重新对齐 DRDY 序号与 DWT 基准 | 实机联调发现：堆不足使 EKF 全部矩阵运算落在地址 0；静止阈值低于噪声底使零漂修正永不触发 |
| 0.4 | 2026-10-26 | ① 驱动拆出 `BMI088_read_imu`（2 次事务）与 `BMI088_read_temperature`（1 次），温度降到 100Hz 与温控同拍，一帧事务 3→2；② `BMI088_com_init()` 运行时把 SPI2 从 3.125MHz 提到 6.25MHz，一帧读取 43→29.9 µs；③ 驱动改用局部缓冲区，消除「半帧」；④ 温度与姿态故障各自降级，互不牵连；⑤ 补上温度 PID 输出限幅与 NaN 保护；⑥ `dt` 限幅单独计入 `dt_clamp_count`，不再混入通信故障计数；⑦ 新增 `spi_time_us`、`sample_rate_hz`、`temp_read_count` 在线诊断；⑧ 加热安全关断跳过重复 CCR 写入 | 1kHz 采样下每帧多读一次温度纯属浪费带宽；`pid_calc` 不做输出限幅使观测值失真；故障计数把「丢 DRDY」和「SPI 失败」混在一起，无法定位 |
| 0.5 | 2026-10-26 | ① 温控积分限幅改为 `max_out / Ki` + 条件积分抗饱和，40℃ 目标由 39.1℃（稳态误差 0.875℃）改为 40.0℃、超调仅 0.125℃；② 静止判定的加速度模长也做 0.2s 低通，`still_reset_count` 从 5 次/25s 降到 0~1 次/30s，静止 yaw 漂移回到 0.0001~0.0018 °/s；③ 新增 `still_reset_count` / `still_reset_reason` / `still_peak_*` / `still_gyro_norm` / `still_accel_norm` 诊断字段，用于定位判定被打断的真实原因 | 实机验证发现：温控积分贴着限幅导致恒温点长期偏低；加速度瞬时跳变（9.25 m/s²）反复打断静止确认，使零漂修正频繁中断、yaw 漂移恶化近 10 倍 |
| 0.6 | 2026-10-26 | ① 新增 `task_time_us` / `task_time_max_us`，把"IMU 任务占用 CPU 时长"变成可观测量；② 依据该数据定位并修复 WS2812 偶发闪停（详见 §9.4，修复落在 `ws2812.cpp`） | 1kHz 高优先级任务把低优先级任务的阻塞时长拉到 127~163 µs，超过了 WS2812 用阻塞 SPI 发送一帧（46.5 µs）时能容忍的 50 µs 锁存门限 |
| 0.7 | 2026-10-26 | ① **新增陀螺野值抑制 `imu_gyro_reject_glitch()`**：逐轴超量程（>36 rad/s）永不放行、变化率 >10000 rad/s²×dt 先保持上一拍、连续两帧偏离才认定为真实机动；② **修正 z 轴 `gyro_bias[]` 初值 0.0000852 → 0.0033 rad/s**（旧值比实测小 38 倍）；③ `yaw_bias_trim` 首次静止确认后直接平均 500 帧**一次定标**，不再从 0 慢积分；④ `IMU_YAW_TRIM_LIMIT` 0.01 → 0.02 rad/s；⑤ 软复位后重置 trim 定标与野值抑制基准 | 归档数据分析发现：板子静止时静止判定用低通角速度单帧从 0.0047 跳到 0.1269 后严格按 0.2s 指数衰减，反推注入 24.8 rad/s（1420°/s），yaw 当场跳 −2.15° 并永久保留（EKF 对 z 轴无绝对观测）；同时硬编码 z 轴零偏失效导致上电后 yaw 以 0.19°/s 漂移数分钟。修复后 8 min 漂移 −0.020°/min、上电 4.8s 累积由 −8.06° 降到 −0.034° |
| 0.8 | 2026-10-26 | ① **静止残差积分加逐帧瞬时角速度门控**（`\|ω\| > 0.15 rad/s` 的帧不计入 trim）；② 文档补 §5.2.2「yaw 零漂传递路径」与实测对照表 | 归档数据里 trim 的阶跃与机动帧一一对应（120.22 s：z 轴角速度 −0.163 rad/s 仍判静止 → Δtrim = −1.9e-4；140.34 s：−0.315 rad/s → −2.1e-4），单次机动留下 0.0115 °/s 固定偏置需 20 s 才衰减；同时确认 EKF 输出的漂移与 1 kHz 陀螺积分仅差 2%，漂移源 100% 在 z 轴零偏而不在 EKF |
| 0.9 | 2026-10-26 | **移除全部诊断观测变量，只用实现必需的字段**：`imu_frame` 精简为姿态/角速度/温度/温控/状态共 12 项；删除 `gyro_raw`、`gyro_bias`、`error_count`、`dt_clamp_count/last`、`temp_read_count`、`still_*`（6 个）、`yaw_bias_trim`、`trim_gate_reject_count`、`gyro_glitch_*`/`gyro_peak_abs`、`yaw_int_raw/filt`、`yaw_drift_*`、`spi_time_us`、`sample_rate_hz`、`task_time_us/_max`；`QEKF_INS` 删除 `UpdateOkCount`/`ResetCount`/`RescaleCount`/`ResetReason`/`DbgQNormSq`；删除 `imu_run_task_step()` 的 DWT 计时包装；`ws2812` 删除 `last/max/slow/corrupt_write_*`、`write_count`、`error_count_` 与其 DWT 计时 | 观测字段在上车后会持续占用 RAM 并增加每帧计算（浮点乘除、DWT 读），而它们对控制与解算没有任何贡献；结论与实测数据已归档到本文档，需要现场排查时用调试器直接看内部静态变量即可。移除后 FLASH 242908 → 241208 B（−1.7 KB），DTCMRAM 81840 → 81624 B |
