# 云台辨识模块使用与操作

本模块从同步采集的角度、速度、实际关节力矩估计单轴物理参数，不发送电机命令，也不自动修改控制器。每个轴、固定机械配置使用独立实例，由单一任务串行调用。接口见 [gimbal_identification.h](gimbal_identification.h)。依赖 [RLS](../../algorithm/rls/rls_last/rls_last.h)，必须关闭 fast-math/finite-math-only。

## 模型与坐标

$$
\tau=J\dot\omega+B\omega+F_c\tanh(\omega/\epsilon)+A\sin(q_g)+C\cos(q_g)
$$

| 参数 | 含义 | 单位 |
| --- | --- | --- |
| J | 输出关节等效惯量，必须为正 | kg·m² |
| B | 粘滞阻尼，非负 | N·m·s/rad |
| Fc | 库仑摩擦幅值，非负 | N·m |
| epsilon | 固定摩擦过渡速度，必须为正 | rad/s |
| A/C | 固定重力平面的正弦/余弦系数，可带符号 | N·m |

Yaw 支持 J/B 或 J/B/Fc；Pitch/Big Pitch 支持 J/B/Fc 或完整五参数。Pitch 三参数模式要求调用方已扣除重力。四参数和 Pitch 两参数被拒绝。

`angle_rad` 是连续关节坐标，`rate_rad_s` 必须为同坐标系导数；不能直接混用转子转速和输出轴角度。`gravity_angle_rad` 是固定平面重力相位，可与关节坐标有固定偏移，但不能在辨识期间任意改变。力矩方向应与角度正方向一致；电流转换力矩须包含减速比、效率及标定约定。

回归使用完整窗口的速度端点差、角度端点差、摩擦积分、重力积分和力矩积分。相邻窗口共享端点，不共享时间区间。

## 接入步骤

1. 用默认配置作为结构初始化，再按轴设置配置 ID、边界、初值及时间限制；默认值是合成起点，不是硬件标定值。
2. 静态分配实例，检查初始化返回值；失败时实例被清空，不应继续当作已初始化对象使用。
3. 只在收到一组新的、完整且同步的反馈后提交一次样本；时间戳是采集时刻，不能用读取缓存的时刻重复提交旧数据。
4. 每拍检查包装层 flags；完整窗口结束时检查 `window_updated` 和 RLS 诊断。
5. 将候选参数保存到上层日志/候选模型；独立验证后再由应用决定如何应用。

```cpp
#include "gimbal_identification.h"

static gimbal_identification_t yaw_ident;

bool init_yaw_identification()
{
    gimbal_identification_config_t config;
    gimbal_identification_default_config(&config, GIMBAL_IDENTIFICATION_YAW);
    config.configuration_id = 1u;
    config.window_s = 0.05f;
    config.max_gap_s = 0.02f;
    config.rls.dt_max_s = 0.1f;
    return gimbal_identification_init(&yaw_ident, &config);
}

uint32_t observe_yaw(const gimbal_identification_sample_t &coherent_sample,
                     gimbal_identification_output_t &result)
{
    return gimbal_identification_update(&yaw_ident, &coherent_sample, &result);
}
```

调用方应在自己的初始化错误分支记录失败，禁止只忽略返回值。示例中的样本必须填写下列字段：

| 字段 | 填写要求 |
| --- | --- |
| timestamp_us | 单调采集时间，uint32 微秒；支持回绕，相邻时间差须小于 2³¹ 微秒 |
| configuration_id | 与初始化配置一致；改变负载/模型配置应重新初始化 |
| segment_id | 连续观测段 ID；关节角重新归零、坐标重锚或换段时递增 |
| angle/rate/gravity_angle/torque | 有限、单位一致、同一采集时刻对应的反馈 |
| feedback_age_s | 最旧必要反馈的保守年龄，不是任务周期 |
| enabled/valid | 本拍辨识使能与数据有效性 |
| operating_conditions_valid | 调用方确认底座和机械配置符合固定轴模型 |
| torque_calibrated/time_aligned | 调用方已独立确认标定与时间对齐；不能因残差小而置 true |
| saturated | 饱和状态；是否接纳由 accept_saturated 决定 |

## 参数配置

- `window_s`：积分时长；50 ms 是已验证的合成基线，不能视为所有机械系统最优。高频运动需防止窗口过长损失信息。
- `max_gap_s`：允许相邻有效采样最大间隔；不要通过放宽它掩盖严重丢帧。
- `rls.dt_min_s <= window_s`，`rls.dt_max_s >= window_s + max_gap_s`。窗口和最大间隔之和须小于 2147 s，两者至少 1 微秒。
- `feedback_timeout_s`：超时即拒绝；所有涉及的反馈都应计入年龄。
- `accept_saturated=false` 为默认。只有使用饱和后的真实、已对齐力矩且模型仍适用时才允许 true；饱和前指令不能作为真实力矩替代。
- `derive_rate_from_angle=false` 为默认。开启后用最近 3~32 点二次最小二乘拟合，在最新时刻求导，仅替换摩擦积分的速度；惯量仍使用原始速度端点，阻尼仍使用角度端点差。
- 重构历史未满时使用原始速度；历史已满但重构失败时显式上报数值故障并清观测。20 点可作实验起点；端点导数噪声比中心斜率大，高阶运动仍有拟合偏差。
- 拟合采用 double 中间量，开启时每拍遍历配置窗口；STM32 构建通过不代表满足实时预算。应用应测量最坏执行时间，尤其在 1 kHz 多轴同时开启时。
- `max_relative_uncertainty` 必须有限非负，0 关闭此门槛。指标为 sqrt(Pii)/abs(theta_i)，没有噪声校准时不是置信区间；接近零的真实 A/C 也可能令门槛长期不能通过。
- `rls.feature_scale/output_scale` 决定归一化；调整会改变信息指标和先验权重，应重新仿真。`forgetting_time_s=0` 默认不遗忘；开启遗忘须单独验证漂移和稳态噪声。

## 结果和异常处理

`window_updated` 只表示本次新窗口被 RLS 接受；`ready` 还要求本次更新满足 RLS 门槛且未超不确定度上限。两者都是每次调用的事件，下一拍可变为 false。不要把最后一次 ready 直接当作持续使能。

`model` 是候选模型；非更新拍仍可能保留已学参数。`output.flags` 和 `output.rls.flags` 是不同命名空间，必须分别记录。`window_duration_s` 非零表示完整窗口已提交，不代表更新成功。

| 标志/状态 | 行为与处理 |
| --- | --- |
| BAD_CONFIG | 初始化/配置异常；检查轴、参数数目、边界、时间范围和阈值 |
| BAD_INPUT / DISABLED / STALE_FEEDBACK | 清积分、重构和 RLS 观测资格，保留参数/P；恢复有效采集 |
| CONFIGURATION_MISMATCH | 拒绝且不推进本实例时钟；匹配实例或重新初始化 |
| BAD_TIMESTAMP | 重复或倒序；保留最后接受的时钟位置，修复采集顺序 |
| GAP / SEGMENT_CHANGE | 清当前积分和速度历史，保留 RLS 历史，当前有效样本可重新锚定 |
| SATURATED | 默认清观测；accept_saturated=true 时只上报并继续 |
| UNQUALIFIED_OPERATION / TORQUE_UNCALIBRATED / TORQUE_UNALIGNED | 条件不成立时不辨识，先修复条件 |
| NUMERIC_FAULT / RLS_REJECTED | 记录包装及 RLS 标志，检查输入尺度、状态和编译选项；不能把此输出直接应用 |
| RLS LOW_EXCITATION | 特征历史继续积累，冻结参数/P；应改善数据的可辨识性，不放宽门槛强行 ready |
| RLS PROJECTED | 参数被投影到边界，本次不 ready；检查边界、模型和单位 |

`reset()` 恢复配置初值/P，清计数及时间；`clear_observations()` 保留已学参数/P和累计更新次数，清观测与时间连续性。改变轴、模型、负载或固定配置时使用 init，不直接改实例成员。角度重锚必须换 segment，即使之前已完成一个窗口。

## 操作和记录流程

1. 先确认反馈单位、方向、标定、时间对齐及现有系统运行边界，保持原控制系统管理运动。
2. 先记录原始速度分支作为对照；只在角度连续且分辨率足够时比较重构分支。
3. 日志包含采集时间、段/配置 ID、原始测量、饱和/年龄、全部 flags、参数、P 对角、信息指标、残差 RMS、更新与 ready。
4. 用不同时间段验证候选模型，分别观察加速、匀速、换向与不同位置的残差。单一匀速不能分离惯量，过小位置范围不能可靠分离重力两列。
5. 若不 ready，依次检查是否有完整窗口、时间/反馈拒绝、持续激励、投影、创新 RMS、各参数信息指标。累计更新数大不表示参数准确。
6. 参数冻结保存后再评估前馈效果，保留现有力矩限制和模式切换处理；本模块不自动下发参数。

## 对接前馈

模型映射到 [feedforward](../../algorithm/feedforward/feedforward.h)：inertia→j，viscous→b，coulomb→fc，friction_velocity→eps，gravity_sin/cos→grav_sin/cos。`dynamic_max`、最大加速度、总力矩上限属于应用配置，不从辨识结果推断。

前馈实现将 eps 下限固定为 0.001 rad/s；辨识若选更小 epsilon，摩擦模型将不一致。前馈重力输入必须沿用辨识的 gravity_angle 相位。切模式时用实测速率 `feedforward_tracker_reset`；首拍只对齐，随后用 `feedforward_tracker_update` 生成有界参考速率/加速度，再调用 `feedforward_calculate`。

动态项先按 dynamic_max 限幅，重力另算，总和再按 torque_limit 限幅；后续与反馈控制器相加后的最终电机命令仍需应用层限幅。非有限输入/中间溢出会使前馈输出全零，不能用零输出反推模型正常。

## 仿真和验收

本轮直接编译生产辨识/RLS 完成主机回环验证。测试源码、运行脚本及临时生成程序已清理，以下保留历史验证设定与结果。MinGW 编译时报告了 RLS 已有的枚举/无符号数条件表达式警告。

回环由双精度 RK4 对象、固定 PD 控制器和 1 kHz 采样构成，实际施加力矩限幅为 ±3 Nm。连续运行 180 s 辨识后冻结参数，在另外 30 s 的独立轨迹上验证力矩预测，并比较 PD 与 PD 加模型前馈的跟踪误差（忽略前 2 s）。训练三正弦幅值为 0.75/0.32/0.15 rad，角频率为 1.1/2.3/4.7 rad/s；验证频率乘 1.19，并改变各项相位。

三轴 × 三模式（理想反馈、量化加噪声、量化加噪声并重构摩擦速度）× 三种子，共 27 组验收。量化步长为角度 0.000096 rad、速度 0.015 rad/s，力矩高斯噪声标准差为 0.0003 Nm，积分窗口保持默认 50 ms。固定验收门槛为最大参数相对误差 <=5%、独立力矩 RMSE <=0.005 Nm、跟踪角度 RMSE 至少降低 75%。2026-11-17 实测 27 组通过：最坏参数误差 0.923%，最坏力矩 RMSE 0.002049 Nm，补偿后最坏角度 RMSE 0.000987 rad。

另保留低频弱激励诊断：角频率降为 0.47/0.83/1.37 rad/s 时，量化 Yaw 的参数误差约 16.6%，同时仍出现 ready。该案例显式断言误差超过 5%，用于记录 ready 不能保证参数准确，不计为精度验收通过。边界用例覆盖时间回绕、重复/倒序、配置错配、断段、丢帧、饱和策略、陈旧/NaN 反馈及恢复；重构用例覆盖 3/20/32 点、采样抖动与回绕；生命周期检查清观测保留参数/P及 reset 恢复初值。

前馈验证使用同构数学模型，不调用生产 feedforward、参考跟踪器或固件控制任务。以上仅为主机软件回环，未验证真实硬件、实测温漂、全部频段、应用层前馈限幅与实时执行预算；上机前仍须完成独立回放及硬件验证。
