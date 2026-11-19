/**
 * @file reference_track_plan.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 带运动约束的参考轨迹跟踪接口
 * @date 2026-10-18
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 轨迹规划目标结构体
 */
typedef struct
{
    float angle_rad;        // 角度，单位：rad
    float rate_rad_s;       // 角速度，单位：rad/s
    float accel_rad_s2;     // 角加速度，单位：rad/s^2
} reference_value_t;

/**
 * @brief 参数配置结构体
 */
typedef struct
{
    float max_rate_rad_s;           // 最大角速度，单位：rad/s
    float max_accel_rad_s2;         // 最大角加速度，单位：rad/s^2
    float max_jerk_rad_s3;          // >0: 三阶阻尼参考跟踪及jerk约束; <=0: 兼容二阶跟踪
    float position_gain_per_s;      // 位置比例增益，单位：1/秒
    float rate_servo_gain_per_s;    // 速率环增益 a=K*(期望角速度-角速度)，单位：1/秒
                                    // jerk启用时带宽w<=min(position_gain,rate_servo_gain/3)
    float dt_min_s;                 // 最小采样时间，单位：秒
    float dt_max_s;                 // 最大采样时间，单位：秒
    bool wrap_angle;                // 是否进行角度环绕处理(最短角规划)

    bool  accel_sat_guard_enable;   // true: 检测到加速度/加加速度被限幅时临时降低位置增益
    float sat_gain_scale;           // 饱和时位置增益缩放系数, 0<scale<1 (建议 0.4~0.6)
    float sat_release_per_s;        // 增益降低/恢复的最大速率, 单位 1/s (建议 1~3); 0 表示保持增益
    float sat_hold_time_s;          // 连续受限使位置增益降到下限后, 位置项暂挂的时长, 单位 秒
                                    // >0 启用超能力守护; 0 关闭 (暂挂期间只跟随目标速度/加速度前馈)
    bool estimate_target_rate;      // 无有效速度输入时由角度估计目标速度与目标加速度；有效速度优先，默认关闭
} reference_config_t;

/**
 * @brief 轨迹规划总结构体
 */
typedef struct
{
    reference_value_t value;            // 目标轨迹规划点
    reference_config_t config;          // 轨迹规划参数配置
    bool is_initialized;                // 是否已初始化

    float gain_scale;                   // 当前位置增益缩放, 1.0=正常, 饱和时 -> sat_gain_scale
    bool accel_saturated;               // 上一帧 accel/jerk 是否被限幅 (供上层 P4 监督读取)
    uint32_t sat_frames;                // 连续饱和帧数 (调试/观察用)
    float sat_time_s;                   // 连续受限累计时间, 未受限时清零
    bool position_hold;                 // true: 位置项暂挂, 本拍只跟随目标速度/加速度前馈
    float position_hold_s;              // 暂挂剩余时间, >0 表示仍暂挂
    float over_capability_time_s;       // 目标“同号且超能力加速度”的持续时长
    float last_rate_increment_rad_s;    // 上一拍目标速度增量, 供超能力检测判同号
    float dt_used_s;                    // 最近成功更新的真实采样时间，失败时保持
    float previous_target_angle_rad;
    float estimated_target_rate_rad_s;        // 目标角速度估计(未过滤的原给速度历史见下)
    float estimated_target_accel_rad_s2;      // 目标角加速度估计, 仅平滑目标(可微分)时有效
    float previous_measured_rate_rad_s;       // 上一拍目标速度原值, 供二阶差分
    bool target_history_valid;
} reference_track_plan_t;

/**
 * @brief 复制配置并从给定角度静止起步。
 * @param plan 调用方持有的实例。
 * @param config 配置；成功后修改原配置不影响实例。
 * @param initial_angle_rad 初始参考角，单位 rad。
 * @return 成功返回 true；空指针返回 false 且不写入；非法配置或角度会清空实例。
 */
bool reference_track_plan_init(reference_track_plan_t *plan, const reference_config_t *config, float initial_angle_rad);

/**
 * @brief 保留配置，将参考速度、加速度及饱和历史清零。
 * @param plan 已初始化的实例。
 * @param angle_rad 新的参考起点，单位 rad，与目标角使用相同零点及正方向。
 * @return 成功返回 true；实例为空、未初始化或角度非有限时返回 false，不修改实例。
 */
bool reference_track_plan_reset(reference_track_plan_t *plan, float angle_rad);

/**
 * @brief 更新参考轨迹规划(含目标角速度/角加速度前馈)
 * @param plan 总体轨迹规划结构体
 * @param target_angle_rad 目标角度(rad)
 * @param omega_ff_rad_s 目标角速度前馈(rad/s), 与目标角同向; rate_valid为true时优先使用
 * @param accel_ff_rad_s2 目标角加速度前馈(rad/s^2), 与目标角同向
 * @param dt_s 时间步长(s)
 * @param output 输出的参考值
 * @param rate_valid 本拍速度输入是否有效，0也可为有效速度；false时按配置估计或使用0
 * @param accel_valid 本拍加速度输入是否有效，false时使用0，不进行二次差分
 * @retval true: 更新成功
 * @note jerk启用时按拍内恒定jerk积分，输出为拍末q/v/a；不是时间最优S曲线。
 *       禁止运行中突降约束使已有状态不可达；此时返回false，由调用方显式处理。
 *       标记有效但非有限的前馈拒绝更新；标记无效的数值忽略。增益缩放越界时拒绝更新。
 *       返回false时状态和output不变；调用方不得将旧输出视为本拍有效参考。
 * @retval false: 更新失败（输入/状态非法或约束不可达）
 **/
bool reference_track_plan_update(reference_track_plan_t *plan, float target_angle_rad,
                                float omega_ff_rad_s, float accel_ff_rad_s2,
                                float dt_s, reference_value_t *output,
                                bool rate_valid, bool accel_valid);
