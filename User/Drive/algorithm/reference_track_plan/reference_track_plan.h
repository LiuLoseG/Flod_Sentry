/**
 * @file reference_track_plan.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
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
typedef struct {
    float max_rate_rad_s;           // 最大角速度，单位：rad/s
    float max_accel_rad_s2;         // 最大角加速度，单位：rad/s^2
    float max_jerk_rad_s3;          // 最大角加加速度，单位：rad/s^3
    float position_gain_per_s;      // 位置比例增益，单位：1/秒
    float dt_min_s;                 // 最小采样时间，单位：秒
    float dt_max_s;                 // 最大采样时间，单位：秒
    bool wrap_angle;                // 是否进行角度环绕处理(最短角规划)

    bool  accel_sat_guard_enable;   // true: 检测到加速度/加加速度被限幅时临时降低位置增益
    float sat_gain_scale;           // 饱和时位置增益缩放系数, 0<scale<1 (建议 0.4~0.6)
    float sat_release_per_s;        // 饱和解除后增益恢复速率, 单位 1/s (建议 1~3)
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
} reference_track_plan_t;

// 初始化参考轨迹规划
bool reference_track_plan_init(reference_track_plan_t *plan, const reference_config_t *config, float initial_angle_rad);

// 重置参考轨迹规划
bool reference_track_plan_reset(reference_track_plan_t *plan, float angle_rad);

// 更新参考轨迹规划
bool reference_track_plan_update(reference_track_plan_t *plan, float target_angle_rad, 
                                float dt_s, reference_value_t *output);
