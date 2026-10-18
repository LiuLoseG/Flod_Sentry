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
    float position_gain_per_s;      // 位置比例增益，单位：1/秒
    float dt_min_s;                 // 最小采样时间，单位：秒
    float dt_max_s;                 // 最大采样时间，单位：秒
    bool wrap_angle;                // 是否进行角度环绕处理(最短角规划)
} reference_config_t;

/**
 * @brief 轨迹规划总结构体
 */
typedef struct
{
    reference_value_t value;            // 目标轨迹规划点
    reference_config_t config;          // 轨迹规划参数配置
    bool is_initialized;                // 是否已初始化
} reference_track_plan_t;

// 初始化参考轨迹规划
bool reference_track_plan_init(reference_track_plan_t *plan, const reference_config_t *config, float initial_angle_rad);

// 重置参考轨迹规划
bool reference_track_plan_reset(reference_track_plan_t *plan, float angle_rad);

// 更新参考轨迹规划
bool reference_track_plan_update(reference_track_plan_t *plan, float target_angle_rad, 
                                float dt_s, reference_value_t *output);
