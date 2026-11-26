/**
 * @file power_ctrl.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-11-26
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// 电机转速从 rpm 转换为 rad/s 的换算系数
#define POWER_RPM_TO_RAD (2.0f * 3.14159265f / 60.0f)

/** @brief 电机输入功率模型，电流使用 A，角速度使用 rad/s，功率使用 W。 */
typedef struct {
    float k1;               // 转速与电流乘积项系数，描述机械做功
    float k2;               // 电流平方项系数，描述铜耗，必须非负
    float k3;               // 转速平方项系数，描述速度相关损耗
    float k4;               // 单电机固定损耗，单位 W
} power_motor_model_t;

/**
 * @brief 单电机功率状态
 */
typedef struct
{
    float speed_rad_s;      // 电机转速，单位 rad/s
    float limited_cmd;      // 限幅后的电机输入指令，单位 A
} motor_power_state_t;

/**
 * @brief 电机功率节点，包含电机状态和功率模型
 */
typedef struct
{
    motor_power_state_t *state; // 电机状态
    const power_motor_model_t *model; // 电机功率模型
} power_node_t;

/**
 * @brief 真实电机对应的平移,旋转,补偿运动分量
 */
typedef struct
{
    power_node_t motor;         // 电机状态和模型
    float translation_cmd;      // 平移产生的有符号电流指令， A
    float rotation_cmd;         // 旋转产生的有符号电流指令， A
    float compensation_cmd;     // 补偿产生的有符号电流指令， A
    float max_cmd;              // 最大允许的有符号电流指令， A
} power_motion_node_t;

/**
 * @brief 加权目标函数使用的运动分量优先级
 */
typedef enum
{
    POWER_PRIORITY_TRANSLATION = 0, // 平移优先
    POWER_PRIORITY_ROTATION = 1,    // 旋转优先
} power_motion_priority_e;

/**
 * @brief 功率运动约束求解状态
 */
typedef enum
{
    POWER_MOTION_OK = 0,                // 原始请求可全部执行
    POWER_MOTION_LIMITED,               // 已找到满足约束的缩放组合
    POWER_MOTION_NO_FEASIBLE_CANDIDATE, // 无可行解，无法满足约束, 输出零电流
    POWER_MOTION_INVALID_INPUT,         // 输入参数无效, 可访问的输出电流清零
} power_motion_status_e;


/** 分配结果；比例范围为 [0, 1]，功率单位为 W。 */
typedef struct {
    float translation_scale;        // 所有轮共用的平移保留比例
    float rotation_scale;           // 所有轮共用的旋转保留比例
    float requested_power;          // 平移、旋转和轮级修正均完整输出时的预测功率
    float allocated_power;          // 最终输出电流对应的预测功率
    power_motion_status_e status;   // 分配状态，调用方须检查后再发送电流
} power_motion_result_t;

