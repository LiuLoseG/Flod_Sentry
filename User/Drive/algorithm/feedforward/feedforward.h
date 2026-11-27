/**
 * @file feedforward.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 轴模型前馈: 由辨识出的物理参数直接算出力矩前馈
 * @date 2026-09-01
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#pragma once

#include "basic_math.h"
#include "general_filter.h"

/*------------------------------------默认配置(配置字段非法时回退到这里)-------------------------------------------*/

/* 参考信号调理的合法调用间隔上限, s
 * 超过它说明任务被长时间抢占, 这一拍的滤波/jerk 积分没有物理意义(会把整段变化压进一拍), 直接重新对齐
 */
#define FEEDFORWARD_DEFAULT_REF_DT_MAX_S 0.02f

/* tanh 速度尺度下限, rad/s
 * eps 太小时 tanh 退化成 sign(阶跃), 摩擦前馈变成 bang-bang, 换向会抖
 */
#define FEEDFORWARD_DEFAULT_EPS_MIN_RAD_S 0.001f

/**
 * @brief 单轴前馈参数(与 gimbal_identification 的模型同构)
 * @note tau = J*acc + B*rate + Fc*tanh(rate/max(eps, eps_min)) + A*sin(q) + C*cos(q)
 *       参数来自在线/离线辨识, 单位与 gimbal_identification_model_t 一致。
 *       eps_min_rad_s 放在最后: 旧的 7 项聚合初始化会把它置 0, 此时按默认值处理。
 */
typedef struct
{
    float j;             // 等效惯量, kg·m^2
    float b;             // 粘滞阻尼, N·m·s/rad
    float fc;            // 库仑摩擦幅值, N·m
    float eps;           // 库仑摩擦 tanh 速度尺度, rad/s
    float grav_sin;      // 重力项 sin 系数, N·m
    float grav_cos;      // 重力项 cos 系数, N·m
    float dynamic_max;   // 动态项限幅(不含重力), N·m; <=0 表示动态项恒为 0
    float eps_min_rad_s; // eps 下限, rad/s; 非有限或 <=0 时取 FEEDFORWARD_DEFAULT_EPS_MIN_RAD_S
} feedforward_param_t;

/**
 * @brief 参考信号调理器配置
 * @note 初始化时传入一份结构体参数; 运行中也可直接改写 tracker->config (如在 Ozone 中), 每次 update 都会重新校验:
 *       限幅/jerk 非有限或 <=0 视为不限制; 截止频率非有限或 <=0 视为不滤波(直通);
 *       ref_dt_max_s 非有限或 <=0 时回退到默认值。
 *       低通由 general_filter 的 low_pass_filter_class 实现(双线性变换 + 预畸变), 用截止频率而不是系数描述。
 */
typedef struct
{
    float max_rate_rad_s;          // 角速度限幅, rad/s; <=0 表示不限幅
    float max_accel_rad_s2;        // 角加速度限幅, rad/s^2; <=0 表示不限幅

    float rate_filter_cutoff_hz;   // 角速度参考低通截止频率, Hz; 非有限或 <=0 表示不滤波
    float accel_filter_cutoff_hz;  // 角加速度参考低通截止频率, Hz; 非有限或 <=0 表示不滤波
    
    float accel_jerk_limit_rad_s3; // 角加速度变化率上限, rad/s^3; <=0 表示不限制
    float ref_dt_max_s;            // 合法调用间隔上限, s; 超过则本拍重新对齐
} feedforward_tracker_config_t;

/**
 * @brief 前馈参考信号调理器
 * @note 目标角速度/角加速度由外部(轨迹规划、自瞄预测、遥控斜坡等)直接给定, 本模块不再对目标角差分,
 *       也不引入实测速率, 只做: 限幅 -> 一阶低通(截止频率按实际 dt 重算) -> 加速度限 jerk。
 *       这样参考信号与反馈完全解耦, 前馈不会被测量噪声带偏。
 */
typedef struct
{
    feedforward_tracker_config_t config; // 可在线修改的配置
    low_pass_filter_class rate_filter;   // 角速度参考低通(截止频率 <=0 时整段不调用)
    low_pass_filter_class accel_filter;  // 角加速度参考低通(截止频率 <=0 时整段不调用)
    float rate_rad_s;                    // 调理后的参考角速度, rad/s
    float accel_rad_s2;                  // 调理后的参考角加速度, rad/s^2
    bool primed;                         // 是否已完成首拍对齐
} feedforward_tracker_t;

/**
 * @brief 前馈输出
 */
typedef struct
{
    float dynamic_nm; // 动态项(J*acc + B*rate + 库仑摩擦), 已按 dynamic_max 限幅
    float gravity_nm; // 重力项 A*sin(q) + C*cos(q), 未限幅
    float total_nm;   // 总前馈 = 动态项 + 重力项, 已按 torque_limit_nm 限幅
} feedforward_output_t;

/**
 * @brief 初始化参考信号调理器(写入配置并清零输出, 首拍直接对齐外部给定)
 * @param config 配置, 为空时使用模块默认结构体配置
 */
void feedforward_tracker_init(feedforward_tracker_t *tracker, const feedforward_tracker_config_t *config);

/**
 * @brief 运行中替换配置, 不清零当前参考值
 * @note 新配置在下一次 update 生效; 若改动幅度大(如限幅收紧), 调用方可自行 reset。
 */
void feedforward_tracker_set_config(feedforward_tracker_t *tracker, const feedforward_tracker_config_t *config);

/** @brief 清零输出, 下一拍重新对齐外部给定的速度(加速度从 0 起按 jerk 限制爬升)。 */
void feedforward_tracker_reset(feedforward_tracker_t *tracker);

/**
 * @brief 输入外部给定的目标角速度/角加速度, 输出调理后的参考值
 * @param target_rate_rad_s   外部给定目标角速度
 * @param target_accel_rad_s2 外部给定目标角加速度
 * @param dt_s                距上次调用的时间, 非法或超过上限时复位
 */
void feedforward_tracker_update(feedforward_tracker_t *tracker, const float &target_rate_rad_s, const float &target_accel_rad_s2, const float &dt_s);

/**
 * @brief 计算单轴力矩前馈
 * @param param            前馈参数
 * @param angle_rad        实测关节角(重力项的相位)
 * @param rate_rad_s       参考角速度(建议取 feedforward_tracker_t::rate_rad_s)
 * @param accel_rad_s2     参考角加速度(建议取 feedforward_tracker_t::accel_rad_s2)
 * @param torque_limit_nm  该轴力矩上限, 用于总前馈收口; <=0 时不做总限幅
 * @param output           输出(可为空, 此时直接返回)
 * @note 任一输入(含模型参数与力矩上限)为非有限值或中间计算溢出时输出全 0 —— 前馈算错的方向可能是助力, 宁可少补不可乱补。
 *       dynamic_max<=0 时跳过动态项计算, 仅计算重力补偿; 仍检查全部输入的有限性。
 */
void feedforward_calculate(const feedforward_param_t *param,
                           const float &angle_rad,
                           const float &rate_rad_s,
                           const float &accel_rad_s2,
                           const float &torque_limit_nm,
                           feedforward_output_t *output);
