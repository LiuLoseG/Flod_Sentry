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

/**
 * @brief 单轴前馈参数(与 gimbal_identification 的模型同构)
 * @note tau = J*acc + B*rate + Fc*tanh(rate/eps) + A*sin(q) + C*cos(q)
 *       参数来自在线/离线辨识, 单位与 gimbal_identification_model_t 一致。
 */
typedef struct
{
    float j;           // 等效惯量, kg·m^2
    float b;           // 粘滞阻尼, N·m·s/rad
    float fc;          // 库仑摩擦幅值, N·m
    float eps;         // 库仑摩擦 tanh 速度尺度, rad/s
    float grav_sin;    // 重力项 sin 系数, N·m
    float grav_cos;    // 重力项 cos 系数, N·m
    float dynamic_max; // 动态项限幅(不含重力), N·m; <=0 表示动态项恒为 0
} feedforward_param_t;

/**
 * @brief 前馈参考速率跟踪器
 * @note 前馈的 J*acc 项要求加速度有界可微。上游给的是阶跃速度指令(如换向 ±0.6rad/s)时,
 *       直接差分会造出上千 rad/s^2 的假加速度(J*acc 可达数 N·m), 完全盖过真实惯量补偿。
 *       因此参考速率必须先过限斜率跟踪, 再差分得到加速度。
 */
typedef struct
{
    float rate_rad_s;   // 当前参考速率
    float accel_rad_s2; // 上一拍算出的参考加速度(每拍刷新)
    bool primed;        // 是否已完成首拍对齐
} feedforward_ref_t;

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
 * @brief 用给定速率播种参考跟踪器(上电/切模式时用实测速率播种, 避免首推前馈冲击)
 */
void feedforward_ref_reset(feedforward_ref_t *ref, const float &rate_rad_s);

/**
 * @brief 限斜率跟踪目标速率, 并差分出参考加速度
 * @param ref                 跟踪器
 * @param target_rate_rad_s   目标速率(通常是速度环给定)
 * @param dt_s                本次调用间隔(s); 非有限、<=0 或 >20ms 视为调度异常, 本拍不产生加速度
 * @param max_accel_rad_s2    参考速率的最大变化率(rad/s^2); <=0 表示不使用加速度前馈(参考直接跟随, accel=0); 非有限时保持参考速率, accel=0
 * @note 结果写在 ref->rate_rad_s / ref->accel_rad_s2, 只受 max_accel 限制, 不受电机力矩限幅影响。
 *       加速度由实际可表示的速率增量差分并限幅; 步长小于浮点精度时速率保持, accel=0。
 */
void feedforward_ref_track(feedforward_ref_t *ref, const float &target_rate_rad_s, const float &dt_s, const float &max_accel_rad_s2);

/**
 * @brief 计算单轴力矩前馈
 * @param param            前馈参数
 * @param angle_rad        实测关节角(重力项的相位)
 * @param rate_rad_s       参考角速度(应来自 feedforward_ref_track)
 * @param accel_rad_s2     参考角加速度(应来自 feedforward_ref_track)
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
