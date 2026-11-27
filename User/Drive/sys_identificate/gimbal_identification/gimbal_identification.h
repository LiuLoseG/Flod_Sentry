/**
 * @file gimbal_identification.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-10-19
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#ifndef GIMBAL_IDENTIFICATION_H
#define GIMBAL_IDENTIFICATION_H

#include "rls_last.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Online integral-regression bridge. One instance per fixed axis/configuration.
 * Pure C99; no allocation, HAL calls, motor commands or SMC configuration edits.
 * The application must synchronize each complete sample before calling update.
 */
typedef enum {
    GIMBAL_IDENTIFICATION_YAW = 0,
    GIMBAL_IDENTIFICATION_PITCH = 1,
    GIMBAL_IDENTIFICATION_BIG_PITCH = 2
} gimbal_identification_axis_t;

#define GIMBAL_IDENTIFICATION_RATE_HISTORY 32u

typedef struct {
    gimbal_identification_axis_t axis;
    float window_s;     // 单个积分窗口的持续时间，单位为秒， config->window_s
    float max_gap_s;    // 相邻两个有效采样点之间允许的最大时间间隔，单位是秒
    float feedback_timeout_s; // 反馈超时时间，单位是秒
    float friction_velocity_rad_s; /* Fixed positive epsilon in tanh(rate/eps). */ // 摩擦速度阈值，单位为弧度每秒，用于计算摩擦力矩
    uint32_t configuration_id;  // 仅用于调试，标识当前配置的唯一 ID
    /* 饱和样本是否仍参与回归。
     * 指令达到限幅时，反馈回来的力矩就是限幅后的真实值，回归目标依然成立，
     * 因此这些样本本身是可用的（只需把 SATURATED 标志上报给上层）。
     * false(默认) = 保持保守行为: 一帧饱和即丢弃整个积分窗口并清空 RLS 激励历史；
     * true         = 只上报 SATURATED 标志，数据照常参与回归。
     * 注意: 若上层在饱和时会把力矩指令改写成非物理值(如额外限幅/斜坡)，则应保持 false。
     */
    bool accept_saturated;
    /* 是否用 angle_rad 的历史做二次最小二乘拟合，在最新采样时刻重构角速度。
     * 角度反馈是 16 bit (LSB≈9.6e-5 rad)，而电机速度报文只有 12 bit
     * (LSB≈1.5e-2 rad/s)。二次拟合能消除匀加速运动下线性斜率的半窗时延，
     * 但端点导数比中心斜率更敏感于噪声，需用实测数据验证窗口长度。
     * 打开后重构速度用于库仑摩擦积分；惯量特征始终使用原始速度端点，避免平滑衰减。
     * 历史未满时摩擦积分使用原始速度；历史已满但重构失败时上报 NUMERIC_FAULT 并清观测。
     * 非匀加速运动仍有拟合误差；本功能不保证任何频段均无相位误差。
     * 需要角度是【连续、无跳变】的关节坐标(与 angle_rad 的要求一致)。
     */
    bool derive_rate_from_angle;
    /* 重构窗口采样数，3..GIMBAL_IDENTIFICATION_RATE_HISTORY，默认 20 (=20ms@1kHz)。
     * 窗口越长越平滑但高阶运动拟合偏差越大。derive_rate_from_angle=false 时被忽略。
     */
    uint16_t rate_window_samples;
    /* 逐参数的归一化相对不确定度上限 = sqrt(P_ii)/|theta_i| (无量纲)。
     * 必须有限且非负；0 表示不检查(默认，保持既有行为)。
     * P 使用归一化测量方差 R=1，未校准实际噪声时该量不是统计置信区间。
     * 小摆幅/病态激励下这个值会很大，用它把"看起来能算但结果不可信"的工况拦下去。
     */
    float max_relative_uncertainty;
    /* Parameter order: J, B, Fc for yaw; J, B, Fc, A, C for pitch/big pitch.
     * Required bounds: minimum[J] > 0; minimum[B/Fc] >= 0.
     * A/C are signed. Bounds and initial values belong to this configuration.
     * RLS dt limits must cover window_s through window_s + max_gap_s.
     */
    rls_last_config_t rls;
} gimbal_identification_config_t;

typedef struct {
    /* Acquisition time of a NEW coherent sample, not the time cached data is
     * read. Unsigned wrap is supported; ordering is unambiguous only when
     * successive timestamps are separated by less than 2^31 microseconds.
     */
    uint32_t timestamp_us;
    uint32_t segment_id;
    uint32_t configuration_id;
    /* Continuous joint coordinate, no shortest wrap. For long-running yaw,
     * rebase before float resolution is lost and START A NEW segment_id.
     * A coordinate shift inside an integral window corrupts delta-angle.
     */
    float angle_rad;
    float rate_rad_s;         /* Derivative of angle in the SAME fixed frame. */
    float gravity_angle_rad;  /* Fixed-plane gravity phase; finite even for yaw. */
    float torque_nm;          /* Time-aligned actual/qualified joint torque. */
    float feedback_age_s;     /* Conservative age of oldest required feedback. */
    bool enabled;
    bool valid;
    bool saturated;
    /* Caller assertions, NOT automatically inferred from the measurements.
     * operating_conditions_valid means fixed base/configuration and the
     * selected model applies. Torque calibration and alignment must already
     * be independently established; low residuals cannot establish either.
     */
    bool operating_conditions_valid;
    bool torque_calibrated;
    bool torque_time_aligned;
} gimbal_identification_sample_t;

typedef struct {
    float inertia_kg_m2;
    float viscous_nm_s_rad;
    float coulomb_nm;
    float friction_velocity_rad_s;
    float gravity_sin_nm;     /* Always zero for the yaw model. */
    float gravity_cos_nm;     /* Always zero for the yaw model. */
} gimbal_identification_model_t;

enum {
    GIMBAL_IDENTIFICATION_DISABLED               = 1u << 0,
    GIMBAL_IDENTIFICATION_BAD_INPUT              = 1u << 1,
    GIMBAL_IDENTIFICATION_SATURATED              = 1u << 2,
    GIMBAL_IDENTIFICATION_STALE_FEEDBACK         = 1u << 3,
    GIMBAL_IDENTIFICATION_BAD_TIMESTAMP          = 1u << 4,
    GIMBAL_IDENTIFICATION_GAP                    = 1u << 5,
    GIMBAL_IDENTIFICATION_SEGMENT_CHANGE         = 1u << 6,
    GIMBAL_IDENTIFICATION_CONFIGURATION_MISMATCH = 1u << 7,
    GIMBAL_IDENTIFICATION_UNQUALIFIED_OPERATION  = 1u << 8,
    GIMBAL_IDENTIFICATION_TORQUE_UNCALIBRATED     = 1u << 9,
    GIMBAL_IDENTIFICATION_TORQUE_UNALIGNED       = 1u << 10,
    GIMBAL_IDENTIFICATION_NUMERIC_FAULT          = 1u << 11,
    GIMBAL_IDENTIFICATION_BAD_CONFIG             = 1u << 12,
    GIMBAL_IDENTIFICATION_RLS_REJECTED            = 1u << 13
};

typedef struct {
    gimbal_identification_model_t model;
    rls_last_output_t rls;
    /* 各参数归一化信息指标 = sqrt(P_ii)/|theta_i|，不是经噪声校准的置信区间。
     * 参数为零、状态无效或协方差无效时取 FLT_MAX；比值溢出时可为无穷大。
     * 注意它的含义: forgetting_time_s = 0(默认)时 P 随更新次数单调收缩, 所以它度量的是
     * "累计了多少信息", 不是"当前噪声有多大"——同一段数据把力矩噪声放大 10 倍,
     * 它几乎不变(噪声水平要看 rls.innovation_rms)。
     * 判读方式: 短时间(几秒)内不再下降 => 激励不足, 该段数据对参数没有新增信息;
     * 到位判据建议用绝对值门槛(大 pitch 取 2 以下), 而不是单纯看 accepted_updates。
     */
    float relative_uncertainty[RLS_LAST_MAX_PARAMETERS];
    /* 本次窗口估计中"看起来能算但不可信"的标记 */
    bool uncertainty_exceeded;
    uint32_t flags; /* Wrapper namespace, independent of rls.flags. */
    float window_duration_s; /* Nonzero only when a full window was submitted. */
    /* Both are per-call events, never stale readiness from an earlier window.
     * window_updated means RLS accepted this newly completed window.
     * ready additionally requires this update to satisfy the RLS readiness
     * gates. Neither flag authorizes applying the estimate to a controller.
     */
    bool window_updated;
    bool ready;
} gimbal_identification_output_t;

/* Storage is exposed only for static allocation. Treat every member as private;
 * changing model/configuration/clock state requires the lifecycle APIs below.
 */
typedef struct {
    gimbal_identification_config_t config;
    rls_last_t estimator;
    rls_last_output_t diagnostics;
    gimbal_identification_sample_t previous;
    float start_angle_rad;
    float start_rate_rad_s;
    float integral_torque;
    float integral_friction;
    float integral_gravity_sin;
    float integral_gravity_cos;
    uint32_t window_start_us;
    uint32_t last_timestamp_us;
    uint32_t window_us;
    uint32_t max_gap_us;
    /* 角度历史环，仅用于 derive_rate_from_angle */
    float    rate_history_angle[GIMBAL_IDENTIFICATION_RATE_HISTORY];
    uint32_t rate_history_us[GIMBAL_IDENTIFICATION_RATE_HISTORY];
    uint8_t  rate_history_count;
    uint8_t  rate_history_next;
    float    derived_rate_rad_s;
    bool     derived_rate_valid;
    bool initialized;
    bool have_timestamp;
    bool window_active;
} gimbal_identification_t;

/* Defaults are synthetic starting values, NOT calibrated hardware parameters. */
void gimbal_identification_default_config(gimbal_identification_config_t *config,
                                          gimbal_identification_axis_t axis);
bool gimbal_identification_init(gimbal_identification_t *identification,
                                const gimbal_identification_config_t *config);
/* Restore initial parameters/P and clear counts, observations and timestamps. */
void gimbal_identification_reset(gimbal_identification_t *identification);
/* Keep learned parameters/P and total accepted-update count; clear readiness,
 * excitation/innovation histories, partial integral and timestamp continuity.
 */
void gimbal_identification_clear_observations(gimbal_identification_t *identification);
/* Samples must arrive once in acquisition order. Invalid/disabled/saturated/
 * stale/unqualified samples and all temporal/segment/configuration breaks drop
 * partial integration and clear RLS observation readiness while retaining the
 * learned parameters/P. accept_saturated allows saturated samples to proceed.
 * Gaps/segment changes retain RLS histories and may anchor a fresh valid sample;
 * they never integrate across the break. Wrong configuration_id is rejected;
 * use its own instance or explicitly reinitialize for the new configuration.
 * Complete adjacent windows share an endpoint, never a time interval.
 */
uint32_t gimbal_identification_update(gimbal_identification_t *identification,
                                      const gimbal_identification_sample_t *sample,
                                      gimbal_identification_output_t *output);

#ifdef __cplusplus
}
#endif
#endif
