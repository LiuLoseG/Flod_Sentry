#pragma once

/**
 * @file    imu_odometer.h
 * @brief   卡尔曼融合里程计 — IMU预测 + 编码器修正 + 自适应打滑检测
 *
 * 4状态卡尔曼滤波器 (仅测量位置, 速度由卡尔曼估计):
 *   状态: [pos_x, vel_x, pos_y, vel_y]
 *   输入: [acc_world_x, acc_world_y]  (IMU加速度, 世界系)
 *   测量: [pos_enc_x, pos_enc_y]      (编码器位置, 不测速度!)
 *
 * 为什么只测位置?
 *   - 编码器位置精度高 (累计轮转角→位移, 误差<1mm)
 *   - 编码器速度精度低 (帧间差分, 噪声大)
 *   - 卡尔曼从位置观测+IMU加速度自动最优估计速度
 *   - 这才是真正的融合: 速度 = f(编码器位置变化, IMU加速度积分)
 *
 * 过程模型:
 *   pos_k = pos_{k-1} + vel_{k-1}*dt + ½*acc*dt²
 *   vel_k = vel_{k-1} + acc*dt
 *
 * 为什么用卡尔曼而不是简单互补滤波?
 *   - 卡尔曼增益根据噪声协方差自动调整 IMU vs 编码器 的信任度
 *   - 编码器准确时 → 测量噪声R小 → 卡尔曼增益大 → 信任编码器
 *   - 轮子打滑时 → 新息(测量-预测)超阈值 → 自适应放大R → 信任IMU
 *   - IMU漂移时 → 过程噪声Q累积 → 协方差P增大 → 卡尔曼增益大 → 信任编码器
 *   - 不需要手动调α, 卡尔曼自动找到最优权重
 *
 * 坐标系: 上电位置=(0,0), 上电朝向=前(pos_x+), 左移→pos_y+
 *===========================================================================*/

#include "hi12_imu.h"
#include "wheel_system_solution.h"
#include "kalman_new.h"
#include "math_matrix.h"
#include "basic_math.h"
#include <cmath>

typedef struct 
{
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
    float yaw;
    float distance;
} fusion_state_t;

class fusion_odometer_class
{
public:
    fusion_state_t state;

    void init(void);
    void reset(void);

    /**
     * @brief 卡尔曼融合更新
     * @param enc_pos   编码器里程计位姿 (世界系, 已用IMU yaw旋转)
     * @param body_speed 机体速度 (vx,vy: m/s, omega: rad/s)
     * @param imu       HI12 IMU (提供yaw和acc_x)
     * @param dt        时间步长 (s)
     */
    void update(wheel_robot_position_t *enc_pos,
                wheel_robot_speed_t   *body_speed,
                volatile hi12_gyro    *imu,
                float                  dt);
    float get_distance(void) const { return state.distance; }

private:
    static constexpr uint32_t N = 4; // state: px,vx,py,vy
    static constexpr uint32_t M = 2; // input: acc_wx, acc_wy
    static constexpr uint32_t K = 2; // meas:  enc_px, enc_py (仅位置!)

    class_filter_kalman<N, M, K> kf;
    class_matrix_f32<N, N> mat_a, mat_q;
    class_matrix_f32<N, M> mat_b;
    class_matrix_f32<K, N> mat_h;
    class_matrix_f32<K, K> mat_r_base;

    float yaw_offs;
    bool  yaw_offs_ok;

    void build_matrices(float dt);
    void check_slip(const class_matrix_f32<K,1> &innov);
};


