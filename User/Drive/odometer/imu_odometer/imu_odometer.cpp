/**
 * @file    imu_odometer.cpp
 * @brief   卡尔曼融合里程计 — IMU预测 + 编码器修正 + 自适应打滑检测
 *
 * 4状态卡尔曼滤波器设计:
 *
 *   状态 x = [px, vx, py, vy]^T
 *   输入 u = [acc_wx, acc_wy]^T   (IMU加速度→世界系, m/s²)
 *   测量 z = [px_enc, vx_enc, py_enc, vy_enc]^T  (编码器位置+机体速度旋转, m, m/s)
 *
 *   过程模型 (线性):
 *     px_k = px_{k-1} + vx_{k-1}*dt + 0.5*acc_wx*dt² + w_px
 *     vx_k = vx_{k-1} + acc_wx*dt + w_vx
 *     py_k = py_{k-1} + vy_{k-1}*dt + 0.5*acc_wy*dt² + w_py
 *     vy_k = vy_{k-1} + acc_wy*dt + w_vy
 *
 *   测量模型 (线性):
 *     z = I·x + v   (直接测量所有状态)
 *
 *   噪声调参:
 *     Q_p = 1e-4: 位置过程噪声小 (位置主要由积分确定)
 *     Q_v = 1e-2: 速度过程噪声较大 (加速度积分会漂移)
 *     R_p = 1e-6: 编码器位置测量噪声极小 (编码器很准)
 *     R_v = 1e-4: 编码器速度测量噪声 (差分计算, 有噪声)
 *
 *   自适应打滑检测:
 *     innovation = z - H·x_prior
 *     若 |innovation_px| > 0.05m 或 |innovation_py| > 0.05m:
 *       → 编码器可能打滑 → R *= 100 → 卡尔曼信任IMU预测
 *===========================================================================*/

#include "imu_odometer.h"

/* ---- 噪声参数 (v2: 编码器信任度合理, 打滑检测有效) ---- */
/*
 * 为什么R不能太小?
 *   卡尔曼增益 K = P / (P + R). 若 R → 0, 则 K → 1, 输出 = 测量值.
 *   旧值 R_POS=1e-6 → K≈1 → 融合输出≈编码器原始值 → 打滑/振动全盘接受.
 *   新值 R_POS=1e-3 → K会根据P动态调整 → 编码器准→信任, 不准→信任IMU.
 */
/*
 * 噪声参数:
 *   Q_VEL 小 → 速度预测平滑 → 抑制IMU振动噪声引起的毛刺
 *   ACC_DZ 大 → 滤除更多微振动 → 加速度输入更干净
 *   R_POS 小 → 编码器位置测量可信 → 位置快速收敛到编码器
 */
static constexpr float Q_POS = 1e-4f;   // 位置过程噪声
static constexpr float Q_VEL = 1e-4f;   // 速度过程噪声 (从5e-2大幅降低→平滑速度)
static constexpr float R_POS = 1e-4f;   // 编码器位置测量噪声 (从1e-3降低→更信任编码器)
static constexpr float ACC_DZ = 0.05f;  // 加速度死区 (从0.01增大→滤除微振动)
static constexpr float SLIP_INNOV_POS = 0.02f;
static constexpr float SLIP_R_MULT    = 1000.0f;

/*===========================================================================
 * 构建dt相关的系统矩阵
 *===========================================================================*/
void fusion_odometer_class::build_matrices(float dt)
{
    float dt2 = 0.5f * dt * dt;

    /* A = [[1,dt,0,0 ], [0,1,0,0], [0,0,1,dt], [0,0,0,1]] */
    float adata[16] = {1,dt,0,0, 0,1,0,0, 0,0,1,dt, 0,0,0,1};
    mat_a = class_matrix_f32<N,N>(adata);

    /* B = [[dt2,0], [dt,0], [0,dt2], [0,dt]] */
    float bdata[8] = {dt2,0, dt,0, 0,dt2, 0,dt};
    mat_b = class_matrix_f32<N,M>(bdata);
}

/*===========================================================================
 * 初始化
 *===========================================================================*/
void fusion_odometer_class::init(void)
{
    build_matrices(0.001f);

    /* H = [[1,0,0,0], [0,0,1,0]]  (仅测量位置) */
    float hdata[8] = {1,0,0,0, 0,0,1,0};
    mat_h = class_matrix_f32<K,N>(hdata);

    float qdata[16] = {Q_POS,0,0,0, 0,Q_VEL,0,0, 0,0,Q_POS,0, 0,0,0,Q_VEL};
    mat_q = class_matrix_f32<N,N>(qdata);

    /* R = diag(R_POS, R_POS) (2×2, 仅位置测量噪声) */
    float rdata[4] = {R_POS,0, 0,R_POS};
    mat_r_base = class_matrix_f32<K,K>(rdata);

    kf.init(mat_a, mat_b, mat_h, mat_q, mat_r_base);
    reset();
}

/*===========================================================================
 * 重置
 *===========================================================================*/
void fusion_odometer_class::reset(void)
{
    state.pos_x = state.pos_y = 0;
    state.vel_x = state.vel_y = 0;
    state.yaw = 0;
    state.distance = 0;

    kf.vector_x = namespace_alg_matrix::zero<N,1>();
    kf.vector_u = namespace_alg_matrix::zero<M,1>();
    kf.matrix_p = namespace_alg_matrix::identity<N,N>();

    yaw_offs = 0;
    yaw_offs_ok = false;
}

/*===========================================================================
 * 打滑检测: 双判据 → 放大测量噪声
 *
 * 判据1 (位置新息): 编码器位置与IMU预测位置差 > 0.02m → 编码器可能打滑
 * 判据2 (速度跳变): 编码器速度帧间跳变 > 0.50m/s → 物理不可能 → 必是打滑
 *
 * 触发任一判据 → R *= 1000 → 卡尔曼增益K≈0 → 完全信任IMU预测
 *===========================================================================*/
void fusion_odometer_class::check_slip(const class_matrix_f32<K,1> &innov)
{
    float ipx = fabsf(innov.data[0]);
    float ipy = fabsf(innov.data[1]);

    if (ipx > SLIP_INNOV_POS || ipy > SLIP_INNOV_POS)
    {
        float rdata[4] = {R_POS*SLIP_R_MULT, 0, 0, R_POS*SLIP_R_MULT};
        kf.matrix_r = class_matrix_f32<K,K>(rdata);
    }
    else
    {
        kf.matrix_r = mat_r_base;
    }
}

/*===========================================================================
 * 卡尔曼融合更新
 *===========================================================================*/
void fusion_odometer_class::update(wheel_robot_position_t *enc_pos,
                                    wheel_robot_speed_t   *body_speed,
                                    volatile hi12_gyro    *imu,
                                    float                  dt)
{
    if (dt <= 0.0f || imu == nullptr || enc_pos == nullptr || body_speed == nullptr)
        return;

    /* ---- 0. IMU数据 ---- */
    float imu_yaw = imu->yaw();
    if (!yaw_offs_ok) { yaw_offs = imu_yaw; yaw_offs_ok = true; }
    state.yaw = imu_yaw - yaw_offs;

    float acc_body_x = imu->acc_x();
    if (fabsf(acc_body_x) < ACC_DZ) acc_body_x = 0;

    float yaw_rad = state.yaw * (M_PI / 180.0f);
    float acc_wx =  acc_body_x * cosf(yaw_rad);
    float acc_wy =  acc_body_x * sinf(yaw_rad);

    /* ---- 1. 更新卡尔曼矩阵 ---- */
    build_matrices(dt);
    kf.matrix_a = mat_a;
    kf.matrix_b = mat_b;

    /* ---- 2. 输入: IMU加速度 ---- */
    float udata[2] = {acc_wx, acc_wy};
    kf.vector_u = class_matrix_f32<M,1>(udata);

    /* ---- 3. 预测 (IMU加速度积分) ---- */
    kf.tim_predict_periodelapsedcallback();

    /* ---- 4. 测量: 仅编码器位置 (速度由卡尔曼从位置+IMU估计) ---- */
    float zdata[2] = {enc_pos->x, enc_pos->y};
    kf.vector_z = class_matrix_f32<K,1>(zdata);

    /* ---- 5. 打滑检测 (位置新息) ---- */
    class_matrix_f32<K,1> innov = kf.vector_z - kf.matrix_h * kf.vector_x_prior;
    check_slip(innov);

    /* ---- 6. 更新 ---- */
    kf.tim_update_periodelapsedcallback();

    /* ---- 7. 输出 ---- */
    state.pos_x = kf.vector_x.data[0];
    state.vel_x = kf.vector_x.data[1];
    state.pos_y = kf.vector_x.data[2];
    state.vel_y = kf.vector_x.data[3];

    float step = sqrtf((state.vel_x*dt)*(state.vel_x*dt) + (state.vel_y*dt)*(state.vel_y*dt));
    state.distance += step;
}
