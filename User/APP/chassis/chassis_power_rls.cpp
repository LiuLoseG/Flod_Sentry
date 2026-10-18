/**
 * @file chassis_power_rls.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-09-29
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "chassis_power_rls.h"
#include "freertos.h"
#include <cmath>

/**
 * @brief Construct a new chassis power rls::chassis power rls object
 * @param rls_enabled_ 
 * @param k1_ 
 * @param k2_ 
 * @param k3_ 
 * @param lambda_ 
 */
void chassis_power_rls::init(const rls_enable_e &__rls_enabled, const float &__k1, const float &__k2, const float &__k3, const float &__lambda)
{
    error_flags = 0U;
    effective_power = 0.0f;
    rls_enabled = __rls_enabled;
    k1 = __k1;
    k2 = __k2;
    k3 = __k3;

    configASSERT(__k1 >= 0);
    configASSERT(__k2 >= 0);
    configASSERT(__k3 >= 0);

    // 域换算: 刻度域 k2 → 扭矩域 k2(供 README 的二次方程 A 项使用)
    k2_torque    = k2 / (Rotor_Torque * Rotor_Torque);
    k3_per_motor = k3 * 0.25f;

    float initParams[2] = {__k1, __k2};
    rls.setParamVector(Matrixf<2, 1>(initParams));
}

/**
 * @brief 
 * @brief Update the measurement values
 * @param now_vel Current velocity of each motor
 * @param now_cur Current current of each motor
 * @param cmd_vel Commanded velocity of each motor
 * @param cmd_cur Commanded current of each motor
 * @param alive_status Alive status of each motor
 */
void chassis_power_rls::update_measure(const float (&__now_vel)[4], const float (&__now_cur)[4], const float (&__cmd_vel)[4], const float (&__cmd_cur)[4], const bool (&__alive_status)[4])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        cfg_val[i].now_vel = __now_vel[i];
        cfg_val[i].now_cur = __now_cur[i];
        cfg_val[i].cmd_vel = __cmd_vel[i];
        cfg_val[i].cmd_cur = __cmd_cur[i];
        cfg_val[i].alive_status = __alive_status[i];
    }

    now_total_power = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (cfg_val[i].alive_status)   // 电机在线参与功率计算
        {
            // 功率模型: P_i = τ_i·ω_i + k1·ω_i² + k2·τ_i² + k3/4
            float torque = Rotor_Torque * cfg_val[i].now_cur;

            now_power[i] = torque * cfg_val[i].now_vel + 
                        k1 * (cfg_val[i].now_vel * cfg_val[i].now_vel) + 
                        k2_torque * (torque * torque) + 
                        k3_per_motor;

            if (now_power[i] < 0.0f)
            {
                    continue;
            }

            now_total_power += now_power[i];
        }

        else    // 电机掉线不参与功率计算, 直接置零
        {
            now_power[i] = 0.0f;
        }
    }
}

/**
 * @brief 底盘功率计算回调函数
 * @note 实时计算和控制底盘功率分配量
 */
void chassis_power_rls::calculate_callback()
{
    // 每次计算前清零
    cmd_total_power = 0.0f;
    omega_total_error = 0.0f;
    sum_power_required = 0.0f;

    // 允许分配的功率池(不超过最大功率)
    allo_catable_power = chassis_power_max;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (cfg_val[i].alive_status)   // 电机在线参与功率计算
        {
            float torque = Rotor_Torque * cfg_val[i].cmd_cur;

            // 功率计算公式: P_i = τ_i·ω_i + k1·ω_i² + k2·τ_i² + k3/4
            cmd_power[i] = torque * cfg_val[i].now_vel + 
                                    k1 * (cfg_val[i].now_vel * cfg_val[i].now_vel) + 
                                    k2_torque * (torque * torque) + 
                                    k3_per_motor;

            cmd_total_power += cmd_power[i];

            // 电机转速误差计算
            omega_error[i] = fabsf(cfg_val[i].cmd_vel - cfg_val[i].now_vel);

            if (cmd_power[i] > 0.0f)
            {
                omega_total_error += omega_error[i];
                sum_power_required += cmd_power[i];
            }
        }

        else    // 电机掉线不参与功率计算, 直接置零
        {
            cmd_power[i] = 0.0f;
            omega_error[i] = 0.0f;
        }
    }

    // 后续添加功率快照

    if(cmd_total_power > chassis_power_max && sum_power_required > 1e-6f)
    {
        // 功率分配置信度 K_coe(README): Σerror 在 [E_lower, E_upper] 内线性插值
        if(omega_total_error > error_power_distribution)
        {
            power_scale = 1.0f;
        }

        else if (omega_total_error > prop_power_distribution)
        {
            power_scale = basic_math_constrain((omega_total_error - prop_power_distribution) / (error_power_distribution - prop_power_distribution), 0.0f, 1.0f);
        }

        else
        {
            power_scale = 0.0f;
        }

        // 集合级分母(先判零再除, 避免 inf/NaN 注入控制量)
        const float err_den = omega_total_error;
        const float pwr_den = sum_power_required;

        // 力矩/电流分配
        for (uint8_t i = 0; i < 4; i++)
        {
            float out = 0.0f;

            if (cfg_val[i].alive_status)
            {
                const float cmd_cur = cfg_val[i].cmd_cur;
                const float omega   = cfg_val[i].now_vel;

                // 负功率(制动回馈)/零指令轮不参与分配, 原指令直通
                if (cmd_power[i] <= 0.0f)
                {
                    power_distribution_out[i] = cmd_cur;
                    continue;
                }

                // 角速度误差占比 / 需求功率占比(权重和 = 1 ⇒ Σ分配功率 = 功率池)
                float power_weight_error = (err_den > 1e-6f) ? (omega_error[i] / err_den) : 0.0f;
                float power_weight_prop = cmd_power[i] / pwr_den;
                float power_weight = power_scale * power_weight_error + (1.0f - power_scale) * power_weight_prop;

                // 该轮分配到的功率
                float alloc_power = power_weight * allo_catable_power;

                const float A = k2_torque;
                const float B = omega;
                const float C = k1 * omega * omega + k3_per_motor - alloc_power;

                float delta = B * B - 4.0f * A * C;
                float torque_out;

                if (delta > 0.0f)   // Δ > 0: 两个实根, 按原指令符号取
                {
                    float sqrt_delta = sqrtf(delta);
                    torque_out = (cmd_cur > 0.0f) ? ((-B + sqrt_delta) / (2.0f * A))
                                                : ((-B - sqrt_delta) / (2.0f * A));
                }

                else                // Δ ≤ 0: 无实根, 取抛物线顶点(可达最小功率点)
                {
                    torque_out = -B / (2.0f * A);
                }

                // Nm → 原始 CAN 刻度(与 cmd_cur 同单位)
                out = torque_out / Rotor_Torque;

                // NaN 参与的任何比较都是 false
                if (out != out)
                {
                    out = cmd_cur;
                }

                else if (cmd_cur >= 0.0f)
                {
                    out = basic_math_constrain(out, 0.0f, cmd_cur);
                }

                else
                {
                    out = basic_math_constrain(out, cmd_cur, 0.0f);
                }
            }

            else // 掉线电机不参与功率分配, 直接置零
            {
                out = 0.0f;  
            }

            power_distribution_out[i] = out;
        }
    }

    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            power_distribution_out[i] = cfg_val[i].cmd_cur;
        }
    }
}

/**
 * @brief 
 */
void chassis_power_rls::set_error_flags()
{
    // 电机离线检测
    if(!is_all_motor_alive()) // 任意电机掉线
    {
        error_flags |= Motor_Disconnected;
    }

    else
    {
        error_flags &= ~Motor_Disconnected;
    }

    // 裁判系统离线检测

    // 功率计/超级电容离线检测
}

/**
 * @brief RLS 计算回调函数
 */
void chassis_power_rls::rls_calculate_callback(float __measured_power)
{
    effective_power = 0;
    samples[0][0]  = 0;
    samples[1][0]  = 0;
    measured_power = __measured_power;

    if (get_error_flags() != 0U) // 任意异常标志置位, 则不进行 RLS 计算
    {
        return;
    }

    for(uint8_t i = 0; i < 4; i++)
    {
        effective_power += (cfg_val[i].now_cur * Rotor_Torque) * cfg_val[i].now_vel;
        samples[0][0] += cfg_val[i].now_vel * cfg_val[i].now_vel;
        samples[1][0] += cfg_val[i].now_cur * cfg_val[i].now_cur;
    }
    
    estimated_power = k1 * samples[0][0] + k2 * samples[1][0] + effective_power + k3;

    params = rls.update(samples, __measured_power - effective_power - k3);
    k1 = fmax(params[0][0], 1e-9f);
    k2 = fmax(params[1][0], 1e-9f);
}

