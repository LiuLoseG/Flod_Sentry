#pragma once

#include "arm_math.h"
#include "basic_math.h"
#include "rls.h"
#include "stdbool.h"

/**
 * @brief RLS enable status
 */
typedef enum
{
    Disable = 0,
    Enable = 1,
} rls_enable_e;

/**
 * @brief Error flags
 */
typedef enum
{
    Motor_Disconnected = 1U,
    Referee_Disconnected = 2U,
    Cap_Disconnected = 4U,
} error_flags_e;

typedef struct
{
    float now_vel;
    float now_cur;
    float cmd_vel;
    float cmd_cur;
    bool alive_status;  // 电机存活状态, true表示存活, false表示掉线
} power_configuration_value;

class chassis_power_rls
{

public:

    void set_chassis_power_max(const float &__chassis_power_max)
    {
        chassis_power_max = __chassis_power_max;
    }

    void init(const rls_enable_e &__rls_enabled = Enable, const float &__k1 = 0.0000443f, const float &__k2 = 0.00000009976f, const float &__k3 = 1.315f, const float &__lambda = 0.99999f);
    
    inline float get_motor_rotor_out(const uint8_t &__index) const;

    void set_error_flags();

    // 返回错误标志位(0 = 无异常)。改为返回值形式，调用处直接用 get_error_flags() != 0U 判断
    uint8_t get_error_flags() const
    {
        return error_flags;
    }

    void update_measure(const float (&__now_vel)[4], const float (&__now_cur)[4], const float (&__cmd_vel)[4], const float (&__cmd_cur)[4], const bool (&__alive_status)[4]);

    void calculate_callback();

    // 传入的是 volatile 功率计数据, 且函数内只读取, 故按值传递(引用无法绑定 volatile)
    void rls_calculate_callback(float __measured_power);

protected:

    // 功率配置值
    power_configuration_value cfg_val[4] = {};

    // 纯"误差分配" 置信度上阈值 K_coe=1
    float error_power_distribution = 20.0f;
    // 等比缩放置信度下阈值 K_coe=0
    float prop_power_distribution  = 15.0f;
    // 电机转矩常数
    float Rotor_Torque = 0.3f / (3591.0f / 187.0f) * (20.0f / 16384.0f);

    uint8_t error_flags; // Error flags

    // RLS 自适应参数使能
    rls_enable_e rls_enabled;

    // 功率计算公式: P_i = τ_i·ω_i + k1·ω_i² + k2·τ_i² + k3/n
    float k1 = 0.0000308553062f;
    float k2 = 0.0000013128074f;
    float k3 = 5.26f;

    // k2 的扭矩域换算值(= k2 / Rotor_Torque²), 在 init() 中计算
    float k2_torque = 0.0f;
    // 单个电机的静态损耗(c = k3/n, n=4), 在 init() 中计算
    float k3_per_motor = 0.0f;

    // RLS 待辨识参数估计器
    Core::Control::Math::RLS<2> rls{1e-5f, 0.99999f};

    // RLS 的样本向量/辨识结果
    Matrixf<2, 1> samples;
    Matrixf<2, 1> params;
    
    // 有效功率(正功率)总和, W
    float effective_power = 0.0f;

    // 功率缩放系数(0~1)
    float power_scale = 1.0f;

    // 测量功率
    float measured_power;
    // 估计功率
    float estimated_power;

    // 总功率分配池
    float allo_catable_power = 0.0f;

    // 裁判系统认定的底盘最大功率值(功率上限), W
    float power_limit_max = 10.0f;
    // 底盘功率最大值(本次实际允许的上限, 综合考虑缓冲能量后的动态值), W
    float chassis_power_max = 10.0f;
    
    // 所需功率总和，只计算正功率部分，负功率不计入总和，W
    float sum_power_required = 0.0f;

    // 初始未分配的功率(控制器计算的功率值), W
    float cmd_power[4] = {0.0f};
    // 初始未分配的总功率(控制器计算的功率值), W
    float cmd_total_power = 0.0f;
    
    // 当前电机拟合实际功率值
    float now_power[4] = {0.0f};
    // 当前底盘总功率值(实际测量的功率值), W
    float now_total_power = 0.0f;

    float omega_error[4] = {0.0f};
    float omega_total_error = 0.0f;

    // 功率分配后的输出值
    float power_distribution_out[4] = {0.0f};

    // 4 个轮电机是否全部在线(仅声明, 定义见本文件末尾)
    inline bool is_all_motor_alive() const;
}; 

/**
 * @brief 获取当前电机应该输出的控制量(电流/扭矩)
 * @param __index 
 * @retval float: 
 */
inline float chassis_power_rls::get_motor_rotor_out(const uint8_t &__index) const
{
    if (__index >= 4)
    {
        return (0.0f);
    }

    return (power_distribution_out[__index]);
}

inline bool chassis_power_rls::is_all_motor_alive() const
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (!cfg_val[i].alive_status)
        {
            return false;
        }
    }

    return true;
}
