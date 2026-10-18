#pragma once

#include "bsp_pwm.h"
#include "stm32H7xx_hal.h"
#include <cmath>
#include <cstdint>
#include "user_lib.h"
#include "pid.h"

/**
 * @brief TB6612 电机驱动器控制模式
 */
typedef enum
{
    Motor_TB6612_Drive_Method_Current = 0,   // 开环控制模式 (PWM 占空比直接控制)
    Motor_TB6612_Drive_Method_Omega = 1,     // 角速度闭环模式
    Motor_TB6612_Drive_Method_Angle = 2,     // 角度闭环模式
} tb6612_drive_mode_e;

/**
 * @brief TB6612 驱动 GPIO 配置结构体
 * @note IN1/IN2 控制方向: IN1=1,IN2=0 → 正转; IN1=0,IN2=1 → 反转; IN1=0,IN2=0 → 刹车
 */
typedef struct
{
    GPIO_TypeDef *in1_port;     // IN1 引脚 GPIO 端口
    uint16_t      in1_pin;      // IN1 引脚编号

    GPIO_TypeDef *in2_port;     // IN2 引脚 GPIO 端口
    uint16_t      in2_pin;      // IN2 引脚编号
} tb6612_link_gpio_t;

/**
 * @brief TB6612 电机接收数据结构体
 */
typedef struct
{
    // 输出轴状态
    float now_angle;              // 输出轴当前总角度, rad
    float now_omega;              // 输出轴当前角速度, rad/s
    int32_t now_round;            // 总圈数

    float rotor_angle;            // 电机转子当前总角度, rad, (rotor_angle / gearbox_rate) = now_angle
    float rotor_omega;            // 电机转子当前角速度, rad/s, (rotor_omega / gearbox_rate) = now_omega

    int32_t total_encoder;        // 总编码器值
    int32_t last_total_encoder;   // 上次总编码器值

    int32_t tim_encoder_round;    // 定时器编码器轮数统计
} tb6612_drive_rx_data_t;

/**
 * @brief TB6612 电机驱动器类
 * @note 与 A4950 对齐设计, 支持 Current/Omega/Angle 三种控制模式
 *       使用 1 路 PWM + 2 路 GPIO(IN1/IN2) 控制电机
 */
class tb6612_drive_class : public pwm_class
{
public:
    // 构造函数
    tb6612_drive_class() = default;

    // pid 角度环控制
    pid_class pid_angle;
    // pid 角速度环控制
    pid_class pid_omega;

    /**
     * @brief 初始化 TB6612 驱动
     * @param pwm_handle     PWM 定时器句柄
     * @param encoder_handle 编码器定时器句柄
     * @param __pwm_channel  PWM 通道
     * @param __link_gpio    IN1/IN2 GPIO 配置
     * @param __drive_method 控制方式
     * @param __control_period 控制周期, s
     * @param __gearbox_rate  减速比
     */
    void init(TIM_HandleTypeDef *pwm_handle, TIM_HandleTypeDef *encoder_handle,
              uint32_t &__pwm_channel, tb6612_link_gpio_t *__link_gpio,
              const tb6612_drive_mode_e &__drive_method = Motor_TB6612_Drive_Method_Current,
              const float &__control_period = 0.005f,
              const float &__gearbox_rate = 30.0f);

    // 获取电机驱动方式
    inline tb6612_drive_mode_e get_drive_method() const;

    // 获取输出轴反馈电机参数
    inline float get_now_angle() const;
    inline float get_now_omega() const;

    // 获取电机转子反馈电机参数
    inline float get_rotor_angle() const;
    inline float get_rotor_omega() const;

    // 获取控制参数
    inline float get_control_period() const;
    inline float get_target_angle() const;
    inline float get_target_omega() const;
    inline float get_target_current() const;
    inline float get_feedback_omega() const;
    inline float get_feedback_current() const;

    // 设置电机控制方式和控制参数
    inline void set_drive_method(const tb6612_drive_mode_e &__drive_method);
    inline void set_control_period(const float &__control_period);
    inline void set_target_angle(const float &__target_angle);
    inline void set_target_omega(const float &__target_omega);
    inline void set_target_current(const float &__target_current);
    inline void set_feedback_omega(const float &__feedback_omega);
    inline void set_feedback_current(const float &__feedback_current);

    // 滤波后的电机反馈数据
    inline float get_filter_angle() const;
    inline float get_filter_omega() const;

    // 计算和执行电机解算和控制任务
    void data_process();                    // 获取电机反馈速度
    void tim_encoder_round_count();         // 定时器编码器溢出次数计数
    void control_calculate_callback();      // 电机闭环控制计算回调函数

private:
    // 常量

    // 电机转子一圈编码器刻度（编码器精度），霍尔编码器为 13 线精度，GMR 为 500 线精度
    const uint16_t Encoder_Num_Per_Round = 13;
    // 编码器倍频系数，取决于定时器的编码器模式设置（AB 相四倍频）
    const uint16_t Encoder_Num_Rate = 4;
    // 编码器模式定时器计数值范围（16 位定时器: 0~65535, 共 65536 个状态）
    static constexpr uint32_t Encoder_Timer_Period = 65536;
    // 电机最大输出占空比归一化值
    const float Max_Duty_Out = 100.0f;
    // 电机最大空载转速, rad/s (200 rpm 换算)
    const float Max_Omega = 200.0f * 0.104719755f;

    // IN1/IN2 GPIO 配置
    tb6612_link_gpio_t link_gpio{};

    float control_period = 0.005f;  // 控制周期, s
    float gearbox_rate = 30.0f;     // 减速比

    float out = 0.0f;               // 电机输出值

    // 读变量
    tb6612_drive_rx_data_t rx_data{};
    // 写变量

    tb6612_drive_mode_e drive_method = Motor_TB6612_Drive_Method_Current;

    float target_angle = 0.0f;      // 目标角度, rad
    float target_omega = 0.0f;      // 目标角速度, rad/s
    float target_current = 0.0f;    // 目标电流（开环占空比控制）
    float feedback_omega = 0.0f;    // 前馈角速度, rad/s
    float feedback_current = 0.0f;  // 前馈电流

    // 滤波后的反馈数据
    float filter_angle = 0.0f;
    float filter_omega = 0.0f;

    // 内部函数
    void pid_calculate();           // PID 计算函数
    void set_motor_direction(float duty);  // 设置电机方向 GPIO
};

/*------------------------------------------------------内联函数实现----------------------------------------------------------------*/

/**
 * @brief 获取电机驱动方式
 * @return tb6612_drive_mode_e 电机驱动方式
 */
inline tb6612_drive_mode_e tb6612_drive_class::get_drive_method() const
{
    return drive_method;
}

/**
 * @brief 获取当前电机角度
 * @return float 当前电机角度, rad
 */
inline float tb6612_drive_class::get_now_angle() const
{
    return rx_data.now_angle;
}

/**
 * @brief 获取当前电机角速度
 * @return float 当前电机角速度, rad/s
 */
inline float tb6612_drive_class::get_now_omega() const
{
    return rx_data.now_omega;
}

/**
 * @brief 获取电机转子角度
 * @return float 电机转子角度, rad
 */
inline float tb6612_drive_class::get_rotor_angle() const
{
    return rx_data.rotor_angle;
}

/**
 * @brief 获取电机转子角速度
 * @return float 电机转子角速度, rad/s
 */
inline float tb6612_drive_class::get_rotor_omega() const
{
    return rx_data.rotor_omega;
}

/**
 * @brief 获取控制周期
 * @return float 控制周期, s
 */
inline float tb6612_drive_class::get_control_period() const
{
    return control_period;
}

/**
 * @brief 获取目标角度
 * @return float 目标角度, rad
 */
inline float tb6612_drive_class::get_target_angle() const
{
    return target_angle;
}

/**
 * @brief 获取目标角速度
 * @return float 目标角速度, rad/s
 */
inline float tb6612_drive_class::get_target_omega() const
{
    return target_omega;
}

/**
 * @brief 获取目标电流
 * @return float 目标电流
 */
inline float tb6612_drive_class::get_target_current() const
{
    return target_current;
}

/**
 * @brief 获取前馈角速度
 * @return float 前馈角速度, rad/s
 */
inline float tb6612_drive_class::get_feedback_omega() const
{
    return feedback_omega;
}

/**
 * @brief 获取前馈电流
 * @return float 前馈电流
 */
inline float tb6612_drive_class::get_feedback_current() const
{
    return feedback_current;
}

/**
 * @brief 设置电机驱动方式
 * @param __drive_method 电机驱动方式
 */
inline void tb6612_drive_class::set_drive_method(const tb6612_drive_mode_e &__drive_method)
{
    drive_method = __drive_method;
}

/**
 * @brief 设置控制周期
 * @param __control_period 控制周期, s
 */
inline void tb6612_drive_class::set_control_period(const float &__control_period)
{
    control_period = __control_period;
}

/**
 * @brief 设置目标角度
 * @param __target_angle 目标角度, rad
 */
inline void tb6612_drive_class::set_target_angle(const float &__target_angle)
{
    target_angle = __target_angle;
}

/**
 * @brief 设置目标角速度
 * @param __target_omega 目标角速度, rad/s
 */
inline void tb6612_drive_class::set_target_omega(const float &__target_omega)
{
    target_omega = __target_omega;
}

/**
 * @brief 设置目标电流
 * @param __target_current 目标电流
 */
inline void tb6612_drive_class::set_target_current(const float &__target_current)
{
    target_current = __target_current;
}

/**
 * @brief 设置前馈角速度
 * @param __feedback_omega 前馈角速度, rad/s
 */
inline void tb6612_drive_class::set_feedback_omega(const float &__feedback_omega)
{
    feedback_omega = __feedback_omega;
}

/**
 * @brief 设置前馈电流
 * @param __feedback_current 前馈电流
 */
inline void tb6612_drive_class::set_feedback_current(const float &__feedback_current)
{
    feedback_current = __feedback_current;
}

/**
 * @brief 获取滤波后的角度
 * @return float 滤波后的角度, rad
 */
inline float tb6612_drive_class::get_filter_angle() const
{
    return filter_angle;
}

/**
 * @brief 获取滤波后的角速度
 * @return float 滤波后的角速度, rad/s
 */
inline float tb6612_drive_class::get_filter_omega() const
{
    return filter_omega;
}