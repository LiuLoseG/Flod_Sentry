#pragma once

#include "bsp_pwm.h"
#include "stm32H7xx_hal.h"
#include <cmath>
#include <cstdint>
#include "user_lib.h"
#include "pid.h"

// #define A4950_MOTOR_ENCODER_PPR 65536

// #define A4950_MOTOR_MAX_RPM 250.0f // 电机空载最大速度，单位：rpm

// #define A4950_SPEED_FILTER 0.95f    // 速度一阶低通滤波系数

// #define A4950_ENCODER_MEASURE_INTERVAL_MS 5.0f   // 编码器测量间隔，单位毫秒
// #define A4950_ENCODER_PPR 11 // 编码器每转脉冲数（Pulse Per Revolution）
// #define A4950_SOFT_RATIO 4 // 译码倍频系数（常见为AB相四倍频）
// #define A4950_REDUCTION_RATIO 30 // 电机减速比 减速比（电机轴转速 / 输出轴转速）

// #define RPM2RADS 0.10471975f // 转速单位转换：RPM转为rad/s的换算系数（2 * π / 60）

typedef enum
{
     Motor_A4950_Drive_Status_Disable = 0, // 电机驱动器禁用
     Motor_A4950_Drive_Status_Enable = 1,  // 电机驱动器使能
} a4950_drive_status_e;

typedef enum
{
     Motor_A4950_Drive_Method_Current = 0,   // 开环控制模式
     Motor_A4950_Drive_Method_Omega = 1,     // 角速度闭环模式
     Motor_A4950_Drive_Method_Angle = 2,     // 角度闭环模式
} a4950_drive_mode_e;

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

     int32_t tim_encoder_round;     // 定时器编码器轮数统计
} a4950_drive_rx_data_t;

/**
 * @brief A4950电机驱动器类
 */
class a4950_drive_class : public pwm_class
{
public:
     // 构造函数
     a4950_drive_class() = default;

    // pid角度环控制
     pid_class pid_angle; 
    // pid角速度环控制
     pid_class pid_omega;

     // 初始化函数，绑定定时器和PWM通道
     void init(TIM_HandleTypeDef *pwm_handle, TIM_HandleTypeDef *encoder_handle, uint32_t &__pwm_channel1, uint32_t &__pwm_channel2, const a4950_drive_mode_e &__drive_method = Motor_A4950_Drive_Method_Current, const float &__control_period = 0.005f, const float &__gearbox_rate = 30.0f);

     // 获取电机状态
     inline a4950_drive_status_e get_status() const;

     // 获取输出轴反馈电机参数
     inline float get_now_angle() const;
     inline float get_now_omega() const;

     // 获取电机转子反馈电机参数
     inline float get_rotor_angle() const;
     inline float get_rotor_omega() const;

     // 获取电机控制方式和控制参数
     inline a4950_drive_mode_e get_drive_method() const;
     inline float get_control_period() const;

     inline float get_target_angle() const;
     inline float get_target_omega() const;

     inline float get_feedback_omega() const;
     inline float get_feedback_current() const;

     // 设置电机控制方式和控制参数
     inline void set_drive_method(const a4950_drive_mode_e &__drive_method);
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
     void data_process();                    // 获取电机反馈速度函数

     void tim_encoder_round_count();         // 定时器编码器溢出次数计数

     void control_calculate_callback();      // 电机闭环控制计算回调函数

private:

     // 编码器一圈脉冲数(输出轴) = 编码器精度 * 编码器倍频系数 * 减速比

     // 常量

     // 电机转子一圈编码器刻度(编码器精度)，霍尔编码器为13线精度，GMR为500线精度。
     const uint16_t Encoder_Num_Per_Round = 13;
     // 编码器倍频系数，取决于定时器的编码器模式设置(倍频系数)
     const uint16_t Encoder_Num_Rate = 4;
     // 编码器模式定时器计数值范围（16位定时器: 0~65535, 共65536个状态）
     static constexpr uint32_t Encoder_Timer_Period = 65536;
     // 电机最大输出的电流值
     const float Max_Current_Out = 100.0f;
     
     float control_period = 0.005f; // 控制周期，单位：秒

     // 减速比, 默认带减速箱
     float gearbox_rate = 30.0f;           

     float out = 0.0f; // 电机输出值

     // 读变量

     // 电机接收数据结构体
     a4950_drive_rx_data_t rx_data;        
     // 电机驱动器状态
     a4950_drive_status_e status = Motor_A4950_Drive_Status_Disable;

     // 写变量

     // 电机驱动器控制方式
     a4950_drive_mode_e drive_method = Motor_A4950_Drive_Method_Current;
     // 目标的角度, rad
     float target_angle = 0.0f;
     // 目标的速度, rad/s
     float target_omega = 0.0f;
     // 目标的电流,(占空比的开环控制)
     float target_current = 0.0f;
     // 前馈电机的角速度, rad/s
     float feedback_omega = 0.0f;
     // 前馈电机开环电流
     float feedback_current = 0.0f;  

     // 滤波后的当前角度
     float filter_angle = 0.0f;
     // 滤波后的速度, rad/s
     float filter_omega = 0.0f;

     // 内部函数

     void pid_calculate(); // PID计算函数
};

/**
 * @brief 获取电机驱动器状态
 * @return 电机驱动器状态
 */
inline a4950_drive_status_e a4950_drive_class::get_status() const
{
     return status;
}

/**
 * @brief 获取当前电机角度
 * @return 当前电机角度，单位：rad
 */
inline float a4950_drive_class::get_now_angle() const
{
     return rx_data.now_angle;
}

/**
 * @brief 获取当前电机角速度
 * @return 当前电机角速度，单位：rad/s
 */
inline float a4950_drive_class::get_now_omega() const
{
     return rx_data.now_omega;
}

/**
 * @brief 获取电机转子角度
 * @return 电机转子角度，单位：rad
 */
inline float a4950_drive_class::get_rotor_angle() const
{
     return rx_data.rotor_angle;
}

/**
 * @brief 获取电机转子角速度
 * @return 电机转子角速度，单位：rad/s
 */
inline float a4950_drive_class::get_rotor_omega() const
{
     return rx_data.rotor_omega;
}

/**
 * @brief 获取电机驱动方式
 * @return 电机驱动方式
 */
inline a4950_drive_mode_e a4950_drive_class::get_drive_method() const
{
     return drive_method;
}

/**
 * @brief 获取控制周期
 * @return 控制周期，单位：秒
 */
inline float a4950_drive_class::get_control_period() const
{
     return control_period;
}

/**
 * @brief 获取目标角度
 * @return 目标角度，单位：rad
 */
inline float a4950_drive_class::get_target_angle() const
{
     return target_angle;
}

/**
 * @brief 获取目标角速度
 * @return 目标角速度，单位：rad/s
 */
inline float a4950_drive_class::get_target_omega() const
{
     return target_omega;
}

/**
 * @brief 获取前馈角速度
 * @return 前馈角速度，单位：rad/s
 */
inline float a4950_drive_class::get_feedback_omega() const
{
     return feedback_omega;
}

/**
 * @brief 获取前馈电流
 * @return 前馈电流，单位：A
 */
inline float a4950_drive_class::get_feedback_current() const
{
     return feedback_current;
}

/**
 * @brief 设置电机驱动方式
 * @param __drive_method 电机驱动方式
 */
inline void a4950_drive_class::set_drive_method(const a4950_drive_mode_e &__drive_method)
{
     drive_method = __drive_method;
}

/**
 * @brief 设置控制周期
 * @param __control_period 控制周期，单位：秒
 */
inline void a4950_drive_class::set_control_period(const float &__control_period)
{
     control_period = __control_period;
}

/**
 * @brief 设置目标角度
 * @param __target_angle 目标角度，单位：rad
 */
inline void a4950_drive_class::set_target_angle(const float &__target_angle)
{
     target_angle = __target_angle;
}

/**
 * @brief 设置目标角速度
 * @param __target_omega 目标角速度，单位：rad/s
 */
inline void a4950_drive_class::set_target_omega(const float &__target_omega)
{
     target_omega = __target_omega;
}

/**
 * @brief 设置目标电流
 * @param __target_current 目标电流，单位：A
 */
inline void a4950_drive_class::set_target_current(const float &__target_current)
{
     target_current = __target_current;
}

/**
 * @brief 设置前馈角速度
 * @param __feedback_omega 前馈角速度，单位：rad/s
 */
inline void a4950_drive_class::set_feedback_omega(const float &__feedback_omega)
{
     feedback_omega = __feedback_omega;
}

/**
 * @brief 设置前馈电流
 * @param __feedback_current 前馈电流，单位：A
 */
inline void a4950_drive_class::set_feedback_current(const float &__feedback_current)
{
     feedback_current = __feedback_current;
}

/**
 * @brief 获取滤波后的角度
 * @return 滤波后的角度，单位：rad
 */
inline float a4950_drive_class::get_filter_angle() const
{
     return filter_angle;
}

/**
 * @brief 获取滤波后的角速度
 * @return 滤波后的角速度，单位：rad/s
 */
inline float a4950_drive_class::get_filter_omega() const
{
     return filter_omega;
}