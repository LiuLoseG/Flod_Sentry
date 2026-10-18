#include "a4950_drive.h"
#include <cstring>

/**
 * @brief 初始化A4950驱动
 * @param pwm_handle PWM定时器句柄
 * @param encoder_handle 编码器定时器句柄
 * @param pwm_channel1 PWM通道1
 * @param pwm_channel2 PWM通道2
 * @param gearbox_rate 减速比
 */
void a4950_drive_class::init(TIM_HandleTypeDef *pwm_handle, TIM_HandleTypeDef *encoder_handle, uint32_t &__pwm_channel1, uint32_t &__pwm_channel2, const a4950_drive_mode_e &__drive_method, const float &__control_period, const float &__gearbox_rate)
{
    // 设置绑定的定时器PWM参数
    set_tim_handle(0, pwm_handle);
    set_tim_handle(1, encoder_handle);

    set_pwm_channel(0, __pwm_channel1);
    set_pwm_channel(1, __pwm_channel2);

    start_tim_encoder(encoder_handle); // 启动编码器接口
    
    start_pwm(0); // 启动PWM输出
    start_pwm(1); // 启动PWM输出
    set_duty_ration(0, 0.0f);
    set_duty_ration(1, 0.0f);

    drive_method = __drive_method;              // 设置电机驱动器控制方式
    control_period = __control_period;          // 设置控制周期
    gearbox_rate = __gearbox_rate;              // 设置减速比

    memset(&rx_data, 0, sizeof(rx_data));
}

/**
 * @brief 获取电机反馈速度
 */
void a4950_drive_class::data_process()
{
    // 原子读取：防止 ISR 在读取 CNT 与 tim_encoder_round 之间更新后者导致偏差
    int32_t round_snapshot;
    uint32_t tmp_encoder;
    do {
        round_snapshot = rx_data.tim_encoder_round;
        tmp_encoder = __HAL_TIM_GET_COUNTER(get_tim_handle(1));
    } while (round_snapshot != rx_data.tim_encoder_round);

    // 获取累计总编码器值（tim_encoder_round 已通过中断补偿定时器溢出）
    rx_data.total_encoder = (int32_t)tmp_encoder + round_snapshot * (int32_t)Encoder_Timer_Period;

    // 转子总角度, 单位: rad
    rx_data.rotor_angle = (float)rx_data.total_encoder / (float)(Encoder_Num_Per_Round * Encoder_Num_Rate) * 2.0f * (float)M_PI;

    // 转子角速度, 单位: rad/s
    rx_data.rotor_omega = (float)(rx_data.total_encoder - rx_data.last_total_encoder) / (float)(Encoder_Num_Per_Round * Encoder_Num_Rate) * 2.0f * (float)M_PI / control_period;

    // 输出轴总角度和角速度值
    rx_data.now_angle = rx_data.rotor_angle / gearbox_rate; // 输出轴总角度，单位：rad
    rx_data.now_omega = rx_data.rotor_omega / gearbox_rate; // 输出轴角速度，单位：rad/s

    // 电机输出轴总转动圈数
    rx_data.now_round = (int32_t)(rx_data.total_encoder / (Encoder_Num_Per_Round * Encoder_Num_Rate * gearbox_rate)); // 总圈数

    // 上次总编码器值
    rx_data.last_total_encoder = rx_data.total_encoder;

}

/**
 * @brief 编码器轮数统计
 */
void a4950_drive_class::tim_encoder_round_count()
{
    // 编码器轮数统计
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(get_tim_handle(1)))
    {
        rx_data.tim_encoder_round--;
    }

    else 
    {
        rx_data.tim_encoder_round++;
    }
}

/**
 * @brief 电机PID计算控制，使用电机滤波后的数据进行PID闭环控制
 */
void a4950_drive_class::pid_calculate()
{
    switch (drive_method)
    {
        case Motor_A4950_Drive_Method_Current:      // 开环控制逻辑
        {
            break;
        }

        case Motor_A4950_Drive_Method_Omega:        // 角速度闭环控制逻辑
        {
            pid_omega.set_target(target_omega + feedback_omega);    // 设置角速度环目标值
            pid_omega.set_now(filter_omega);                        // 设置滤波后的角速度作为当前值
            pid_omega.pid_calculate_callback();                    // 计算角速度环PID输出值

            target_current = pid_omega.get_out();                  // 获取角速度环PID输出值作为电流环目标值
            break;
        }

        case Motor_A4950_Drive_Method_Angle:        // 角度闭环控制逻辑
        {
            pid_angle.set_target(target_angle);                    // 设置角度环目标值
            pid_angle.set_now(filter_angle);                        // 设置滤波后的角度作为当前值
            pid_angle.pid_calculate_callback();                    // 计算角度环PID输出值

            target_omega = pid_angle.get_out();                    // 获取角度环PID输出值作为角速度环目标值

            pid_omega.set_target(target_omega + feedback_omega);    // 设置角速度环目标值
            pid_omega.set_now(filter_omega);                        // 设置滤波后的角速度作为当前值
            pid_omega.pid_calculate_callback();                    // 计算角速度环PID输出值

            target_current = pid_omega.get_out();                  // 获取角速度环PID输出值作为电流环目标
            break;
        }
    }
}

void a4950_drive_class::control_calculate_callback()
{
    // 执行PID计算
    pid_calculate();

    out = (target_current + feedback_current) / Max_Current_Out; // 电机输出值为目标电流和反馈电流的平均值
    basic_math_constrain(out, -1.0f, 1.0f); // 限幅到[-1, 1]

    // 设置PWM占空比
    if (out >= 0.0f)
    {
        set_duty_ration(0, 1.0f);    // 正转
        set_duty_ration(1, 1.0f - out);
    }
    else
    {
        set_duty_ration(0, 1.0f + out);   // 反转
        set_duty_ration(1, 1.0f);
    }
}