#include "tb6612_drive.h"
#include <cstring>

/**
 * @brief 初始化 TB6612 驱动
 * @param pwm_handle PWM 定时器句柄
 * @param encoder_handle 编码器定时器句柄
 * @param __pwm_channel PWM 通道
 * @param __link_gpio IN1/IN2 GPIO 配置
 * @param __drive_method 控制方式
 * @param __control_period 控制周期
 * @param __gearbox_rate 减速比
 */
void tb6612_drive_class::init(TIM_HandleTypeDef *pwm_handle, TIM_HandleTypeDef *encoder_handle,
                               uint32_t &__pwm_channel, tb6612_link_gpio_t *__link_gpio,
                               const tb6612_drive_mode_e &__drive_method,
                               const float &__control_period,
                               const float &__gearbox_rate)
{
    // 设置绑定的定时器 PWM 参数
    set_tim_handle(0, pwm_handle);
    set_tim_handle(1, encoder_handle);

    set_pwm_channel(0, __pwm_channel);

    // 保存 GPIO 配置
    if (__link_gpio != nullptr)
    {
        link_gpio = *__link_gpio;
    }

    // 启动编码器接口
    start_tim_encoder(encoder_handle);

    // 启动 PWM 输出并初始化为 0 占空比
    start_pwm(0);
    set_duty_ration(0, 0.0f);

    drive_method = __drive_method;
    control_period = __control_period;
    gearbox_rate = __gearbox_rate;

    // 初始化为刹车状态（IN1=0, IN2=0）
    HAL_GPIO_WritePin(link_gpio.in1_port, link_gpio.in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(link_gpio.in2_port, link_gpio.in2_pin, GPIO_PIN_RESET);

    memset(&rx_data, 0, sizeof(rx_data));
}

/**
 * @brief 设置电机方向 GPIO
 * @param duty 输出值（正 → 正转，负 → 反转，零 → 刹车）
 */
void tb6612_drive_class::set_motor_direction(float duty)
{
    if (duty > 0.0f)
    {
        // 正转: IN1=1, IN2=0
        HAL_GPIO_WritePin(link_gpio.in1_port, link_gpio.in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(link_gpio.in2_port, link_gpio.in2_pin, GPIO_PIN_RESET);
    }
    else if (duty < 0.0f)
    {
        // 反转: IN1=0, IN2=1
        HAL_GPIO_WritePin(link_gpio.in1_port, link_gpio.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(link_gpio.in2_port, link_gpio.in2_pin, GPIO_PIN_SET);
    }
    else
    {
        // 刹车: IN1=0, IN2=0
        HAL_GPIO_WritePin(link_gpio.in1_port, link_gpio.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(link_gpio.in2_port, link_gpio.in2_pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief 获取电机反馈速度
 */
void tb6612_drive_class::data_process()
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

    // 转子总角度, rad
    rx_data.rotor_angle = (float)rx_data.total_encoder / (float)(Encoder_Num_Per_Round * Encoder_Num_Rate) * 2.0f * (float)PI;

    // 转子角速度, rad/s
    rx_data.rotor_omega = (float)(rx_data.total_encoder - rx_data.last_total_encoder) / (float)(Encoder_Num_Per_Round * Encoder_Num_Rate) * 2.0f * (float)PI / control_period;

    // 输出轴总角度和角速度值
    rx_data.now_angle = rx_data.rotor_angle / gearbox_rate;
    rx_data.now_omega = rx_data.rotor_omega / gearbox_rate;

    // 电机输出轴总转动圈数
    rx_data.now_round = (int32_t)(rx_data.total_encoder / (Encoder_Num_Per_Round * Encoder_Num_Rate * gearbox_rate));

    // 保存上次总编码器值
    rx_data.last_total_encoder = rx_data.total_encoder;
}

/**
 * @brief 编码器轮数统计
 */
void tb6612_drive_class::tim_encoder_round_count()
{
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
 * @brief 电机 PID 计算控制，使用电机滤波后的数据进行 PID 闭环控制
 */
void tb6612_drive_class::pid_calculate()
{
    switch (drive_method)
    {
        case Motor_TB6612_Drive_Method_Current:      // 开环控制逻辑
        {
            break;
        }

        case Motor_TB6612_Drive_Method_Omega:        // 角速度闭环控制逻辑
        {
            pid_omega.set_target(target_omega + feedback_omega);
            pid_omega.set_now(filter_omega);
            pid_omega.pid_calculate_callback();

            target_current = pid_omega.get_out();
            break;
        }

        case Motor_TB6612_Drive_Method_Angle:        // 角度闭环控制逻辑
        {
            pid_angle.set_target(target_angle);
            pid_angle.set_now(filter_angle);
            pid_angle.pid_calculate_callback();

            target_omega = pid_angle.get_out();

            pid_omega.set_target(target_omega + feedback_omega);
            pid_omega.set_now(filter_omega);
            pid_omega.pid_calculate_callback();

            target_current = pid_omega.get_out();
            break;
        }
    }
}

/**
 * @brief 电机闭环控制计算回调函数
 */
void tb6612_drive_class::control_calculate_callback()
{
    // 执行 PID 计算
    pid_calculate();

    // 计算输出占空比（归一化到 [-1, 1]）
    out = (target_current + feedback_current) / Max_Duty_Out;
    basic_math_constrain(out, -1.0f, 1.0f);

    // 设置电机方向 GPIO
    set_motor_direction(out);

    // 设置 PWM 占空比（取绝对值）
    set_duty_ration(0, fabsf(out));
}
