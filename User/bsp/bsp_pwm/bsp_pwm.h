#pragma once

#include "tim.h"
#include "stm32h7xx_hal.h"

class pwm_class
{
    public:

        static constexpr uint8_t channel_amount = 2;
        // 旧接口：设置PWM相关参数
        void set_tim_handle(TIM_HandleTypeDef *tim_handle) { tim_handle_[0] = tim_handle; };

        void set_pwm_channel(uint32_t pwm_channel) { pwm_channel_[0] = pwm_channel; };

        // 设置通道最大占空比。限制只能收紧，不能通过后续调用放宽。
        void set_max_duty_ratio(float max_duty_ratio);

        void set_duty_ration(float duty_ratio);

        void start_pwm(void);

        void stop_pwm(void);

        // 新接口：设置PWM相关参数
        void set_tim_handle(uint8_t handle_index, TIM_HandleTypeDef *tim_handle) 
        { 
            if (handle_index < channel_amount)
            {
                tim_handle_[handle_index] = tim_handle; 
            }
        };

        void set_pwm_channel(uint8_t channel_index, uint32_t pwm_channel)
        {
            if (channel_index < channel_amount)
            {
                pwm_channel_[channel_index] = pwm_channel;
            }
        }

        // 设置通道最大占空比。每个 pwm_class 实例独立配置，默认 100%。
        void set_max_duty_ratio(uint8_t channel_index, float max_duty_ratio);

        void set_duty_ration(uint8_t channel_index, float duty_ratio);

        void start_pwm(uint8_t channel_index);

        void stop_pwm(uint8_t channel_index);

        TIM_HandleTypeDef* get_tim_handle(uint8_t handle_index) { return tim_handle_[handle_index]; }

        void start_tim_encoder(TIM_HandleTypeDef *encoder_handle)
        {
            if (encoder_handle != nullptr)
            {
                // 启动编码器接口的计数功能
                HAL_TIM_Encoder_Start(encoder_handle, TIM_CHANNEL_1|TIM_CHANNEL_2);
                // 清除更新中断标志，避免一开始就触发中断
                __HAL_TIM_CLEAR_FLAG(encoder_handle, TIM_FLAG_UPDATE);
                // 使能更新中断，用于编码器计数溢出时的处理
                __HAL_TIM_ENABLE_IT(encoder_handle, TIM_IT_UPDATE);
            }
        }

    private:

        float sanitize_duty_ratio(uint8_t channel_index, float duty_ratio) const;
        void write_duty_ratio(uint8_t channel_index, float duty_ratio);

        // PWM 句柄,handle0为pwm控制器的句柄，handle1为编码器的句柄
        TIM_HandleTypeDef *tim_handle_[channel_amount] = {nullptr, nullptr};    

        uint32_t pwm_channel_[channel_amount] = {0, 0};

        uint32_t pwm_value_[channel_amount] = {0, 0};

        float max_duty_ratio_[channel_amount] = {1.0f, 1.0f};

};
