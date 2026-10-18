#include "bsp_pwm.h"

void pwm_class::set_duty_ration(float duty_ratio)
{
    if (duty_ratio < 0.0f || duty_ratio > 1.0f)
    {
        return;
    }

    if (tim_handle_[0] == nullptr)
    {
        return;
    }

    pwm_value_[0] = static_cast<uint32_t>(duty_ratio * (tim_handle_[0]->Init.Period + 1));
    __HAL_TIM_SetCompare(tim_handle_[0], pwm_channel_[0], pwm_value_[0]);
}

void pwm_class::set_duty_ration(uint8_t channel_index, float duty_ratio)
{
    if (duty_ratio < 0.0f || duty_ratio > 1.0f)
    {
        return;
    }

    if (tim_handle_[0] == nullptr || channel_index >= channel_amount)
    {
        return;
    }

    pwm_value_[channel_index] = static_cast<uint32_t>(duty_ratio * (tim_handle_[0]->Init.Period + 1));
    __HAL_TIM_SetCompare(tim_handle_[0], pwm_channel_[channel_index], pwm_value_[channel_index]);
}
