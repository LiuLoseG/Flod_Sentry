#include "bsp_pwm.h"

#include <algorithm>
#include <cmath>

float pwm_class::sanitize_duty_ratio(uint8_t channel_index, float duty_ratio) const
{
    // Invalid floating point input must never preserve a previous dangerous
    // compare value. Fail closed instead.
    if (!std::isfinite(duty_ratio) || duty_ratio <= 0.0f)
    {
        return 0.0f;
    }

    const float max_duty_ratio = max_duty_ratio_[channel_index];
    return (duty_ratio > max_duty_ratio) ? max_duty_ratio : duty_ratio;
}

void pwm_class::set_max_duty_ratio(float max_duty_ratio)
{
    set_max_duty_ratio(0U, max_duty_ratio);
}

void pwm_class::set_max_duty_ratio(uint8_t channel_index, float max_duty_ratio)
{
    if ((channel_index >= channel_amount) || !std::isfinite(max_duty_ratio))
    {
        return;
    }

    max_duty_ratio = std::max(0.0f, std::min(1.0f, max_duty_ratio));
    // A configured safety limit is monotonic: it can only be tightened.
    if (max_duty_ratio < max_duty_ratio_[channel_index])
    {
        max_duty_ratio_[channel_index] = max_duty_ratio;
    }
}

void pwm_class::write_duty_ratio(uint8_t channel_index, float duty_ratio)
{
    if (tim_handle_[0] == nullptr || channel_index >= channel_amount)
    {
        return;
    }

    const float safe_duty_ratio = sanitize_duty_ratio(channel_index, duty_ratio);
    const uint32_t period = tim_handle_[0]->Init.Period;
    const uint64_t period_counts = static_cast<uint64_t>(period) + 1ULL;
    uint64_t compare = static_cast<uint64_t>(safe_duty_ratio * static_cast<float>(period_counts));

    // Keep the compare value within ARR. This also prevents an ARR overflow
    // from turning a nominally safe request into an always-on output.
    if (compare > period)
    {
        compare = period;
    }

    pwm_value_[channel_index] = static_cast<uint32_t>(compare);
    __HAL_TIM_SetCompare(tim_handle_[0], pwm_channel_[channel_index], pwm_value_[channel_index]);
}

void pwm_class::set_duty_ration(float duty_ratio)
{
    write_duty_ratio(0U, duty_ratio);
}

void pwm_class::start_pwm(void)
{
    if (tim_handle_[0] == nullptr)
    {
        return;
    }

    // Explicitly clear CCR before enabling the output, even if the timer
    // handle was reused by another owner.
    write_duty_ratio(0U, 0.0f);
    if (HAL_TIM_PWM_Start(tim_handle_[0], pwm_channel_[0]) != HAL_OK)
    {
        write_duty_ratio(0U, 0.0f);
    }
}

void pwm_class::stop_pwm(void)
{
    if (tim_handle_[0] == nullptr)
    {
        return;
    }

    (void)HAL_TIM_PWM_Stop(tim_handle_[0], pwm_channel_[0]);
    write_duty_ratio(0U, 0.0f);
}

void pwm_class::set_duty_ration(uint8_t channel_index, float duty_ratio)
{
    write_duty_ratio(channel_index, duty_ratio);
}

void pwm_class::start_pwm(uint8_t channel_index)
{
    if (tim_handle_[0] == nullptr || channel_index >= channel_amount)
    {
        return;
    }

    write_duty_ratio(channel_index, 0.0f);
    if (HAL_TIM_PWM_Start(tim_handle_[0], pwm_channel_[channel_index]) != HAL_OK)
    {
        write_duty_ratio(channel_index, 0.0f);
    }
}

void pwm_class::stop_pwm(uint8_t channel_index)
{
    if (tim_handle_[0] == nullptr || channel_index >= channel_amount)
    {
        return;
    }

    (void)HAL_TIM_PWM_Stop(tim_handle_[0], pwm_channel_[channel_index]);
    write_duty_ratio(channel_index, 0.0f);
}
