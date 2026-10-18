#pragma once

#include "bsp_pwm.h"
#include "basic_math.h"
#include "tim.h"

/* Exported macros -----------------------------------------------------------*/

extern const float BUZZER_FREQUENCY_A0;
extern const float BUZZER_FREQUENCY_A0_SHARP;
extern const float BUZZER_FREQUENCY_B0_FLAT;
extern const float BUZZER_FREQUENCY_B0;
extern const float BUZZER_FREQUENCY_C1;
extern const float BUZZER_FREQUENCY_C1_SHARP;
extern const float BUZZER_FREQUENCY_D1_FLAT;
extern const float BUZZER_FREQUENCY_D1;
extern const float BUZZER_FREQUENCY_D1_SHARP;
extern const float BUZZER_FREQUENCY_E1_FLAT;
extern const float BUZZER_FREQUENCY_E1;
extern const float BUZZER_FREQUENCY_F1;
extern const float BUZZER_FREQUENCY_F1_SHARP;
extern const float BUZZER_FREQUENCY_G1_FLAT;
extern const float BUZZER_FREQUENCY_G1;
extern const float BUZZER_FREQUENCY_G1_SHARP;
extern const float BUZZER_FREQUENCY_A1_FLAT;
extern const float BUZZER_FREQUENCY_A1;
extern const float BUZZER_FREQUENCY_A1_SHARP;
extern const float BUZZER_FREQUENCY_B1_FLAT;
extern const float BUZZER_FREQUENCY_B1;
extern const float BUZZER_FREQUENCY_C2;
extern const float BUZZER_FREQUENCY_C2_SHARP;
extern const float BUZZER_FREQUENCY_D2_FLAT;
extern const float BUZZER_FREQUENCY_D2;
extern const float BUZZER_FREQUENCY_D2_SHARP;
extern const float BUZZER_FREQUENCY_E2_FLAT;
extern const float BUZZER_FREQUENCY_E2;
extern const float BUZZER_FREQUENCY_F2;
extern const float BUZZER_FREQUENCY_F2_SHARP;
extern const float BUZZER_FREQUENCY_G2_FLAT;
extern const float BUZZER_FREQUENCY_G2;
extern const float BUZZER_FREQUENCY_G2_SHARP;
extern const float BUZZER_FREQUENCY_A2_FLAT;
extern const float BUZZER_FREQUENCY_A2;
extern const float BUZZER_FREQUENCY_A2_SHARP;
extern const float BUZZER_FREQUENCY_B2_FLAT;
extern const float BUZZER_FREQUENCY_B2;
extern const float BUZZER_FREQUENCY_C3;
extern const float BUZZER_FREQUENCY_C3_SHARP;
extern const float BUZZER_FREQUENCY_D3_FLAT;
extern const float BUZZER_FREQUENCY_D3;
extern const float BUZZER_FREQUENCY_D3_SHARP;
extern const float BUZZER_FREQUENCY_E3_FLAT;
extern const float BUZZER_FREQUENCY_E3;
extern const float BUZZER_FREQUENCY_F3;
extern const float BUZZER_FREQUENCY_F3_SHARP;
extern const float BUZZER_FREQUENCY_G3_FLAT;
extern const float BUZZER_FREQUENCY_G3;
extern const float BUZZER_FREQUENCY_G3_SHARP;
extern const float BUZZER_FREQUENCY_A3_FLAT;
extern const float BUZZER_FREQUENCY_A3;
extern const float BUZZER_FREQUENCY_A3_SHARP;
extern const float BUZZER_FREQUENCY_B3_FLAT;
extern const float BUZZER_FREQUENCY_B3;
extern const float BUZZER_FREQUENCY_C4;
extern const float BUZZER_FREQUENCY_C4_SHARP;
extern const float BUZZER_FREQUENCY_D4_FLAT;
extern const float BUZZER_FREQUENCY_D4;
extern const float BUZZER_FREQUENCY_D4_SHARP;
extern const float BUZZER_FREQUENCY_E4_FLAT;
extern const float BUZZER_FREQUENCY_E4;
extern const float BUZZER_FREQUENCY_F4;
extern const float BUZZER_FREQUENCY_F4_SHARP;
extern const float BUZZER_FREQUENCY_G4_FLAT;
extern const float BUZZER_FREQUENCY_G4;
extern const float BUZZER_FREQUENCY_G4_SHARP;
extern const float BUZZER_FREQUENCY_A4_FLAT;
extern const float BUZZER_FREQUENCY_A4;
extern const float BUZZER_FREQUENCY_A4_SHARP;
extern const float BUZZER_FREQUENCY_B4_FLAT;
extern const float BUZZER_FREQUENCY_B4;
extern const float BUZZER_FREQUENCY_C5;
extern const float BUZZER_FREQUENCY_C5_SHARP;
extern const float BUZZER_FREQUENCY_D5_FLAT;
extern const float BUZZER_FREQUENCY_D5;
extern const float BUZZER_FREQUENCY_D5_SHARP;
extern const float BUZZER_FREQUENCY_E5_FLAT;
extern const float BUZZER_FREQUENCY_E5;
extern const float BUZZER_FREQUENCY_F5;
extern const float BUZZER_FREQUENCY_F5_SHARP;
extern const float BUZZER_FREQUENCY_G5_FLAT;
extern const float BUZZER_FREQUENCY_G5;
extern const float BUZZER_FREQUENCY_G5_SHARP;
extern const float BUZZER_FREQUENCY_A5_FLAT;
extern const float BUZZER_FREQUENCY_A5;
extern const float BUZZER_FREQUENCY_A5_SHARP;
extern const float BUZZER_FREQUENCY_B5_FLAT;
extern const float BUZZER_FREQUENCY_B5;
extern const float BUZZER_FREQUENCY_C6;
extern const float BUZZER_FREQUENCY_C6_SHARP;
extern const float BUZZER_FREQUENCY_D6_FLAT;
extern const float BUZZER_FREQUENCY_D6;
extern const float BUZZER_FREQUENCY_D6_SHARP;
extern const float BUZZER_FREQUENCY_E6_FLAT;
extern const float BUZZER_FREQUENCY_E6;
extern const float BUZZER_FREQUENCY_F6;
extern const float BUZZER_FREQUENCY_F6_SHARP;
extern const float BUZZER_FREQUENCY_G6_FLAT;
extern const float BUZZER_FREQUENCY_G6;
extern const float BUZZER_FREQUENCY_G6_SHARP;
extern const float BUZZER_FREQUENCY_A6_FLAT;
extern const float BUZZER_FREQUENCY_A6;
extern const float BUZZER_FREQUENCY_A6_SHARP;
extern const float BUZZER_FREQUENCY_B6_FLAT;
extern const float BUZZER_FREQUENCY_B6;
extern const float BUZZER_FREQUENCY_C7;
extern const float BUZZER_FREQUENCY_C7_SHARP;
extern const float BUZZER_FREQUENCY_D7_FLAT;
extern const float BUZZER_FREQUENCY_D7;
extern const float BUZZER_FREQUENCY_D7_SHARP;
extern const float BUZZER_FREQUENCY_E7_FLAT;
extern const float BUZZER_FREQUENCY_E7;
extern const float BUZZER_FREQUENCY_F7;
extern const float BUZZER_FREQUENCY_F7_SHARP;
extern const float BUZZER_FREQUENCY_G7_FLAT;
extern const float BUZZER_FREQUENCY_G7;
extern const float BUZZER_FREQUENCY_G7_SHARP;
extern const float BUZZER_FREQUENCY_A7_FLAT;
extern const float BUZZER_FREQUENCY_A7;
extern const float BUZZER_FREQUENCY_A7_SHARP;
extern const float BUZZER_FREQUENCY_B7_FLAT;
extern const float BUZZER_FREQUENCY_B7;
extern const float BUZZER_FREQUENCY_C8;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 板载蜂鸣器
 *
 */
class buzzer_class : public pwm_class
{
    public:

        void init(const float &__frequency, const float &__loudness);

        inline void set_frequency(const float &__frequency);

        inline void set_loudness(const float &__loudness);

        inline void set_sound(const float &__frequency, const float &__loudness);

    protected:

        float frequency = 4000.0f;     // 蜂鸣器频率
        float loudness = 0.0f;         // 蜂鸣器响度，范围为0.0到1.0

};

/**
 * @brief 设置蜂鸣器频率,音色
 * @param __frequency 蜂鸣器频率, 单位Hz
 */
inline void buzzer_class::set_frequency(const float &__frequency)
{
    frequency = basic_math_constrain(__frequency, 0.0f, 20000.0f); // 将频率限制在20Hz到20kHz范围内

    float arr;

    if (frequency == 0.0f)
    {
        arr = 0.0f; // 如果频率为0，设置ARR为0以停止蜂鸣器
    }

    else
    {
        arr = 1000000.0f / frequency; // 计算ARR值
    }

    __HAL_TIM_SetAutoreload(get_tim_handle(0), static_cast<uint32_t>(arr - 1)); // 设置ARR寄存器

    set_duty_ration( loudness * 0.5f); // 设置占空比
}

/**
 * @brief 设置蜂鸣器响度，音量
 *
 * @param loudness 蜂鸣器响度, 0-1
 */
inline void buzzer_class::set_loudness(const float &__loudness)
{
    loudness = basic_math_constrain(__loudness, 0.0f, 1.0f); // 将响度限制在0.0到1.0范围内

    float arr;

    if (frequency == 0.0f)
    {
        arr = 0.0f; // 如果频率为0，设置ARR为0以停止蜂鸣器
    }

    else
    {
        arr = 1000000.0f / frequency; // 计算ARR值
    }

    __HAL_TIM_SetAutoreload(get_tim_handle(0), static_cast<uint32_t>(arr - 1)); // 设置ARR寄存器

    set_duty_ration(loudness * 0.5f); // 设置占空比
}

/**
 * @brief 设置蜂鸣器声音
 *
 * @param frequency 蜂鸣器频率, 单位Hz
 * @param loudness 蜂鸣器响度, 0-1
 */

inline void buzzer_class::set_sound(const float &__frequency, const float &__loudness)
{
    frequency = basic_math_constrain(__frequency, 0.0f, 20000.0f); // 将频率限制在20Hz到20kHz范围内
    loudness = basic_math_constrain(__loudness, 0.0f, 1.0f); // 将响度限制在0.0到1.0范围内

    float arr;

    if (frequency == 0.0f)
    {
        arr = 0.0f; // 如果频率为0，设置ARR为0以停止蜂鸣器
    }

    else
    {
        arr = 1000000.0f / frequency; // 计算ARR值
    }

    __HAL_TIM_SetAutoreload(get_tim_handle(0), static_cast<uint32_t>(arr - 1)); // 设置ARR寄存器

    set_duty_ration(loudness * 0.5f); // 设置占空比
}