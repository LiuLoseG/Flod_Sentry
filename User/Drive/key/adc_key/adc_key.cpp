/**
 * @file adc_key.cpp
 * @author HNAU-Robot (19038200162@163.com)
 * @brief ADC按键状态检测驱动
 * @date 2026-07-16
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#include "adc_key.h"
#include "bsp_dwt.h"

/**
 * @brief 判断浮点数的符号
 * @param value: 输入的浮点数
 * @retval 0: 正数或零, 1: 负数
 */
inline uint8_t judge_float_sign(float value)
{
    // return ((*(uint32_t*)&value) >> 31);
    return (value < 0.0f ? 1 : 0);
}

/**
 * @brief 获取LCD按键的状态
 * @retval lcd_key_t: 当前按键状态
 * @note  通过ADC采集的电压值判断按键状态，进行消抖处理
 */
lcd_key_t get_lcd_key_state(void)
{
    static lcd_key_t confirmed_key = LCD_KEY_NONE;  // 已确认的稳定状态
    static lcd_key_t pending_key   = LCD_KEY_NONE;  // 待确认的候选状态
    static float  hold_since_ms = 0;                // 候选状态开始时刻(ms)

    float adc_val = bsp_get_adc_lcd_key_voltage();

    int8_t key_num = -1;

    adc_val -= 3.63f; // 偏移电压值，调整为0附近

    while (judge_float_sign(adc_val))
    {
        key_num++;
        adc_val += 0.66f; // 每个按键的电压间隔为0.33V
    }

    lcd_key_t raw_key = (lcd_key_t)key_num;
    float now_ms = bsp_dwt_get_timeline_ms();

    // 原始值与候选值不同 → 更新候选，重新计时
    if (raw_key != pending_key)
    {
        pending_key   = raw_key;
        hold_since_ms = now_ms;
    }
    // 候选状态保持超过消抖时间 → 确认为稳定状态
    else if ((now_ms - hold_since_ms) >= (float)ADC_KEY_DEBOUNCE_MS)
    {
        confirmed_key = pending_key;
    }

    return confirmed_key;
}