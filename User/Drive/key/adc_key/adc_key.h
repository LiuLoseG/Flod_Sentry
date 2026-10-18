#pragma once

#include "bsp_adc.h"

// 消抖时间（毫秒），使用dwt定时器,与任务调用频率无关
#define ADC_KEY_DEBOUNCE_MS  20U

typedef enum
{
    LCD_KEY_NONE = 0,
    LCD_KEY_RIGHT,
    LCD_KEY_LEFT,
    LCD_KEY_UP,
    LCD_KEY_DOWN,
    LCD_KEY_PRESS
} lcd_key_t;

lcd_key_t get_lcd_key_state(void);

