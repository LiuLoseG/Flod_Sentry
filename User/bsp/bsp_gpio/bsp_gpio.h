#pragma once

#include "gpio.h"
#include "stdint.h"
#include <stdint.h>

/**
 * @brief GPIO中断触发模式
 */
typedef enum
{
    GPIO_Exti_Mode_Rising = 0,              // 上升沿
    GPIO_Exti_Mode_Falling,                 // 下降沿
    GPIO_Exti_Mode_Rising_Falling,          // 双边沿
    GPIO_Exti_Mode_None,                    // 无中断
} gpio_exti_mode;

/**
 * @brief GPIO状态结构体
 */
typedef struct gpio_status
{
    GPIO_TypeDef *port;                     // GPIO端口
    uint16_t pin;                           // GPIO引脚
    GPIO_PinState state;                    // GPIO状态

    gpio_exti_mode exti_mode;               // 中断触发模式
    void (*callback)(void);                 // 中断回调函数
} gpio_config_t;


