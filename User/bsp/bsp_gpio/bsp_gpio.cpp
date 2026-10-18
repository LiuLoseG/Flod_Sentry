#include "bsp_gpio.h"


void bsp_gpio_init(gpio_config_t *gpio, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state, gpio_exti_mode exti_mode, void (*callback)(void))
{
    gpio->port = port;
    gpio->pin = pin;
    gpio->state = state;
    gpio->exti_mode = exti_mode;
    gpio->callback = callback;

    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = (exti_mode == GPIO_Exti_Mode_None) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_IT_RISING_FALLING; // 根据中断模式设置GPIO模式
    GPIO_InitStruct.Pull = GPIO_NOPULL; // 不使用上拉或下拉电阻
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // 设置初始状态
    HAL_GPIO_WritePin(port, pin, state);

    // 配置中断
    if (exti_mode != GPIO_Exti_Mode_None)
    {
        uint32_t irq_number;

        // 根据引脚号选择对应的中断号
        switch (pin)
        {
            case GPIO_PIN_0: irq_number = EXTI0_IRQn; break;
            case GPIO_PIN_1: irq_number = EXTI1_IRQn; break;
            case GPIO_PIN_2: irq_number = EXTI2_IRQn; break;
            case GPIO_PIN_3: irq_number = EXTI3_IRQn; break;
            case GPIO_PIN_4: irq_number = EXTI4_IRQn; break;
            case GPIO_PIN_5: irq_number = EXTI9_5_IRQn; break;
            case GPIO_PIN_6: irq_number = EXTI9_5_IRQn; break;
            case GPIO_PIN_7: irq_number = EXTI9_5_IRQn; break;
            case GPIO_PIN_8: irq_number = EXTI9_5_IRQn; break;
            case GPIO_PIN_9: irq_number = EXTI9_5_IRQn; break;
            case GPIO_PIN_10: irq_number = EXTI15_10_IRQn; break;
            case GPIO_PIN_11: irq_number = EXTI15_10_IRQn; break;
            case GPIO_PIN_12: irq_number = EXTI15_10_IRQn; break;
            case GPIO_PIN_13: irq
        }
    }
}

// ----------------- GPIO API -----------------
// 都是对HAL的形式上的封装

void bsp_gpio_toggle(gpio_config_t *gpio)
{
    HAL_GPIO_TogglePin(gpio->port, gpio->pin);
}

void bsp_gpio_set(gpio_config_t *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_SET);
}

void bsp_gpio_reset(gpio_config_t *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_RESET);
}

GPIO_PinState bsp_gpio_read(gpio_config_t *gpio)
{
    return HAL_GPIO_ReadPin(gpio->port, gpio->pin);
}