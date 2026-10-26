#include "ws2812.h"

#include "bsp_spi.hpp"
#include "cmsis_os2.h"
#include "stm32h7xx_hal_gpio.h"
#include <cmath>
#include <cstring>
#include <cstdint>

#include "bsp_uart.h"
#include "usart.h"

// #include "servo_drive.h"



// 纯黑
const ws2812_color WS2812_COLOR_BLACK = {0, 0, 0};
// 纯白
const ws2812_color WS2812_COLOR_WHITE = {255, 255, 255};

// 纯红
const ws2812_color WS2812_COLOR_RED = {255, 0, 0};
// 纯绿
const ws2812_color WS2812_COLOR_GREEN = {0, 255, 0};
// 纯蓝
const ws2812_color WS2812_COLOR_BLUE = {0, 0, 255};

// 黄色
const ws2812_color WS2812_COLOR_YELLOW = {255, 255, 0};
// 青色
const ws2812_color WS2812_COLOR_CYAN = {0, 255, 255};
// 品红
const ws2812_color WS2812_COLOR_MAGENTA = {255, 0, 255};

// servo_class servo1;
// servo_class servo2;

/**
 * @brief WS2812单个灯珠初始化, 默认灯灭
 *
 * @param __red 红色分量
 * @param __green 绿色分量
 * @param __blue 蓝色分量
 */

void ws2812_class::color_init(const uint8_t & __red, const uint8_t & __green, const uint8_t & __blue)
{
    spi_manager_object_ = &spi6_manager_object; // 绑定SPI6

    color_.red = __red;
    color_.green = __green;
    color_.blue = __blue;
}

/**
 * @brief 发送当前颜色到 WS2812 灯珠
 */
bool ws2812_class::ws2812_write_color(void)
{
    if (spi_manager_object_ == nullptr)
    {
        return false;
    }

    uint8_t *tmp_buffer = spi_manager_object_->get_tx_buffer();
    if (tmp_buffer == nullptr)
    {
        return false;
    }

    memset(tmp_buffer, 0, total_bytes); // 只清要发的 total_bytes 字节, 帧尾天然为低电平

    for (uint8_t i = 0; i < 8; i++) 
    {
        tmp_buffer[7 - i] = (color_.green & (1 << i)) ? LEVEL_1_ : LEVEL_0_;
        tmp_buffer[15 - i] = (color_.red & (1 << i)) ?  LEVEL_1_ : LEVEL_0_;
        tmp_buffer[23 - i] = (color_.blue & (1 << i)) ? LEVEL_1_ : LEVEL_0_;
    }

    const uint8_t status = bsp_spi_transmit_data(spi_manager_object_->get_spi_handler(), nullptr, 0, GPIO_PIN_SET, total_bytes);
    return (status == 0U);
}

// void ws2812_color_flicker(void)
// {
//     static uint16_t mod10 = 0;
//     mod10++;
//     if (mod10 == 10)
//     {
//         mod10 = 0;

//         if (red >= 18)
//         {
//             red_minus_flag = true;
//         }
//         else if (red == 0)
//         {
//             red_minus_flag = false;
//         }
//         if (green >= 18)
//         {
//             green_minus_flag = true;
//         }
//         else if (green == 0)
//         {
//             green_minus_flag = false;
//         }
//         if (blue >= 18)
//         {
//             blue_minus_flag = true;
//         }
//         else if (blue == 0)
//         {
//             blue_minus_flag = false;
//         }

//         if (red_minus_flag)
//         {
//             red--;
//         }
//         else
//         {
//             red++;
//         }
//         if (green_minus_flag)
//         {
//             green--;
//         }
//         else
//         {
//             green++;
//         }
//         if (blue_minus_flag)
//         {
//             blue--;
//         }
//         else
//         {
//             blue++;
//         }

//         ws2812.set_rgb(red, green, blue);
//         ws2812.ws2812_write_color();
//     }
// }

// extern "C" void ws2812_task(void *argument)
// {

//     bsp_spi_init(&hspi6, nullptr); // 初始化SPI6
//     osDelay(100); // 延时100ms
//     // bsp_uart_init(&huart10, user_uart_callback, 1); // 初始化UART10
//     ws2812.color_init(0, 0, 0); // 初始化为红色
//     servo_init(); // 初始化舵机

//     for (;;) 
//     {
//         // uart_send_data(&huart10, (uint8_t*)"WS2812 Task Running\r\n", 22); // 发送调试信息
//         ws2812_color_flicker();
//         servo_control(angle1, angle2); // 控制舵机1转到90度，舵机2转到270度
//         osDelay(1); // 延时100ms
//     }
// }
