/**
 * @file drv_usb.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的USB通信初始化与配置流程
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/**
 * 双缓冲接收部分位于usbd_cdc_if.c中, 需要在usbd_cdc_if.c中, 如有需要可在此处修改
 * 具体函数为 static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
 * 此外, 为保证编译通过, user-include部分中, 需要额外声明 typedef int bool
 */

#pragma once

/* Includes ------------------------------------------------------------------*/
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define USB_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*usb_callback)(uint8_t *buffer, uint16_t length);

/**
 * @brief USB通信处理结构体
 */
struct struct_usb_manage_object
{
    UART_HandleTypeDef *uart_handler;
    usb_callback callback_function;

    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t rx_buffer_0[USB_BUFFER_SIZE];
    uint8_t rx_buffer_1[USB_BUFFER_SIZE];
    uint8_t *rx_buffer_active;
    uint8_t *rx_buffer_ready; // 另一个缓冲区, 用于接收数据时切换

    // 接收时间戳
    uint64_t rx_time_stamp;
};

/* Exported variables --------------------------------------------------------*/

extern bool init_finished;

extern struct struct_usb_manage_object usb0_manage_object;

/* Exported function declarations --------------------------------------------*/
// 本头文件被C(usbd_cdc_if.c)与C++文件共同包含, 需C链接保证跨语言调用
#ifdef __cplusplus
extern "C" {
#endif

void bsp_usb_init(usb_callback callback_function);

uint8_t bsp_usb_transmit(uint8_t *data, uint16_t length);

void bsp_usb_receive_callback(uint16_t size);

#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/