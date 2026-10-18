#pragma once

#include "main.h"
#include "usart.h"
#include "bsp_dwt.h"
#include <string.h>
#include "stdbool.h"

#define UART_BUFFER_SIZE 256U

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*uart_callback)(uint8_t *data, uint16_t length);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *uart_handler;
    uart_callback callback_function;

    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t rx_buffer_0[UART_BUFFER_SIZE];
    uint8_t rx_buffer_1[UART_BUFFER_SIZE];
    // 正在接收的缓冲区
    uint8_t *rx_buffer_active;
    // 接收完毕的缓冲区
    uint8_t *rx_buffer_ready;

    // 接收时间戳
    uint64_t rx_time_stamp;
} uart_manger_object;

extern bool init_finished;

extern uart_manger_object uart1_manager;
extern uart_manger_object uart2_manager;
extern uart_manger_object uart3_manager;
extern uart_manger_object uart4_manager;
extern uart_manger_object uart5_manager;
extern uart_manger_object uart7_manager;
extern uart_manger_object uart10_manager;

void bsp_uart_init(UART_HandleTypeDef *huart, uart_callback callback);
uint8_t bsp_uart_transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
