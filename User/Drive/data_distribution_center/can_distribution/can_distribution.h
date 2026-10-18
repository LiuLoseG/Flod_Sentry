#pragma once

#include "bsp_can.h"

// 单条can分发层最大注册数量
#define CAN_DEVICE_REGISTER_MAX 8

// 统一CAN接收分发回调函数类型
typedef void (*can_rx_client_t)(const volatile FDCAN_RxFrame_TypeDef *frame);

// 统一CAN接收分发注册表结构体
typedef struct
{
    uint32_t can_rx_id;                     // CAN RX ID
    can_rx_client_t rx_client;              // CAN接收对象回调函数指针
} can_rx_client_table_t;

/**
 * @brief can分发层注册函数
 * @param hcan_id CAN总线号，1/2/3
 * @param can_rx_id 要匹配的标准帧ID
 * @param rx_client 匹配后调用的接收处理函数（须为ISR安全的纯数据解析）
 * @note 由各业务模块在init阶段调用，中断里由fdcanX_rx_callback查表转发
 */
void can_distribution_register(uint8_t hcan_id, uint32_t can_rx_id, can_rx_client_t rx_client);

