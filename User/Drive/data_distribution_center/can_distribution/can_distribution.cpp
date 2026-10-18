/**
 * @file can_distribution.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 统一的can分发层，提供can总线接收数据的统一分发接口
 * @note  提供了一个统一的can分发层，允许各业务模块注册自己的can接收处理函数，并在can中断中进行分发。
        通过这种方式，各业务模块可以独立处理自己的can数据，而无需关心其他模块的实现细节。
        无需依赖任何业务模块，模块通过 can_distribution_register() 注册即可，彻底解耦。
 * @date 2026-08-20
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "can_distribution.h"

// 注册can分发层静态数组表
static can_rx_client_table_t can_rx_client_table[3][CAN_DEVICE_REGISTER_MAX] = {0}; // [0]=CAN1, [1]=CAN2, [2]=CAN3
static uint8_t can_rx_client_count[3] = {0}; // [0]=CAN1, [1]=CAN2, [2]=CAN3

/**
 * @brief can分发层注册函数
 * @note  该函数允许各业务模块注册自己的can接收处理函数，并在can中断中进行分发。
        通过这种方式，各业务模块可以独立处理自己的can数据，而无需关心其他模块的实现细节。
 * @param hcan_id CAN总线号，1表示CAN1，2表示CAN2，3表示CAN3
 * @param can_rx_id can接收ID
 * @param rx_client can接收回调函数指针
 */
void can_distribution_register(uint8_t hcan_id, uint32_t can_rx_id, can_rx_client_t rx_client)
{
    if (hcan_id < 1 || hcan_id > 3 || rx_client == NULL)
    {
        return; // 无效的CAN总线号或回调函数指针
    }

    uint8_t idx = hcan_id - 1;

    // 重复ID检测，防止同一ID被注册多次
    for (uint8_t i = 0; i < can_rx_client_count[idx]; i++)
    {
        if (can_rx_client_table[idx][i].can_rx_id == can_rx_id)
        {
            return; // 该ID已注册，直接返回
        }
    }

    if (can_rx_client_count[idx] >= CAN_DEVICE_REGISTER_MAX)
    {
        return; // 超过最大注册数量
    }

    // 将注册信息添加到静态数组表中
    can_rx_client_table[idx][can_rx_client_count[idx]++] = (can_rx_client_table_t){can_rx_id, rx_client};
}

/**
 * @brief can分发层分发函数
 * @note  该函数在can中断中被调用，用于将接收到的can数据分发给各个注册的处理函数。
 * @param idx can总线索引，0表示CAN1，1表示CAN2，2表示CAN3
 * @param frame 接收到的can帧
 */
static void can_distribution_dispatch(uint8_t idx, const volatile FDCAN_RxFrame_TypeDef *frame)
{
    if (frame == NULL)
    {
        return; // 空帧指针
    }

    for (uint8_t i = 0; i < can_rx_client_count[idx]; i++)  // 遍历注册表，查找匹配的can_rx_id
    {
        if (can_rx_client_table[idx][i].can_rx_id == frame->header.Identifier)
        {
            can_rx_client_table[idx][i].rx_client(frame); // 调用注册的回调函数
            return; // 同一帧只喂给一个注册者
        }
    }
}

// 绑定到CAN中断回调函数(强定义确保链接时使用该版本)
void fdcan1_rx_callback(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    can_distribution_dispatch(0, frame); // CAN1
}

void fdcan2_rx_callback(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    can_distribution_dispatch(1, frame); // CAN2
}

void fdcan3_rx_callback(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    can_distribution_dispatch(2, frame); // CAN3
}

