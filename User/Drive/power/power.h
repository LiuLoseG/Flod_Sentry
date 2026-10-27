/**
 * @file power.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 功率计通信协议 —— 纯二进制，小端，定长 15 字节
 *        帧格式: [AA] [电压 float32] [电流 float32] [功率 float32] [校验和 uint8] [55]
 *        校验和: 帧头 + 电压 + 电流 + 功率 共 13 字节求和
 * @date 2026-09-26
 * @version 0.3
 * @copyright HNAU-Robot (c) 2026
 */

#pragma once

#include <stdint.h>
#include "bsp_uart.h"

/* ---------- 协议常量 ---------- */

#define POWER_HEADER          0xAAu     /**< 帧头 */
#define POWER_TAIL            0x55u     /**< 帧尾 */
#define POWER_FRAME_LENGTH    15u       /**< 整帧字节数 */
#define POWER_CHECKSUM_LEN    13u       /**< 校验和覆盖的字节数（头+3个float） */

/* ---------- 数据结构 ---------- */

/**
 * @brief 功率计解析结果（浮点值，可直接使用）
 */
typedef struct
{
    float voltage;    /**< 电压, V */
    float current;    /**< 电流, A */
    float power;      /**< 功率, W */
} power_data_t;

extern volatile power_data_t power_data;

/**
 * @brief 解析功率计数据（从 UART DMA 接收缓冲区调用）
 * @param buf 数据缓冲区指针
 * @param len 数据长度
 */
void power_data_analysis(uint8_t *buf, uint16_t len);
