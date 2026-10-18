/**
 * @file absolute_encoder.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 单圈 / 多圈绝对值编码器 CAN2.0B 通信协议
 * @note  基于 CAN2.0B 标准帧（11-bit ID），默认波特率 500kbps。
 *        协议帧格式: [LEN][设备ID][指令FUNC][数据DATA...]
 *        CAN 标识符 = 设备ID（默认 0x01）。
 *
 *        单圈编码器: 掉电后仅记住 0~360° 内位置，超过 360° 从 0 重新计数。
 *        多圈编码器: 如钟表"分针+时针"，掉电后同时记住圈数 + 圈内位置，
 *                    超过指定圈数后从 0 重新计数。
 * @date 2026-07-28
 * @version 0.4
 * @copyright HNAU-Robot (c) 2026
 */
#pragma once

#include "bsp_can.h"
#include <cstdint>

/*=========================================== 指令码枚举 ===========================================*/

/** @brief 绝对值编码器 CAN 指令码 */
typedef enum
{
    ABS_ENCODER_FUNC_READ       = 0x01,     // 读取编码器值（返回32位无符号数）
    ABS_ENCODER_FUNC_SET_ID     = 0x02,     // 设置编码器设备ID（0~255）
    ABS_ENCODER_FUNC_SET_BAUD   = 0x03,     // 设置 CAN 通信波特率
    ABS_ENCODER_FUNC_SET_MODE   = 0x04,     // 设置编码器工作模式
} abs_encoder_func_e;

/*=========================================== 波特率枚举 ===========================================*/

/** @brief 编码器 CAN 波特率（修改后需重新上电生效） */
typedef enum
{
    ABS_ENCODER_BAUD_500K   = 0x00,     // 500 kbps（默认）
    ABS_ENCODER_BAUD_1M     = 0x01,     // 1 Mbps
    ABS_ENCODER_BAUD_250K   = 0x02,     // 250 kbps
    ABS_ENCODER_BAUD_125K   = 0x03,     // 125 kbps
    ABS_ENCODER_BAUD_100K   = 0x04,     // 100 kbps
} abs_encoder_baud_e;

/*=========================================== 工作模式枚举 ===========================================*/

/** @brief 编码器工作模式 */
typedef enum
{
    ABS_ENCODER_MODE_QUERY   = 0x00,     // 查询模式：主机主动轮询
    ABS_ENCODER_MODE_AUTO    = 0xAA,     // 自动回发模式：编码器主动上报
} abs_encoder_mode_e;

/*=========================================== 编码器类型枚举 ===========================================*/

/** @brief 绝对值编码器类型（单圈 / 多圈） */
typedef enum
{
    ABS_ENCODER_TYPE_SINGLE_TURN = 0,     // 单圈：0~360°，超过则从 0 重新计数
    ABS_ENCODER_TYPE_MULTI_TURN  = 1,     // 多圈：圈内位置 + 圈数，超过指定圈数后归零
} abs_encoder_type_e;

/*=========================================== 编码器状态 ===========================================*/

/** @brief 编码器在线状态 */
typedef enum
{
    Abs_Encoder_Status_Disable = 0,     // 离线
    Abs_Encoder_Status_Enable  = 1,     // 在线
} abs_encoder_status_e;

/*=========================================== CAN 帧结构体 ===========================================*/

/**
 * @brief 编码器 CAN 帧结构（协议原始层）
 * @note  Data[0] = LEN（总字节数，包含 LEN 自身）
 *        Data[1] = 设备ID
 *        Data[2] = 指令码 FUNC
 *        Data[3..6] = 数据域（0~4字节，小端序）
 */
typedef struct __attribute__((packed))
{
    uint8_t len;        // Data[0]: 帧总长度 (3 + data_bytes)
    uint8_t dev_id;     // Data[1]: 设备地址 (0~255)
    uint8_t func;       // Data[2]: 指令码
    uint8_t data[5];    // Data[3~7]: 数据域 + 填充
} abs_encoder_frame_t;

/*=========================================== 编码器处理后的数据 ===========================================*/

/**
 * @brief 编码器解析后的数据
 */
typedef struct
{
    uint32_t raw_encoder;       // 原始编码器值（32位无符号）
    float    angle_rad;         // 当前角度 (rad), [0, 2π)
    float    angle_deg;         // 当前角度 (°),   [0, 360)
    // ---- 多圈编码器扩展字段 ----
    uint16_t turns;             // 当前圈数（多圈模式有效）
    float    total_angle_rad;   // 累计总角度 (rad)，含圈数（多圈模式有效）
    float    total_angle_deg;   // 累计总角度 (°)，含圈数（多圈模式有效）
} abs_encoder_processed_data_t;

/*=========================================== 绝对值编码器类 ===========================================*/

/**
 * @brief 单圈/多圈绝对值编码器 CAN 通信类
 *
 * @note  协议采用 CAN2.0B 标准帧，CAN 标识符 = 设备ID。
 *        默认: CAN ID=0x01, 波特率=500kbps, 查询模式, 单圈类型。
 *
 *        单圈模式: raw_encoder 直接映射到 [0, 360°)。
 *        多圈模式: 需调用 set_multi_turn(total_turns, single_resolution_bits)
 *                  配置总圈数和单圈分辨率位数，然后 raw_encoder 被拆分为
 *                  [圈数: 高位] + [圈内位置: 低位]。
 *
 *        典型用法:
 *        @code
 *        abs_encoder_class yaw_encoder;
 *        yaw_encoder.init(&hfdcan1, 0x01);
 *
 *        // 多圈编码器: 4096 圈, 单圈 17 位分辨率
 *        yaw_encoder.set_encoder_type(ABS_ENCODER_TYPE_MULTI_TURN);
 *        yaw_encoder.set_multi_turn(4096, 17);
 *
 *        // 设置为自动回发模式（可选）
 *        yaw_encoder.set_mode(ABS_ENCODER_MODE_AUTO);
 *
 *        // CAN 接收中断中调用
 *        void HAL_FDCAN_RxFifo0Callback(...) {
 *            fdcan_receive_data(...);
 *            yaw_encoder.can_rx_callback();
 *        }
 *
 *        // 主循环中
 *        yaw_encoder.alive_check();
 *        if (yaw_encoder.get_status() == Abs_Encoder_Status_Enable) {
 *            float angle = yaw_encoder.get_angle_rad();
 *            float total = yaw_encoder.get_total_angle_deg();
 *        }
 *        @endcode
 */
class abs_encoder_class
{
public:
    // ==================== 初始化 ====================

    /**
     * @brief 初始化编码器
     * @param hcan   FDCAN 句柄指针
     * @param can_id 编码器 CAN 标识符（= 设备ID，默认 0x01）
     */
    void init(const FDCAN_HandleTypeDef *__hcan, const uint8_t &__can_id = 0x01);

    // ==================== CAN 通信回调 ====================

    /** @brief CAN 接收回调，在 HAL_FDCAN_RxFifo0Callback / RxFifo1Callback 中调用 */
    void can_rx_callback(void);

    /** @brief 编码器存活检测，在主循环中周期性调用 */
    void alive_check(void);

    // ==================== 读取指令 ====================

    /** @brief 发送读取编码器值命令 (FUNC=0x01) */
    void read_encoder(void);

    /*--------------------------------设置编码器参数------------------------------------------------------- */

    /**
     * @brief 设置编码器设备ID (FUNC=0x02)
     * @param new_id 新设备ID (0~255)
     */
    void set_device_id(uint8_t new_id);

    /**
     * @brief 设置 CAN 通信波特率 (FUNC=0x03)
     * @param baud 波特率枚举值
     * @note  修改后需重新上电才生效！主机也应同步修改为相同波特率。
     */
    void set_baud_rate(abs_encoder_baud_e baud);

    /**
     * @brief 设置编码器工作模式 (FUNC=0x04)
     * @param mode ABS_ENCODER_MODE_QUERY(0x00)=查询, ABS_ENCODER_MODE_AUTO(0xAA)=自动回发
     */
    void set_mode(abs_encoder_mode_e mode);

    /*--------------------------------多圈编码器配置-------------------------------------------------------*/

    /**
     * @brief 设置编码器类型（单圈 / 多圈）
     * @param type ABS_ENCODER_TYPE_SINGLE_TURN / ABS_ENCODER_TYPE_MULTI_TURN
     */
    void set_encoder_type(abs_encoder_type_e type);

    /**
     * @brief 配置多圈编码器参数
     * @param total_turns          总圈数（超过后从 0 重新计数）
     * @param single_resolution_bits 单圈分辨率位数（如 17 表示单圈 2^17=131072 个位置）
     * @note  调用后自动设置编码器类型为 ABS_ENCODER_TYPE_MULTI_TURN
     */
    void set_multi_turn(uint16_t total_turns, uint8_t single_resolution_bits);

/*-----------------------------------编码器数据获取-------------------------------------------------------*/

    /** @brief 获取在线状态 */
    inline abs_encoder_status_e get_status(void) const;

    /** @brief 获取当前角度 (rad), [0, 2π) */
    inline float get_angle_rad(void) const;

    /** @brief 获取当前角度 (°), [0, 360) */
    inline float get_angle_deg(void) const;

    /** @brief 获取原始编码器值（32位无符号） */
    inline uint32_t get_raw_encoder(void) const;

    /** @brief 获取最近一次设置命令的结果码（0x00=成功） */
    inline uint8_t get_cmd_result(void) const;

    /** @brief 获取当前设备 CAN ID */
    inline uint8_t get_can_id(void) const;

    // ---- 多圈编码器数据获取 ----

    /** @brief 获取当前圈数（多圈模式有效） */
    inline uint16_t get_turns(void) const;

    /** @brief 获取累计总角度 (rad)，含圈数（多圈模式有效） */
    inline float get_total_angle_rad(void) const;

    /** @brief 获取累计总角度 (°)，含圈数（多圈模式有效） */
    inline float get_total_angle_deg(void) const;

private:
    // ==================== 硬件绑定 ====================
    volatile FDCAN_RxFrame_TypeDef *can_rx_frame;  // 绑定的 CAN 接收帧
    FDCAN_HandleTypeDef *hcan;                      // FDCAN 句柄
    uint8_t  can_id;                                // 当前 CAN 标识符

    // ==================== 内部变量 ====================
    uint32_t flag     = 0;       // 当前时刻接收 flag
    uint32_t pre_flag = 0;       // 前一时刻接收 flag

    // ==================== 数据 ====================
    abs_encoder_status_e         encoder_status;     // 编码器在线状态
    abs_encoder_processed_data_t rx_data;            // 编码器处理后的数据
    uint8_t cmd_result = 0;                         // 最近一次设置命令结果码

    // ==================== 多圈编码器配置 ====================
    abs_encoder_type_e encoder_type = ABS_ENCODER_TYPE_SINGLE_TURN;  // 编码器类型
    uint16_t multi_turn_total      = 0;      // 多圈总圈数（0=未配置/单圈模式）
    uint8_t  multi_turn_single_bits = 0;     // 单圈分辨率位数
    uint32_t multi_turn_single_mask = 0;     // 单圈位置掩码（由位数计算）
    uint32_t multi_turn_shift       = 0;     // 圈数位移（= 单圈位数）

    // ==================== 内部函数 ====================

    /** @brief 数据处理，从 can_rx_frame_->data 解析编码器数据 */
    void data_process(void);

    /** @brief 构造并发送 CAN 帧 */
    void send_frame(abs_encoder_func_e func, const uint8_t *tx_data, uint8_t data_len);

    /** @brief 解析读取响应帧 */
    void parse_read_response(const uint8_t *data);

    /** @brief 多圈模式: 从 raw_encoder 拆分圈数和圈内位置 */
    void parse_multi_turn(void);

    /** @brief 解析设置类响应帧 */
    void parse_set_response(const uint8_t *data);

    /** @brief 原始编码器值转弧度 */
    static float encoder_to_rad(uint32_t raw);

    /** @brief 原始编码器值转角度 */
    static float encoder_to_deg(uint32_t raw);
};

/*=========================================== 内联函数实现 ===========================================*/

/**
 * @brief 获取在线状态
 * @return abs_encoder_status_e 在线状态
 */
inline abs_encoder_status_e abs_encoder_class::get_status(void) const
{
    return (encoder_status);
}

/**
 * @brief 获取当前角度 (rad)
 * @return float 当前角度, rad
 */
inline float abs_encoder_class::get_angle_rad(void) const
{
    return (rx_data.angle_rad);
}

/**
 * @brief 获取当前角度 (°)
 * @return float 当前角度, °
 */
inline float abs_encoder_class::get_angle_deg(void) const
{
    return (rx_data.angle_deg);
}

/**
 * @brief 获取原始编码器值
 * @return uint32_t 原始32位编码器值
 */
inline uint32_t abs_encoder_class::get_raw_encoder(void) const
{
    return (rx_data.raw_encoder);
}

/**
 * @brief 获取最近一次设置命令的结果码
 * @return uint8_t 0x00=成功, 非0=错误码
 */
inline uint8_t abs_encoder_class::get_cmd_result(void) const
{
    return (cmd_result);
}

/**
 * @brief 获取当前设备 CAN ID
 * @return uint8_t CAN ID
 */
inline uint8_t abs_encoder_class::get_can_id(void) const
{
    return (can_id);
}

/*=========================================== 多圈编码器内联 getter ===========================================*/

/**
 * @brief 获取当前圈数（多圈模式有效）
 * @return uint16_t 圈数
 */
inline uint16_t abs_encoder_class::get_turns(void) const
{
    return (rx_data.turns);
}

/**
 * @brief 获取累计总角度 (rad)，含圈数（多圈模式有效）
 * @return float 累计总角度, rad
 */
inline float abs_encoder_class::get_total_angle_rad(void) const
{
    return (rx_data.total_angle_rad);
}

/**
 * @brief 获取累计总角度 (°)，含圈数（多圈模式有效）
 * @return float 累计总角度, °
 */
inline float abs_encoder_class::get_total_angle_deg(void) const
{
    return (rx_data.total_angle_deg);
}