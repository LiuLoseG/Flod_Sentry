/**
 * @file absolute_encoder.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 单圈 / 多圈绝对值编码器 CAN2.0B 通信协议实现
 * @date 2026-07-28
 * @version 0.4
 * @copyright HNAU-Robot (c) 2026
 */
#include "absolute_encoder.h"

#include <cstring>

/*=========================================== 常量 ===========================================*/

#define M_TWOPI         6.283185307179586f       // 2π
#define M_360_DEG       360.0f
#define M_UINT32_MAX_F  4294967296.0f            // 2^32

/*=========================================== 初始化 ===========================================*/

/**
 * @brief 初始化编码器
 * @param hcan   FDCAN 句柄指针
 * @param can_id 编码器 CAN 标识符（= 设备ID，默认 0x01）
 */
void abs_encoder_class::init(const FDCAN_HandleTypeDef *__hcan, const uint8_t &__can_id)
{
    if (__hcan == nullptr)
    {
        return;
    }

    // 绑定 CAN 接收帧
    if (__hcan->Instance == FDCAN1)
    {
        can_rx_frame = &fdcan1_rxframe;
    }
    else if (__hcan->Instance == FDCAN2)
    {
        can_rx_frame = &fdcan2_rxframe;
    }
    else if (__hcan->Instance == FDCAN3)
    {
        can_rx_frame = &fdcan3_rxframe;
    }

    hcan  = const_cast<FDCAN_HandleTypeDef *>(__hcan);
    can_id = __can_id;

    encoder_status = Abs_Encoder_Status_Disable;
    memset(&rx_data, 0, sizeof(rx_data));
}

/*=========================================== CAN 接收回调 ===========================================*/

/**
 * @brief CAN 接收回调，在 HAL_FDCAN_RxFifo0Callback / RxFifo1Callback 中调用
 * @note  不在此处按 Identifier 过滤，因为 SET_ID(0x02) 的回复帧使用新 ID 作为
 *        CAN 标识符，若过滤会丢弃该回复。具体帧校验在 data_process() 中完成。
 */
void abs_encoder_class::can_rx_callback(void)
{
    // 滑动窗口, 判断编码器是否在线
    flag += 1;

    data_process();
}

/*=========================================== 存活检测 ===========================================*/

/**
 * @brief 编码器存活检测，在主循环中周期性调用
 */
void abs_encoder_class::alive_check(void)
{
    // 判断该时间段内是否接收过编码器数据
    if (flag == pre_flag)
    {
        // 编码器断开连接
        encoder_status = Abs_Encoder_Status_Disable;
    }
    else
    {
        // 编码器连接正常
        encoder_status = Abs_Encoder_Status_Enable;
    }
    pre_flag = flag;
}

/*=========================================== 读取指令 ===========================================*/

/**
 * @brief 发送读取编码器值命令 (FUNC=0x01)
 * @note  下发: [LEN=4][DEV_ID][FUNC=0x01][DATA=0x00]
 *        返回: [LEN=7][DEV_ID][FUNC=0x01][DATA0..DATA3]（32位小端序）
 */
void abs_encoder_class::read_encoder(void)
{
    uint8_t dummy = 0x00;
    send_frame(ABS_ENCODER_FUNC_READ, &dummy, 1);
}

/*=========================================== 设置指令 ===========================================*/

/**
 * @brief 设置编码器设备ID (FUNC=0x02)
 * @param new_id 新设备ID (0~255)
 * @note  下发: [LEN=4][DEV_ID][FUNC=0x02][新ID]
 *        返回: [LEN=4][新ID][FUNC=0x02][结果]  0=成功, 非0=错误码
 */
void abs_encoder_class::set_device_id(uint8_t new_id)
{
    send_frame(ABS_ENCODER_FUNC_SET_ID, &new_id, 1);
}

/**
 * @brief 设置 CAN 通信波特率 (FUNC=0x03)
 * @param baud 波特率枚举值
 * @note  下发: [LEN=4][DEV_ID][FUNC=0x03][波特率码]
 *        修改后需重新上电才生效！主机也应同步修改为相同波特率。
 */
void abs_encoder_class::set_baud_rate(abs_encoder_baud_e baud)
{
    uint8_t baud_code = static_cast<uint8_t>(baud);
    send_frame(ABS_ENCODER_FUNC_SET_BAUD, &baud_code, 1);
}

/**
 * @brief 设置编码器工作模式 (FUNC=0x04)
 * @param mode ABS_ENCODER_MODE_QUERY(0x00)=查询, ABS_ENCODER_MODE_AUTO(0xAA)=自动回发
 * @note  下发: [LEN=4][DEV_ID][FUNC=0x04][模式]
 *        返回: [LEN=4][DEV_ID][FUNC=0x04][结果]
 */
void abs_encoder_class::set_mode(abs_encoder_mode_e mode)
{
    uint8_t mode_code = static_cast<uint8_t>(mode);
    send_frame(ABS_ENCODER_FUNC_SET_MODE, &mode_code, 1);
}

/*=========================================== 多圈编码器配置 ===========================================*/

/**
 * @brief 设置编码器类型（单圈 / 多圈）
 * @param type ABS_ENCODER_TYPE_SINGLE_TURN / ABS_ENCODER_TYPE_MULTI_TURN
 */
void abs_encoder_class::set_encoder_type(abs_encoder_type_e type)
{
    encoder_type = type;

    // 切回单圈时清空多圈配置
    if (type == ABS_ENCODER_TYPE_SINGLE_TURN)
    {
        multi_turn_total      = 0;
        multi_turn_single_bits = 0;
        multi_turn_single_mask = 0;
        multi_turn_shift       = 0;
    }
}

/**
 * @brief 配置多圈编码器参数
 * @param total_turns          总圈数（超过后从 0 重新计数）
 * @param single_resolution_bits 单圈分辨率位数（如 17 表示单圈 2^17=131072 个位置）
 * @note  调用后自动设置编码器类型为 ABS_ENCODER_TYPE_MULTI_TURN
 *
 *        数据手册建议: 选择圈数稍大于实际需要测量的圈数，这样在整个行程范围内
 *        任意位置掉电（或掉电后转动），都能正确记忆位置。
 */
void abs_encoder_class::set_multi_turn(uint16_t total_turns, uint8_t single_resolution_bits)
{
    if (total_turns == 0 || single_resolution_bits == 0 || single_resolution_bits > 31)
    {
        return;  // 参数非法
    }

    encoder_type            = ABS_ENCODER_TYPE_MULTI_TURN;
    multi_turn_total        = total_turns;
    multi_turn_single_bits  = single_resolution_bits;
    multi_turn_single_mask  = (1UL << single_resolution_bits) - 1;
    multi_turn_shift        = single_resolution_bits;
}

/*=========================================== 数据处理 ===========================================*/

/**
 * @brief 数据处理，从 can_rx_frame_->data 解析编码器数据
 */
void abs_encoder_class::data_process(void)
{
    const uint8_t *rx_buf = can_rx_frame->data;

    // 帧格式: Data[0]=LEN, Data[1]=DEV_ID, Data[2]=FUNC, Data[3..]=DATA
    uint8_t len    = rx_buf[0];
    uint8_t dev_id = rx_buf[1];
    uint8_t func   = rx_buf[2];

    // 基本校验: LEN 合法性
    if (len < 3 || len > 8) return;

    // CAN 标识符 / 数据域 DEV_ID 双重校验
    // SET_ID(0x02) 回复帧: CAN 标识符=新ID, Data[1]=新ID, 两者均不等于旧 can_id
    // 因此对 SET_ID 仅校验 CAN 标识符与 DATA[1] 一致, 不校验与本地 can_id 的关系
    if (func == ABS_ENCODER_FUNC_SET_ID)
    {
        // SET_ID 回复: Identifier 应与 Data[1]（新ID）一致
        if (can_rx_frame->header.Identifier != dev_id) return;
    }
    else
    {
        // 其他帧: CAN 标识符和 Data[1] 均需匹配本地 can_id
        if (can_rx_frame->header.Identifier != can_id) return;
        if (dev_id != can_id) return;
    }

    // 根据指令码分派解析
    switch (func)
    {
        case ABS_ENCODER_FUNC_READ:
            parse_read_response(&rx_buf[3]);
            break;

        case ABS_ENCODER_FUNC_SET_ID:
            // 仅结果码为 0x00 时更新本地 ID
            if (rx_buf[3] == 0x00)
            {
                can_id = dev_id;
            }
            parse_set_response(&rx_buf[3]);
            break;

        case ABS_ENCODER_FUNC_SET_BAUD:
        case ABS_ENCODER_FUNC_SET_MODE:
            parse_set_response(&rx_buf[3]);
            break;

        default:
            break;
    }
}

/*=========================================== 内部：发送帧 ===========================================*/

/**
 * @brief 构造并发送 CAN 帧
 * @param func     指令码
 * @param tx_data  待发送数据（1~4字节）
 * @param data_len 数据字节数（0~4）
 */
void abs_encoder_class::send_frame(abs_encoder_func_e func,
                                   const uint8_t *tx_data, uint8_t data_len)
{
    if (data_len > 4) return;

    uint8_t frame[8] = {0};

    // Data[0] = LEN = 1(LEN) + 1(DEV_ID) + 1(FUNC) + data_len
    uint8_t total_len = 3 + data_len;
    frame[0] = total_len;
    // Data[1] = 设备ID
    frame[1] = can_id;
    // Data[2] = 指令码
    frame[2] = static_cast<uint8_t>(func);
    // Data[3..] = 数据域（小端序）
    if (tx_data != nullptr && data_len > 0)
    {
        memcpy(&frame[3], tx_data, data_len);
    }

    // 按实际帧长度发送（total_len ≤ 8），不填充至 8 字节
    bsp_fdcan_send(hcan, can_id, frame, total_len);
}

/*=========================================== 内部：响应解析 ===========================================*/

/**
 * @brief 解析读取响应帧 (FUNC=0x01)
 * @note  响应: [LEN=7][DEV_ID][FUNC=0x01][DATA0..DATA3]（4字节小端序）
 *        示例: [07][01][01][45][23][01][00] → 编码器值 = 0x00012345
 */
void abs_encoder_class::parse_read_response(const uint8_t *data)
{
    // 解析 32 位编码器值（小端序：data[0] 最低字节）
    rx_data.raw_encoder = static_cast<uint32_t>(data[0])
                        | (static_cast<uint32_t>(data[1]) << 8)
                        | (static_cast<uint32_t>(data[2]) << 16)
                        | (static_cast<uint32_t>(data[3]) << 24);

    if (encoder_type == ABS_ENCODER_TYPE_MULTI_TURN && multi_turn_total > 0)
    {
        // 多圈模式: 从 raw_encoder 中拆分圈数和圈内位置
        parse_multi_turn();
    }
    else
    {
        // 单圈模式: raw_encoder 直接映射到 [0, 360°)
        rx_data.angle_deg = encoder_to_deg(rx_data.raw_encoder);
        rx_data.angle_rad = encoder_to_rad(rx_data.raw_encoder);

        // 单圈模式下多圈字段清零
        rx_data.turns = 0;
        rx_data.total_angle_deg = rx_data.angle_deg;
        rx_data.total_angle_rad = rx_data.angle_rad;
    }

    // NaN/inf 运行时安全钳位
    if (rx_data.angle_rad != rx_data.angle_rad ||
        rx_data.angle_rad > 1e10f || rx_data.angle_rad < -1e10f)
        rx_data.angle_rad = 0.0f;
    if (rx_data.angle_deg != rx_data.angle_deg ||
        rx_data.angle_deg > 1e10f || rx_data.angle_deg < -1e10f)
        rx_data.angle_deg = 0.0f;
    if (rx_data.total_angle_rad != rx_data.total_angle_rad ||
        rx_data.total_angle_rad > 1e10f || rx_data.total_angle_rad < -1e10f)
        rx_data.total_angle_rad = 0.0f;
    if (rx_data.total_angle_deg != rx_data.total_angle_deg ||
        rx_data.total_angle_deg > 1e10f || rx_data.total_angle_deg < -1e10f)
        rx_data.total_angle_deg = 0.0f;
}

/**
 * @brief 多圈模式: 从 raw_encoder 拆分圈数和圈内位置
 * @note  raw_encoder 位域布局（由硬件决定）:
 *        [31 .. single_bits] = 圈数（高位）
 *        [single_bits-1 .. 0] = 圈内位置（低位）
 *
 *        例如: 单圈 17 位分辨率 + 15 位圈数 = 32 位
 *        raw_encoder[16:0]  → 圈内位置 (0 ~ 131071)
 *        raw_encoder[31:17] → 圈数      (0 ~ 32767)
 */
void abs_encoder_class::parse_multi_turn(void)
{
    // 拆分圈数和圈内位置
    uint32_t single_pos = rx_data.raw_encoder & multi_turn_single_mask;
    uint32_t turn_count = (rx_data.raw_encoder >> multi_turn_shift) & 0xFFFF;

    rx_data.turns = static_cast<uint16_t>(turn_count);

    // 圈内角度: single_pos / 2^single_bits * 360°
    float single_max  = static_cast<float>(1UL << multi_turn_single_bits);
    rx_data.angle_deg = static_cast<float>(single_pos) * (M_360_DEG / single_max);
    rx_data.angle_rad = static_cast<float>(single_pos) * (M_TWOPI / single_max);

    // 累计总角度 = 圈数 * 360° + 圈内角度
    rx_data.total_angle_deg = static_cast<float>(turn_count) * M_360_DEG + rx_data.angle_deg;
    rx_data.total_angle_rad = static_cast<float>(turn_count) * M_TWOPI + rx_data.angle_rad;
}

/**
 * @brief 解析设置类响应帧 (FUNC=0x02/0x03/0x04)
 * @note  响应: [LEN=4][DEV_ID][FUNC][结果]
 *        结果: 0x00=成功, 非0=错误码
 */
void abs_encoder_class::parse_set_response(const uint8_t *data)
{
    // Data[3] = 结果码: 0x00=成功, 非0=错误码
    cmd_result = data[0];
}

/*=========================================== 静态工具函数 ===========================================*/

/**
 * @brief 原始编码器值转弧度
 * @param raw 原始32位编码器值
 * @return 角度 (rad), [0, 2π)
 */
float abs_encoder_class::encoder_to_rad(uint32_t raw)
{
    return static_cast<float>(raw) * (M_TWOPI / M_UINT32_MAX_F);
}

/**
 * @brief 原始编码器值转角度
 * @param raw 原始32位编码器值
 * @return 角度 (°), [0, 360)
 */
float abs_encoder_class::encoder_to_deg(uint32_t raw)
{
    return static_cast<float>(raw) * (M_360_DEG / M_UINT32_MAX_F);
}