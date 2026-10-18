/**
 * @file can_board_communication.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief can板间通讯设计
 * @note 该模块实现了板间通信的发送和接收功能，支持多板间通信的注册和数据处理。通过该模块，各个板子可以通过CAN总线进行数据交换，实现协同工作。
 * @date 2026-09-08
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "can_board_communication.h"
#include "main.h"
#include "crc8_crc16.h"

void can_board_communication_class::init(const FDCAN_HandleTypeDef *hcan, const uint16_t &__can_id, const uint16_t &__master_id, const uint16_t &__transmit_frequency)
{
    if (hcan == nullptr)
    {
        return;
    }

    else if (hcan->Instance == FDCAN1)
    {
        can_rx_frame = &fdcan1_rxframe;
    }

    else if (hcan->Instance == FDCAN2)
    {
        can_rx_frame = &fdcan2_rxframe;
    }

    else if (hcan->Instance == FDCAN3)
    {
        can_rx_frame = &fdcan3_rxframe;
    }

    can_id = __can_id;
    master_id = __master_id;
    transmit_frequency = __transmit_frequency;

    // 初始化发送和接收缓冲区
    vx_control = 0.0f;
    vy_control = 0.0f;
    vz_control = 0.0f;
    rotation_matrix_yaw_control = 0.0f;

    memset(tx_can_frame, 0, sizeof(tx_can_frame));
    memset(tx_packet_buf, 0, sizeof(tx_packet_buf));
    memset(rx_packet_buf, 0, sizeof(rx_packet_buf));
    memset(rx_unpack_buf, 0, sizeof(rx_unpack_buf));

    recv_state = 0;         //接收状态
    cur_recv_len = 0;       //当前已接收数据长度
    update_flag = 0;        //数据更新标志位
}

/**
 * @brief 传输数据转换协议
 */
void can_board_communication_class::can_comm_tx_transformation(void)
{
    tx_data.control_mode = control_mode;  // 设置控制模式
    tx_data.vx = float_to_int16(vx_control, 3.0f, 20000);  // 将浮点数转换为int16_t类型，范围[-3.0, 3.0]映射到[-10000, 10000]
    tx_data.vy = float_to_int16(vy_control, 3.0f, 20000);  // 将浮点数转换为int16_t类型，范围[-3.0, 3.0]映射到[-10000, 10000]
    tx_data.vz = float_to_int16(vz_control, 10.0f, 30000);  // 将浮点数转换为int16_t类型，范围[-10.0, 10.0]映射到[-20000, 20000]
    tx_data.rotation_matrix_yaw = float_to_int16(rotation_matrix_yaw_control, 3.141592f, 32767);  // 将浮点数转换为int16_t类型，范围[-π, π]映射到[-32768, 32767]
}

/**
 * @brief 数据传输
 * @note 协议帧格式: 帧头(0x5A,1byte) + 数据长度(1byte) + 数据段(n-byte) + CRC8(1byte) + 帧尾(0x55,1byte)
 *       CRC8 使用 crc8_crc16 模块的 append_crc8_check_sum 计算，仅覆盖数据段
 *       由于CAN单帧最多8字节，自动分包发送
 */
void can_board_communication_class::transmit_data()
{
    uint8_t send_len;

    // 空指针检查
    if(can_rx_frame == nullptr)
    {
        return;
    }

    // 频率控制(限制发送速率)
    if ((transmit_frequency == 0) || ((HAL_GetTick() - time_stamp) < (1000 / transmit_frequency)))
    {
        return;
    }

    // 更新时间戳
    time_stamp = HAL_GetTick();

    // 解析数据，更新 tx_data
    can_comm_tx_transformation();  

    // 构建协议帧
    tx_packet_buf[0] = Sof_Header;                                      // 帧头(0x5A)
    tx_packet_buf[1] = tx_data_len;                                     // 数据长度
    memcpy(&tx_packet_buf[2], &tx_data, tx_data_len);                   // 数据段

    // 计算CRC8校验码，只对数据段进行CRC8校验
    append_crc8_check_sum(&tx_packet_buf[2], tx_data_len + 1);
    tx_packet_buf[tx_data_len + 3] = Eof_Tail;                                        // 帧尾(0x55)

    for(uint8_t i = 0; i < Can_Protocol_Size + tx_data_len; i += 8)
    {
        // 计算每次发送的长度，确保不超过8字节
        send_len = (Can_Protocol_Size + tx_data_len - i) >= 8 ? 8 : (Can_Protocol_Size + tx_data_len - i);

        // 将数据拷贝到发送缓冲区
        memcpy(tx_can_frame, tx_packet_buf + i, send_len);

        bsp_fdcan_send(can_rx_frame->hcan, master_id, tx_can_frame, send_len);
    }
}

/**
 * @brief CAN接收数据缓冲区数据复位函数
 * @note 用于在接收数据异常或需要重新初始化时，
        清空接收状态、当前接收长度和数据更新标志位，确保接收缓冲区处于初始状态。
 */
void can_board_communication_class::can_comm_rx_reset(void)
{
    // 清空接收缓冲区
    memset(rx_packet_buf, 0, sizeof(rx_packet_buf)); 
    recv_state = 0;         //接收状态
    cur_recv_len = 0;       //当前已接收数据长度
}

/**
 * @brief 解析接收到的CAN数据(底盘发送给云台数据)
 */
void can_board_communication_class::rx_packet()
{
    if(can_rx_frame == nullptr)
    {
        return;
    }

    /* 
     * can_rx_frame 是 volatile 全局变量, 在 CAN 中断中被修改。
     * 为防 ISR 在读过程中修改数据导致不一致, 先整体拷贝到局部变量再操作。
     */
    uint8_t frame_data[8];
    uint8_t frame_len;

    {
        /* 关中断保证拷贝操作的原子性 */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        for (uint32_t i = 0; i < 8; i++)
        {
            frame_data[i] = can_rx_frame->data[i];  // volatile-aware 逐字节拷贝
        }
        frame_len = can_rx_frame->rx_len;
        __set_PRIMASK(primask);  // 恢复原始中断状态（而非无条件开中断）
    }

    if (frame_len == 0 || frame_len > 8)
    {
        return;
    }

    // 之前尚未开始接收且此次包里第一个位置是帧头
    if(frame_data[0] == Sof_Header && recv_state == 0)  
    {
        // 帧头后的后一个字节刚好等于can通信协议规定的接收数据长度
        if(frame_data[1] == rx_data_len)  
        {
            // 标记为正在接收状态
            recv_state = 1;  
        }
        else
        {
            // 数据长度不对，直接丢弃
            return;
        }
    }

    // 已收到帧头且处于正在接收状态
    if(recv_state)
    {
        // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
        if (cur_recv_len + frame_len > rx_data_len + Can_Protocol_Size)
        {
            // 重置接收状态，丢弃错误接收数据
            can_comm_rx_reset();  
            return;
        }

        // 将接收到的数据拷贝到接收缓冲区
        memcpy((void *)&rx_packet_buf[cur_recv_len], (const void *)frame_data, frame_len);  

        // 更新当前接收长度
        cur_recv_len += frame_len;  

        // 如果当前接收长度等于总长度，说明接收完成(进行CRC8校验和帧尾检查)
        if (cur_recv_len == rx_data_len + Can_Protocol_Size)
        {
            if (rx_packet_buf[rx_data_len + Can_Protocol_Size - 1] == Eof_Tail)  // 检查帧尾
            {
                // 校验CRC8
                if (verify_crc8_check_sum(&rx_packet_buf[2], rx_data_len + 1))
                {
                    // CRC校验通过，解析数据段到rx_data结构体
                    memcpy(&rx_unpack_buf, &rx_packet_buf[2], rx_data_len);  
                    update_flag = 1;  // 设置数据更新标志位
                }
            }

            // 重置接收状态,进行下一次数据接收
            can_comm_rx_reset();  
            return;
        }
    }
}

/**
 * @brief 数据接收解析函数
 * @note 将 rx_unpack_buf 中的原始数据解析为 chassis_to_gimbal_data_t 结构体
 */
void can_board_communication_class::rx_unpack(void)
{
    if(update_flag)
    {
        // 将解析后的数据段转换为实际控制量
        chassis_x = int16_to_float(rx_unpack_buf[0] | (rx_unpack_buf[1] << 8), 3.0f, 20000);  // 将int16_t类型转换为浮点数，范围[-20000, 20000]映射到[-3.0, 3.0]
        chassis_y = int16_to_float(rx_unpack_buf[2] | (rx_unpack_buf[3] << 8), 3.0f, 20000);  // 将int16_t类型转换为浮点数，范围[-20000, 20000]映射到[-3.0, 3.0]
        chassis_yaw = int16_to_float(rx_unpack_buf[4] | (rx_unpack_buf[5] << 8), 10.0f, 30000); // 将int16_t类型转换为浮点数，范围[-30000, 30000]映射到[-10.0, 10.0]
        chassis_angular = int16_to_float(rx_unpack_buf[6] | (rx_unpack_buf[7] << 8), 2.0f * PI, 30000); // 将int16_t类型转换为浮点数，范围[-32768, 32767]映射到[-2π, 2π]
        update_flag = 0; // 清除数据更新标志位
    }
}


