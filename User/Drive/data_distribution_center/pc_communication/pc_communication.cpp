/**
 * @file pc_communication.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief PC与MCU通信模块实现文件(USB设备数据分发)
 * @date 2026-08-26
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "pc_communication.h"
#include <cstring>

/**
 * @brief 初始化MCU到PC的视觉数据传输类
 * @param __cmd_id 
 * @param __data_len 
 * @param __transmit_frequency 
 */
void mcu_to_pc_vision_class::init(const uint8_t &__cmd_id, const uint8_t &__data_len, const uint16_t &__transmit_frequency)
{
    // 初始化数据ID、数据长度和发送频率
    cmd_id = __cmd_id;
    data_len = __data_len;
    transmit_frequency = __transmit_frequency;

    // 发送数据清零
    memset(imu_quaternions, 0, sizeof(imu_quaternions));
    imu_yaw_angle = 0.0f;
    imu_yaw_omega = 0.0f;
    imu_pitch_angle = 0.0f;
    imu_pitch_omega = 0.0f;
    bullet_speed = 0.0f;
    bullet_count = 0;
    time_stamp = 0;
}

/**
 * @brief 更新IMU数据
 * @param __imu_quaternion_w 
 * @param __imu_quaternion_x 
 * @param __imu_quaternion_y 
 * @param __imu_quaternion_z 
 * @param __imu_yaw_angle (rad)
 * @param __imu_yaw_omega (rad/s)
 * @param __imu_pitch_angle (rad)
 * @param __imu_pitch_omega (rad/s)
 */
void mcu_to_pc_vision_class::imu_data_update(const float &__imu_quaternion_w, const float &__imu_quaternion_x, const float &__imu_quaternion_y, const float &__imu_quaternion_z, 
    const float &__imu_yaw_angle, const float &__imu_yaw_omega, const float &__imu_pitch_angle, const float &__imu_pitch_omega)
{
    // 更新IMU数据
    imu_quaternions[0] = __imu_quaternion_w;
    imu_quaternions[1] = __imu_quaternion_x;
    imu_quaternions[2] = __imu_quaternion_y;
    imu_quaternions[3] = __imu_quaternion_z;
    imu_yaw_angle = __imu_yaw_angle;
    imu_yaw_omega = __imu_yaw_omega;
    imu_pitch_angle = __imu_pitch_angle;
    imu_pitch_omega = __imu_pitch_omega;
}

/**
 * @brief 更新子弹数据
 * @param __bullet_speed
 * @param __bullet_count
 */
void mcu_to_pc_vision_class::bullet_data_update(const float &__bullet_speed, const uint16_t &__bullet_count)
{
    // 更新子弹数据
    bullet_speed = __bullet_speed;
    bullet_count = __bullet_count;
}

/**
 * @brief 传输数据给PC端
 * @note 该函数需要在主循环中以固定频率调用, 以保证数据传输的实时性
 * @note 帧格式: | SOF(0x5A) | LEN | CMD_ID(小端2B) | CRC8(帧头) | DATA | CRC16(整包) |
 */
void mcu_to_pc_vision_class::transmit_data(void)
{

    // 频率控制(限制发送速率)
    if ((transmit_frequency == 0) || ((HAL_GetTick() - time_stamp) < (1000 / transmit_frequency)))
    {
        return;
    }
    
    // 更新时间戳
    time_stamp = HAL_GetTick();

    // 帧缓冲区: 帧头5字节 + 数据段39字节 + CRC16 2字节
    uint8_t tx_buf[5u + data_len + 2u];
    uint8_t offset = 0;

    // 1. 帧头: SOF + LEN + CMD_ID(小端) + CRC8(帧头校验)
    tx_buf[offset++] = Sof_Header;                              // 帧头 0x5A
    tx_buf[offset++] = data_len;                                // 数据长度
    tx_buf[offset++] = (uint8_t)(cmd_id & 0xFF);                // cmd_id 低字节
    tx_buf[offset++] = (uint8_t)((cmd_id >> 8) & 0xFF);         // cmd_id 高字节
    append_crc8_check_sum(tx_buf, offset + 1u);                 // 帧头CRC8: 覆盖SOF+LEN+CMD_ID, 写入第5字节
    offset++;                                                   // 跳过CRC8字节

    // 2. 数据段: 敌方装甲板颜色
    memcpy(&tx_buf[offset], &enemy_color, sizeof(enemy_color));         offset += sizeof(enemy_color);
    // 四元数
    memcpy(&tx_buf[offset], imu_quaternions, sizeof(imu_quaternions));  offset += sizeof(imu_quaternions);
    // 欧拉角与角速度
    memcpy(&tx_buf[offset], &imu_yaw_angle, sizeof(imu_yaw_angle));     offset += sizeof(imu_yaw_angle);
    memcpy(&tx_buf[offset], &imu_yaw_omega, sizeof(imu_yaw_omega));     offset += sizeof(imu_yaw_omega);
    memcpy(&tx_buf[offset], &imu_pitch_angle, sizeof(imu_pitch_angle)); offset += sizeof(imu_pitch_angle);
    memcpy(&tx_buf[offset], &imu_pitch_omega, sizeof(imu_pitch_omega)); offset += sizeof(imu_pitch_omega);
    // 子弹数据
    memcpy(&tx_buf[offset], &bullet_speed, sizeof(bullet_speed)); offset += sizeof(bullet_speed);
    memcpy(&tx_buf[offset], &bullet_count, sizeof(bullet_count)); offset += sizeof(bullet_count);

    // 3. 整包CRC16(覆盖帧头+数据段)
    append_crc16_check_sum(tx_buf, offset + 2u);
    offset += 2u;

    // 4. 通过USB发送
    bsp_usb_transmit(tx_buf, offset);
}

/**
 * @brief 初始化PC到MCU的视觉数据传输类
 * @param __cmd_id 
 * @param __data_len 
 */
void pc_to_mcu_vision_class::init(const uint8_t &__cmd_id, const uint8_t &__data_len)
{
    // 初始化数据ID、数据长度
    cmd_id = __cmd_id;
    data_len = __data_len;

    // 解包数据清零
    memset(&rx_data, 0, sizeof(auto_aim_data_t));
}

/**
 * @brief 自瞄数据协议解包
 * @param sof_address 
 */
void pc_to_mcu_vision_class::data_unpack(const uint8_t *sof_address)
{
    memcpy(&rx_data, sof_address, sizeof(auto_aim_data_t));

        // 更新当前解包数据
    if (rx_data.control_mode != 0)
    {
        now_yaw_angle = rx_data.target_yaw_angle;
        now_ff_yaw_w = rx_data.feedforward_yaw_omega;
        now_ff_yaw_a = rx_data.feedforward_yaw_accelerate;

        now_pitch_angle = rx_data.target_pitch_angle;
        now_ff_pitch_w = rx_data.feedforward_pitch_omega;
        now_ff_pitch_a = rx_data.feedforward_pitch_accelerate;
    }
}
