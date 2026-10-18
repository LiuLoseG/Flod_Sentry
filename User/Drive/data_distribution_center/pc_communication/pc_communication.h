/**
 * @file pc_communication.h
 * @author Robin Liu (19038200162@163.com)
 * @brief PC与MCU通信模块头文件(USB设备数据分发)
 * @date 2026-08-26
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#pragma once

#include "crc8_crc16.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include "bsp_usb.h"


typedef struct __attribute__((__packed__))
{
    // 控制模式，0:不控制 1:控云台不开火 2:控云台且开火
    uint8_t control_mode;
    // 目标角度、前馈角速度、前馈加速度
    float target_yaw_angle;
    float feedforward_yaw_omega;
    float feedforward_yaw_accelerate;

    float target_pitch_angle;
    float feedforward_pitch_omega;
    float feedforward_pitch_accelerate;
} auto_aim_data_t;

/**
 * @brief MCU到PC的自瞄视觉数据传输类
 */
class mcu_to_pc_vision_class
{
public:
    void init(const uint8_t &__cmd_id = 0x01, const uint8_t &__data_len = 39, const uint16_t &__transmit_frequency = 1000);
    
    inline void get_time_stamp(uint32_t &__time_stamp) const;

    inline void set_cmd_id(const uint8_t &__cmd_id);

    inline void set_data_len(const uint8_t &__data_len);

    inline void set_transmit_frequency(const uint16_t &__transmit_frequency);

    inline void enemy_color_data_update(const uint8_t &__enemy_color);

    void imu_data_update(const float &__imu_quaternion_w, const float &__imu_quaternion_x, const float &__imu_quaternion_y, const float &__imu_quaternion_z, 
        const float &__imu_yaw_angle, const float &__imu_yaw_omega, const float &__imu_pitch_angle, const float &__imu_pitch_omega);

    void bullet_data_update(const float &__bullet_speed, const uint16_t &__bullet_count);

    void transmit_data(void);

protected:

    // 帧头(协议规定 0x5A)
    const uint8_t Sof_Header = 0x5A; 

    // 数据长度(只计算数据段的长度，帧头、数据长度、数据ID、CRC16不计入)
    uint8_t data_len = 39; 
    // 数据命令ID
    uint16_t cmd_id = 0x01;
    // 发送频率(默认1000Hz)
    uint16_t transmit_frequency = 1000; 
    // 发送时间戳
    uint32_t time_stamp = 0; 

    // 敌方装甲板颜色  0：未获取  1：红色   2：蓝色
    uint8_t enemy_color = 0;
    // 四元数
    float imu_quaternions[4];
    // imu欧拉角及其角速度
    float imu_yaw_angle;
    float imu_yaw_omega;
    float imu_pitch_angle;
    float imu_pitch_omega;
    // 子弹速度
    float bullet_speed;
    // 子弹发射次数
    uint16_t bullet_count;
};

/**
 * @brief PC到MCU的自瞄视觉数据传输类
 */
class pc_to_mcu_vision_class
{
public:

    // 解包后的数据读取
    inline uint8_t get_control_mode() const;
    inline float get_yaw_angle() const;
    inline float get_ff_yaw_w() const;
    inline float get_ff_yaw_a() const;

    inline float get_pitch_angle() const;
    inline float get_ff_pitch_w() const;
    inline float get_ff_pitch_a() const;

    inline uint8_t get_cmd_id() const;

    void init(const uint8_t &__cmd_id = 0x11, const uint8_t &__data_len = 5);
    
    inline void set_cmd_id(const uint8_t &__cmd_id);

    inline void set_data_len(const uint8_t &__data_len);

    void data_unpack(const uint8_t *sof_address);

protected:

    // 帧头
    const uint8_t Sof_Header = 0x5A; 

    // 数据长度
    uint8_t data_len = 5; 
    // 数据命令ID
    uint16_t cmd_id = 0x11;
    // 解包后的自瞄数据结构体
    auto_aim_data_t rx_data; 

    float now_yaw_angle;
    float now_ff_yaw_w;
    float now_ff_yaw_a;
    float now_pitch_angle;
    float now_ff_pitch_w;
    float now_ff_pitch_a;
};

/**
 * @brief 获取MCU到PC的自瞄视觉数据传输类的时间戳
 * @param __time_stamp 
 */
inline void mcu_to_pc_vision_class::get_time_stamp(uint32_t &__time_stamp) const
{
    __time_stamp = time_stamp;
}

/**
 * @brief 设置MCU到PC的自瞄视觉数据传输类的命令ID
 * @param __cmd_id 
 */
inline void mcu_to_pc_vision_class::set_cmd_id(const uint8_t &__cmd_id)
{
    cmd_id = __cmd_id;
}

/**
 * @brief 设置MCU到PC的自瞄视觉数据传输类的数据长度
 * @param __data_len 
 */
inline void mcu_to_pc_vision_class::set_data_len(const uint8_t &__data_len)
{
    data_len = __data_len;
}

/**
 * @brief 设置MCU到PC的自瞄视觉数据传输类的发送频率
 * @param __transmit_frequency 
 */
inline void mcu_to_pc_vision_class::set_transmit_frequency(const uint16_t &__transmit_frequency)
{
    transmit_frequency = __transmit_frequency;
}

/**
 * @brief 设置MCU到PC的自瞄视觉数据传输类的敌方装甲板颜色
 * @param __enemy_color 
 */
inline void mcu_to_pc_vision_class::enemy_color_data_update(const uint8_t &__enemy_color)
{
    enemy_color = __enemy_color;
}

/**
 * @brief 设置PC到MCU的自瞄视觉数据传输类的命令ID
 */
inline void pc_to_mcu_vision_class::set_cmd_id(const uint8_t &__cmd_id)
{
    cmd_id = __cmd_id;
}

/**
 * @brief 设置PC到MCU的自瞄视觉数据传输类的数据长度
 */
inline void pc_to_mcu_vision_class::set_data_len(const uint8_t &__data_len)
{
    data_len = __data_len;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据传输类的控制模式
 * @retval uint8_t: 
 */
inline uint8_t pc_to_mcu_vision_class::get_control_mode() const
{
    return rx_data.control_mode;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据传输类的偏航角度
 * @retval float: 
 */
inline float pc_to_mcu_vision_class::get_yaw_angle() const
{
    return now_yaw_angle;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据yaw角速度前馈
 * @retval float: 
 */
inline float pc_to_mcu_vision_class::get_ff_yaw_w() const
{
    return now_ff_yaw_w;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据yaw角加速度前馈
 * @retval float: 
 */
inline float pc_to_mcu_vision_class::get_ff_yaw_a() const
{
    return now_ff_yaw_a;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据pitch角度
 * @retval float: 
 */
inline float pc_to_mcu_vision_class::get_pitch_angle() const
{
    return now_pitch_angle;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据pitch角速度前馈
 * @retval float: 
 */
inline float pc_to_mcu_vision_class::get_ff_pitch_w() const
{
    return now_ff_pitch_w;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据pitch角加速度前馈
 * @retval float: 
 */
inline float pc_to_mcu_vision_class::get_ff_pitch_a() const
{
    return now_ff_pitch_a;
}

/**
 * @brief 获取PC到MCU的自瞄视觉数据传输类的命令ID
 * @retval uint8_t: 
 */
inline uint8_t pc_to_mcu_vision_class::get_cmd_id() const
{
    return (uint8_t)cmd_id;
}
