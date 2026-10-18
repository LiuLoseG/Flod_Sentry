/**
 * @file can_board_communication.h
 * @author Robin Liu (19038200162@163.com)
 * @brief can板间通讯设计
 * @note 该模块实现了板间通信的发送和接收功能，支持多板间通信的注册和数据处理。通过该模块，各个板子可以通过CAN总线进行数据交换，实现协同工作。
 * @date 2026-09-08
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#pragma once

#include "bsp_can.h"
#include "user_lib.h"
#include "crc8_crc16.h"

/**
 * @brief 云台发送给底盘的数据结构体
 */
typedef struct __attribute__((packed))
{
    // 控制模式(0x00: 底盘无力模式, 0x01: 底盘跟随模式，0x02: 底盘小陀螺模式)
    uint8_t control_mode;  
    // 底盘电机x方向线速度(单位: m/s)
    int16_t vx;
    // 底盘y方向线速度(单位: m/s)
    int16_t vy;
    // 底盘旋转角速度(单位: rad/s)
    int16_t vz;
    // 底盘旋转矩阵的偏转角度(单位: rad)
    int16_t rotation_matrix_yaw;
} gimbal_to_chassis_data_t;

/**
 * @brief 底盘发送给云台的数据结构体
 */
typedef struct
{
    int16_t x;     // 底盘x方向移动路程(单位: m)
    int16_t y;     // 底盘y方向移动路程(单位: m)
    int16_t yaw;   // 底盘旋转偏航角(单位: rad)
    int16_t angular;    // 底盘旋转角速度(单位: rad/s)
} chassis_to_gimbal_data_t;

/**
 * @brief can板间通讯类
 */
class can_board_communication_class
{
public:

    // 初始化函数
    void init(const FDCAN_HandleTypeDef *hcan, const uint16_t &__can_id = 0x16, const uint16_t &__master_id = 0x6, const uint16_t &__transmit_frequency = 1000);

/*--------------------------------------------------设置板间通讯参数----------------------------------------------------*/
    inline uint16_t get_can_id(void) const;
    inline uint16_t get_master_id(void) const;

    inline uint8_t get_tx_data_len(void) const;
    inline uint8_t get_rx_data_len(void) const;

    inline uint16_t get_transmit_frequency(void) const;
    inline uint32_t get_time_stamp(void) const;

    inline float get_chassis_x(void) const;
    inline float get_chassis_y(void) const;
    inline float get_chassis_yaw(void) const;
    inline float get_chassis_angular(void) const;

/*--------------------------------------------------设置板间通讯参数----------------------------------------------------*/
    inline void set_can_id(const uint16_t &__can_id);
    inline void set_master_id(const uint16_t &__master_id);

    inline void set_tx_data_len(const uint8_t &__tx_data_len);
    inline void set_rx_data_len(const uint8_t &__rx_data_len);

    inline void set_transmit_frequency(const uint16_t &__transmit_frequency);

    inline void set_control_mode(const uint8_t &__control_mode);
    inline void set_vx_control(const float &__vx_control);
    inline void set_vy_control(const float &__vy_control);
    inline void set_vz_control(const float &__vz_control);
    inline void set_rotation_matrix_yaw_control(const float &__rotation_matrix_yaw_control);

/*------------------------------------------------------解析函数--------------------------------------------------------*/
    // can板间通讯，接收原始数据获取函数(获取原始数据)
    void rx_packet(void);

    // can板间通讯，数据接收解析函数(解包)
    void rx_unpack(void);

    // can板间通讯，打包发送数据函数(整包发送数据)
    void transmit_data(void);
    
protected:
    // 初始化相关常量

    // 绑定的CAN
    volatile FDCAN_RxFrame_TypeDef *can_rx_frame;

    // 常量

    // 帧头(协议规定 0x5A)
    static constexpr uint8_t Sof_Header = 0x5A; 
    // 帧尾(协议规定 0x55)
    static constexpr uint8_t Eof_Tail = 0x55;

    // 数据段最大长度
    static constexpr uint8_t Can_Information_Size = 60;
    // 数据协议段长度(包含帧头(1byte)、数据长度(1byte)、CRC8(1byte)、帧尾(1byte))
    static constexpr uint8_t Can_Protocol_Size = 4;

    // 本板接收CAN ID(从ID)
    uint16_t can_id = 0x16;
    // 本板发送CAN ID(主ID)
    uint16_t master_id = 0x6;

    // 内部变量

    // 发送数据长度(数据段长度)
    uint8_t tx_data_len = 9;
    // 接收数据长度(数据段)
    uint8_t rx_data_len = 8; 

    // 发送频率(默认1000Hz)
    uint16_t transmit_frequency = 1000; 
    // 发送时间戳
    uint32_t time_stamp = 0; 

    // 输出量

    // 控制模式 (0x00: 底盘无力模式, 0x01: 底盘跟随模式，0x02: 底盘小陀螺模式)
    uint8_t control_mode;  
    // 底盘x方向线速度控制量
    float vx_control;
    // 底盘y方向线速度控制量  
    float vy_control;  
    // 底盘旋转角速度控制量
    float vz_control;  
    // 底盘旋转矩阵的偏转角度控制量
    float rotation_matrix_yaw_control;

    // 发送云台数据给底盘
    gimbal_to_chassis_data_t tx_data;  

    // 读变量

    // 接收缓冲区(帧头 + 数据长度 + 数据段 + crc8 + 帧尾)
    uint8_t rx_packet_buf[Can_Information_Size + Can_Protocol_Size]; 
    // 解析处理的数据段
    uint8_t rx_unpack_buf[Can_Information_Size];

    volatile uint8_t recv_state = 0;                  // 接收状态
    volatile uint8_t cur_recv_len = 0;                // 当前已接收数据长度
    volatile uint8_t update_flag = 0;                 // 数据更新标志位

    // 写变量

    // 底盘发送给云台的数据结构体
    float chassis_x;     // 底盘x方向移动路程(单位: m)
    float chassis_y;     // 底盘y方向移动路程(单位: m)
    float chassis_yaw;   // 底盘旋转偏航角(单位: rad)
    float chassis_angular;  // 底盘旋转角速度(单位: rad/s)

    // 读写变量

    // 一帧can报文发送缓冲区
    uint8_t tx_can_frame[8];
    // 发送缓冲区(协议 + 数据段)最大长度缓存区
    uint8_t tx_packet_buf[Can_Information_Size + Can_Protocol_Size]; // 发送缓冲区(帧头 + 数据段)

    // 内部函数

    // 发送数据解析转换函数(真实的控制数据转换为协议数据)
    void can_comm_tx_transformation(void);

    // can接收数据缓冲区数据复位函数
    void can_comm_rx_reset(void);
    
};

/**
 * @brief 获取can板间通讯类的CAN ID
 * @retval uint16_t: 
 */
inline uint16_t can_board_communication_class::get_can_id(void) const
{
    return can_id;
}

/**
 * @brief 获取can板间通讯类的主ID
 * @retval uint16_t: 
 */
inline uint16_t can_board_communication_class::get_master_id(void) const
{
    return master_id;
}

/**
 * @brief 获取can板间通讯类的发送数据长度
 * @retval uint8_t: 
 */
inline uint8_t can_board_communication_class::get_tx_data_len(void) const
{
    return tx_data_len;
}

/**
 * @brief 获取can板间通讯类的接收数据长度
 * @retval uint8_t: 
 */
inline uint8_t can_board_communication_class::get_rx_data_len(void) const
{
    return rx_data_len;
}

/**
 * @brief 获取can板间通讯类的发送频率
 * @retval uint16_t: 
 */
inline uint16_t can_board_communication_class::get_transmit_frequency(void) const
{
    return transmit_frequency;
}

/**
 * @brief 获取can板间通讯类的发送时间戳
 * @retval uint32_t: 
 */
inline uint32_t can_board_communication_class::get_time_stamp(void) const
{
    return time_stamp;
}

/**
 * @brief 获取can板间通讯类的底盘x方向移动路程
 * @retval float: 
 */
inline float can_board_communication_class::get_chassis_x(void) const
{
    return chassis_x;
}

/**
 * @brief 获取can板间通讯类的底盘y方向移动路程
 * @retval float: 
 */
inline float can_board_communication_class::get_chassis_y(void) const
{
    return chassis_y;
}

/**
 * @brief 获取can板间通讯类的底盘旋转偏航角
 * @retval float: 
 */
inline float can_board_communication_class::get_chassis_yaw(void) const
{
    return chassis_yaw;
}

/**
 * @brief 获取can板间通讯类的底盘旋转角速度
 * @note 该函数用于获取底盘的旋转角速度
 * @retval float: 
 */
inline float can_board_communication_class::get_chassis_angular(void) const
{
    return chassis_angular;
}

/**
 * @brief 设置can板间通讯类的CAN ID
 * @param __can_id 
 */
inline void can_board_communication_class::set_can_id(const uint16_t &__can_id)
{
    can_id = __can_id;
}

/**
 * @brief 设置can板间通讯类的主ID
 * @param __master_id 
 */
inline void can_board_communication_class::set_master_id(const uint16_t &__master_id)
{
    master_id = __master_id;
}

/**
 * @brief 设置can板间通讯类的发送数据长度
 * @param __tx_data_len 
 */
inline void can_board_communication_class::set_tx_data_len(const uint8_t &__tx_data_len)
{
    tx_data_len = __tx_data_len;
}

/**
 * @brief 设置can板间通讯类的接收数据长度
 * @param __rx_data_len 
 */
inline void can_board_communication_class::set_rx_data_len(const uint8_t &__rx_data_len)
{
    rx_data_len = __rx_data_len;
}

/**
 * @brief 设置can板间通讯类的发送频率
 * @param __transmit_frequency 
 */
inline void can_board_communication_class::set_transmit_frequency(const uint16_t &__transmit_frequency)
{
    transmit_frequency = __transmit_frequency;
}

/**
 * @brief 设置can板间通讯类的底盘控制模式
 * @param __control_mode 
 */
inline void can_board_communication_class::set_control_mode(const uint8_t &__control_mode)
{
    control_mode = __control_mode;
}

/**
 * @brief 设置can板间通讯类的底盘x方向线速度控制量
 * @param __vx_control 
 */
inline void can_board_communication_class::set_vx_control(const float &__vx_control)
{
    vx_control = __vx_control;
}

/**
 * @brief 设置can板间通讯类的底盘y方向线速度控制量
 * @param __vy_control 
 */
inline void can_board_communication_class::set_vy_control(const float &__vy_control)
{
    vy_control = __vy_control;
}

/**
 * @brief 设置can板间通讯类的底盘旋转角速度控制量
 * @param __vz_control 
 */
inline void can_board_communication_class::set_vz_control(const float &__vz_control)
{
    vz_control = __vz_control;
}

/**
 * @brief 设置can板间通讯类的底盘旋转矩阵的偏转角度控制量
 * @param __rotation_matrix_yaw_control 
 */
inline void can_board_communication_class::set_rotation_matrix_yaw_control(const float &__rotation_matrix_yaw_control)
{
    rotation_matrix_yaw_control = __rotation_matrix_yaw_control;
}

