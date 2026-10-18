#pragma once

#include "remote_center.h"
#include "pc_communication.h"
#include "can_board_communication.h"
#include "hi12_imu.h"

#include "basic_math.h"
#include "fsm.h"

#include "robot_def.h"

class robot_decision_class;

// 热量检测状态类型
typedef enum
{
    Robot_Fsm_Heat_Status_Stop = 0,      // 停止状态
    Robot_Fsm_Heat_Status_Ready,         // 准备状态
    Robot_Fsm_Heat_Status_Suspect,       // 热量降低，怀疑状态
    Robot_Fsm_Heat_Status_Confirm,       // 热量降低，确定状态
} robot_fsm_heat_status_e;

// 发射类型状态类型(单发、连发、停火)
typedef enum
{
    Robot_Fsm_Fire_Status_Stop = 0,         // 停止状态
    Robot_Fsm_Fire_Status_Free,             // 空闲状态
    Robot_Fsm_Fire_Status_Press,            // 按下状态
    Robot_Fsm_Fire_Status_Single,           // 单发状态
    Robot_Fsm_Fire_Status_Repeat,           // 连发状态
} robot_fsm_fire_e;

/*-------------------------发射机构用于决策的源数据--------------------------------*/

// 发射机构热量检测数据
typedef struct
{
    float fric_r_omega;  // 右摩擦轮电机角速度
    float fric_r_torque;  // 右摩擦轮电机扭矩
    uint8_t shoot_now_mode;  // 发射机构当前模式
} shoot_decision_source_t;

/*-------------------------云台控制用于决策的源数据--------------------------------*/

// 云台控制用于决策的源数据
typedef struct
{
    float big_yaw_angle;
    float small_yaw_angle;
    float vision_gyr_z;
} gimbal_decision_source_t;

/*-----------------------------机器人决策状态机------------------------------------*/

/**
 * @brief 机器人热量检测状态机
 */
class robot_heat_fsm_class : public fsm_class<4>
{

public:

    // 机器人决策对象指针(观测)
    robot_decision_class *decision; 

    // 重写基函数的状态转移函数
    void fsm_calculate_callback() override;

    // 获取当前热量值
    float get_now_heat();

    // 获取总弹量
    uint16_t get_totol_ammunition_num();

protected:
    // 初始化相关常量

    // 常量

    // 摩擦轮单发增长热量值
    const float Single_Heat_Increase = 10.0f;

    // 内部变量

    // 读变量

    // 当前热量值
    float now_heat = 0.0f;

    // 累计发射弹量
    uint16_t total_ammunition_num = 0;
};

/**
 * @brief 机器人开火选择状态机(单发、连发、停火)
 */
class robot_fire_fsm_class : public fsm_class<5>
{

public:
    // 机器人决策对象指针(观测)
    robot_decision_class *decision; 

     // 重写基函数的状态转移函数
    void fsm_calculate_callback() override;

    robot_fsm_fire_e get_now_fire_status() const;

protected:
    // 初始化相关常量

    // 常量
    const uint32_t Repeat_Fire_Time_Threshold = 500;  // 单发模式持续时间阈值, 超过则认为单发结束
    // 内部变量

    // 读变量
    robot_fsm_fire_e now_fire_status = Robot_Fsm_Fire_Status_Stop;  // 当前开火状态

};

/**
 * @brief 机器人决策类
 */
class robot_decision_class
{

public:

    // 遥控器对象指针(观测)
    crsf_class *crsf; 
    // IMU对象指针(观测)
    volatile hi12_gyro *hi12_imu;                   

    robot_heat_fsm_class heat_fsm; // 机器人热量检测状态机对象
    friend class robot_heat_fsm_class; // 声明友元类, 允许访问私有成员

    robot_fire_fsm_class fire_fsm; // 机器人开火选择状态机对象
    friend class robot_fire_fsm_class; // 声明友元类, 允许访问私有成员

    // MCU到PC的自瞄视觉数据传输类对象
    mcu_to_pc_vision_class mcu_to_pc_aim; 

    // PC到MCU的自瞄视觉数据传输类对象
    pc_to_mcu_vision_class pc_to_mcu_aim;
    
    // CAN板间通讯类对象
    can_board_communication_class can_board_communication; 

    inline float get_heat_maximum() const; // 获取热量上限

    inline float get_heat_cd() const; // 获取热量冷却系数

    inline float get_gimbal_for_chassis_offset() const; // 获取云台对底盘的偏航角度补偿量

    inline void set_gimbal_for_chassis_offset(float __offset_yaw); // 设置云台对底盘的偏航角度补偿量

protected:
    // 初始化相关常量

    //常量

    // 摩擦轮单次判定发弹阈值, 超出被认为发射子弹
    const float Fric_Torque_Threshold = -0.050f;
    // 摩擦轮速度判定发弹阈值, 超出则说明已经开机
    const float Fric_Omega_Threshold = -FRICT_MOTOR_SPEED_SET + 4.0f;
    // 摩擦轮单次判定发弹确认时间, 超过这个时间则认为发射子弹
    const uint32_t Fric_Torque_Confirm_Time = 30;  // 单位: ms

    // 内部变量  

    // 云台对底盘的偏航角度补偿量(单位: rad)
    float gimbal_for_chassis_offset = 0.0f;

    // 热量冷却系数, 每秒热量降低值
    float heat_cd = 30.0f;  
    // 热量上限
    float heat_maximum = 260.0f;  

    // 读变量
};

/**
 * @brief 获取当前热量值
 */
inline float robot_heat_fsm_class::get_now_heat()
{
    return now_heat;
}

/**
 * @brief 获取总弹量
 */
inline uint16_t robot_heat_fsm_class::get_totol_ammunition_num()
{
    return total_ammunition_num;
}

/**
 * @brief 获取热量上限
 * @retval float: 热量上限
 */
inline float robot_decision_class::get_heat_maximum() const
{
    return heat_maximum;
}

/**
 * @brief 获取热量冷却系数
 * @retval float: 热量冷却系数
 */
inline float robot_decision_class::get_heat_cd() const
{
    return heat_cd;
}

/**
 * @brief 获取当前开火状态
 * @retval robot_fsm_fire_e: 当前开火状态
 */
inline robot_fsm_fire_e robot_fire_fsm_class::get_now_fire_status() const
{
    return now_fire_status;
}

/**
 * @brief 设置云台对底盘的偏航角度补偿量
 * @param offset 偏航角度补偿量 (单位: rad)
 */
inline float robot_decision_class::get_gimbal_for_chassis_offset() const
{
    return gimbal_for_chassis_offset;
}

/**
 * @brief 设置云台对底盘的偏航角度补偿量
 * @param offset 偏航角度补偿量 (单位: rad)
 */
inline void robot_decision_class::set_gimbal_for_chassis_offset(float __offset_yaw)
{
    gimbal_for_chassis_offset = __offset_yaw;
}

extern shoot_decision_source_t shoot_decision_source;  
extern gimbal_decision_source_t gimbal_decision_source;  

extern robot_decision_class robot_decision;

void robot_data_transfer_analyze(uint8_t *buffer, uint16_t length);
