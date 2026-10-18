#pragma once

#include "dji_motor_new.h"
#include "frequency_filter.h"
#include "general_filter.h"
#include "pid.h"
#include "fsm.h"
#include "slope_math.h"
#include "remote_center.h"
#include "robot_def.h"

#include "robot_decision.h"

// 发射机构控制方式
typedef enum
{
    Shoot_Method_Zero_Force = 0,  // 零力矩模式
    Shoot_Method_Manual,          // 遥控器控制模式
    Shoot_Method_Auto             // 自瞄控制模式
} shoot_method_e;

// 发射机构控制模式
typedef enum
{
    Shoot_Disable = 0,  // 禁止发射(发射机构电机失能)
    Shoot_Single,       // 单发模式
    Shoot_Repeat,       // 连发模式
    Shoot_Ceasefire,    // 停止开火模式(摩擦轮正常控制, 拨盘电机停止转动)
    Shoot_Auto,         // 自动模式
} shoot_mode_e;

// 卡弹处理状态机状态类型
typedef enum
{
    Shoot_Fsm_Jamming_Status_Normal = 0,  // 正常状态
    Shoot_Fsm_Jamming_Status_Suspect,     // 卡弹怀疑状态
    Shoot_Fsm_Jamming_Status_Confirm,     // 卡弹确认状态
    Shoot_Fsm_Jamming_Status_Process,     // 卡弹处理状态
} shoot_fsm_jamming_status_e;

// 发射机构观测与控制值
typedef struct
{
    float fric_r_omega;  // 右摩擦轮电机角速度
    float fric_l_omega;  // 左摩擦轮电机角速度
    float fric_r_torque;  // 右摩擦轮电机扭矩
    float fric_l_torque;  // 左摩擦轮电机扭矩

    float trigger_angle;  // 拨盘电机角度
    float trigger_omega;  // 拨盘电机角速度
    float trigger_torque;  // 拨盘电机扭矩
} shoot_observate_control_t;

// 发射机构电机前馈值
typedef struct
{
    float fric_r_toque;  // 右摩擦轮电机扭矩前馈
    float fric_l_toque;  // 左摩擦轮电机扭矩前馈

    float trigger_omega;    // 拨盘电机角速度前馈
    float trigger_toque;    // 拨盘电机扭矩前馈
} shoot_motor_feedforward_t;

// 卡弹状态机
class shoot_jamming_fsm_class : public fsm_class<5>
{

public:

    // 重写基函数的状态转移函数
    void fsm_calculate_callback() override;

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 拨弹盘堵转扭矩阈值, 超出被认为卡弹
    const float Driver_Torque_Threshold = 0.4f;
    // 卡弹判定持续时间阈值, 超出被认为卡弹
    const uint32_t Jamming_Suspect_Time_Threshold = 100;  // 单位: ms
    // 卡弹处理持续时间阈值, 超出则认为处理结束
    const uint32_t Jamming_Solving_Time_Threshold = 280;  // 单位: ms
    // 卡弹处理回退角度(回转半发弹丸角度)
    const float Driver_Back_Angle = TRIGGER_PILL_TURN_RAD * 1.5f;

    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

typedef struct
{

/*--------------------------外部观测对象绑定--------------------------------*/
    crsf_class *crsf; // 遥控器对象指针(观测)
    shoot_decision_source_t *source; // 用于决策的源数据(修改)
    robot_decision_class *decision; // 机器人决策对象指针(观测)
    
    shoot_jamming_fsm_class jamming_fsm; // 卡弹状态机对象

    adaptive_filter_class fric_r_filter; // 自适应低通滤波器对象
    adaptive_filter_class fric_l_filter; // 自适应低通滤波器对象
    adaptive_filter_class trigger_filter; // 自适应低通滤波器对象

/*--------------------------控制和观测量--------------------------------*/
    shoot_observate_control_t now_value;      // 当前观测值
    shoot_observate_control_t target_value;   // 目标值
    shoot_observate_control_t filter_value;   // 滤波值
    shoot_motor_feedforward_t feedforward_value;  // 前馈值

/*--------------------------摩擦轮电机斜坡函数--------------------------------*/
    slope_class fric_r_slope;  // 右摩擦轮电机斜坡函数对象
    slope_class fric_l_slope;  // 左摩擦轮电机斜坡函数对象

/*--------------------------电机对象创建--------------------------------*/
    dji_c620_motor_class fric_r_motor;      // 右摩擦轮电机
    dji_c620_motor_class fric_l_motor;      // 左摩擦轮电机
    dji_c610_motor_class trigger_motor;     // 拨盘电机

/*--------------------------控制模式--------------------------------*/
    shoot_method_e now_method;  // 当前控制模式
    shoot_mode_e now_mode;    // 当前发射模式
    shoot_mode_e last_mode;   // 上一帧发射模式

    uint8_t allow_flag;         // 允许发射标志位, 0:禁止发射, 1:允许发射
    bool fric_spin_up_flag;      // 摩擦轮电机转速上升标志位, 0:未上升, 1:已上升
    
    float Heat_Balance_Omega;   // 热量平衡值, 拨盘射速

} shoot_t;

