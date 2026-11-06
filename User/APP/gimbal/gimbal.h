#pragma once

#include "dm_motor.h"
#include "smc_control.h"
#include "remote_center.h"

typedef struct
{
    float angle; // 角度
    float speed; // 速度
} motor_state_t;

typedef enum
{
    Gimbal_Disable = 0,
    Gimbal_Enabale = 1,
} gimbal_state_e;

/**
 * @brief 云台结构体
 */
typedef struct
{
    crsf_class *crsf; // 遥控器对象指针

    gimbal_state_e state; // 云台状态

    cSMC smc_yaw; // yaw轴滑模控制器

    motor_state_t tgt_yaw; // 目标角度
    motor_state_t now_yaw; // 当前角度
    
    dm_motor_normal_class motor_yaw; // yaw轴电机
    
} gimbal_smc_t;

