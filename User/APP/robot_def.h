/**
 * @file robot_def.h
 * @author Robin Liu (19038200162@163.com)
 * @brief 机器人相关物理参数宏定义文件
 * @note 该文件用于定义机器人相关的物理参数和宏定义，便于在整个项目中统一使用和管理这些参数。
         通过集中定义，可以提高代码的可维护性和可读性，避免在多个文件中重复定义相同的参数。
 * @date 2026-07-19
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#pragma once

/*--------------------------------------------------机器人整体控制-------------------------------------------------------*/

// 机器人单双板设定
#define ROBOT_AMOUNT_BOARD 2 // 单板模式(1)/双板模式(2)

/*--------------------------------------------------底盘-------------------------------------------------------*/   

// 底盘最大平动速度，单位m/s
#define CHASSIS_MAX_MOVE_SPEED 1.5f 
// 底盘最大转动速度，单位rad/s
#define CHASSIS_MAX_ROTATE_SPEED 5.0f

/*--------------------------------------------------云台-------------------------------------------------------*/

#define YAW_SMALL_TORQUE_LIMIT 1.0f
#define PITCH_TORQUE_LIMIT 1.8f
#define PITCH_GRAVITY_COMPENSATION -0.85f

// 云台yaw轴对齐时是否大于0度, 若大于则设为1，否则设为0
#define YAW_RAD_MIDDLE_THAN_ZERO 1

// yaw电机中间位置角度 (单位: rad)
#define YAW_MOTOR_MID_ANGLE 1.64509869f

// pitch轴电机中间位置角度 (单位: rad)
#define PITCH_MOTOR_MID_ANGLE 0.377454847f
// 小yaw电机中间位置角度 (单位: rad)          
#define YAW_SMALL_MOTOR_MID_ANGLE 3.50284529f      

// 云台PITCH轴限辐，0.780412078 - 1.320412078 = -0.599690202 留0.04f的余量
#define PITCH_MIN_OFFSET -0.54f 
// 云台PITCH轴限辐，2.02034688 - 1.38010228 = 0.6402446 留0.08f的余量           
#define PITCH_MAX_OFFSET 0.55f               

// 云台YAW轴限辐，2.88848591 - 3.49364138 = -0.60515547 留0.04f的余量
#define YAW_SMALL_MIN_OFFSET -0.56515547f 
// 云台YAW轴限辐，4.1033988 - 3.49364138 = 0.60899019 留0.06f的余量         
#define YAW_SMALL_MAX_OFFSET 0.54899019f           

// 小yaw电机单次角度增量系数 (单位: rad/单位输入)
#define YAW_SMALL_ANGLE_INCREMENT_RATION    0.0035f 
// pitch电机单次角度增量系数 (单位: rad/单位输入)       
#define PITCH_ANGLE_INCREMENT_RATION        0.0020f        

/*--------------------------------------------------发射机构-------------------------------------------------------*/

// 摩擦轮电机速度设定
#define FRICT_MOTOR_SPEED_SET 650.0f

// 摩擦轮电机加速完成的速度差阈值, 超过被认为加速完成
#define FRICT_MOTOR_SPEED_DELTA_THRESHOLD 5.0f

// 摩擦轮电机扭矩自适应滤波参数(观测/热量检测用: 平稳深度平滑, 突变快速跟踪)
#define FRICT_TORQUE_ALPHA_MIN 0.04f   // 平稳段滤波系数(等效低通约6.4Hz@1kHz)
#define FRICT_TORQUE_ALPHA_MAX 0.95f   // 突变段滤波系数(快速跟踪)
#define FRICT_TORQUE_ERR_LOW   0.010f  // 偏差低于此值使用α_min
#define FRICT_TORQUE_ERR_HIGH  0.030f  // 偏差高于此值使用α_max(真实力矩事件)

// 拨盘电机齿轮减速比
#define TRTGGER_TOOTHED_REDUCTION 2.5f    

// 拨盘满载弹丸数量
#define TRIGGER_PILL_NUM        9.0f               

// 拨盘电机每发射一发弹丸转动的角度值，单位rad
#define TRIGGER_PILL_TURN_RAD ((TRTGGER_TOOTHED_REDUCTION * 2.0f * PI) / TRIGGER_PILL_NUM)       

// 拨盘电机默认旋转频率18HZ(单位: Hz)
#define TRIGGER_AMMO_FREQUENCY_DEFAULT   18.0f
// 拨盘电机默认旋转速度18HZ(单位: rad/s)
#define TRIGGER_DRIVER_OMEGA_DEFAULT ((TRIGGER_AMMO_FREQUENCY_DEFAULT / 9.0f * TRTGGER_TOOTHED_REDUCTION) * 2.0f * PI)

// 拨盘电机扭矩自适应滤波参数(卡弹检测用: 平稳深度平滑, 突变快速跟踪)
#define TRIGGER_TORQUE_ALPHA_MIN 0.04f   // 平稳段滤波系数(等效低通约6.6Hz@1kHz)
#define TRIGGER_TORQUE_ALPHA_MAX 0.95f   // 突变段滤波系数(快速跟踪)
#define TRIGGER_TORQUE_ERR_LOW   0.05f   // 偏差低于此值使用α_min
#define TRIGGER_TORQUE_ERR_HIGH  0.60f   // 偏差高于此值使用α_max(卡弹更早触发快速跟踪)

// 停火热量阀值
#define HEAT_CEASEFIRE_THRESHOLD 20.0f
// 减速热量阀值
#define HEAT_SLOWDOWN_THRESHOLD 50.0f
