/**
 * @file chassis.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 底盘控制实现
 * @date 2026-09-06
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "chassis.h"
#include "arm_math.h"

#include "basic_math.h"
#include "can_distribution.h"
#include "fdcan.h"
#include "freertos.h"
#include "cmsis_os2.h"

#include "pid.h"
#include "wave_generator.h"
#include "sys_phased_freq.h"

#include "general_filter.h"

#include <math.h>

// 底盘控制总结构体实例
chassis_t chassis; 

wave_generator_t motor_wave;

/*------------------------------------底盘电机can接收对象(注册到统一分发层）-------------------------------------------*/
static void chassis_wheel_0_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.wheel_motor[0].can_rx_callback();     // CAN1 0x201
}

static void chassis_wheel_1_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.wheel_motor[1].can_rx_callback(); // CAN1 0x202
}

static void chassis_wheel_2_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.wheel_motor[2].can_rx_callback();   // CAN1 0x203
}

static void chassis_wheel_3_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.wheel_motor[3].can_rx_callback();   // CAN1 0x204
}

static void chassis_steer_0_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.steer_motor[0].can_rx_callback();   // CAN2 0x205
}

static void chassis_steer_1_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.steer_motor[1].can_rx_callback();   // CAN2 0x206
}

static void chassis_steer_2_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.steer_motor[2].can_rx_callback();   // CAN2 0x207
}

static void chassis_steer_3_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    chassis.steer_motor[3].can_rx_callback();   // CAN2 0x208
}

/**
 * @brief 底盘初始化函数
 * @note  初始化底盘相关参数和电机
*/
void chassis_init(void)
{
    // 1. 初始化绑定观测对象
    chassis.crsf = &crsf;
    chassis.decision = &robot_decision;
    chassis.power = &power_data; // 功率计对象指针

    // 2. 初始化电机参数
    chassis.wheel_motor[0].init(&hfdcan1, Motor_DJI_ID_0x201, DJI_Motor_Control_Method_Torque, 268.0f/17.0f); // 左前轮电机
    chassis.wheel_motor[1].init(&hfdcan1, Motor_DJI_ID_0x202, DJI_Motor_Control_Method_Torque, 268.0f/17.0f); // 左后轮电机
    chassis.wheel_motor[2].init(&hfdcan1, Motor_DJI_ID_0x203, DJI_Motor_Control_Method_Torque, 268.0f/17.0f); // 右后轮电机
    chassis.wheel_motor[3].init(&hfdcan1, Motor_DJI_ID_0x204, DJI_Motor_Control_Method_Torque, 268.0f/17.0f); // 右前轮电机

    chassis.steer_motor[0].init(&hfdcan2, Motor_DJI_ID_0x205, DJI_Motor_Control_Method_Torque); // 左前转向电机
    chassis.steer_motor[1].init(&hfdcan2, Motor_DJI_ID_0x206, DJI_Motor_Control_Method_Torque); // 左后转向电机
    chassis.steer_motor[2].init(&hfdcan2, Motor_DJI_ID_0x207, DJI_Motor_Control_Method_Torque); // 右后转向电机
    chassis.steer_motor[3].init(&hfdcan2, Motor_DJI_ID_0x208, DJI_Motor_Control_Method_Torque); // 右前转向电机

    // 3. 初始化速度斜坡函数
    chassis.slope_speed[0].init(0.0045f, 0.0045f ,Slope_First_TARGET); // X轴速度斜坡
    chassis.slope_speed[1].init(0.0045f, 0.0045f ,Slope_First_TARGET); // Y轴速度斜坡
    chassis.slope_speed[2].init(0.0045f, 0.0045f, Slope_First_TARGET); // 角速度斜坡

    // 4. 初始化滤波器（一阶IIR低通，截止频率30Hz，采样周期1ms）
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.lowpass_filter[i].init(50.0f, 0.002f);
    }

    // 5. 舵向电机PID控制初始化
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.pid_steer_angle[i].init(12.5f, 0.0f, 0.1f, 0.0f, 1.0f, 8.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE); // 电机角度环PID初始化
        chassis.pid_steer_omega[i].init(0.12f, 0.98f, 0.0f, 0.0f, 0.85f, 1.2f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE); // 电机速度环PID初始化
    }

    // 轮向电机角速度PID控制
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.pid_wheel_omega[i].init(0.85f, 0.0f, 0.0f, 0.0f, 0.5f, 1.2f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE); // 电机速度环PID初始化
    }

    // 6. 设置底盘锁定模式
    chassis.now_method = Chassis_Lock_Method; 
    chassis.now_mode = Chassis_Zero_Force_Mode; // 默认底盘无力模式

    // 底盘功率控制初始化
    // chassis.control_power.init(15.0f, 0.0f, 1.0f);
    chassis.control_power.init();
    chassis.control_power.set_chassis_power_max(20.0f); // 设置底盘功率上限 20W(无电容/缓冲反馈, 暂用固定值)
    chassis.tgt_steer_state[0].angle = 1.0f; // 初始化目标舵向角度为 0

    wave_generator_init(&motor_wave, WAVE_SINE, 0.1f, 2.0f, 0.0f);
    wave_generator_enable(&motor_wave);

    can_distribution_register(1, 0x201,  chassis_wheel_0_motor_rx_client);
    can_distribution_register(1, 0x202,  chassis_wheel_1_motor_rx_client);
    can_distribution_register(1, 0x203,  chassis_wheel_2_motor_rx_client);
    can_distribution_register(1, 0x204,  chassis_wheel_3_motor_rx_client);

    can_distribution_register(2, 0x205, chassis_steer_0_motor_rx_client);
    can_distribution_register(2, 0x206, chassis_steer_1_motor_rx_client);
    can_distribution_register(2, 0x207, chassis_steer_2_motor_rx_client);
    can_distribution_register(2, 0x208, chassis_steer_3_motor_rx_client);

    SAFE_DAEMON_PROCESS_REGISTER("chassis_wheel_0", chassis.wheel_motor[0], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("chassis_wheel_1", chassis.wheel_motor[1], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("chassis_wheel_2", chassis.wheel_motor[2], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("chassis_wheel_3", chassis.wheel_motor[3], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);

    SAFE_DAEMON_PROCESS_REGISTER("chassis_steer_0", chassis.steer_motor[0], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("chassis_steer_1", chassis.steer_motor[1], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("chassis_steer_2", chassis.steer_motor[2], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("chassis_steer_3", chassis.steer_motor[3], SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
}
/**
 * 底盘模式切换
 * @brief 根据遥控器拨动开关状态，切换底盘工作模式
 */
void chassis_update_mode(void)
{

#if ROBOT_AMOUNT_BOARD

    switch (chassis.crsf->get_left_toggle())
    {
        case CRSF_SWITCH_DOWN:
            chassis.now_method = Chassis_Lock_Method; // 底盘锁定模式
            break;
        case CRSF_SWITCH_MID:
            chassis.now_method = Chassis_Manual_Method; // 底盘手动模式
            break;
        case CRSF_SWITCH_UP:
            chassis.now_method = Chassis_Auto_Method; // 底盘自动模式
            break;
        default:
            break;
    }

#else
    // 高四位表示底盘控制方式(自锁/手动/自动)
    switch ((chassis.decision->can_board_communication.get_control_mode() >> 4) & 0xF)
    {
        case CRSF_SWITCH_DOWN:
            chassis.now_method = Chassis_Lock_Method; // 底盘锁定模式
            break;
        case CRSF_SWITCH_MID:
            chassis.now_method = Chassis_Manual_Method; // 底盘手动模式
            break;
        case CRSF_SWITCH_UP:
            chassis.now_method = Chassis_Auto_Method; // 底盘自动模式
            break;

        default:
            chassis.now_method = Chassis_Lock_Method; // 默认底盘锁定模式
            break;
    }

    // 高四位表示底盘控制方式(自锁/手动/自动)
    switch ((chassis.decision->can_board_communication.get_control_mode() >> 4) & 0xF)
    {
        case 0:
            chassis.now_method = Chassis_Lock_Method; // 底盘锁定模式
            break;

        case 1:
            chassis.now_method = Chassis_Manual_Method; // 底盘手动模式
            break;

        case 2:
            chassis.now_method = Chassis_Auto_Method; // 底盘自动模式
            break;

        default:
            chassis.now_method = Chassis_Lock_Method; // 默认底盘锁定模式
            break;
    }

    // 低四位表示底盘控制模式(无力/跟随/小陀螺)
    switch (chassis.decision->can_board_communication.get_control_mode() & 0xF)
    {
        case 0:
            chassis.now_mode = Chassis_Zero_Force_Mode; // 底盘无力模式
            break;

        case 1:
            chassis.now_mode = Chassis_Follow_Mode; // 底盘跟随模式
            break;

        case 2:
            chassis.now_mode = Chassis_Rotate_Mode; // 底盘小陀螺模式
            break;

        default:
            chassis.now_mode = Chassis_Zero_Force_Mode; // 默认底盘无力模式
            break;
    }
#endif

}

/**
 * @brief 更新底盘测量值
 * @note  获取电机滤波后的反馈速度值,以及底盘正运动学解算，计算底盘的实际速度
 */
void chassis_update_measure(void)
{
    // 获取CAN板间通讯获取的旋转矩阵偏航角度
    chassis.yaw_angular_offset = robot_decision.can_board_communication.get_rotation_matrix_yaw_control(); 

    // 更新电机当前角度和速度值
    for (uint8_t i = 0; i < 4; i++)
    {
        // 获取舵向电机当前角度值
        // 舵向位置环使用连续总角度，避免每圈边界处目标角跳变。
        chassis.now_steer_state[i].angle = chassis.steer_motor[i].get_now_angle(); 
        chassis.now_steer_state[i].speed = chassis.steer_motor[i].get_now_omega(); 

        // 获取轮向电机当前速度值
        chassis.now_wheel_state[i].angle = chassis.wheel_motor[i].get_now_angle();
        chassis.now_wheel_state[i].speed = chassis.wheel_motor[i].get_now_omega(); 
    }

    // 更新舵轮解算的相关参数
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.mtr_stat.now_angle[i] = chassis.now_steer_state[i].angle;
        chassis.mtr_stat.now_speed[i] = chassis.now_wheel_state[i].speed;
    }

    // 更新滤波后的轮电机速度值（一阶IIR低通，30Hz截止）
    
    // 使用底盘正运动学解算，计算底盘的实际速度
   
}

/**
 * @brief 底盘整体遥控器控制函数
 * @note  根据远程控制指令，更新底盘的期望速度和期望位置等控制参数，进行底盘的远程控制
 */
void chassis_whole_remote_control(void)
{

#if ROBOT_AMOUNT_BOARD == 1     // 单板(一块H7)

    // 直接使用遥控器控制底盘速度
    chassis.target_robot_speed.vx = basic_math_dead_constrain((chassis.crsf->get_right_x() * CHASSIS_MAX_MOVE_SPEED), -CHASSIS_MAX_MOVE_SPEED, CHASSIS_MAX_MOVE_SPEED, 0.05f); // 将遥控器右摇杆Y轴值映射为底盘的x轴速度，并进行限幅和死区处理
    chassis.target_robot_speed.vy = basic_math_dead_constrain((chassis.crsf->get_right_y() * CHASSIS_MAX_MOVE_SPEED), -CHASSIS_MAX_MOVE_SPEED, CHASSIS_MAX_MOVE_SPEED, 0.05f); // 将遥控器右摇杆X轴值映射为底盘的y轴速度，并进行限幅和死区处理
    chassis.target_robot_speed.vw = basic_math_dead_constrain((chassis.crsf->get_wheel() * CHASSIS_MAX_ROTATE_SPEED), -CHASSIS_MAX_ROTATE_SPEED, CHASSIS_MAX_ROTATE_SPEED, 0.05f); // 将遥控器滚轮值映射为底盘的角速度，并进行限幅和死区处理

#elif ROBOT_AMOUNT_BOARD == 2   // 双板(两块H7)

    // 使用云台板发送的数据进行底盘控制
    if(chassis.now_mode == Chassis_Zero_Force_Mode)     // 底盘无力模式
    {
        chassis.target_robot_speed.vx = 0.0f; // 底盘无力模式下，底盘的目标速度为0
        chassis.target_robot_speed.vy = 0.0f; // 底盘无力模式下，底盘的目标速度为0
        chassis.target_robot_speed.vw = 0.0f; // 底盘无力模式下，底盘的目标角速度为0
    }

    else if(chassis.now_mode == Chassis_Follow_Mode)    // 底盘跟随模式
    {
        // 底盘跟随模式下，底盘的目标速度为CAN板间通讯获取的速度值
        chassis.target_robot_speed.vx = basic_math_dead_constrain(robot_decision.can_board_communication.get_vx_control(), -CHASSIS_MAX_MOVE_SPEED, CHASSIS_MAX_MOVE_SPEED, 0.085f); // 将CAN板间通讯获取的vx值映射为底盘的x轴速度，并进行限幅和死区处理
        chassis.target_robot_speed.vy = basic_math_dead_constrain(robot_decision.can_board_communication.get_vy_control(), -CHASSIS_MAX_MOVE_SPEED, CHASSIS_MAX_MOVE_SPEED, 0.085f); // 将CAN板间通讯获取的vy值映射为底盘的y轴速度，并进行限幅和死区处理
        chassis.target_robot_speed.vw = basic_math_dead_constrain(robot_decision.can_board_communication.get_vz_control(), -CHASSIS_MAX_ROTATE_SPEED, CHASSIS_MAX_ROTATE_SPEED, 0.085f); // 将CAN板间通讯获取的wz值映射为底盘的角速度，并进行限幅和死区处理
    }

    else if(chassis.now_mode == Chassis_Rotate_Mode)    // 底盘小陀螺模式
    {
        // 底盘小陀螺模式下，底盘的目标速度为CAN板间通讯获取的速度值
        chassis.target_robot_speed.vx = basic_math_dead_constrain(robot_decision.can_board_communication.get_vx_control(), -CHASSIS_MAX_MOVE_SPEED, CHASSIS_MAX_MOVE_SPEED, 0.05f); // 将CAN板间通讯获取的vx值映射为底盘的x轴速度，并进行限幅和死区处理
        chassis.target_robot_speed.vy = basic_math_dead_constrain(robot_decision.can_board_communication.get_vy_control(), -CHASSIS_MAX_MOVE_SPEED, CHASSIS_MAX_MOVE_SPEED, 0.05f); // 将CAN板间通讯获取的vy值映射为底盘的y轴速度，并进行限幅和死区处理
        chassis.target_robot_speed.vw = CHASSIS_MAX_ROTATE_SPEED; // 底盘小陀螺模式下，底盘的目标角速度为3.5rad/s
    }

#endif

}

/**
 * @brief 底盘整体自动控制函数()
 */
void chassis_whole_auto_control(void)
{
    // 这里可以添加底盘自动控制的逻辑，例如根据传感器数据或预设路径进行控制
}

// 底盘整体移动控制函数，根据底盘工作模式，选择不同的控制方式
void chassis_overall_mobile_control(void)
{
    float sin_deflection = 0.0f, cos_deflection = 0.0f;

    // 根据当前控制方式选择目标速度来源(先更新target_robot_speed，再计算旋转矩阵)
    switch (chassis.now_method)
    {
        case Chassis_Lock_Method:
            // 底盘锁定模式下，目标速度直接置零
            chassis.target_robot_speed.vx = 0.0f;
            chassis.target_robot_speed.vy = 0.0f;
            chassis.target_robot_speed.vw = 0.0f;
            break;

        case Chassis_Manual_Method:
            chassis_whole_remote_control(); // 底盘整体遥控器控制 → 更新target_robot_speed
            break;

        case Chassis_Auto_Method:
            chassis_whole_auto_control(); // 底盘整体自动运动控制 → 更新target_robot_speed
            break;

        default:
            break;
    }

    // 旋转矩阵的单位正余弦x,y分量(使用最新的target_robot_speed和yaw_angular_offset)
    cos_deflection = arm_cos_f32(chassis.yaw_angular_offset);
    sin_deflection = arm_sin_f32(chassis.yaw_angular_offset);

    // 将目标速度进行旋转矩阵变换，从云台坐标系转换到底盘坐标系
    chassis.map_robot_speed.vx = chassis.target_robot_speed.vx * cos_deflection - chassis.target_robot_speed.vy * sin_deflection; // 将目标速度进行旋转矩阵变换
    chassis.map_robot_speed.vy = chassis.target_robot_speed.vx * sin_deflection + chassis.target_robot_speed.vy * cos_deflection; // 将目标速度进行旋转矩阵变换
    chassis.map_robot_speed.vw = chassis.target_robot_speed.vw; // 角速度不需要旋转矩阵变换

    chassis.slope_speed[0].set_target(chassis.map_robot_speed.vx);      // 设置斜坡函数的目标值为底盘的期望速度
    // chassis.slope_speed[0].set_now_real(chassis.now_robot_speed.vx); // 设置斜坡函数的当前真实值为底盘的实际速度
    chassis.slope_speed[0].slope_calculate_callback();                  // 计算斜坡函数的输出值
    chassis.slope_output_speed.vx = chassis.slope_speed[0].get_out();   // 获取斜坡函数的输出值作为底盘的期望速度

    chassis.slope_speed[1].set_target(chassis.map_robot_speed.vy);      // 设置斜坡函数的目标值为底盘的期望速度
    // chassis.slope_speed[1].set_now_real(chassis.now_robot_speed.vy); // 设置斜坡函数的当前真实值为底盘的实际速度
    chassis.slope_speed[1].slope_calculate_callback();                  // 计算斜坡函数的输出值
    chassis.slope_output_speed.vy = chassis.slope_speed[1].get_out();   // 获取斜坡函数的输出值作为底盘的期望速度

    chassis.slope_speed[2].set_target(chassis.map_robot_speed.vw);      // 设置斜坡函数的目标值为底盘的期望角速度
    // chassis.slope_speed[2].set_now_real(chassis.now_robot_speed.vw); // 设置斜坡函数的当前真实值为底盘的实际角速度
    chassis.slope_speed[2].slope_calculate_callback();                  // 计算斜坡函数的输出值
    chassis.slope_output_speed.vw = chassis.slope_speed[2].get_out();   // 获取斜坡函数的输出值作为底盘的期望角速度
}

/**
 * @brief 底盘电机计算函数
 * @note  计算电机控制参数
 */
void chassis_motor_calculate(void)
{
    // 1. 运动学解算，计算轮子目标速度
    steer_wheel_inverse_kinematics(&chassis.mtr_stat, chassis.slope_output_speed.vx, chassis.slope_output_speed.vy, chassis.slope_output_speed.vw, chassis.now_mode); // 计算轮子目标速度

    // 2. 舵向电机PID计算，计算电机控制参数
    for (uint8_t i = 0; i < 4; i++)
    {
        // 位置环
        chassis.pid_steer_angle[i].set_now(chassis.now_steer_state[i].angle); // 设置电机当前角度值
        chassis.pid_steer_angle[i].set_target(chassis.mtr_stat.optimize_angle[i]); // 设置电机目标角度值
        chassis.pid_steer_angle[i].pid_calculate_callback(); // 计算电机控制参数
        chassis.tgt_steer_state[i].speed = chassis.pid_steer_angle[i].get_out(); // 获取电机控制参数

        // 速度环
        chassis.pid_steer_omega[i].set_now(chassis.now_steer_state[i].speed); // 设置电机当前速度值
        chassis.pid_steer_omega[i].set_target(chassis.tgt_steer_state[i].speed); // 设置电机目标速度值
        chassis.pid_steer_omega[i].pid_calculate_callback(); // 计算电机控制参数
        chassis.steer_motor[i].set_target_torque(chassis.pid_steer_omega[i].get_out()); // 设置电机目标转矩值
        chassis.steer_motor[i].external_calculate_callback();
    }

    // 轮向电机PID计算，计算电机控制参数
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.pid_wheel_omega[i].set_now(chassis.now_wheel_state[i].speed);
        chassis.pid_wheel_omega[i].set_target(chassis.mtr_stat.optimize_speed[i]);
        chassis.pid_wheel_omega[i].pid_calculate_callback();
        chassis.wheel_motor[i].set_target_torque(chassis.pid_wheel_omega->get_out()); // 设置轮向电机目标转矩值
        chassis.wheel_motor[i].external_calculate_callback();
    }

    // 发送数据
    bsp_can_1_to_4_transmit(&hfdcan2, 0x1fe, can2_0x1fe_tx_data, Can_transmit_frequency_1000Hz); // 发送电机控制参数
    bsp_can_1_to_4_transmit(&hfdcan1, 0x200, can1_0x200_tx_data, Can_transmit_frequency_1000Hz); // 发送电机控制参数
}

/**
 * @brief 底盘电机PID测试
 */
void chasiss_motor_test(void)
{
    // 2. 电机PID计算，计算电机控制参数
    for (uint8_t i = 0; i < 4; i++)
    {
        // 位置环
        chassis.pid_steer_angle[i].set_now(chassis.now_steer_state[i].angle); // 设置电机当前角度值
        chassis.pid_steer_angle[i].set_target(chassis.tgt_steer_state[i].angle); // 设置电机目标角度值
        chassis.pid_steer_angle[i].pid_calculate_callback(); // 计算电机控制参数
        chassis.tgt_steer_state[i].speed = chassis.pid_steer_angle[i].get_out(); // 获取电机控制参数

        // 速度环
        chassis.pid_steer_omega[i].set_now(chassis.now_steer_state[i].speed); // 设置电机当前速度值
        chassis.pid_steer_omega[i].set_target(chassis.tgt_steer_state[i].speed); // 设置电机目标速度值
        chassis.pid_steer_omega[i].pid_calculate_callback(); // 计算电机控制参数
        chassis.steer_motor[i].set_target_torque(chassis.pid_steer_omega[i].get_out()); // 设置电机目标转矩值
        chassis.steer_motor[i].external_calculate_callback();
    }

    // 轮向电机PID计算，计算电机控制参数
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.pid_wheel_omega[i].set_now(chassis.now_wheel_state[i].speed);
        chassis.pid_wheel_omega[i].set_target(chassis.tgt_wheel_state[i].speed);
        chassis.pid_wheel_omega[i].pid_calculate_callback();
        chassis.wheel_motor[i].set_target_torque(chassis.pid_wheel_omega->get_out()); // 设置轮向电机目标转矩值
        chassis.wheel_motor[i].external_calculate_callback();
    }

    // 发送数据
    bsp_can_1_to_4_transmit(&hfdcan2, 0x1fe, can2_0x1fe_tx_data, Can_transmit_frequency_1000Hz); // 发送电机控制参数
    bsp_can_1_to_4_transmit(&hfdcan1, 0x200, can1_0x200_tx_data, Can_transmit_frequency_1000Hz); // 发送电机控制参数
}

/*
void chassis_power_calculate(void)
{
    // initial_give_power = rotor_toque * chassis.wheel_motor[1].get_out() * chassis.wheel_motor[1].get_rotor_omega() + 
    //                      Power_K1 * chassis.wheel_motor[1].get_rotor_omega() * chassis.wheel_motor[1].get_rotor_omega() + 
    //                      Power_K2 * chassis.wheel_motor[1].get_out() * chassis.wheel_motor[1].get_out() * rotor_toque * rotor_toque + Power_C;

    // rx_give_power = rotor_toque * chassis.wheel_motor[1].get_rotor_current() * chassis.wheel_motor[1].get_rotor_omega() + 
    //                 Power_K1 * chassis.wheel_motor[1].get_rotor_omega() * chassis.wheel_motor[1].get_rotor_omega() + 
    //                 Power_K2 * chassis.wheel_motor[1].get_rotor_current() * chassis.wheel_motor[1].get_rotor_current() * rotor_toque * rotor_toque + Power_C;

    // lowpass_filter_rx.set_now_input(rx_give_power);
    // lowpass_filter_rx.low_calculate_callback();

    // lowpass_filter_initial.set_now_input(initial_give_power);
    // lowpass_filter_initial.low_calculate_callback();

    // lowpass_filter_power.set_now_input((float)chassis.power->power);
    // lowpass_filter_power.low_calculate_callback();

    chassis.motor_rotor_omega[0] = chassis.wheel_motor[0].get_rotor_omega();
    chassis.motor_rotor_omega[1] = chassis.wheel_motor[1].get_rotor_omega();
    chassis.motor_rotor_omega[2] = chassis.wheel_motor[2].get_rotor_omega();
    chassis.motor_rotor_omega[3] = chassis.wheel_motor[3].get_rotor_omega();

    chassis.motor_rotor_current[0] = chassis.wheel_motor[0].get_rotor_current();
    chassis.motor_rotor_current[1] = chassis.wheel_motor[1].get_rotor_current();
    chassis.motor_rotor_current[2] = chassis.wheel_motor[2].get_rotor_current();
    chassis.motor_rotor_current[3] = chassis.wheel_motor[3].get_rotor_current();

    // 运动学逆解

    // 转子目标角速度
    chassis.motor_target_omega[0] = chassis.target_motor_status.wheel_speed[0] * (3591.0f / 187.0f);
    chassis.motor_target_omega[1] = chassis.target_motor_status.wheel_speed[1] * (3591.0f / 187.0f);
    chassis.motor_target_omega[2] = chassis.target_motor_status.wheel_speed[2] * (3591.0f / 187.0f);
    chassis.motor_target_omega[3] = chassis.target_motor_status.wheel_speed[3] * (3591.0f / 187.0f);

    chassis.pid_steer_omega[0].set_now(chassis.now_motor_status.wheel_speed[0]);
    chassis.pid_steer_omega[1].set_now(chassis.now_motor_status.wheel_speed[1]);
    chassis.pid_steer_omega[2].set_now(chassis.now_motor_status.wheel_speed[2]);
    chassis.pid_steer_omega[3].set_now(chassis.now_motor_status.wheel_speed[3]);

    chassis.pid_steer_omega[0].set_target(chassis.target_motor_status.wheel_speed[0]);
    chassis.pid_steer_omega[1].set_target(chassis.target_motor_status.wheel_speed[1]);
    chassis.pid_steer_omega[2].set_target(chassis.target_motor_status.wheel_speed[2]);
    chassis.pid_steer_omega[3].set_target(chassis.target_motor_status.wheel_speed[3]);

    chassis.pid_steer_omega[0].pid_calculate_callback();
    chassis.pid_steer_omega[1].pid_calculate_callback();
    chassis.pid_steer_omega[2].pid_calculate_callback();
    chassis.pid_steer_omega[3].pid_calculate_callback();

    chassis.motor_target_output[0] = chassis.pid_steer_omega[0].get_out() / (0.3f / (3591.0f / 187.0f)) / (3591.0f / 187.0f) * (16384.0f / 20.0f);
    chassis.motor_target_output[1] = chassis.pid_steer_omega[1].get_out() / (0.3f / (3591.0f / 187.0f)) / (3591.0f / 187.0f) * (16384.0f / 20.0f);
    chassis.motor_target_output[2] = chassis.pid_steer_omega[2].get_out() / (0.3f / (3591.0f / 187.0f)) / (3591.0f / 187.0f) * (16384.0f / 20.0f);
    chassis.motor_target_output[3] = chassis.pid_steer_omega[3].get_out() / (0.3f / (3591.0f / 187.0f)) / (3591.0f / 187.0f) * (16384.0f / 20.0f);

    // chassis.motor_out[0] = 0.0f;
    // chassis.motor_out[1] = 0.0f;
    // chassis.motor_out[2] = 0.0f;
    // chassis.motor_out[3] = 0.0f;

    // chassis.control_power.update_measure(chassis.motor_rotor_current, chassis.motor_rotor_omega);
    // chassis.control_power.power_calculate_callback(chassis.motor_out, chassis.motor_rotor_omega);

    // chassis.wheel_motor[0].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(0));
    // chassis.wheel_motor[1].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(1));
    // chassis.wheel_motor[2].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(2));
    // chassis.wheel_motor[3].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(3));

    // chassis.wheel_motor[0].power_control_calculate_callback(chassis.motor_out[0]);
    // chassis.wheel_motor[1].power_control_calculate_callback(chassis.motor_out[1]);
    // chassis.wheel_motor[2].power_control_calculate_callback(chassis.motor_out[2]);
    // chassis.wheel_motor[3].power_control_calculate_callback(chassis.motor_out[3]);

    chassis.control_power.update_measure(chassis.motor_rotor_omega, chassis.motor_rotor_current, chassis.motor_target_omega, chassis.motor_target_output, chassis.motor_alive_status);
    chassis.control_power.calculate_callback();
    chassis.control_power.rls_calculate_callback(chassis.power->power);

    chassis.wheel_motor[0].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(0));
    chassis.wheel_motor[1].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(1));
    chassis.wheel_motor[2].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(2));
    chassis.wheel_motor[3].power_control_calculate_callback(chassis.control_power.get_motor_rotor_out(3));

    bsp_can_1_to_4_transmit(&hfdcan1, 0x200, can1_0x200_tx_data, Can_transmit_frequency_1000Hz); // 发送电机控制参数
}
*/

/**
 * @brief 底盘任务函数
 */
extern "C" void chassis_task(void *argument)
{
    // 初始化底盘相关参数和电机
    chassis_init(); 
    // chassis_init_test(); // 初始化底盘相关参数和电机(测试用)

    // 延时等待底盘初始化完成
    osDelay(1000); 

    for(;;)
    {
        // 1. 更新底盘控制模式
        chassis_update_mode();

        // 2. 更新底盘测量值
        chassis_update_measure();

        // 3. 底盘整体移动控制
        chassis_overall_mobile_control();

        // 4. 底盘电机计算
        // chasiss_motor_test();
        chassis_motor_calculate();   // 常规闭环控制
        // chassis_motor_sys_indef();   // 扫频辨识占用本帧(开环力矩输出)
        // chassis_power_calculate();
        // chassis_lqr_motor_test(); // LQR 控制器测试

        // chassis_motor_control_test(); // 底盘电机控制测试

        // 5. 延时等待下一次循环
        osDelay(2); 
    }
}
