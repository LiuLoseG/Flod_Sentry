/**
 * @file shoot.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 摩擦轮电机控制
 * @date 2026-07-16
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#include "shoot.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "dji_motor_new.h"
#include "freertos.h"

// 统一CAN分发层
#include "can_distribution.h"

// 安全守护进程(电机离线检测)
#include "safe_daemon_process.h"

#include "pid.h"
#include "robot_def.h"
#include "wave_generator.h"
#include <cmath>

shoot_t shoot;

// PID波形调节器
wave_generator_t trigger_wave;

/*------------------------------------发射机构电机can接收对象(注册到统一分发层）-------------------------------------------*/
static void shoot_fric_r_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    shoot.fric_r_motor.can_rx_callback();     // CAN2 0x201
}

static void shoot_fric_l_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    shoot.fric_l_motor.can_rx_callback();     // CAN2 0x202
}

static void shoot_trigger_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    shoot.trigger_motor.can_rx_callback();    // CAN2 0x203 拨盘电机
}

/**
 * @brief 发射机构初始化
 * @note 初始化摩擦轮电机、拨盘电机、滤波器、PID控制器和斜坡函数
 */
void shoot_init(void)
{
    // 绑定遥控器对象
    shoot.crsf = &crsf;
    shoot.source = &shoot_decision_source;
    shoot.decision = &robot_decision;

    // 滤波器初始化
    shoot.fric_r_filter.init(FRICT_TORQUE_ALPHA_MIN, FRICT_TORQUE_ALPHA_MAX, FRICT_TORQUE_ERR_LOW, FRICT_TORQUE_ERR_HIGH);
    shoot.fric_l_filter.init(FRICT_TORQUE_ALPHA_MIN, FRICT_TORQUE_ALPHA_MAX, FRICT_TORQUE_ERR_LOW, FRICT_TORQUE_ERR_HIGH);
    shoot.trigger_filter.init(TRIGGER_TORQUE_ALPHA_MIN, TRIGGER_TORQUE_ALPHA_MAX, TRIGGER_TORQUE_ERR_LOW, TRIGGER_TORQUE_ERR_HIGH);

    // 电机初始化
    shoot.fric_r_motor.init(&hfdcan2, Motor_DJI_ID_0x201, DJI_Motor_Control_Method_Omega, 1.0f);
    shoot.fric_l_motor.init(&hfdcan2, Motor_DJI_ID_0x202, DJI_Motor_Control_Method_Omega, 1.0f);

    shoot.trigger_motor.init(&hfdcan2, Motor_DJI_ID_0x203, DJI_Motor_Control_Method_Angle);

    //pid初始化
    shoot.fric_r_motor.pid_omega.init(0.00135f, 0.0f, 0.0f, 0.0f, 5.0f, 20.0f);
    shoot.fric_l_motor.pid_omega.init(0.00135f, 0.0f, 0.0f, 0.0f, 5.0f, 20.0f);

    shoot.trigger_motor.pid_angle.init(30.0f, 0.0f, 0.0f, 0.0f, 5.0f, 35.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    shoot.trigger_motor.pid_omega.init(0.85f, 0.0f, 0.004f, 0.0f, 5.0f, 10.0f);

    // 斜坡函数初始化，摩擦轮电机转速缓慢上升
    shoot.fric_r_slope.init(0.12f, 0.12f, Slope_First_REAL);
    shoot.fric_l_slope.init(0.12f, 0.12f, Slope_First_REAL);

    shoot.target_value.fric_r_omega = 0.0f;
    shoot.target_value.fric_l_omega = 0.0f;
    shoot.target_value.trigger_angle = 0.0f;

    shoot.now_method = Shoot_Method_Zero_Force;
    shoot.now_mode = Shoot_Disable;
    shoot.last_mode = Shoot_Disable;
    shoot.allow_flag = 0;               // 0:禁止发射(未达到发射条件) 1:允许发射
    shoot.fric_spin_up_flag = false;    // 摩擦轮电机转速上升标志位, 0:未上升, 1:已上升

    // 波形生成器初始化，拨盘电机角度跟踪波形
    wave_generator_init(&trigger_wave, WAVE_SQUARE, 0.2f, 1.7453f, 0.0f);
    wave_generator_enable(&trigger_wave);

    // 注册 CAN 接收分发规则（统一分发层）
    can_distribution_register(2, 0x201, shoot_fric_r_motor_rx_client);
    can_distribution_register(2, 0x202, shoot_fric_l_motor_rx_client);
    can_distribution_register(2, 0x203, shoot_trigger_motor_rx_client);  

    // 注册电机离线检测（安全守护进程统一调用 alive_check, 只检测状态不做安全动作）
    //     检测周期由 SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS 决定, 即掉线判定时延(单位 ms)
    SAFE_DAEMON_PROCESS_REGISTER("fric_r_motor", shoot.fric_r_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("fric_l_motor", shoot.fric_l_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("trigger_motor", shoot.trigger_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
}

/**
 * @brief 更新发射机构控制模式
 */
void shoot_update_mode(void)
{
    // 卡弹状态机计算
    shoot.jamming_fsm.fsm_calculate_callback();  // 卡弹状态机计算

    switch (shoot.crsf->get_left_toggle())
    {
        case CRSF_SWITCH_UP:
        {
            shoot.now_method = Shoot_Method_Auto;

            // 获取发射状态机的当前发射状态, 0:允许发射, 1:禁止发射
            if((shoot.decision->pc_to_mcu_aim.get_control_mode()==2))
            {
                shoot.allow_flag = 1;   // 允许发射
                shoot.now_mode = Shoot_Auto;
            }

            else 
            {
                shoot.allow_flag = 0;   // 允许发射
                shoot.now_mode = Shoot_Ceasefire;
            
            }
            break;
        }
        
        case CRSF_SWITCH_MID:
        {
            shoot.now_method = Shoot_Method_Manual;
            
            if(shoot.decision->fire_fsm.get_now_fire_status() == Robot_Fsm_Fire_Status_Repeat)
            {
                shoot.now_mode = Shoot_Repeat;
            }

            else if(shoot.decision->fire_fsm.get_now_fire_status() == Robot_Fsm_Fire_Status_Single)
            {
                shoot.now_mode = Shoot_Single;
            }

            else if(shoot.decision->fire_fsm.get_now_fire_status() == Robot_Fsm_Fire_Status_Stop)
            {
                shoot.now_mode = Shoot_Ceasefire;
            }

            else if(shoot.decision->fire_fsm.get_now_fire_status() == Robot_Fsm_Fire_Status_Free)
            {
                shoot.now_mode = Shoot_Ceasefire;
            }

            break;
        }

        case CRSF_SWITCH_DOWN:
        {
            shoot.now_method = Shoot_Method_Zero_Force;
            shoot.now_mode = Shoot_Disable;
            break;
        }

        default:
        {
            shoot.now_method = Shoot_Method_Zero_Force;
            shoot.now_mode = Shoot_Disable;
            break;
        }
    }
}

/**
 * @brief 更新测量值
 */
void shoot_update_measure(void)
{
    // 更新当前观测值
    shoot.now_value.fric_r_omega = shoot.fric_r_motor.get_rotor_omega();
    shoot.now_value.fric_l_omega = shoot.fric_l_motor.get_rotor_omega();
    shoot.now_value.fric_r_torque = shoot.fric_r_motor.get_now_torque();
    shoot.now_value.fric_l_torque = shoot.fric_l_motor.get_now_torque();

    shoot.now_value.trigger_angle = shoot.trigger_motor.get_now_angle();
    shoot.now_value.trigger_omega = shoot.trigger_motor.get_now_omega();
    shoot.now_value.trigger_torque = shoot.trigger_motor.get_now_torque();

    // 更新滤波值 (用于观测, 不参与控制)
    shoot.filter_value.fric_r_omega = shoot.now_value.fric_r_omega;
    shoot.filter_value.fric_l_omega = shoot.now_value.fric_l_omega;
    // 摩擦轮扭矩自适应滤波: 平稳时深度平滑, 真实力矩事件时快速跟踪
    shoot.fric_r_filter.set_now_input(shoot.now_value.fric_r_torque);
    shoot.fric_l_filter.set_now_input(shoot.now_value.fric_l_torque);
    shoot.fric_r_filter.adaptive_calculate_callback();
    shoot.fric_l_filter.adaptive_calculate_callback();
    shoot.filter_value.fric_r_torque = shoot.fric_r_filter.get_out();
    shoot.filter_value.fric_l_torque = shoot.fric_l_filter.get_out();

    shoot.filter_value.trigger_angle = shoot.now_value.trigger_angle;
    shoot.filter_value.trigger_omega = shoot.now_value.trigger_omega;
    // 拨盘扭矩自适应滤波: 平稳时深度平滑, 真实力矩事件时快速跟踪
    shoot.trigger_filter.set_now_input(shoot.now_value.trigger_torque);
    shoot.trigger_filter.adaptive_calculate_callback();
    shoot.filter_value.trigger_torque = shoot.trigger_filter.get_out();
}

/**
 * @brief 发射机构摩擦轮转速准备就绪状态
 * @retval true: 准备就绪
 * @retval false: 未就绪
 */
bool shoot_fric_omega_ready(void)
{
    return (fabs(shoot.now_value.fric_r_omega + FRICT_MOTOR_SPEED_SET) < FRICT_MOTOR_SPEED_DELTA_THRESHOLD
        && fabs(shoot.now_value.fric_l_omega - FRICT_MOTOR_SPEED_SET) < FRICT_MOTOR_SPEED_DELTA_THRESHOLD);
}

/**
 * @brief 摩擦轮电机失能切换提速
 */
void shoot_frict_motor_spin_up(void)
{
    if (shoot.last_mode == Shoot_Disable && shoot.now_mode != Shoot_Disable)
    {
        shoot.fric_spin_up_flag = true;  // 摩擦轮电机提速
    }

    else if(shoot.now_mode == Shoot_Disable)
    {
        shoot.fric_spin_up_flag = false;  // 摩擦轮电机停止
    }

    if(shoot.fric_spin_up_flag && shoot_fric_omega_ready())
    {
        shoot.fric_spin_up_flag = false;  // 摩擦轮电机提速完成
    }

    else if(shoot.now_mode != Shoot_Disable && shoot.fric_spin_up_flag)
    {
        shoot.now_mode = Shoot_Ceasefire;  // 摩擦轮电机提速中, 进入停火模式
    }
}

/**
 * @brief 发射机构控制决策(控制目标设定)
 */
void shoot_control_decision(void)
{
    // 摩擦轮电机失能切换提速
    shoot_frict_motor_spin_up();  

    switch (shoot.now_mode)
    {
        case Shoot_Disable:   // 失能  
        {
            shoot.trigger_motor.set_control_method(DJI_Motor_Control_Method_Torque);

            // 摩擦轮电机目标转速设为0，拨盘电机保持当前位置
            shoot.target_value.fric_r_omega = 0.0f;
            shoot.target_value.fric_l_omega = 0.0f;

            shoot.target_value.trigger_angle = shoot.now_value.trigger_angle;
            shoot.target_value.trigger_omega = 0.0f;

            // 摩擦轮电机斜坡函数计算
            shoot.fric_r_motor.set_target_omega(shoot.fric_r_slope.get_out());
            shoot.fric_l_motor.set_target_omega(shoot.fric_l_slope.get_out());

            // 拨盘电机保持当前位置
            shoot.trigger_motor.set_target_torque(0.0f);
            break;
        }

        case Shoot_Ceasefire:   // 停火
        {
            // 停止发射模式下，摩擦轮电机正常控制，拨盘电机停止转动
            shoot.target_value.fric_r_omega = -FRICT_MOTOR_SPEED_SET;
            shoot.target_value.fric_l_omega = FRICT_MOTOR_SPEED_SET;
            shoot.target_value.trigger_omega = 0.0f;

            // 摩擦轮电机控制函数
            if(shoot.fric_spin_up_flag) // 摩擦轮电机提速中，使用斜坡函数控制摩擦轮电机转速
            {
                shoot.fric_r_motor.set_target_omega(shoot.fric_r_slope.get_out());
                shoot.fric_l_motor.set_target_omega(shoot.fric_l_slope.get_out());
            }

            else    // 摩擦轮电机提速完成，直接控制摩擦轮电机转速
            {
                shoot.fric_r_motor.set_target_omega(shoot.target_value.fric_r_omega);
                shoot.fric_l_motor.set_target_omega(shoot.target_value.fric_l_omega);
            }

            // shoot.fric_r_motor.set_target_omega(shoot.target_value.fric_r_omega);
            // shoot.fric_l_motor.set_target_omega(shoot.target_value.fric_l_omega);

            if(shoot.trigger_motor.get_control_method() == DJI_Motor_Control_Method_Angle)
            {
                shoot.target_value.trigger_omega = shoot.trigger_motor.get_target_omega();
                shoot.trigger_motor.set_target_angle(shoot.target_value.trigger_angle);
            }

            else if(shoot.trigger_motor.get_control_method() == DJI_Motor_Control_Method_Omega)
            {
                shoot.target_value.trigger_angle = shoot.now_value.trigger_angle;
                shoot.trigger_motor.set_target_omega(shoot.target_value.trigger_omega);
            }

            else 
            {
                shoot.trigger_motor.set_target_torque(0.0f);
            }

            break;
        }

        case Shoot_Single:   // 单发
        {
            shoot.trigger_motor.set_control_method(DJI_Motor_Control_Method_Angle);
            shoot.fric_r_motor.set_control_method(DJI_Motor_Control_Method_Omega);
            shoot.fric_l_motor.set_control_method(DJI_Motor_Control_Method_Omega);

            // 单发模式下，摩擦轮电机正常控制，拨盘电机按照波形生成器输出角度进行控制
            shoot.target_value.fric_r_omega = -FRICT_MOTOR_SPEED_SET;
            shoot.target_value.fric_l_omega = FRICT_MOTOR_SPEED_SET;
            shoot.target_value.trigger_omega = shoot.trigger_motor.get_target_omega();

            // 摩擦轮电机控制函数
            shoot.fric_r_motor.set_target_omega(shoot.fric_r_slope.get_out());
            shoot.fric_l_motor.set_target_omega(shoot.fric_l_slope.get_out());

            // 热量控制机制
            if ((shoot.decision->heat_fsm.get_now_heat() + 30.0f) <= shoot.decision->get_heat_maximum())
            {
                shoot.target_value.trigger_angle = shoot.now_value.trigger_angle + TRIGGER_PILL_TURN_RAD;  // 拨盘电机转动一个弹丸的角度
                shoot.trigger_motor.set_target_angle(shoot.target_value.trigger_angle);
            }

            shoot.now_mode = Shoot_Ceasefire;  // 单发模式执行完毕后，进入停火模式

            break;
        }

        case Shoot_Repeat:   // 连发
        {
            shoot.trigger_motor.set_control_method(DJI_Motor_Control_Method_Omega);
            shoot.fric_r_motor.set_control_method(DJI_Motor_Control_Method_Omega);
            shoot.fric_l_motor.set_control_method(DJI_Motor_Control_Method_Omega);

            // 连发模式下，摩擦轮电机正常控制，拨盘电机按照波形生成器输出角度进行控制
            shoot.target_value.fric_r_omega = -FRICT_MOTOR_SPEED_SET;
            shoot.target_value.fric_l_omega = FRICT_MOTOR_SPEED_SET;
            shoot.target_value.trigger_angle = shoot.now_value.trigger_angle; 

            // 拨盘电机默认旋转速度18HZ(单位: rad/s)
            shoot.Heat_Balance_Omega = ((shoot.decision->get_heat_cd() / 10.0f) / TRIGGER_PILL_NUM) * (TRTGGER_TOOTHED_REDUCTION * 2.0f * PI);

            if(TRIGGER_DRIVER_OMEGA_DEFAULT <= shoot.Heat_Balance_Omega)
            {
                shoot.target_value.trigger_omega = TRIGGER_DRIVER_OMEGA_DEFAULT;
            }

            else
            {
                // 热量平衡，拨盘电机按照热量比例调整旋转速度
                shoot.target_value.trigger_omega = (shoot.Heat_Balance_Omega - TRIGGER_DRIVER_OMEGA_DEFAULT) / shoot.decision->get_heat_maximum()
                * (shoot.decision->heat_fsm.get_now_heat() + 30.0f) + TRIGGER_DRIVER_OMEGA_DEFAULT;
            }

            // 摩擦轮电机控制函数
            shoot.fric_r_motor.set_target_omega(shoot.fric_r_slope.get_out());
            shoot.fric_l_motor.set_target_omega(shoot.fric_l_slope.get_out());

            shoot.trigger_motor.set_target_omega(shoot.target_value.trigger_omega);


            break;
        }

        case Shoot_Auto:    // 自瞄发射
        {
            float tmp_delta_heat = shoot.decision->get_heat_maximum() - shoot.decision->heat_fsm.get_now_heat();

            if(tmp_delta_heat >= HEAT_SLOWDOWN_THRESHOLD)   // 热量充足, 拨盘电机按照默认射速旋转
            {
                shoot.target_value.trigger_omega = TRIGGER_DRIVER_OMEGA_DEFAULT;
            }

            else if(tmp_delta_heat < HEAT_SLOWDOWN_THRESHOLD && tmp_delta_heat >= HEAT_CEASEFIRE_THRESHOLD) // 热量不足, 拨盘电机按照热量平衡射速旋转
            {
                shoot.target_value.trigger_omega = ((TRIGGER_AMMO_FREQUENCY_DEFAULT * (HEAT_CEASEFIRE_THRESHOLD - tmp_delta_heat) + shoot.decision->get_heat_cd() / 10.0f * (tmp_delta_heat - HEAT_SLOWDOWN_THRESHOLD)) 
                                                    / (HEAT_CEASEFIRE_THRESHOLD - HEAT_SLOWDOWN_THRESHOLD)) * ((TRTGGER_TOOTHED_REDUCTION * 2.0f * PI) / TRIGGER_PILL_NUM);
            }

            else if(tmp_delta_heat < HEAT_CEASEFIRE_THRESHOLD)  // 热量过低, 拨盘电机停止旋转
            {
                shoot.target_value.trigger_omega = 0.0f;
            }

            shoot.trigger_motor.set_control_method(DJI_Motor_Control_Method_Omega);
            shoot.fric_r_motor.set_control_method(DJI_Motor_Control_Method_Omega);
            shoot.fric_l_motor.set_control_method(DJI_Motor_Control_Method_Omega);

            shoot.trigger_motor.set_target_omega(shoot.target_value.trigger_omega);
            shoot.fric_r_motor.set_target_omega(-FRICT_MOTOR_SPEED_SET);
            shoot.fric_l_motor.set_target_omega(FRICT_MOTOR_SPEED_SET);


            break;
        }

        default:
        {
            break;
        }
    }

        shoot.fric_r_slope.set_target(shoot.target_value.fric_r_omega);
        shoot.fric_r_slope.set_now_real(shoot.now_value.fric_r_omega);
        shoot.fric_r_slope.slope_calculate_callback();

        shoot.fric_l_slope.set_target(shoot.target_value.fric_l_omega);
        shoot.fric_l_slope.set_now_real(shoot.now_value.fric_l_omega);
        shoot.fric_l_slope.slope_calculate_callback();

    shoot.last_mode = shoot.now_mode;  // 更新上一帧发射模式

}

/**
 * @brief 发射机构电机控制计算
 */
void shoot_motor_calculate(void)
{

    if(shoot.jamming_fsm.get_now_status_serial() == Shoot_Fsm_Jamming_Status_Normal || shoot.jamming_fsm.get_now_status_serial() == Shoot_Fsm_Jamming_Status_Suspect)
    {
        // 发射机构控制决策(控制目标设定)
        shoot_control_decision();
    }

    // 摩擦轮电机控制计算
    shoot.fric_r_motor.internal_calculate_callback();
    shoot.fric_l_motor.internal_calculate_callback();

    // 拨盘电机控制计算
    shoot.trigger_motor.internal_calculate_callback(); 

    bsp_can_1_to_4_transmit(&hfdcan2, 0x200, can2_0x200_tx_data, Can_transmit_frequency_1000Hz);
    // bsp_can_1_to_4_transmit(&hfdcan3, 0x200, can3_0x200_tx_data, Can_transmit_frequency_1000Hz);
}

/**
 * @brief 更新发射机构用于决策的源数据
 * @param void
 * @return void
 */
void shoot_robot_decision_source_update(void)
{
    shoot.source->fric_r_omega = shoot.now_value.fric_r_omega;
    // 热量检测使用自适应滤波后的扭矩, 抑制噪声尖峰误判, 真实发射事件仍快速跟踪
    shoot.source->fric_r_torque = shoot.filter_value.fric_r_torque;
    shoot.source->shoot_now_mode = (uint8_t)shoot.now_mode;
}

void shoot_motor_parameter_test(void)
{
    shoot_update_measure();

    shoot.target_value.trigger_angle = wave_generator_calc(&trigger_wave);
    shoot.trigger_motor.set_target_angle(shoot.target_value.trigger_angle);
    shoot.trigger_motor.internal_calculate_callback();
    bsp_can_1_to_4_transmit(&hfdcan3, 0x200, can3_0x200_tx_data, Can_transmit_frequency_1000Hz);
}

/**
 * @brief shoot任务
 * @param argument
 */
extern "C" void shoot_task(void *argument)
{
    // 初始化发射机构
    shoot_init();

    shoot.jamming_fsm.init();  // 卡弹状态机初始化

    // 延时1秒，等待电机初始化完成
    osDelay(1000);  
    
    for(;;)
    {
        // 更新发射机构控制模式
        shoot_update_mode();

        // 更新发射机构测量值
        shoot_update_measure();

        // 发射机构电机控制计算
        shoot_motor_calculate();

        // 更新发射机构用于决策的源数据
        shoot_robot_decision_source_update();
        // shoot_motor_parameter_test();
        osDelay(1);
    }
}

/**
 * @brief 卡弹状态机计算回调函数
 */
void shoot_jamming_fsm_class::fsm_calculate_callback()
{
    status[now_status_serial].count_time++;

    switch (now_status_serial)
    {
        case Shoot_Fsm_Jamming_Status_Normal:   // 正常模式
        {
            if (fabsf(shoot.filter_value.trigger_torque) > Driver_Torque_Threshold)
            {
                // 正常模式检测到大扭矩-->卡弹嫌疑状态
                set_status(Shoot_Fsm_Jamming_Status_Suspect);
            }
            break;
        }

        case Shoot_Fsm_Jamming_Status_Suspect:   // 卡弹嫌疑状态
        {
            if (status[now_status_serial].count_time > Jamming_Suspect_Time_Threshold)
            {
                // 长时间大扭矩-->卡弹确认状态
                set_status(Shoot_Fsm_Jamming_Status_Confirm);
            }

            else if (fabsf(shoot.filter_value.trigger_torque) <= Driver_Torque_Threshold)
            {
                // 扭矩恢复正常-->正常状态
                set_status(Shoot_Fsm_Jamming_Status_Normal);
            }
            break;
        }

        case Shoot_Fsm_Jamming_Status_Confirm:   // 卡弹确认状态
        {
            shoot.trigger_motor.set_control_method(DJI_Motor_Control_Method_Angle);
            shoot.trigger_motor.set_target_angle(shoot.trigger_motor.get_now_angle() - Driver_Back_Angle);  // 拨盘电机回退角度
            shoot.target_value.trigger_angle = shoot.trigger_motor.get_now_angle() - Driver_Back_Angle;
            // 切换电机控制模式和控制角度-->卡弹处理状态
            set_status(Shoot_Fsm_Jamming_Status_Process);
            break;
        }

        case Shoot_Fsm_Jamming_Status_Process:   // 卡弹处理状态
        {
            if (status[now_status_serial].count_time > Jamming_Solving_Time_Threshold)
            {
                // 卡弹处理时间结束-->正常状态
                set_status(Shoot_Fsm_Jamming_Status_Normal);
            }
            break;
        }

    }
}

