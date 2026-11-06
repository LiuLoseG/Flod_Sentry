#include "gimbal.h"
#include "dm_motor.h"
#include "fdcan.h"

#include "basic_math.h"
#include "freertos.h"
#include "cmsis_os2.h"

#include "bsp_dwt.h"

// 统一CAN分发层
#include "can_distribution.h"

gimbal_smc_t gimbal_smc;

static void gimbal_yaw_smc_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal_smc.motor_yaw.can_rx_callback();
}

/**
 * @brief 初始化云台控制器
 */
void gimbal_smc_init()
{
    gimbal_smc.crsf = &crsf;

    gimbal_smc.motor_yaw.init(&hfdcan1, 0x11, 0x01, Motor_DM_Control_Method_Normal_MIT, 3.1415926f, 35.0f, 10.0f);

    gimbal_smc.smc_yaw.Init();
    gimbal_smc.smc_yaw.SetParam(0.0004f, 75.0f, 65.0f, 0.03f, 1.0f, 0.3f, EXPONENT, 0.001f);

    can_distribution_register(0x01, 0x11, gimbal_yaw_smc_motor_rx_client);

    gimbal_smc.state = Gimbal_Disable;
}

/**
 * @brief 设置云台控制器模式
 * @param mode 模式选择
 */
void gimbal_smc_mode(void)
{
    if (gimbal_smc.crsf->get_left_toggle() == CRSF_SWITCH_MID)
    {   
        gimbal_smc.state = Gimbal_Enabale;
    }

    else
    {
        gimbal_smc.state = Gimbal_Disable;
    }
}

/**
 * @brief 云台控制器更新函数
 */
void gimbal_smc_update(void)
{
    gimbal_smc.now_yaw.angle = gimbal_smc.motor_yaw.get_total_angle();
    gimbal_smc.now_yaw.speed = gimbal_smc.motor_yaw.get_now_omega();
}

/**
 * @brief 云台控制器设置目标函数
 */
void gimbal_smc_set_target(void)
{
    if(gimbal_smc.state == Gimbal_Disable)
    {
        gimbal_smc.tgt_yaw.angle = gimbal_smc.now_yaw.angle;
        gimbal_smc.tgt_yaw.speed = 0.0f;
    }

}

/**
 * @brief 滑模控制器计算函数
 */
void gimbal_smc_control(void)
{
    if(gimbal_smc.state == Gimbal_Disable)
    {
        gimbal_smc.motor_yaw.set_control_torque(0.0f);
    }

    else
    {
        gimbal_smc.smc_yaw.ErrorUpdate(gimbal_smc.tgt_yaw.angle, gimbal_smc.now_yaw.angle, gimbal_smc.now_yaw.speed);
        gimbal_smc.smc_yaw.SmcCalculate();
        gimbal_smc.motor_yaw.set_control_torque(basic_math_constrain(gimbal_smc.smc_yaw.Out(), -0.3f, 0.3f));
    }

    // else
    // {
    //     gimbal_smc.motor_yaw.set_control_torque(basic_math_constrain(0.03f, -0.3f, 0.3f));
    // }

    gimbal_smc.motor_yaw.calculate_callback();
}

extern "C" void gimbal_task(void *argument)
{
    gimbal_smc_init();
    osDelay(1000);

    for(;;)
    {
        
        gimbal_smc_mode();
        gimbal_smc_update();
        gimbal_smc_set_target();
        gimbal_smc_control();

        osDelay(1);
    }
}

