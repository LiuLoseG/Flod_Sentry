#include "steering_wheel.h"
#include "cmsis_os2.h"
#include "freertos.h"

steering_wheel_t steer_wheel;

// 初始化
void steer_wheel_init(void)
{
    // 初始化舵轮电机对象
    steer_wheel.dm_steer_motor.init(&hfdcan2, 0x11, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 3.141593f, 45.0f, 10.0f, 10.0f);
    steer_wheel.dm_drive_motor.init(&hfdcan2, 0x12, 0x02, Motor_DM_Control_Method_NORMAL_OMEGA, 12.5f, 45.0f, 10.0f, 10.0f);

    // 初始化舵轮PID控制器
    steer_wheel.steer_angle_pid.init(5.4f, 0.0f, 0.032f, 3.5f, 1.5f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.2f, PID_D_First_ENABLE);
    // 舵轮速度PID: Kp=0.01, Kd=0.0003 (阻尼项抵消EMA相位滞后), D-first模式抑制微分噪声
    // 调参策略: 先调Kp至微振, 再加Kd至振荡消失, 最后微调Kp恢复响应速度
    // steer_wheel.steer_speed_pid.init(0.032f, 0.012f, 0.0f, 0.0f, 2.0f, 10.0f);
    steer_wheel.steer_speed_pid.init(0.051f, 0.75f, 0.0f, 1.2f, 1.5f, 10.0f, 0.001f, 0.035f, 0.0f, 0.0f, 2.0f, PID_D_First_ENABLE);
    // steer_wheel.steer_speed_pid.init(0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 10.0f);
    // steer_wheel.drive_pid.init(0.0f, 0.0f, 0.0f, 0.0f, 2.0f, 10.0f);

    // 初始化轮电机状态结构体
    for (int i = 0; i < 4; i++)
    {
        steer_wheel.ref_status.steer_angle[i] = 0.0f;
        steer_wheel.ref_status.wheel_speed[i] = 0.0f;
    }

    steer_wheel.crsf = &crsf; // 将遥控器对象指针赋值

    for (int i = 0; i < 3; i++)
    {
        steer_wheel.slope_speed[i].init(0.04f, 0.04f); // 初始化速度斜坡函数
    }

    // for (int i = 0; i < 4; i++)
    // {
    //     steer_wheel.slope_angle[i].init(0.008f, 0.008f); // 初始化角度斜坡函数
    // }

}

void steer_motor_enable(void)
{
    steer_wheel.dm_steer_motor.can_send_enter();
    osDelay(10); // 延时等待舵轮电机使能完成
    steer_wheel.dm_drive_motor.can_send_enter();

    // steer_wheel.dm_steer_motor.set_control_omega(3.0f); // 设置舵轮电机的目标角速度为3rad/s
}

void steer_wheel_update_mode(void)
{
    switch (steer_wheel.crsf->get_right_toggle())
    {
        case CRSF_SWITCH_DOWN:
            steer_wheel.steer_mode = Steer_Lock_Mode; // 舵轮锁定模式
            break;
        case CRSF_SWITCH_MID:
            steer_wheel.steer_mode = Steer_Manual_Mode; // 舵轮手动模式
            break;

        default:
            break;
    }
}

// 更新舵轮测量值
void steer_wheel_update_measure(void)
{
    float raw_omega = steer_wheel.dm_steer_motor.get_now_omega(); // 只读一次, 避免重复读取数据不一致

    steer_wheel.act_status.wheel_speed[0] = raw_omega;
    // steer_wheel.act_status.steer_angle[0] = steer_wheel.dm_steer_motor.get_now_angle();
    steer_wheel.act_status.steer_angle[0] = steer_wheel.dm_steer_motor.get_now_angle(); // 获取舵轮电机的总角度值

    // 自适应低通滤波器: 死区+变α, 零振铃, 兼顾响应速度与平滑度
    // 死区阈值 0.05 rad/s — 略大于编码器单步量化噪声(~0.03), 消除静态抖动
    // 大偏差(>0.5 rad/s): α=0.30, τ≈3ms — 快速跟踪真实运动
    // 小偏差(≤0.5 rad/s): α=0.06, τ≈17ms — 深度压制量化噪声
    static float ema_state = 0.0f;
    float error = raw_omega - ema_state;

    // 死区: 小于阈值的偏差视为量化噪声, 强制归零
    if (fabsf(error) < 0.05f)
    {
        error = 0.0f;
    }

    float alpha = (fabsf(error) > 0.5f) ? 0.30f : 0.06f;
    ema_state += alpha * error;
    steer_wheel.act_steer_angular[0] = ema_state;
}

void steer_wheel_whole_remote_control(void)
{
    steer_wheel.map_vx = basic_math_dead_constrain((steer_wheel.crsf->get_right_x() * STEER_MAX_MOVE_SPEED), -STEER_MAX_MOVE_SPEED, STEER_MAX_MOVE_SPEED, 0.05f); // 将遥控器右摇杆Y轴值映射为底盘的x轴速度，并进行限幅和死区处理
    steer_wheel.map_vy = basic_math_dead_constrain((steer_wheel.crsf->get_right_y() * STEER_MAX_MOVE_SPEED), -STEER_MAX_MOVE_SPEED, STEER_MAX_MOVE_SPEED, 0.05f); // 将遥控器右摇杆X轴值映射为底盘的y轴速度，并进行限幅和死区处理
    steer_wheel.map_vz = basic_math_dead_constrain((steer_wheel.crsf->get_left_y() * STEER_MAX_ROTATE_SPEED), -STEER_MAX_ROTATE_SPEED, STEER_MAX_ROTATE_SPEED, 0.15f); // 将遥控器滚轮值映射为底盘的角速度，并进行限幅和死区处理
}

// 舵轮整体移动控制函数，根据舵轮工作模式，选择不同的控制方式
void steer_wheel_mobile_control(void)
{
    switch (steer_wheel.steer_mode)
    {
        case Steer_Lock_Mode:
            steer_wheel.ref_speed_robot.vx = 0.0f;
            steer_wheel.ref_speed_robot.vy = 0.0f;
            steer_wheel.ref_speed_robot.omega = 0.0f;
            break;

        case Steer_Manual_Mode:
            steer_wheel_whole_remote_control(); // 舵轮整体遥控器控制
            break;

        default:
            break;
    }

    steer_wheel.slope_speed[0].set_target(steer_wheel.map_vx); // 设置斜坡函数的目标值为舵轮的期望速度
    steer_wheel.slope_speed[0].set_now_real(steer_wheel.ref_speed_robot.vx); // 设置斜坡函数的当前真实值为舵轮的实际速度
    steer_wheel.slope_speed[0].slope_calculate_callback(); // 计算斜坡函数的输出值
    steer_wheel.ref_speed_robot.vx = steer_wheel.slope_speed[0].get_out(); // 获取斜坡函数的输出值作为舵轮的期望速度

    steer_wheel.slope_speed[1].set_target(steer_wheel.map_vy); // 设置斜坡函数的目标值为舵轮的期望速度
    steer_wheel.slope_speed[1].set_now_real(steer_wheel.ref_speed_robot.vy); // 设置斜坡函数的当前真实值为舵轮的实际速度
    steer_wheel.slope_speed[1].slope_calculate_callback(); // 计算斜坡函数的输出值
    steer_wheel.ref_speed_robot.vy = steer_wheel.slope_speed[1].get_out(); // 获取斜坡函数的输出值作为舵轮的期望速度

    steer_wheel.slope_speed[2].set_target(steer_wheel.map_vz); // 设置斜坡函数的目标值为舵轮的期望角速度
    steer_wheel.slope_speed[2].set_now_real(steer_wheel.ref_speed_robot.omega); // 设置斜坡函数的当前真实值为舵轮的实际角速度
    steer_wheel.slope_speed[2].slope_calculate_callback(); // 计算斜坡函数的输出值
    steer_wheel.ref_speed_robot.omega = steer_wheel.slope_speed[2].get_out(); // 获取斜坡函数的输出值作为舵轮的期望角速度
}

// 舵轮电机计算函数
void steer_wheel_motor_calculate(void)
{
    // 舵轮运动学解算函数
    steer_wheel_inverse_kinematics(&steer_wheel.ref_status, steer_wheel.ref_speed_robot.vx, steer_wheel.ref_speed_robot.vy, steer_wheel.ref_speed_robot.omega, steer_wheel.act_status.steer_angle);

    // 舵轮PID控制器计算
    // steer_wheel.slope_angle[0].set_target(steer_wheel.ref_status.steer_angle[0]); // 设置斜坡函数的目标值为舵轮的期望角速度
    // steer_wheel.slope_angle[0].set_now_real(steer_wheel.act_status.steer_angle[0]); // 设置斜坡函数的当前真实值为舵轮的实际角速度
    // steer_wheel.slope_angle[0].slope_calculate_callback(); // 计算斜坡函数的输出值
    // steer_wheel.motor_angle[0] = steer_wheel.slope_angle[0].get_out();

    steer_wheel.steer_angle_pid.set_target(steer_wheel.ref_status.steer_angle[0]);
    steer_wheel.steer_angle_pid.set_now(steer_wheel.act_status.steer_angle[0]);
    steer_wheel.steer_angle_pid.pid_calculate_callback();
    steer_wheel.steer_angular[0] = steer_wheel.steer_angle_pid.get_out();

    steer_wheel.steer_speed_pid.set_target(steer_wheel.steer_angular[0]);
    steer_wheel.steer_speed_pid.set_now(steer_wheel.act_steer_angular[0]);
    steer_wheel.steer_speed_pid.pid_calculate_callback();
    steer_wheel.steer_torque[0] = steer_wheel.steer_speed_pid.get_out();
    // steer_wheel.steer_torque[0] = steer_wheel.ref_status.steer_angle[0]; // 直接将舵轮目标角度作为舵轮电机的控制扭矩值

    // steer_wheel.drive_pid.set_target(steer_wheel.ref_status.wheel_speed[0]);
    // steer_wheel.drive_pid.set_now(steer_wheel.act_status.wheel_speed[0]);
    // steer_wheel.drive_pid.pid_calculate_callback();
    // steer_wheel.dm_drive_motor.set_target_omega(steer_wheel.drive_pid.get_out());
    // steer_wheel.motor_control_wheel[0] = steer_wheel.drive_pid.get_out();
    steer_wheel.wheel_angular[0] = steer_wheel.ref_status.wheel_speed[0];
}

void steer_motor_control(void)
{

    steer_wheel.dm_steer_motor.set_control_torque(steer_wheel.steer_torque[0]);
    steer_wheel.dm_drive_motor.set_control_omega(steer_wheel.wheel_angular[0]);

    steer_wheel.dm_steer_motor.calculate_callback();
    steer_wheel.dm_drive_motor.calculate_callback();
}

void steer_wheel_test(void)
{
    steer_wheel_update_mode(); // 更新舵轮工作模式
    steer_wheel_update_measure(); // 更新舵轮测量值
    steer_wheel_motor_calculate(); // 舵轮电机计算函数
    // steer_wheel.dm_steer_motor.set_control_torque(0.1f);

    // steer_wheel.dm_steer_motor.calculate_callback();
}

extern "C" void steering_wheel_task(void *argument)
{

    steer_wheel_init(); // 初始化舵轮相关参数和电机
    steer_motor_enable(); // 使能舵轮电机
    osDelay(500); // 延时等待舵轮初始化完成

    for(;;)
    {
        steer_wheel_update_mode(); // 更新舵轮工作模式
        steer_wheel_update_measure(); // 更新舵轮测量值
        steer_wheel_mobile_control(); // 舵轮整体移动控制函数
        steer_wheel_motor_calculate(); // 舵轮电机计算函数
        // steer_wheel_test(); // 舵轮测试函数
        steer_motor_control(); // 舵轮电机控制函数
        osDelay(1);
    }
}

void fdcan2_rx_callback(const volatile FDCAN_RxFrame_TypeDef *rx_frame)
{
    if (rx_frame == NULL) {
        return;
    }

    switch (rx_frame->header.Identifier)
    {

        case 0x11: // 根据实际ID进行判断
        {
            // 处理接收到的CAN消息，参数根据需要调整
            steer_wheel.dm_steer_motor.can_rx_callback();
            break;
        }
        case 0x12: // 根据实际ID进行判断
        {
            // 处理接收到的CAN消息，参数根据需要调整
            steer_wheel.dm_drive_motor.can_rx_callback();
            break;
        }

        default:
            break;
    }
}