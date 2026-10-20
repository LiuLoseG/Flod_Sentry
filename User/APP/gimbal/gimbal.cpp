#include "gimbal.h"
#include "arm_math.h"
#include "basic_math.h"
#include "freertos.h"
#include "cmsis_os2.h"

// PID波形调节器
#include "pid.h"

// 统一CAN分发层
#include "can_distribution.h"

// 安全守护进程(电机离线检测)
#include "safe_daemon_process.h"
#include "wave_generator.h"
#include "bsp_dwt.h"

// 定义全局云台对象
gimbal_t gimbal;

// 波形发生器
wave_generator_t yaw_wave;
wave_generator_t big_yaw_wave;

float pitch_c_angle = 0.00065f;
float yaw_c_angle = 0.00112f;

static float pitch_ff_coeff[3] = {
    -0.298674f,  // c[0] = K (弹簧/重力刚度)  Nm/rad
    -0.318108f,  // c[1] = B (阻尼系数)      Nm/(rad/s)
    -0.000000f   // c[2] = J (等效惯量)      Nm/(rad/s²)
};

/** @brief 前馈常数偏置 (v7, 因果滤波匹配MCU辨识) */
#define PITCH_FF_BIAS_C      0.022863f

#define PITCH_FF_COULOMB     0.0f

#define PITCH_FF_VELOCITY_GAIN  0.85f

#define PITCH_FF_REF_LPF_ALPHA  0.05f

#define PITCH_FF_REF_DOT_OLS_ORDER  5

#define PITCH_FF_OMEGA_MAX  3.5f

#define PITCH_FF_ACC_LIMIT  18.0f

/** @brief Pitch非线性静态补偿点数 */
#define PITCH_FF_STATIC_NUM  5

// 静态补偿表 (角度→扭矩, 由稳态数据辨识)
// v6.1线性模型K=-0.395已足够强, 静态非线性残差极小, 全部设为0
static const float pitch_ff_static_angle[PITCH_FF_STATIC_NUM] = {
    -0.45f, -0.28f, 0.0f, 0.28f, 0.45f
};
static const float pitch_ff_static_torque[PITCH_FF_STATIC_NUM] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

static float yaw_small_ff_coeff[3] = {
    +0.098415f,  // c[0] = K (刚度)      Nm/rad
    +0.170000f,  // c[1] = B (阻尼)      Nm/(rad/s)  [v3: 0.11→0.17, 实测辨识 0.171±0.009]
    +0.000000f   // c[2] = J (等效惯量)  Nm/(rad/s²)
};

#define YAW_SMALL_FF_BIAS_C  0.0f

/** @brief 小yaw库仑摩擦补偿 (sign(ω), 补偿微小cogging力矩) */
#define YAW_SMALL_FF_COULOMB  0.005f

#define YAW_SMALL_FF_VELOCITY_GAIN  0.55f

#define YAW_SMALL_FF_OMEGA_MAX  0.75f

#define YAW_SMALL_FF_RATE_LIMIT  10.0f

#define YAW_SMALL_FF_TORQUE_RATE_LIMIT  0.5f

/** @brief 大yaw角速度指令速率限制(rad/s²): 平滑Follow↔Scan切换引起的目标角速度阶跃, 抑制速度环力矩尖峰 */
#define BIG_YAW_OMEGA_SLEW  32.0f

#define YAW_SMALL_FF_ENC_LPF_ALPHA  0.111f

#define YAW_SMALL_FF_DELTA_LPF_ALPHA  0.05f

#define YAW_SMALL_FF_REF_DOT_OLS_ORDER  5

/** @brief 大yaw回中参数 */
#define BIG_YAW_RECENTER_FULL      0.52f   // 原 0.35: 满回中幅值提高, 让工作区间保持低回中
#define BIG_YAW_RECENTER_MIN_FRAC  0.25f   // 原 0.32: 小角度回中基础比例降低, 释放小yaw高响应

// 轨迹规划器参数配置
reference_config_t yaw_small_track_plan_config = {
    .max_rate_rad_s = 1.2f,           // 最大角速度，单位：rad/s
    .max_accel_rad_s2 = 1.8f,        // 最大角加速度，单位：rad/s^2
    .position_gain_per_s = 1.0f,      // 位置比例增益，单位：1/秒
    .dt_min_s = 0.0009f,               // 最小采样时间，单位：秒
    .dt_max_s = 0.005f,                // 最大采样时间，单位：秒
    .wrap_angle = true                // 是否进行角度环绕处理(最短角规划)
};

/*------------------------------------云台电机can接收对象(注册到统一分发层）-------------------------------------------*/
static void gimbal_pitch_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.pitch_motor.can_rx_callback();     // CAN1 0x11
}

static void gimbal_yaw_small_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.yaw_small_motor.can_rx_callback(); // CAN1 0x205
}

static void gimbal_yaw_big_motor_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    gimbal.yaw_big_motor.can_rx_callback();   // CAN3 0x141
}

/**
 * @brief 云台初始化
 */
void gimbal_init(void)
{
    // 1.绑定陀螺仪对象和遥控器对象
    gimbal.hi12_imu = &hi12_data;
    gimbal.crsf = &crsf;
    gimbal.source = &gimbal_decision_source;  // 绑定云台控制用于决策的源数据指针
    gimbal.decision = &robot_decision;  // 绑定机器人决策对象指针

    // 2.初始化电机对象
    gimbal.yaw_big_motor.init(&hfdcan3, 1, Motor_LK_Control_Method_Normal_Torque);  // LK电机，外部PID闭环，扭矩模式
    gimbal.yaw_small_motor.init(&hfdcan1, Motor_DJI_ID_0x205, DJI_Motor_Control_Method_Torque); // DJI电机，内部PID闭环，角度模式
    gimbal.pitch_motor.init(&hfdcan1, 0x11, 0x01, Motor_DM_Control_Method_Normal_MIT, 3.14159f, 30.0f, 10.0f);  // DM电机，外部PID闭环，MIT模式

    // 3.初始化小yaw电机的角度环和角速度环的PID参数
    gimbal.rc_yaw_small_pid.pid_angle.init(11.5f, 0.45f, 0.38f, 2.5f, 1.0f, 3.0f, 0.001f, 0.0f, 0.045f, 0.1f, 0.15f, PID_D_First_DISABLE);
    gimbal.rc_yaw_small_pid.pid_omega.init(0.3f, 0.0f, 0.0f, 0.0f, 0.5f, 1.2f);

    gimbal.pc_yaw_small_pid.pid_angle.init(13.8f, 0.0f, 0.98f, 0.0f, 1.0f, 3.0f, 0.001f, 0.0f, 0.10f, 0.24f, 0.32f, PID_D_First_ENABLE);
    gimbal.pc_yaw_small_pid.pid_omega.init(0.7f, 0.0f, 0.023f, 0.0f, 0.5f, 1.2f, 0.001f, 0.0f, 0.2f, 0.65f, 1.0f, PID_D_First_ENABLE);

    // 4.初始化大yaw电机的角度环和角速度环的PID参数
    gimbal.rc_yaw_big_angle_pid.init(0.62f, 0.0f, 0.242f, 0.0f, 1.0f, 3.0f, 0.001f, 0.0f, 0.105f, 0.2f, 0.25f, PID_D_First_ENABLE);

    gimbal.pc_yaw_big_pid.pid_angle.init(4.20f, 0.0f, 0.15f, 0.0f, 1.0f, 9.2f, 0.001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);
    gimbal.pc_yaw_big_pid.pid_omega.init(0.385f, 0.195f, 0.0f, 0.0f, 0.5f, 3.2f, 0.001f, 0.0f, 0.56f, 0.8f, 1.0f, PID_D_First_ENABLE);
    // 5.初始化pitch电机的角度环和角速度环的PID参数
    // 角度环Ki=0 (无积分), 前馈通过Feedforward_Calculate独立提供
    // 注意: pid_class的k_f是简单差分前馈, 由Feedforward_Calculate的OLS前馈替代, 设为0避免重复
    gimbal.rc_pitch_pid.pid_angle.init(11.68f, 0.0f, 0.25f, 0.0f, 2.0f, 10.0f, 0.001f, 0.0f, 0.0f, 0.0f, 0.1f, PID_D_First_ENABLE);
    gimbal.rc_pitch_pid.pid_omega.init(0.185f, 0.0f, 0.003f, 0.0f, 1.0f, 3.2f);

    gimbal.pc_pitch_pid.pid_angle.init(14.8f, 0.1f, 0.03f, 0.0f, 1.0f, 3.5f, 0.001f, 0.0f, 0.052f, 0.12f, 0.125f, PID_D_First_ENABLE);
    gimbal.pc_pitch_pid.pid_omega.init(0.75f, 0.1f, 0.0021f, 0.0f, 0.4f, 3.2f, 0.001f, 0.0f, 0.32f, 0.65f, 0.8f, PID_D_First_ENABLE);

    // 6.初始化上一帧目标角度
    gimbal.prev_target_small_yaw_angle = gimbal.target_value.small_yaw_angle;

    // 7.初始化云台前馈量
    Feedforward_Init(&gimbal.pc_pitch_ffc, 0.62f, pitch_ff_coeff, 0.008f, PITCH_FF_REF_DOT_OLS_ORDER, 2);
    Feedforward_Init(&gimbal.pc_yaw_small_ffc, 0.50f, yaw_small_ff_coeff, 0.003f, YAW_SMALL_FF_REF_DOT_OLS_ORDER, 2);

    // 8.滤波器初始化
    gimbal.chassis_angular_lpf.init(5.0f, 0.001f);  // 底盘角速度低通滤波器
    gimbal.filter_yaw.init(12.0f, 0.001f);
    gimbal.filter_pitch.init(6.0f, 0.001f);
    // gimbal.filter_imu_pitch_angle.init(8.0f, 0.001f);
    // gimbal.filter_pitch_pitch_omega.init(8.0f, 0.001f);

    gimbal.pitch_gravity_comp_filter.init(5.0f, 0.001f);

    gimbal.feedforward_value.big_yaw_omega = 0.0f;
    gimbal.feedforward_value.big_yaw_torque = 0.0f;

    gimbal.feedforward_value.small_yaw_omega = 0.0f;
    gimbal.feedforward_value.small_yaw_torque = 0.0f;

    gimbal.prev_small_yaw_ff_omega = 0.0f;  // 速率限制器的初始状态
    gimbal.prev_big_yaw_omega = 0.0f;   // 大yaw角速度指令速率限制器初始状态
    gimbal.big_yaw_omega_rl_cnt = 0;
    gimbal.prev_pitch_ff_torque = 0.0f;     // pitch力矩前馈速率限制器初始状态
    gimbal.yaw_small_ff_dc_mean = 0.0f;     // DC blocker 初始状态
    gimbal.yaw_small_ff_enc_lpf = 0.0f;     // 前馈参考用编码器低通初始状态
    gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 前馈参考用视觉误差低通初始状态
    gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 直接播种到当前目标, 避免上电爬升
    gimbal.prev_pitch_ff_omega = 0.0f;      // [拐点缓冲] 加速度限制器初始状态

    gimbal.feedforward_value.pitch_omega = 0.0f;
    gimbal.feedforward_value.pitch_torque = 0.0f;

    // 初始化云台重力/摩擦力补偿量
    gimbal.pitch_gravity_comp.amount = 0.0f;
    gimbal.pitch_gravity_comp.factor[0] = -1.015506f;
    gimbal.pitch_gravity_comp.factor[1] = -0.05028f;
    gimbal.pitch_gravity_comp.factor[2] = 0.081351f;
    gimbal.pitch_gravity_comp.factor[3] = -0.0417713f;

    // gimbal.rls.setParamVector(Matrixf<4, 1>(gimbal.pitch_gravity_comp.factor));

    // 初始化IMU稳定状态
    gimbal.imu_frame_cnt = 0;
    gimbal.imu_attitude_ready = 0;

    gimbal.pit_towards_flag = 0;
    gimbal.yaw_towards_flag = 0;

    // 8.初始化云台模式
    gimbal.now_method = Gimbal_Method_Zero_Force;
    gimbal.last_method = Gimbal_Method_Zero_Force;
    gimbal.now_mode = Gimbal_Auto_Follow;
    gimbal.last_mode = Gimbal_Auto_Follow;

    // 9. 电机波形跟踪调试
    wave_generator_init(&yaw_wave, WAVE_SINE, 0.1f, 0.5f, 0.0f);
    wave_generator_enable(&yaw_wave);

    wave_generator_init(&big_yaw_wave, WAVE_SINE, 0.1f, 1.0f, 0.0f);
    wave_generator_enable(&big_yaw_wave);

    // 10. 注册 CAN 接收分发规则（统一分发层，各总线仅一个强定义回调）
    can_distribution_register(1, 0x11,  gimbal_pitch_motor_rx_client);
    can_distribution_register(1, 0x205, gimbal_yaw_small_motor_rx_client);
    can_distribution_register(3, 0x141, gimbal_yaw_big_motor_rx_client);

    // 11. 注册电机离线检测（安全守护进程统一调用 alive_check, 只检测状态不做安全动作）
    //     检测周期由 SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS 决定, 即掉线判定时延(单位 ms)
    SAFE_DAEMON_PROCESS_REGISTER("gimbal_pitch_motor", gimbal.pitch_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("gimbal_yaw_small_motor", gimbal.yaw_small_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    SAFE_DAEMON_PROCESS_REGISTER("gimbal_yaw_big_motor", gimbal.yaw_big_motor, SAFE_DAEMON_PROCESS_PERIOD_MOTOR_MS);
    // 12. 初始化云台参考轨迹规划器
    reference_track_plan_init(&gimbal.yaw_small_track_plan, &yaw_small_track_plan_config, 0.0f);
}

/**
 * @brief 更新云台控制模式
 */
void gimbal_update_mode(void)
{
    switch (gimbal.crsf->get_left_toggle())
    {
        case CRSF_SWITCH_TRIG_MID_UP:               // 中拨→上拨过渡
        case CRSF_SWITCH_UP:                        // 左拨动开关上拨
        {
            gimbal.now_method = Gimbal_Method_Auto;
            break;
        }

        case CRSF_SWITCH_TRIG_UP_MID:               // 上拨→中拨过渡
        case CRSF_SWITCH_TRIG_DOWN_MID:             // 下拨→中拨过渡
        case CRSF_SWITCH_MID:                       // 左拨动开关中拨
        {
            gimbal.now_method = Gimbal_Method_Manual;
            break;
        }

        case CRSF_SWITCH_TRIG_MID_DOWN:             // 中拨→下拨过渡
        case CRSF_SWITCH_DOWN:                      // 左拨动开关下拨
        {
            gimbal.now_method = Gimbal_Method_Zero_Force;
            break;
        }

        default:
        {
            gimbal.now_method = Gimbal_Method_Zero_Force;
            break;
        }
    }
}

/**
 * @brief 更新云台测量值
 * @note 使用底盘系作为云台的参考系，更新云台的角度和角速度测量值
 */
void gimbal_update_measure(void)
{
    // 更新云台角速度 (来自于 IMU 陀螺仪)
    // gimbal.now_value.small_yaw_omega   = gimbal.hi12_imu->gyr_z() * BASIC_MATH_DEG_TO_RAD;  //  小yaw电机角速度来自于IMU陀螺仪

    //旋转矩阵消除z轴偏置导致的角速度分量偏转
    gimbal.now_value.small_yaw_omega = (gimbal.hi12_imu->gyr_z() * BASIC_MATH_DEG_TO_RAD) * arm_cos_f32(gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD) - (gimbal.hi12_imu->gyr_x() *  BASIC_MATH_DEG_TO_RAD) * arm_sin_f32(gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD);

    gimbal.now_value.pitch_omega = gimbal.hi12_imu->gyr_y() * BASIC_MATH_DEG_TO_RAD;        //  pitch电机角速度来自于IMU陀螺仪

    // 更新大yaw电机的反馈角度，根据小yaw电机编码器的角度来计算大小yaw的相对角度差，用于大yaw电机跟随小yaw电机。
    gimbal.now_value.big_yaw_angle = -(gimbal.yaw_small_motor.get_single_angle() - YAW_SMALL_MOTOR_MID_ANGLE);
    // gimbal.now_value.big_yaw_angle = gimbal.yaw_big_motor.get_single_angle();
    gimbal.now_value.big_yaw_omega = gimbal.yaw_big_motor.get_now_omega();  //  大yaw电机角速度来自于大yaw电机编码器     

    // 小yaw电机编码器角度反馈值
    gimbal.now_s_yaw_angle = gimbal.yaw_small_motor.get_single_angle() - YAW_SMALL_MOTOR_MID_ANGLE;

    // 更新小yaw电机的反馈角度，根据小yaw电机编码器的角度来计算小yaw电机的实际角度。
    if(gimbal.yaw_small_motor.get_single_angle() == 0.0f || gimbal.yaw_small_motor.get_single_angle() >= 2*PI)
    {
        gimbal.now_value.small_yaw_angle = 0.0f;
    }
    else
    {
        gimbal.now_value.small_yaw_angle = gimbal.yaw_small_motor.get_single_angle() - YAW_SMALL_MOTOR_MID_ANGLE;     
    }

    /* ---- 姿态角需要等待 IMU 滤波器收敛 ---- */
    if (gimbal.hi12_imu->tag() == HIPNUC_ID_HI91)
    {
        if (gimbal.imu_frame_cnt < 2000)
        {
            gimbal.imu_frame_cnt++;
        }

        if (gimbal.imu_frame_cnt >= 2000 && !gimbal.imu_attitude_ready)
        {
            // IMU 滤波器已收敛, 解锁姿态角更新 
            gimbal.imu_attitude_ready = 1;

            // 首次使用时, 将当前 IMU 姿态一次性设置到 now_value, 避免后续突然跳变
            gimbal.now_value.gimbal_yaw_angle = gimbal.hi12_imu->yaw() * BASIC_MATH_DEG_TO_RAD;
            gimbal.now_value.pitch_angle = gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD;

            // 同步目标值, 防止 IMU 收敛前 target 已积累偏移, 收敛后 now 跳变导致两者产生巨大误差
            gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;
        }
    }

    if (gimbal.imu_attitude_ready)
    {
        // IMU 已稳定, 正常更新姿态角
        gimbal.now_value.gimbal_yaw_angle = gimbal.hi12_imu->yaw() * BASIC_MATH_DEG_TO_RAD;
        gimbal.now_value.pitch_angle = gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD;

        gimbal.pitch_limit_angle = (gimbal.hi12_imu->roll() * BASIC_MATH_DEG_TO_RAD
                                + (gimbal.pitch_motor.get_now_angle() - PITCH_MOTOR_MID_ANGLE));

    }

    // 更新滤波值 (用于观测, 不参与控制)
    gimbal.filter_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;
    gimbal.filter_value.small_yaw_angle   = gimbal.now_value.small_yaw_angle;
    gimbal.filter_value.small_yaw_omega   = gimbal.now_value.small_yaw_omega;

    gimbal.filter_value.pitch_angle = gimbal.now_value.pitch_angle;
    gimbal.filter_value.pitch_omega = gimbal.now_value.pitch_omega;

    gimbal.filter_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;

    // 获取当前真实值
    gimbal.filter_yaw.set_now_input(gimbal.decision->pc_to_mcu_aim.get_yaw_angle());
    gimbal.filter_pitch.set_now_input(gimbal.decision->pc_to_mcu_aim.get_pitch_angle());
    gimbal.chassis_angular_lpf.set_now_input(gimbal.decision->can_board_communication.get_chassis_angular());

    // gimbal.filter_imu_pitch_angle.set_now_input(gimbal.now_value.pitch_angle);
    // gimbal.filter_pitch_pitch_omega.set_now_input(gimbal.now_value.pitch_omega);

    // 对当前数据进行滤波处理
    gimbal.filter_yaw.low_calculate_callback();
    gimbal.filter_pitch.low_calculate_callback();
    gimbal.chassis_angular_lpf.low_calculate_callback();

    if(fabs(gimbal.chassis_angular_lpf.get_out()) < 0.1f)
    {
        gimbal.feedforward_value.chassis_angular = 0.0f;
    }

    else
    {
        gimbal.feedforward_value.chassis_angular = basic_math_constrain(gimbal.chassis_angular_lpf.get_out(), -PI, PI);
    }

    // gimbal.filter_imu_pitch_angle.low_calculate_callback();
    // gimbal.filter_pitch_pitch_omega.low_calculate_callback();
}

/**
 * @brief 更新云台决策源数据
 */
void gimbal_robot_decision_source_update(void)
{
    gimbal.source->big_yaw_angle = basic_math_loop_constrain(gimbal.yaw_big_motor.get_single_angle(), -PI, PI);
    gimbal.source->small_yaw_angle = gimbal.yaw_small_motor.get_single_angle();
    gimbal.source->vision_gyr_z = gimbal.now_value.small_yaw_omega;
}

/**
 * @brief 更新云台控制决策(控制目标设定)
 */
void gimbal_control_decision(void)
{
    if (gimbal.now_method == Gimbal_Method_Zero_Force)  // 零力矩模式(电机失能)
    {
        gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
        gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
        gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
        gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
        gimbal.pc_yaw_big_pid.pid_angle.set_integral_error(0.0f);
        gimbal.pc_yaw_big_pid.pid_omega.set_integral_error(0.0f);

        gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
        gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
        gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
        gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
        gimbal.rc_yaw_big_angle_pid.set_integral_error(0.0f);

        gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;
        gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
        gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
        gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;

        gimbal.target_value.small_yaw_omega = gimbal.now_value.small_yaw_omega;
        gimbal.target_value.pitch_omega = gimbal.now_value.pitch_omega;
    }

    else if (gimbal.now_method == Gimbal_Method_Manual) // 遥控器控制模式(操作手接管)
    {
        if (gimbal.last_method != Gimbal_Method_Manual)
        {
            gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
            gimbal.pc_yaw_big_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_yaw_big_pid.pid_omega.set_integral_error(0.0f);

            gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
            gimbal.rc_yaw_big_angle_pid.set_integral_error(0.0f);

            gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;
            gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
            gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
            gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;

            // 同步前馈状态, 防止模式切换后前馈微分项产生瞬态跳变
            gimbal.prev_target_small_yaw_angle = gimbal.target_value.small_yaw_angle;
            gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 delta=0, 状态同步为0
            gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种到当前目标, 防切换瞬态
            gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
        }

        else
        {
            float yaw_small_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_y(), -1.0f, 1.0f, 0.065f) * YAW_SMALL_ANGLE_INCREMENT_RATION;
            float pitch_angle_increment = basic_math_dead_constrain(gimbal.crsf->get_left_x(), -1.0f, 1.0f, 0.065f) * PITCH_ANGLE_INCREMENT_RATION;

            gimbal.target_value.gimbal_yaw_angle = basic_math_loop_constrain((gimbal.target_value.gimbal_yaw_angle + yaw_small_angle_increment), -PI, PI);

            {
                float delta = basic_math_loop_constrain(gimbal.target_value.gimbal_yaw_angle - gimbal.now_value.gimbal_yaw_angle, -PI, PI);
                gimbal.target_value.small_yaw_angle = basic_math_constrain(gimbal.now_value.small_yaw_angle + delta, YAW_SMALL_MIN_OFFSET, YAW_SMALL_MAX_OFFSET);
            }
            
            gimbal.target_value.pitch_angle = basic_math_constrain(gimbal.target_value.pitch_angle + pitch_angle_increment, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET);

            {
                // 检测小yaw是否超出正常工作范围 (偏移域 ±偏移量*1.2倍裕量)
                // 单圈角度 → 偏移角度 = single_angle - MID
                float small_yaw_offset = gimbal.yaw_small_motor.get_single_angle() - YAW_SMALL_MOTOR_MID_ANGLE;
                if (small_yaw_offset <= YAW_SMALL_MIN_OFFSET * 1.2f ||
                    small_yaw_offset >= YAW_SMALL_MAX_OFFSET * 1.2f)
                {
                    // 小yaw接近机械限位, 大yaw保持当前位置不动, 防止硬碰撞
                    gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;
                }

                else
                {
                    gimbal.target_value.big_yaw_angle = 0.0f;  // 大yaw回中跟随
                }
            }
        }
    }

    else if (gimbal.now_method == Gimbal_Method_Auto) // 自瞄控制模式(视觉接管)
    {
        if(gimbal.last_method != Gimbal_Method_Auto)
        {
            gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
            gimbal.pc_yaw_big_pid.pid_angle.set_integral_error(0.0f);
            gimbal.pc_yaw_big_pid.pid_omega.set_integral_error(0.0f);

            gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
            gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
            gimbal.rc_yaw_big_angle_pid.set_integral_error(0.0f);

            gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;
            gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
            gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
            gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;

            // 重播种前馈参考用的编码器低通, 避免模式切换时因滤波器状态残留产生前馈台阶
            gimbal.yaw_small_ff_enc_lpf = gimbal.now_value.small_yaw_angle;
            gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 target=now ⇒ delta=0
            gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种, 防切换瞬态
            gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
        }

        else 
        {
            // 自瞄未识别到目标，进行索敌扫频
            if(gimbal.now_mode == Gimbal_Auto_Scan)
            {
                gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;
                // 编码器进行当前角度的参考
                gimbal.now_value.small_yaw_angle = gimbal.now_s_yaw_angle;

                if(gimbal.last_mode != Gimbal_Auto_Scan)
                {
                    gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.pc_yaw_big_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_yaw_big_pid.pid_omega.set_integral_error(0.0f);

                    gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.rc_yaw_big_angle_pid.set_integral_error(0.0f);

                    wave_generator_reset(&yaw_wave);
                    wave_generator_reset(&big_yaw_wave);

                    gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;

                    gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
                    gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;
                    gimbal.target_value.small_yaw_angle = gimbal.now_s_yaw_angle;
                    // 重播种前馈参考用的编码器低通, 避免模式切换时因滤波器状态残留产生前馈台阶
                    gimbal.yaw_small_ff_enc_lpf = gimbal.now_value.small_yaw_angle;
                    gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 target=now ⇒ delta=0
                    gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种, 防切换瞬态
                    gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
                }

                else
                {
                    gimbal.target_value.big_yaw_omega = wave_generator_calc(&big_yaw_wave);

                    if(gimbal.yaw_towards_flag == 0)
                    {
                        gimbal.target_value.small_yaw_angle += yaw_c_angle;

                        if(gimbal.target_value.small_yaw_angle >= (YAW_SMALL_MAX_OFFSET - 0.05f))
                        {
                            gimbal.yaw_towards_flag = 1;
                        }
                    }

                    else 
                    {
                        gimbal.target_value.small_yaw_angle -= yaw_c_angle;

                        if(gimbal.target_value.small_yaw_angle <= (YAW_SMALL_MIN_OFFSET + 0.05f))
                        {
                            gimbal.yaw_towards_flag = 0;
                        }
                    }

                    gimbal.target_value.small_yaw_angle = basic_math_constrain(gimbal.target_value.small_yaw_angle, YAW_SMALL_MIN_OFFSET, YAW_SMALL_MAX_OFFSET);

                    if(gimbal.pit_towards_flag == 0)
                    {
                        gimbal.target_value.pitch_angle += pitch_c_angle;

                        if(gimbal.target_value.pitch_angle >= (gimbal.pitch_limit_angle + PITCH_MAX_OFFSET - 0.18f))
                        {
                            gimbal.pit_towards_flag = 1;
                        }
                    }

                    else 
                    {
                        gimbal.target_value.pitch_angle -= pitch_c_angle;

                        if(gimbal.target_value.pitch_angle <= (gimbal.pitch_limit_angle + PITCH_MIN_OFFSET + 0.15f))
                        {
                            gimbal.pit_towards_flag = 0;
                        }
                    }

                    gimbal.target_value.pitch_angle = basic_math_constrain(gimbal.target_value.pitch_angle, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET);
                }
            }

            // 自瞄识别到目标，进行跟随
            else if(gimbal.now_mode == Gimbal_Auto_Follow)  
            {
                if(gimbal.last_mode != Gimbal_Auto_Follow)
                {
                    gimbal.pc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.pc_yaw_big_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.pc_yaw_big_pid.pid_omega.set_integral_error(0.0f);

                    gimbal.rc_pitch_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_pitch_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_angle.set_integral_error(0.0f);
                    gimbal.rc_yaw_small_pid.pid_omega.set_integral_error(0.0f);
                    gimbal.rc_yaw_big_angle_pid.set_integral_error(0.0f);

                    wave_generator_reset(&yaw_wave);
                    wave_generator_reset(&big_yaw_wave);

                    gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;

                    gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
                    gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;
                    gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
                    // 重播种前馈参考用的编码器低通, 避免模式切换时因滤波器状态残留产生前馈台阶
                    gimbal.yaw_small_ff_enc_lpf = gimbal.now_value.small_yaw_angle;
                    gimbal.yaw_small_ff_delta_lpf = 0.0f;   // 切换瞬间 target=now ⇒ delta=0
                    gimbal.pitch_ff_ref_lpf = gimbal.target_value.pitch_angle;  // [数据平滑] 重播种, 防切换瞬态
                    gimbal.prev_pitch_ff_omega = 0.0f;                          // [拐点缓冲] 清空限制器状态
                }

                else
                {
                    /*----------------------------视觉控制段------------------------------*/
                    // gimbal.target_value.gimbal_yaw_angle = basic_math_loop_constrain(wave_generator_calc(&yaw_wave), -PI, PI);
                    gimbal.target_value.gimbal_yaw_angle = basic_math_loop_constrain(gimbal.filter_yaw.get_out(), -PI, PI);
                    gimbal.target_value.pitch_angle = basic_math_constrain(gimbal.filter_pitch.get_out(), gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET);

                    float delta = basic_math_loop_constrain(gimbal.target_value.gimbal_yaw_angle - gimbal.now_value.gimbal_yaw_angle, -PI, PI);
                    gimbal.target_value.small_yaw_angle = basic_math_constrain(gimbal.now_value.small_yaw_angle + delta, YAW_SMALL_MIN_OFFSET, YAW_SMALL_MAX_OFFSET);

                    float small_yaw_offset = gimbal.yaw_small_motor.get_single_angle() - YAW_SMALL_MOTOR_MID_ANGLE;
                    
                    if (small_yaw_offset <= YAW_SMALL_MIN_OFFSET * 1.12f ||
                        small_yaw_offset >= YAW_SMALL_MAX_OFFSET * 1.12f)
                    {
                        // 小yaw接近机械限位, 大yaw保持当前位置不动, 防止硬碰撞
                        gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;
                    }
                    
                    else
                    {
                        float now_big = gimbal.now_value.big_yaw_angle;
                        float a = basic_math_abs(now_big);

                        float n = a / BIG_YAW_RECENTER_FULL;
                        float rho = BIG_YAW_RECENTER_MIN_FRAC + 0.92f * n*n*n*n;   // 只把 n*n 换成 n^4
                            rho = basic_math_constrain(rho, 0.0f, 1.0f);                             // a>=A_FULL 即满回中

                            gimbal.target_value.big_yaw_angle = (1.0f - rho) * now_big;
                    }

                    /*----------------------------参数调节段------------------------------*/
                    // gimbal.target_value.gimbal_yaw_angle = wave_generator_calc(&yaw_wave);
                    // // gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;
                    // gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;  // pitch保持当前角度
                    // // gimbal.target_value.gimbal_yaw_angle = basic_math_loop_constrain(wave_generator_get_value(&yaw_wave), -PI, PI);

                    // {
                    //     float delta = basic_math_loop_constrain(gimbal.target_value.gimbal_yaw_angle - gimbal.now_value.gimbal_yaw_angle, -PI, PI);
                    //     gimbal.target_value.small_yaw_angle = basic_math_constrain(gimbal.now_value.small_yaw_angle + delta, YAW_SMALL_MIN_OFFSET, YAW_SMALL_MAX_OFFSET);
                    // }

                    // {
                    //     float small_yaw_offset = gimbal.yaw_small_motor.get_single_angle() - YAW_SMALL_MOTOR_MID_ANGLE;
                        
                    //     if (small_yaw_offset <= YAW_SMALL_MIN_OFFSET * 1.12f ||
                    //         small_yaw_offset >= YAW_SMALL_MAX_OFFSET * 1.12f)
                    //     {
                    //         // 小yaw接近机械限位, 大yaw保持当前位置不动, 防止硬碰撞
                    //         gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;
                    //     }
                        
                    //     else
                    //     {
                    //         gimbal.target_value.big_yaw_angle = 0.0f;  // 大yaw回中跟随
                    //     }
                    // }
                }
            }

            // 自瞄未识别到目标，且不在索敌扫频模式下，保持当前角度不动
            else
            {
                gimbal.target_value.gimbal_yaw_angle = gimbal.now_value.gimbal_yaw_angle;
                gimbal.target_value.small_yaw_angle = gimbal.now_value.small_yaw_angle;
                gimbal.target_value.pitch_angle = gimbal.now_value.pitch_angle;
                gimbal.target_value.big_yaw_angle = gimbal.now_value.big_yaw_angle;
            }
        }
    }
}

/**
 * @brief 更新云台补偿量
 * @note 更新云台的重力补偿、摩擦力补偿等
 */
static void gimbal_update_compensation(void)
{
    if(gimbal.now_value.pitch_angle > -0.6f && gimbal.now_value.pitch_angle < 0.5f && gimbal.imu_attitude_ready == 1)
    {
        gimbal.pitch_gravity_comp.amount =  gimbal.pitch_gravity_comp.factor[3] * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle +
                                            gimbal.pitch_gravity_comp.factor[2] * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle +
                                            gimbal.pitch_gravity_comp.factor[1] * gimbal.target_value.pitch_angle +
                                            gimbal.pitch_gravity_comp.factor[0];
    }
    else
    {
        gimbal.pitch_gravity_comp.amount = 0.0f;
    }

    gimbal.pitch_gravity_comp_filter.set_now_input(gimbal.pitch_gravity_comp.amount);
    gimbal.pitch_gravity_comp_filter.low_calculate_callback();

    // gimbal.samples[0][0]  = 1;
    // gimbal.samples[1][0]  = gimbal.target_value.pitch_angle;
    // gimbal.samples[2][0]  = gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle;
    // gimbal.samples[3][0]  = gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle * gimbal.target_value.pitch_angle;

    // gimbal.params = gimbal.rls.update(gimbal.samples, gimbal.pitch_motor.get_now_torque());
    // gimbal.pitch_gravity_comp.factor[0] = gimbal.params[0][0];
    // gimbal.pitch_gravity_comp.factor[1] = gimbal.params[1][0];
    // gimbal.pitch_gravity_comp.factor[2] = gimbal.params[2][0];
    // gimbal.pitch_gravity_comp.factor[3] = gimbal.params[3][0];
}

/**
 * @brief 非线性静态补偿查表 (线性插值)
 * @param angle 目标角度 (rad)
 * @return 补偿扭矩 (Nm)
 */
static float pitch_static_lookup(float angle)
{
    // 边界处理
    if (angle <= pitch_ff_static_angle[0])
        return pitch_ff_static_torque[0];
    if (angle >= pitch_ff_static_angle[PITCH_FF_STATIC_NUM - 1])
        return pitch_ff_static_torque[PITCH_FF_STATIC_NUM - 1];
    
    // 线性插值
    for (int i = 0; i < PITCH_FF_STATIC_NUM - 1; i++)
    {
        if (angle >= pitch_ff_static_angle[i] && angle < pitch_ff_static_angle[i + 1])
        {
            float t = (angle - pitch_ff_static_angle[i]) / (pitch_ff_static_angle[i + 1] - pitch_ff_static_angle[i]);
            return pitch_ff_static_torque[i] + t * (pitch_ff_static_torque[i + 1] - pitch_ff_static_torque[i]);
        }
    }
    return 0.0f;
}

/**
 * @brief 更新云台前馈控制量
 * @note 更新云台的前馈控制量, 用于补偿系统的动态响应
 *       前馈策略:
 *         1. 动态前馈: Feedforward_Calculate 计算 K*θ + B*ω + J*α
 *         2. 速度前馈: Ref_dot 经增益后给角度环, 减小PID负担
 *         3. 静态补偿: 查表修正弹簧/重力的非线性残差
 *         4. 常数偏置: 补偿系统零位偏移
 */
static void gimbal_update_feedforward(void)
{
    gimbal.pitch_ff_ref_lpf += PITCH_FF_REF_LPF_ALPHA *
        (gimbal.target_value.pitch_angle - gimbal.pitch_ff_ref_lpf);
    Feedforward_Calculate(&gimbal.pc_pitch_ffc, gimbal.pitch_ff_ref_lpf);
    
    {
        float raw_ff_omega = basic_math_constrain(
            gimbal.pc_pitch_ffc.Ref_dot * PITCH_FF_VELOCITY_GAIN,
            -PITCH_FF_OMEGA_MAX, PITCH_FF_OMEGA_MAX);
        float dt = gimbal.pc_pitch_ffc.dt;
        if (dt > 0.05f) dt = 0.015f;  // 安全钳位, 防止首次调用或长时间暂停后的dt异常
        float max_delta = PITCH_FF_ACC_LIMIT * dt;
        gimbal.feedforward_value.pitch_omega = basic_math_constrain(
            raw_ff_omega,
            gimbal.prev_pitch_ff_omega - max_delta,
            gimbal.prev_pitch_ff_omega + max_delta);
        gimbal.prev_pitch_ff_omega = gimbal.feedforward_value.pitch_omega;
    }
    
    float dynamic_ff = gimbal.pc_pitch_ffc.Output;
    float coulomb_ff = (gimbal.pc_pitch_ffc.Ref_dot > 0.001f) ? PITCH_FF_COULOMB : 
                       ((gimbal.pc_pitch_ffc.Ref_dot < -0.001f) ? -PITCH_FF_COULOMB : 0.0f);
    float static_ff = pitch_static_lookup(gimbal.target_value.pitch_angle);
    float raw_torque = dynamic_ff + coulomb_ff + static_ff + PITCH_FF_BIAS_C;
    {
        float dt = gimbal.pc_pitch_ffc.dt;
        if (dt > 0.05f) dt = 0.015f;
        float max_delta = 20.0f * dt;  // 20 Nm/s 速率限制
        gimbal.feedforward_value.pitch_torque = basic_math_constrain(
            raw_torque,
            gimbal.prev_pitch_ff_torque - max_delta,
            gimbal.prev_pitch_ff_torque + max_delta);
        gimbal.prev_pitch_ff_torque = gimbal.feedforward_value.pitch_torque;
    }

    gimbal.yaw_small_ff_enc_lpf += YAW_SMALL_FF_ENC_LPF_ALPHA *
        (gimbal.now_value.small_yaw_angle - gimbal.yaw_small_ff_enc_lpf);
    gimbal.yaw_small_ff_delta_lpf += YAW_SMALL_FF_DELTA_LPF_ALPHA *
        ((gimbal.target_value.small_yaw_angle - gimbal.now_value.small_yaw_angle)
         - gimbal.yaw_small_ff_delta_lpf);
    float yaw_small_ff_ref = gimbal.yaw_small_ff_delta_lpf + gimbal.yaw_small_ff_enc_lpf;
    float dc_blocked = yaw_small_ff_ref - gimbal.yaw_small_ff_dc_mean;
    gimbal.yaw_small_ff_dc_mean += 0.0003f * (yaw_small_ff_ref - gimbal.yaw_small_ff_dc_mean);
    Feedforward_Calculate(&gimbal.pc_yaw_small_ffc, dc_blocked);
    // 角速度前馈: 增益缩放 + 安全硬限幅 + 速率限制(平滑拐点冲击)
    {
        float raw_ff_omega = basic_math_constrain(
            gimbal.pc_yaw_small_ffc.Ref_dot * YAW_SMALL_FF_VELOCITY_GAIN,
            -YAW_SMALL_FF_OMEGA_MAX, YAW_SMALL_FF_OMEGA_MAX);
        // 速率限制: 限制每帧变化量, 平滑三角波拐点处ff_omega方向突变
        // v3(260916): 阈值 3.5 → 12.0 rad/s², 详见 YAW_SMALL_FF_RATE_LIMIT 注释
        float dt = gimbal.pc_yaw_small_ffc.dt;
        if (dt > 0.05f) dt = 0.015f;  // 安全钳位, 防止首次调用或长时间暂停后的dt异常
        float max_delta = YAW_SMALL_FF_RATE_LIMIT * dt;
        gimbal.feedforward_value.small_yaw_omega = basic_math_constrain(
            raw_ff_omega,
            gimbal.prev_small_yaw_ff_omega - max_delta,
            gimbal.prev_small_yaw_ff_omega + max_delta);
        gimbal.prev_small_yaw_ff_omega = gimbal.feedforward_value.small_yaw_omega;
    }
    float yaw_coulomb_ff = (gimbal.pc_yaw_small_ffc.Ref_dot > 0.001f) ? YAW_SMALL_FF_COULOMB : 
                           ((gimbal.pc_yaw_small_ffc.Ref_dot < -0.001f) ? -YAW_SMALL_FF_COULOMB : 0.0f);
    gimbal.feedforward_value.small_yaw_torque = gimbal.pc_yaw_small_ffc.Output + yaw_coulomb_ff + YAW_SMALL_FF_BIAS_C;
}

/**
 * @brief 更新云台电机控制
 */
void gimbal_motor_calculate(void)
{
    if (gimbal.now_method == Gimbal_Method_Zero_Force)  // 零力矩模式
    {
        gimbal.yaw_big_motor.set_control_torque(0.0f);
        gimbal.yaw_small_motor.set_target_torque(0.0f);
        gimbal.pitch_motor.set_control_torque(0.0f);
    }

    else if (gimbal.now_method == Gimbal_Method_Manual) // 遥控器控制模式
    {
        // 更新云台前馈量
        // gimbal_update_feedforward();

/*----------------------------------------------小yaw电机rc--------------------------------------------------------*/
        // gimbal.yaw_small_motor.set_target_angle(gimbal.target_value.small_yaw_angle);     // 设置小yaw电机的目标角度
        // gimbal.yaw_small_motor.set_feedforward_omega(gimbal.feedforward_value.small_yaw_omega);
        // gimbal.yaw_small_motor.set_feedforward_torque(gimbal.feedforward_value.small_yaw_torque);
        // gimbal.yaw_small_motor.set_filter_angle(gimbal.now_value.small_yaw_angle);        // 设置小yaw电机的滤波角度
        // gimbal.yaw_small_motor.set_filter_omega(gimbal.now_value.small_yaw_omega);        // 设置小yaw电机的滤波角速度

        gimbal.rc_yaw_small_pid.pid_angle.set_target(gimbal.target_value.small_yaw_angle);
        gimbal.rc_yaw_small_pid.pid_angle.set_now(gimbal.now_value.small_yaw_angle);
        gimbal.rc_yaw_small_pid.pid_angle.pid_calculate_callback();
        gimbal.target_value.small_yaw_omega = gimbal.rc_yaw_small_pid.pid_angle.get_out();

        gimbal.rc_yaw_small_pid.pid_omega.set_target(gimbal.target_value.small_yaw_omega);
        gimbal.rc_yaw_small_pid.pid_omega.set_now(gimbal.now_value.small_yaw_omega);
        gimbal.rc_yaw_small_pid.pid_omega.pid_calculate_callback();
        gimbal.yaw_small_motor.set_target_torque(gimbal.rc_yaw_small_pid.pid_omega.get_out());

/*----------------------------------------------pitch电机rc--------------------------------------------------------*/
        // pitch电机使用外部PID闭环控制，计算角度环和角速度环的输出扭矩
        gimbal.rc_pitch_pid.pid_angle.set_target(basic_math_constrain(gimbal.target_value.pitch_angle, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET));
        gimbal.rc_pitch_pid.pid_angle.set_now(gimbal.now_value.pitch_angle);
        gimbal.rc_pitch_pid.pid_angle.pid_calculate_callback();
        gimbal.target_value.pitch_omega = gimbal.rc_pitch_pid.pid_angle.get_out();

        gimbal.rc_pitch_pid.pid_omega.set_target(gimbal.target_value.pitch_omega);
        gimbal.rc_pitch_pid.pid_omega.set_now(gimbal.now_value.pitch_omega);
        gimbal.rc_pitch_pid.pid_omega.pid_calculate_callback();
        gimbal.pitch_motor.set_control_torque(-gimbal.rc_pitch_pid.pid_omega.get_out() + PITCH_GRAVITY_COMPENSATION);
        
/*----------------------------------------------大yaw电机rc--------------------------------------------------------*/
        gimbal.rc_yaw_big_angle_pid.set_target(gimbal.target_value.big_yaw_angle);  // 设置大yaw电机的目标角度
        gimbal.rc_yaw_big_angle_pid.set_now(gimbal.now_value.big_yaw_angle);  // 设置大yaw电机的当前角度
        gimbal.rc_yaw_big_angle_pid.pid_calculate_callback();  // 计算大yaw电机的角度环输出
        gimbal.yaw_big_motor.set_control_torque(gimbal.rc_yaw_big_angle_pid.get_out());  // 设置大yaw电机的控制扭矩
    }   

    else if (gimbal.now_method == Gimbal_Method_Auto)   // 自瞄控制模式
    {
/*----------------------------------------------小yaw电机pc--------------------------------------------------------*/
        // 更新云台前馈量
        // gimbal_update_feedforward();

/*----------------------------------------------小yaw电机pc--------------------------------------------------------*/

        gimbal.pc_yaw_small_pid.pid_angle.set_target(gimbal.target_value.small_yaw_angle);
        gimbal.pc_yaw_small_pid.pid_angle.set_now(gimbal.now_value.small_yaw_angle);
        gimbal.pc_yaw_small_pid.pid_angle.pid_calculate_callback();
        gimbal.target_value.small_yaw_omega = gimbal.pc_yaw_small_pid.pid_angle.get_out() + gimbal.feedforward_value.small_yaw_omega;

        gimbal.pc_yaw_small_pid.pid_omega.set_target(gimbal.target_value.small_yaw_omega);
        gimbal.pc_yaw_small_pid.pid_omega.set_now(gimbal.now_value.small_yaw_omega);
        gimbal.pc_yaw_small_pid.pid_omega.pid_calculate_callback();
        gimbal.yaw_small_motor.set_target_torque(basic_math_constrain(gimbal.pc_yaw_small_pid.pid_omega.get_out() + gimbal.feedforward_value.small_yaw_torque, -YAW_SMALL_TORQUE_LIMIT, YAW_SMALL_TORQUE_LIMIT));

/*----------------------------------------------pitch电机pc--------------------------------------------------------*/
        // pitch电机使用外部PID闭环控制，计算角度环和角速度环的输出扭矩
        gimbal.pc_pitch_pid.pid_angle.set_target(basic_math_constrain(gimbal.target_value.pitch_angle, gimbal.pitch_limit_angle + PITCH_MIN_OFFSET, gimbal.pitch_limit_angle + PITCH_MAX_OFFSET));
        gimbal.pc_pitch_pid.pid_angle.set_now(gimbal.now_value.pitch_angle);
        // gimbal.pc_pitch_pid.pid_angle.set_now(gimbal.filter_imu_pitch_angle.get_out());
        gimbal.pc_pitch_pid.pid_angle.pid_calculate_callback();
        gimbal.target_value.pitch_omega = gimbal.pc_pitch_pid.pid_angle.get_out() + gimbal.feedforward_value.pitch_omega;
        // gimbal.target_value.pitch_omega = gimbal.pc_pitch_pid.pid_angle.get_out();
        // gimbal.target_value.pitch_omega = gimbal.pc_pitch_pid.pid_angle.get_out();

        gimbal.pc_pitch_pid.pid_omega.set_target(basic_math_constrain(gimbal.target_value.pitch_omega, -1.8f, 1.8f));
        gimbal.pc_pitch_pid.pid_omega.set_now(gimbal.now_value.pitch_omega);
        // gimbal.pc_pitch_pid.pid_omega.set_now(gimbal.filter_pitch_pitch_omega.get_out());
        gimbal.pc_pitch_pid.pid_omega.pid_calculate_callback();
        gimbal.pitch_motor.set_control_torque(basic_math_constrain((-gimbal.pc_pitch_pid.pid_omega.get_out() + gimbal.feedforward_value.pitch_torque + gimbal.pitch_gravity_comp_filter.get_out()), -PITCH_TORQUE_LIMIT, PITCH_TORQUE_LIMIT));
        // gimbal.pitch_motor.set_control_torque(-gimbal.pc_pitch_pid.pid_omega.get_out());
        // gimbal.pitch_motor.set_control_torque(-gimbal.pc_pitch_pid.pid_omega.get_out());
        
/*----------------------------------------------大yaw电机pc--------------------------------------------------------*/
        if(gimbal.now_mode == Gimbal_Auto_Follow)
        {
            gimbal.pc_yaw_big_pid.pid_angle.set_target(gimbal.target_value.big_yaw_angle);  // 设置大yaw电机的目标角度
            gimbal.pc_yaw_big_pid.pid_angle.set_now(gimbal.now_value.big_yaw_angle);  // 设置大yaw电机的当前角度
            gimbal.pc_yaw_big_pid.pid_angle.pid_calculate_callback();  // 计算大yaw电机的角度环输出
            gimbal.target_value.big_yaw_omega = gimbal.pc_yaw_big_pid.pid_angle.get_out();  // 计算大yaw电机的目标角速度
        }

        // 大yaw角速度指令速率限制: 平滑Follow↔Scan切换瞬间,
        // 位置环输出(可达数rad/s)与扫频正弦(offset=0)之间的目标值阶跃, 抑制速度环P项力矩尖峰
        {
            float dt = bsp_dwt_get_deltat32(&gimbal.big_yaw_omega_rl_cnt);
            if (dt > 0.05f) dt = 0.001f;   // 安全钳位: 首次调用或长时间暂停后的dt异常
            float max_delta = BIG_YAW_OMEGA_SLEW * dt;
            gimbal.target_value.big_yaw_omega = basic_math_constrain(gimbal.target_value.big_yaw_omega, gimbal.prev_big_yaw_omega - max_delta, gimbal.prev_big_yaw_omega + max_delta);
            gimbal.prev_big_yaw_omega = gimbal.target_value.big_yaw_omega;
        }

        gimbal.pc_yaw_big_pid.pid_omega.set_target(gimbal.target_value.big_yaw_omega + gimbal.feedforward_value.chassis_angular);  // 设置大yaw电机的目标角速度
        gimbal.pc_yaw_big_pid.pid_omega.set_now(gimbal.now_value.big_yaw_omega);  // 设置大yaw电机的当前角速度
        gimbal.pc_yaw_big_pid.pid_omega.pid_calculate_callback();  // 计算大yaw电机的角速度环输出
        gimbal.yaw_big_motor.set_control_torque(gimbal.pc_yaw_big_pid.pid_omega.get_out());  // 设置大yaw电机的控制扭矩
    }

    gimbal.yaw_small_motor.external_calculate_callback();                       // 执行小yaw电机的外部PID闭环计算和控制
    bsp_can_1_to_4_transmit(&hfdcan1, 0x1fe, can1_0x1fe_tx_data, Can_transmit_frequency_1000Hz);  // 发送小yaw电机的CAN数据
    gimbal.pitch_motor.calculate_callback();
    gimbal.yaw_big_motor.calculate_callback();

    gimbal.last_method = gimbal.now_method;
    gimbal.last_mode = gimbal.now_mode;
}

/**
 * @brief 云台测试任务用于调试和验证云台电机参数
 * @note 该任务在实际应用中可用于验证云台电机的控制性能和参数设置，确保云台在不同控制模式下的稳定性和响应速度。
 */
void gimbal_motor_parameter_test()
{
    wave_generator_calc(&yaw_wave);
    reference_track_plan_update(&gimbal.yaw_small_track_plan, wave_generator_get_value(&yaw_wave), 0.001f, &gimbal.yaw_small_plan_output);
}

extern "C" void gimbal_task(void *argument)
{
    // 初始化云台
    gimbal_init();
    osDelay(1000);

    for(;;)
    {
        // 更新云台测量值
        gimbal_update_measure();

        // 更新云台控制模式
        gimbal_update_mode();

        // 更新云台决策源数据
        gimbal_robot_decision_source_update();

        // 更新云台控制决策
        gimbal_control_decision();

        // 更新云台前馈量
        gimbal_update_feedforward();

        gimbal_update_compensation();

        // 更新云台电机控制
        gimbal_motor_calculate();

        // 云台测试任务
        gimbal_motor_parameter_test();

        // 任务执行频率1000Hz
        osDelay(1);
    }
}




