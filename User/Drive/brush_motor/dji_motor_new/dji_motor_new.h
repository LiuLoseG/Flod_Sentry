#pragma once

#include "bsp_can.h"
#include "pid.h"
#include "basic_math.h"

/**
 * @brief 大疆状态
 *
 */
typedef enum
{
    Motor_DJI_Status_Disable = 0,
    Motor_DJI_Status_Enable = 1,
} dji_motor_status_e;

/**
 * @brief 大疆电机类型
 *
 */
typedef enum
{
    // 大疆电机ID, 0x201~0x204, 
    Motor_DJI_ID_0x201 = 1,     // GM3508_ID_1、GM2006_ID_1
    Motor_DJI_ID_0x202,         // GM3508_ID_2、GM2006_ID_2
    Motor_DJI_ID_0x203,         // GM3508_ID_3、GM2006_ID_3
    Motor_DJI_ID_0x204,         // GM3508_ID_4、GM2006_ID_4

    Motor_DJI_ID_0x205,         // GM6020_ID_1、GM3508_ID_5、GM2006_ID_5
    Motor_DJI_ID_0x206,         // GM6020_ID_2、GM3508_ID_6、GM2006_ID_6
    Motor_DJI_ID_0x207,         // GM6020_ID_3、GM3508_ID_7、GM2006_ID_7
    Motor_DJI_ID_0x208,         // GM6020_ID_4、GM3508_ID_8、GM2006_ID_8
    Motor_DJI_ID_0x209,         // GM6020_ID_5
    Motor_DJI_ID_0x20A,         // GM6020_ID_6
    Motor_DJI_ID_0x20B,         // GM6020_ID_7
} dji_motor_id_e;

/**
 * @brief 大疆控制方式
 *
 */
typedef enum
{
    DJI_Motor_Control_Method_Voltage = 0,
    DJI_Motor_Control_Method_Current,
    DJI_Motor_Control_Method_Torque,
    DJI_Motor_Control_Method_Omega,
    DJI_Motor_Control_Method_Angle,
} dji_motor_control_method_e;

/**
 * @brief 大疆电机驱动版本, 分配CAN_ID时使用, 一般都是default, 6020电机使用
 *
 */
typedef enum
{
    Motor_DJI_GM6020_Driver_Version_Default = 0,
    Motor_DJI_GM6020_Driver_Version_6020,
} dji_motor_gm6020_driver_version_e;

/**
 * @brief 是否开启功率控制, 此时电机须电流作为输出模式, 不可电压控制
 *
 */
typedef enum
{
    Motor_DJI_Power_Limit_Status_Disable = 0,
    Motor_DJI_Power_Limit_Status_Enable,
} dji_motor_power_limit_status_e;

/**
 * @brief 大疆电机源数据
 *
 */
typedef struct  __attribute__((packed))
{
    uint16_t encoder_reverse;
    int16_t omega_reverse;
    int16_t current_reverse;
    uint8_t temperature;
    uint8_t reserved;
} dji_motor_source_data_t;

/**
 * @brief 大疆电机经过处理的数据
 *
 */
typedef struct
{
    // 输出轴状态
    float now_angle;            // 输出轴当前总角度, rad
    float single_angle;         // 输出轴当前单圈角度, rad, 以 0 为中心归一化
    float now_omega;            // 输出轴当前角速度, rad/s
    float now_torque;           // 输出轴当前扭矩, Nm
    float now_temperature;      // 当前温度
    float now_power;            // 当前功率

    // 电机转子状态
    float rotor_angle;          // 电机转子当前总角度, rad, (rotor_angle / gearbox_rate) = now_angle
    float rotor_omega;          // 电机转子当前角速度, rad/s, (rotor_omega / gearbox_rate) = now_omega
    float rotor_current;        // 电机转子原始输出电流, (A * Current_To_Out), (rotor_current / Current_To_Out * Current_To_Torque * gearbox_rate) = now_torque

    uint32_t pre_encoder;       // 上一时刻编码器值
    int32_t total_encoder;      // 总编码器值
    int32_t total_round;        // 总圈数
} dji_motor_processed_data_t;

/**
 * @brief Reusable, GM6020无刷电机, 单片机控制输出电压, 新驱动支持电流
 *
 */
class dji_gm6020_motor_class
{
public:

    // pid角度环控制
    pid_class pid_angle; 
    // pid角速度环控制
    pid_class pid_omega;

    void init(const FDCAN_HandleTypeDef *hcan, const dji_motor_id_e &__can_rx_id, const dji_motor_control_method_e &__motor_dji_control_method = DJI_Motor_Control_Method_Angle, const int32_t &__encoder_offset = 0, const float &__nearest_angle = 0.0f);

    // 获取电机状态
    inline dji_motor_status_e get_status() const;

    inline float get_current_torque_rate() const;           // 获取电机反馈的电流扭矩转换系数

    // 获取电机输出轴反馈数据
    inline float get_now_angle() const;

    inline float get_single_angle() const;

    inline float get_now_omega() const;

    inline float get_now_torque() const;

    inline float get_now_temperature() const;

    inline float get_now_power() const;

    // 获取电机转子状态和数据
    inline float get_rotor_angle() const;

    inline float get_rotor_omega() const;

    inline float get_rotor_current() const;

    // 获取电机控制方式和控制参数
    inline dji_motor_control_method_e get_control_method() const;

    inline float get_target_angle() const;

    inline float get_target_omega() const;

    inline float get_target_torque() const;

    inline float get_feedforward_omega() const;

    inline float get_feedforward_torque() const;

    // 设置电机的控制方式和控制参数
    inline void set_control_method(const dji_motor_control_method_e &__motor_dji_control_method);

    inline void set_target_angle(const float &__target_angle);

    inline void set_target_omega(const float &__target_omega);

    inline void set_target_torque(const float &__target_torque);

    inline void set_feedforward_omega(const float &__feedforward_omega);

    inline void set_feedforward_torque(const float &__feedforward_torque);

    // 滤波处理后的电机反馈数据
    inline void set_filter_angle(const float &__filter_angle);

    inline void set_filter_omega(const float &__filter_omega);

    inline void set_filter_torque(const float &__filter_torque);

    // 计算和执行电机解算和控制任务
    void can_rx_callback();

    void alive_check();

    void internal_calculate_callback();         // 使用电机反馈的原始数据，进行PID闭环控制

    void external_calculate_callback();         // 使用滤波或处理后的传感器数据，进行PID闭环控制

protected:

// 初始化相关变量

    // 绑定的CAN
    volatile FDCAN_RxFrame_TypeDef *can_rx_frame;
    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    dji_motor_id_e can_rx_id;
    // 发送缓存区
    uint8_t *tx_data;
    // 编码器偏移
    int32_t encoder_offset;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float nearest_angle;

    // 常量

    // 功率计算系数
    const float POWER_K_0 = 0.5807780373237382f;
    const float POWER_K_1 = 0.000495794444214091f;
    const float POWER_K_2 = 27.17379907809995f;
    const float POWER_A = 0.9334894382537696f;

    // 一圈编码器刻度
    const uint16_t Encoder_Num_Per_Round = 8192;

    // 扭矩电流常数
    const float Current_To_Torque = 0.741f;
    // 扭矩电流到输出刻度的转化系数
    const float Current_To_Out = 16384.0f / 3.0f;
    // 最大输出刻度
    const float Out_Max = 16384.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag = 0;
    // 首帧反馈标志, 复位/上电后首帧建立编码器基准, 防止误计圈
    bool first_fb_flag = true;
    // 输出量
    float out = 0.0f;

    // 读变量

    // 电机状态
    dji_motor_status_e motor_status = Motor_DJI_Status_Disable;
    // 电机对外接口信息
    dji_motor_processed_data_t rx_data;

    // 写变量

    // 读写变量

    // 电机控制方式
    dji_motor_control_method_e control_method = DJI_Motor_Control_Method_Angle;
    // 目标的角度, rad
    float target_angle = 0.0f;
    // 目标的速度, rad/s
    float target_omega = 0.0f;
    // 目标的扭矩, Nm
    float target_torque = 0.0f;
    // 前馈的速度, rad/s
    float feedforward_omega = 0.0f;
    // 前馈的扭矩, Nm
    float feedforward_torque = 0.0f;

    // 滤波后的当前角度
    float filter_angle = 0.0f;
    // 滤波后的速度, rad/s
    float filter_omega = 0.0f;
    // 滤波后的扭矩, Nm
    float filter_torque = 0.0f;

    // 内部函数

    void data_process();

    void pid_internal_calculate();

    void pid_external_calculate();

    void output();
};

/**
 * @brief Reusable, C610无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class dji_c610_motor_class
{
public:
    // pid角度环控制
    pid_class pid_angle;
    // pid角速度环控制
    pid_class pid_omega;

    void init(const FDCAN_HandleTypeDef *hcan, const dji_motor_id_e &__can_rx_id, const dji_motor_control_method_e &__motor_dji_control_method = DJI_Motor_Control_Method_Omega, const float &__gearbox_rate = 36.0f);

    // 获取电机状态和数据
    inline dji_motor_status_e get_status() const;

    inline float get_gearbox_rate() const;                  // 获取减速比

    inline float get_current_torque_rate() const;           // 获取电机反馈的电流扭矩转换系数

    inline float get_now_angle() const;

    inline float get_single_angle() const;

    inline float get_now_omega() const;

    inline float get_now_torque() const;

    inline float get_now_temperature() const;

    // 获取电机转子状态和数据
    inline float get_rotor_angle() const;

    inline float get_rotor_omega() const;

    inline float get_rotor_current() const;

    // 获取电机的控制方式和控制参数
    inline dji_motor_control_method_e get_control_method() const;

    inline float get_target_angle() const;

    inline float get_target_omega() const;

    inline float get_target_torque() const;

    inline float get_feedforward_omega() const;

    inline float get_feedforward_torque() const;

    // 设置电机的控制方式和控制参数
    inline void set_control_method(const dji_motor_control_method_e &__motor_dji_control_method);

    inline void set_target_angle(const float &__target_angle);

    inline void set_target_omega(const float &__target_omega);

    inline void set_target_torque(const float &__target_torque);

    inline void set_feedforward_omega(const float &__feedforward_omega);

    inline void set_feedforward_torque(const float &__feedforward_torque);

    // 滤波处理后的电机反馈数据
    inline void set_filter_angle(const float &__filter_angle);

    inline void set_filter_omega(const float &__filter_omega);

    inline void set_filter_torque(const float &__filter_torque);

    // 计算和执行电机解算和控制任务
    void can_rx_callback();

    void alive_check();

    void internal_calculate_callback();         // 使用电机反馈的原始数据，进行PID闭环控制

    void external_calculate_callback();         // 使用滤波或处理后的传感器数据，进行PID闭环控制

protected:
    // 初始化相关常量

    // 绑定的CAN
    volatile FDCAN_RxFrame_TypeDef *can_rx_frame;

    // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    dji_motor_id_e can_rx_id;

     // 发送缓存区
    uint8_t *tx_data;
    // 减速比, 默认带减速箱
    float gearbox_rate;

    // 常量

    // 一圈编码器刻度
    const uint16_t Encoder_Num_Per_Round = 8192;

    // 扭矩电流常数, 减速前
    const float Current_To_Torque = 0.18f / 36.0f;
    // 扭矩电流到输出的转化系数
    const float Current_To_Out = 10000.0f / 10.0f;
    // 最大输出刻度
    const float Out_Max = 10000.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag = 0;
    // 首帧反馈标志, 复位/上电后首帧建立编码器基准, 防止误计圈
    bool first_fb_flag = true;
    // 输出量
    float out = 0.0f;

    // 读变量

    // 电机状态
    dji_motor_status_e motor_status = Motor_DJI_Status_Disable;
    // 电机对外接口信息
    dji_motor_processed_data_t rx_data;

    // 写变量

    // 读写变量

    // 电机控制方式
    dji_motor_control_method_e control_method = DJI_Motor_Control_Method_Omega;
    // 目标的角度, rad
    float target_angle = 0.0f;
    // 目标的速度, rad/s
    float target_omega = 0.0f;
    // 目标的扭矩, Nm
    float target_torque = 0.0f;
    // 前馈的速度, rad/s
    float feedforward_omega = 0.0f;
    // 前馈的扭矩, Nm
    float feedforward_torque = 0.0f;

    // 滤波后的当前角度
    float filter_angle = 0.0f;
    // 滤波后的速度, rad/s
    float filter_omega = 0.0f;
    // 滤波后的扭矩, Nm
    float filter_torque = 0.0f;

    // 内部函数

    void data_process();

    void pid_internal_calculate();

    void pid_external_calculate();

    void output();
};

/**
 * @brief Reusable, C620无刷电调, 自带电流环, 单片机控制输出电流
 *
 */
class dji_c620_motor_class
{
public:
    // pid角度环控制
    pid_class pid_angle;
    // pid角速度环控制
    pid_class pid_omega;

    void init(const FDCAN_HandleTypeDef *hcan, const dji_motor_id_e &__can_rx_id, const dji_motor_control_method_e &__motor_dji_control_method = DJI_Motor_Control_Method_Omega, const float &__gearbox_rate = 3591.0f / 187.0f);

    // 获取电机输出轴状态和数据
    inline dji_motor_status_e get_status() const;

    inline float get_gearbox_rate() const;                  // 获取减速比

    inline float get_current_torque_rate() const;           // 获取电机反馈的电流扭矩转换系数

    inline float get_now_angle() const;

    inline float get_single_angle() const;

    inline float get_now_omega() const;

    inline float get_now_torque() const;

    inline float get_now_temperature() const;

    inline float get_now_power() const;

    // 获取电机转子状态和数据
    inline float get_rotor_angle() const;

    inline float get_rotor_omega() const;

    inline float get_rotor_current() const;

    // 获取电机的控制方式和控制参数
    inline dji_motor_control_method_e get_control_method() const;

    inline float get_target_angle() const;

    inline float get_target_omega() const;

    inline float get_target_torque() const;

    inline float get_feedforward_omega() const;

    inline float get_feedforward_torque() const;

    // 设置电机的控制方式和控制参数
    inline void set_control_method(const dji_motor_control_method_e &__motor_dji_control_method);

    inline void set_target_angle(const float &__target_angle);

    inline void set_target_omega(const float &__target_omega);

    inline void set_target_torque(const float &__target_torque);

    inline void set_feedforward_omega(const float &__feedforward_omega);

    inline void set_feedforward_torque(const float &__feedforward_torque);

    // 滤波处理后的电机反馈数据
    inline void set_filter_angle(const float &__filter_angle);
    
    inline void set_filter_omega(const float &__filter_omega);

    inline void set_filter_torque(const float &__filter_torque);

    // 计算和执行电机解算和控制任务
    void can_rx_callback();

    void alive_check();

    void internal_calculate_callback();         // 使用电机反馈的原始数据，进行PID闭环控制

    void external_calculate_callback();         // 使用滤波或处理后的传感器数据，进行PID闭环控制

protected:
    // 初始化相关常量

    // 绑定的CAN
    volatile FDCAN_RxFrame_TypeDef *can_rx_frame;

        // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
    dji_motor_id_e can_rx_id;

     // 发送缓存区
    uint8_t *tx_data;
    // 减速比, 默认带减速箱
    float gearbox_rate;

    // 常量

    // 功率计算系数
    const float POWER_K_0 = 1.1532294690949845f;
    const float POWER_K_1 = 0.00003165414968530437f;
    const float POWER_K_2 = 683.7802569066574f;
    const float POWER_A = 0.8267041157447231f;

    // 一圈编码器刻度
    const uint16_t Encoder_Num_Per_Round = 8192;

    // 扭矩电流常数, 减速前
    const float Current_To_Torque = 0.3f / (3591.0f / 187.0f);
    // 扭矩电流到输出的转化系数
    const float Current_To_Out = 16384.0f / 20.0f;
    // 最大输出刻度
    const float Out_Max = 16384.0f;

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag = 0;
    // 首帧反馈标志, 复位/上电后首帧建立编码器基准, 防止误计圈
    bool first_fb_flag = true;
    // 输出量
    float out = 0.0f;

    // 读变量

    // 电机状态
    dji_motor_status_e motor_status = Motor_DJI_Status_Disable;
    // 电机对外接口信息
    dji_motor_processed_data_t rx_data;

    // 写变量

    // 读写变量

    // 电机控制方式
    dji_motor_control_method_e control_method = DJI_Motor_Control_Method_Angle;
    // 目标的角度, rad
    float target_angle = 0.0f;
    // 目标的速度, rad/s
    float target_omega = 0.0f;
    // 目标的扭矩, Nm
    float target_torque = 0.0f;
    // 前馈的速度, rad/s
    float feedforward_omega = 0.0f;
    // 前馈的扭矩, Nm
    float feedforward_torque = 0.0f;

    // 滤波后的当前角度
    float filter_angle = 0.0f;
    // 滤波后的速度, rad/s
    float filter_omega = 0.0f;
    // 滤波后的扭矩, Nm
    float filter_torque = 0.0f;

    // 内部函数

    void data_process();

    void pid_internal_calculate();

    void pid_external_calculate();

    void output();
};

/*------------------------------------------------------DJI_GM6020----------------------------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return dji_motor_status_e 电机状态
 */
inline dji_motor_status_e dji_gm6020_motor_class::get_status() const
{
    return (motor_status);
}

/**
 * @brief 获取电机反馈的电流扭矩转换系数
 *
 * @return float 电流扭矩转换系数
 */
inline float dji_gm6020_motor_class::get_current_torque_rate() const
{
    return (Current_To_Out / Current_To_Torque);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float dji_gm6020_motor_class::get_now_angle() const
{
    return (rx_data.now_angle);
}

/**
 * @brief 获取输出轴当前单圈角度
 * @retval float: 单圈角度, rad, 范围 (-PI, PI]
 */
inline float dji_gm6020_motor_class::get_single_angle() const
{
    return (rx_data.single_angle);
}

/**
 * @brief 获取当前电机的角速度, rad/s
 *
 * @return float 当前的角速度, rad/s
 */
inline float dji_gm6020_motor_class::get_now_omega() const
{
    return (rx_data.now_omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float dji_gm6020_motor_class::get_now_torque() const
{
    return (rx_data.now_torque);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return float 当前的温度, K
 */
inline float dji_gm6020_motor_class::get_now_temperature() const
{
    return (rx_data.now_temperature);
}

/**
 * @brief 获取当前的功率, W
 *
 * @return float 当前的功率, W
 */
inline float dji_gm6020_motor_class::get_now_power() const
{
    return (rx_data.now_power);
}

/**
 * @brief 获取电机转子当前的角度, rad
 *
 * @return float 电机转子当前的角度, rad
 */
inline float dji_gm6020_motor_class::get_rotor_angle() const
{
    return (rx_data.rotor_angle);
}

/**
 * @brief 获取电机转子当前的角速度, rad/s
 *
 * @return float 电机转子当前的角速度, rad/s
 */
inline float dji_gm6020_motor_class::get_rotor_omega() const
{
    return (rx_data.rotor_omega);
}

/**
 * @brief 获取电机转子当前的电流, A
 *
 * @return float 电机转子当前的电流, A
 */
inline float dji_gm6020_motor_class::get_rotor_current() const
{
    return (rx_data.rotor_current);
}

/**
 * @brief 获取电机控制方式
 *
 * @return dji_motor_control_method_e 电机控制方式
 */
inline dji_motor_control_method_e dji_gm6020_motor_class::get_control_method() const
{
    return (control_method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float dji_gm6020_motor_class::get_target_angle() const
{
    return (target_angle);
}

/**
 * @brief 获取目标电机的角速度, rad/s
 *
 * @return float 目标电机的角速度, rad/s
 */
inline float dji_gm6020_motor_class::get_target_omega() const
{
    return (target_omega);
}

/**
 * @brief 获取目标电机的扭矩, Nm
 *
 * @return float 目标电机的扭矩, Nm
 */
inline float dji_gm6020_motor_class::get_target_torque() const
{
    return (target_torque);
}

/**
 * @brief 获取电机前馈角速度, rad/s
 *
 * @return float 获取电机前馈角速度, rad/s
 */
inline float dji_gm6020_motor_class::get_feedforward_omega() const
{
    return (feedforward_omega);
}

/**
 * @brief 获取电机前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float dji_gm6020_motor_class::get_feedforward_torque() const
{
    return (feedforward_torque);
}

/**
 * @brief 设置电机控制方式
 *
 * @param __motor_dji_control_method 电机控制方式
 */
inline void dji_gm6020_motor_class::set_control_method(const dji_motor_control_method_e &__motor_dji_control_method)
{
    control_method = __motor_dji_control_method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __target_angle 目标的角度, rad
 */
inline void dji_gm6020_motor_class::set_target_angle(const float &__target_angle)
{
    target_angle = __target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __target_omega 目标的速度, rad/s
 */
inline void dji_gm6020_motor_class::set_target_omega(const float &__target_omega)
{
    target_omega = __target_omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __target_torque 目标的扭矩, Nm
 */
inline void dji_gm6020_motor_class::set_target_torque(const float &__target_torque)
{
    target_torque = __target_torque;
}

/**
 * @brief 设定前馈的角速度, rad/s
 *
 * @param __feedforward_omega 前馈的角速度, rad/s
 */
inline void dji_gm6020_motor_class::set_feedforward_omega(const float &__feedforward_omega)
{
    feedforward_omega = __feedforward_omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __feedforward_torque 前馈的扭矩, Nm
 */
inline void dji_gm6020_motor_class::set_feedforward_torque(const float &__feedforward_torque)
{
    feedforward_torque = __feedforward_torque;
}

/**
 * @brief 设置滤波后的角度
 *
 * @param __filter_angle 滤波后的角度, rad
 */
inline void dji_gm6020_motor_class::set_filter_angle(const float &__filter_angle)
{
    filter_angle = __filter_angle;
}

/**
 * @brief 设置滤波后的角速度
 *
 * @param __filter_omega 滤波后的角速度, rad/s
 */
inline void dji_gm6020_motor_class::set_filter_omega(const float &__filter_omega)
{
    filter_omega = __filter_omega;
}

/**
 * @brief 设置滤波后的扭矩
 *
 * @param __filter_torque 滤波后的扭矩, Nm
 */
inline void dji_gm6020_motor_class::set_filter_torque(const float &__filter_torque)
{
    filter_torque = __filter_torque;
}

/*------------------------------------------------------DJI_C610_GM2006----------------------------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return dji_motor_status_e 电机状态
 */
inline dji_motor_status_e dji_c610_motor_class::get_status() const
{
    return (motor_status);
}

/**
 * @brief 获取减速比
 *
 * @return float 减速比
 */
inline float dji_c610_motor_class::get_gearbox_rate() const
{
    return (gearbox_rate);
}

/**
 * @brief 获取电机反馈的电流扭矩转换系数
 *
 * @return float 电流扭矩转换系数
 */
inline float dji_c610_motor_class::get_current_torque_rate() const
{
    return (Current_To_Out / Current_To_Torque / gearbox_rate);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float dji_c610_motor_class::get_now_angle() const
{
    return (rx_data.now_angle);
}

/**
 * @brief 获取输出轴当前单圈角度
 * @retval float: 单圈角度, rad, 范围 (-PI/gearbox_rate, PI/gearbox_rate]
 */
inline float dji_c610_motor_class::get_single_angle() const
{
    return (rx_data.single_angle);
}

/**
 * @brief 获取当前电机的角速度, rad/s
 *
 * @return float 当前的角速度, rad/s
 */
inline float dji_c610_motor_class::get_now_omega() const
{
    return (rx_data.now_omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float dji_c610_motor_class::get_now_torque() const
{
    return (rx_data.now_torque);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return float 当前的温度, K
 */
inline float dji_c610_motor_class::get_now_temperature() const
{
    return (rx_data.now_temperature);
}

/**
 * @brief 获取电机转子当前的角度, rad
 *
 * @return float 电机转子当前的角度, rad
 */
inline float dji_c610_motor_class::get_rotor_angle() const
{
    return (rx_data.rotor_angle);
}

/**
 * @brief 获取电机转子当前的角速度, rad/s
 *
 * @return float 电机转子当前的角速度, rad/s
 */
inline float dji_c610_motor_class::get_rotor_omega() const
{
    return (rx_data.rotor_omega);
}

/**
 * @brief 获取电机转子当前的电流, A
 *
 * @return float 电机转子当前的电流, A
 */
inline float dji_c610_motor_class::get_rotor_current() const
{
    return (rx_data.rotor_current);
}

/**
 * @brief 获取电机控制方式
 *
 * @return dji_motor_control_method_e 电机控制方式
 */
inline dji_motor_control_method_e dji_c610_motor_class::get_control_method() const
{
    return (control_method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float dji_c610_motor_class::get_target_angle() const
{
    return (target_angle);
}

/**
 * @brief 获取目标电机的角速度, rad/s
 *
 * @return float 目标电机的角速度, rad/s
 */
inline float dji_c610_motor_class::get_target_omega() const
{
    return (target_omega);
}

/**
 * @brief 获取目标电机的扭矩, Nm
 *
 * @return float 目标电机的扭矩, Nm
 */
inline float dji_c610_motor_class::get_target_torque() const
{
    return (target_torque);
}

/**
 * @brief 获取电机前馈角速度, rad/s
 *
 * @return float 获取电机前馈角速度, rad/s
 */
inline float dji_c610_motor_class::get_feedforward_omega() const
{
    return (feedforward_omega);
}

/**
 * @brief 获取电机前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float dji_c610_motor_class::get_feedforward_torque() const
{
    return (feedforward_torque);
}

/**
 * @brief 设置电机控制方式
 *
 * @param __motor_dji_control_method 电机控制方式
 */
inline void dji_c610_motor_class::set_control_method(const dji_motor_control_method_e &__motor_dji_control_method)
{
    control_method = __motor_dji_control_method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __target_angle 目标的角度, rad
 */
inline void dji_c610_motor_class::set_target_angle(const float &__target_angle)
{
    target_angle = __target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __target_omega 目标的速度, rad/s
 */
inline void dji_c610_motor_class::set_target_omega(const float &__target_omega)
{
    target_omega = __target_omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __target_torque 目标的扭矩, Nm
 */
inline void dji_c610_motor_class::set_target_torque(const float &__target_torque)
{
    target_torque = __target_torque;
}

/**
 * @brief 设定前馈的角速度, rad/s
 *
 * @param __feedforward_omega 前馈的角速度, rad/s
 */
inline void dji_c610_motor_class::set_feedforward_omega(const float &__feedforward_omega)
{
    feedforward_omega = __feedforward_omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __feedforward_torque 前馈的扭矩, Nm
 */
inline void dji_c610_motor_class::set_feedforward_torque(const float &__feedforward_torque)
{
    feedforward_torque = __feedforward_torque;
}

/**
 * @brief 设置滤波后的角度
 *
 * @param __filter_angle 滤波后的角度, rad
 */
inline void dji_c610_motor_class::set_filter_angle(const float &__filter_angle)
{
    filter_angle = __filter_angle;
}

/**
 * @brief 设置滤波后的角速度
 *
 * @param __filter_omega 滤波后的角速度, rad/s
 */
inline void dji_c610_motor_class::set_filter_omega(const float &__filter_omega)
{
    filter_omega = __filter_omega;
}

/**
 * @brief 设置滤波后的扭矩
 *
 * @param __filter_torque 滤波后的扭矩, Nm
 */
inline void dji_c610_motor_class::set_filter_torque(const float &__filter_torque)
{
    filter_torque = __filter_torque;
}
/*------------------------------------------------------DJI_C620_GM3508----------------------------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return dji_motor_status_e 电机状态
 */
inline dji_motor_status_e dji_c620_motor_class::get_status() const
{
    return (motor_status);
}

/**
 * @brief 获取减速比
 *
 * @return float 减速比
 */
inline float dji_c620_motor_class::get_gearbox_rate() const
{
    return (gearbox_rate);
}

/**
 * @brief 获取电机反馈的电流扭矩转换系数
 *
 * @return float 电流扭矩转换系数
 */
inline float dji_c620_motor_class::get_current_torque_rate() const
{
    return (Current_To_Out / Current_To_Torque / gearbox_rate);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
inline float dji_c620_motor_class::get_now_angle() const
{
    return (rx_data.now_angle);
}

/**
 * @brief 获取输出轴当前单圈角度
 * @retval float: 单圈角度, rad, 范围 (-PI/gearbox_rate, PI/gearbox_rate]
 */
inline float dji_c620_motor_class::get_single_angle() const
{
    return (rx_data.single_angle);
}

/**
 * @brief 获取当前电机的角速度, rad/s
 *
 * @return float 当前的角速度, rad/s
 */
inline float dji_c620_motor_class::get_now_omega() const
{
    return (rx_data.now_omega);
}

/**
 * @brief 获取当前的扭矩, Nm
 *
 * @return float 当前的扭矩, Nm
 */
inline float dji_c620_motor_class::get_now_torque() const
{
    return (rx_data.now_torque);
}

/**
 * @brief 获取当前的温度, K
 *
 * @return float 当前的温度, K
 */
inline float dji_c620_motor_class::get_now_temperature() const
{
    return (rx_data.now_temperature);
}

/**
 * @brief 获取当前的功率, W
 *
 * @return float 当前的功率, W
 */
inline float dji_c620_motor_class::get_now_power() const
{
    return (rx_data.now_power);
}

/**
 * @brief 获取电机转子当前的角度, rad
 *
 * @return float 电机转子当前的角度, rad
 */
inline float dji_c620_motor_class::get_rotor_angle() const
{
    return (rx_data.rotor_angle);
}

/**
 * @brief 获取电机转子当前的角速度, rad/s
 *
 * @return float 电机转子当前的角速度, rad/s
 */
inline float dji_c620_motor_class::get_rotor_omega() const
{
    return (rx_data.rotor_omega);
}

/**
 * @brief 获取电机转子当前的电流, A
 *
 * @return float 电机转子当前的电流, A
 */
inline float dji_c620_motor_class::get_rotor_current() const
{
    return (rx_data.rotor_current);
}

/**
 * @brief 获取电机控制方式
 *
 * @return dji_motor_control_method_e 电机控制方式
 */
inline dji_motor_control_method_e dji_c620_motor_class::get_control_method() const
{
    return (control_method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
inline float dji_c620_motor_class::get_target_angle() const
{
    return (target_angle);
}

/**
 * @brief 获取目标电机的角速度, rad/s
 *
 * @return float 目标电机的角速度, rad/s
 */
inline float dji_c620_motor_class::get_target_omega() const
{
    return (target_omega);
}

/**
 * @brief 获取目标电机的扭矩, Nm
 *
 * @return float 目标电机的扭矩, Nm
 */
inline float dji_c620_motor_class::get_target_torque() const
{
    return (target_torque);
}

/**
 * @brief 获取电机前馈角速度, rad/s
 *
 * @return float 获取电机前馈角速度, rad/s
 */
inline float dji_c620_motor_class::get_feedforward_omega() const
{
    return (feedforward_omega);
}

/**
 * @brief 获取电机前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float dji_c620_motor_class::get_feedforward_torque() const
{
    return (feedforward_torque);
}

/**
 * @brief 设置电机控制方式
 *
 * @param __motor_dji_control_method 电机控制方式
 */
inline void dji_c620_motor_class::set_control_method(const dji_motor_control_method_e &__motor_dji_control_method)
{
    control_method = __motor_dji_control_method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __target_angle 目标的角度, rad
 */
inline void dji_c620_motor_class::set_target_angle(const float &__target_angle)
{
    target_angle = __target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __target_omega 目标的速度, rad/s
 */
inline void dji_c620_motor_class::set_target_omega(const float &__target_omega)
{
    target_omega = __target_omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __target_torque 目标的扭矩, Nm
 */
inline void dji_c620_motor_class::set_target_torque(const float &__target_torque)
{
    target_torque = __target_torque;
}

/**
 * @brief 设定前馈的角速度, rad/s
 *
 * @param __feedforward_omega 前馈的角速度, rad/s
 */
inline void dji_c620_motor_class::set_feedforward_omega(const float &__feedforward_omega)
{
    feedforward_omega = __feedforward_omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __feedforward_torque 前馈的扭矩, Nm
 */
inline void dji_c620_motor_class::set_feedforward_torque(const float &__feedforward_torque)
{
    feedforward_torque = __feedforward_torque;
}

/**
 * @brief 设置滤波后的角度
 *
 * @param __filter_angle 滤波后的角度, rad
 */
inline void dji_c620_motor_class::set_filter_angle(const float &__filter_angle)
{
    filter_angle = __filter_angle;
}

/**
 * @brief 设置滤波后的角速度
 *
 * @param __filter_omega 滤波后的角速度, rad/s
 */
inline void dji_c620_motor_class::set_filter_omega(const float &__filter_omega)
{
    filter_omega = __filter_omega;
}

/**
 * @brief 设置滤波后的扭矩
 *
 * @param __filter_torque 滤波后的扭矩, Nm
 */
inline void dji_c620_motor_class::set_filter_torque(const float &__filter_torque)
{
    filter_torque = __filter_torque;
}