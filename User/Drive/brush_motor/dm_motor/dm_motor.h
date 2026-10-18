#pragma once

#include "bsp_can.h"
#include "basic_math.h"
#include "pid.h"

/**
 * @brief 达妙电机状态
 *
 */
typedef enum
{
    Motor_DM_Status_Disable = 0,
    Motor_DM_Status_Enable,
} dm_motor_status;

/**
 * @brief 达妙电机的ID枚举类型, 一拖四模式用
 *
 */
typedef enum
{
    Motor_DM_ID_0x201 = 1,          // DM3519_ID_1
    Motor_DM_ID_0x202,              // DM3519_ID_2
    Motor_DM_ID_0x203,              // DM3519_ID_3
    Motor_DM_ID_0x204,              // DM3519_ID_4
    Motor_DM_ID_0x205,              // DM3519_ID_5
    Motor_DM_ID_0x206,              // DM3519_ID_6
    Motor_DM_ID_0x207,              // DM3519_ID_7
    Motor_DM_ID_0x208,              // DM3519_ID_8
} dm_motor_id_1_to_4;

/**
 * @brief 达妙电机控制状态, 传统模式有效
 *
 */
typedef enum
{
    Motor_DM_Control_Status_Disable = 0x0,                     // 失能
    Motor_DM_Control_Status_Enable = 0x1,                      // 使能
    Motor_DM_Control_Status_Sensor_Error = 0x5,                // 读出传感器错误
    Motor_DM_Control_Status_Parameter_Error = 0x6,             // 读取电机参数错误
    Motor_DM_Control_Status_Overvoltage = 0x8,                 // 超压
    Motor_DM_Control_Status_Undervoltage = 0x9,                // 欠压 
    Motor_DM_Control_Status_Overcurrent = 0xA,                 // 过流
    Motor_DM_Control_Status_MOS_Overtemperature = 0xB,         // MOS过温
    Motor_DM_Control_Status_Rotor_Overtemperature = 0xC,       // 线圈过温
    Motor_DM_Control_Status_Lose_Connection = 0xD,             // 通讯丢失
    Motor_DM_Control_Status_MOS_Overload = 0xE,                // MOS过载
} dm_motor_control_status;

/**
 * @brief 达妙电机控制方式
 *
 */
typedef enum
{
    Motor_DM_Control_Method_Normal_MIT = 0,         // 传统模式, MIT控制报文
    Motor_DM_Control_Method_Angle_MIT,              // 传统模式, MIT控制报文, 角度控制
    Motor_DM_Control_Method_Omega_MIT,              // 传统模式, MIT控制报文, 角速度控制

    Motor_DM_Control_Method_Normal_Angle_Omega,     // 传统模式, 位置速度控制报文, 角度+角速度控制
    Motor_DM_Control_Method_Normal_Omega,           // 传统模式, 速度控制报文, 角速度控制
    Motor_DM_Control_Method_Normal_EMIT,            // 传统模式, EMIT控制报文, 角度+角速度+电流控制

    Motor_DM_Control_Method_1_To_4_Current,         // 一拖四模式, 电流控制报文
    Motor_DM_Control_Method_1_To_4_Omega,           // 一拖四模式, 角速度控制报文
    Motor_DM_Control_Method_1_To_4_Angle,           // 一拖四模式, 角度控制报文
} dm_motor_control_method;

/**
 * @brief 达妙电机传统模式源数据
 *
 */
typedef struct __attribute__((packed))
{
    uint8_t can_id : 4;
    uint8_t control_status_enum : 4;
    uint16_t angle_reverse;
    uint8_t omega_11_4;
    uint8_t omega_3_0_torque_11_8;
    uint8_t torque_7_0;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
} dm_motor_rx_data_normal;

/**
 * @brief 达妙电机一拖四模式源数据
 *
 */
typedef struct __attribute__((packed))
{
    uint16_t encoder_reverse;       // 编码器值[0,8191]-[0-360°]
    int16_t omega_reverse;          // 角速度值[rpm]
    int16_t current_reverse;        // 电流值[-16384,16384]-[-20.5A,20.5A]
    uint8_t temperature;            // 线圈温度值[℃]
    uint8_t motor_status;           // 电机状态值[]
} dm_motor_rx_data_1_to_4;

/**
 * @brief 达妙电机常规源数据, MIT控制报文
 *
 */
typedef struct __attribute__((packed))
{
    uint16_t control_angle_reverse;
    uint8_t control_omega_11_4;
    uint8_t control_omega_3_0_k_p_11_8;
    uint8_t k_p_7_0;
    uint8_t k_d_11_4;
    uint8_t k_d_3_0_control_torque_11_8;
    uint8_t control_torque_7_0;
} dm_motor_tx_data_mit;

/**
 * @brief 达妙电机常规源数据, 位置速度控制报文
 *
 */
typedef struct __attribute__((packed))
{
    float control_angle = 0.0f;
    float control_omega = 0.0f;
} dm_motor_tx_data_position_velocity;

/**
 * @brief 达妙电机常规源数据, 速度控制报文
 *
 */
typedef struct __attribute__((packed))
{
    float control_omega = 0.0f;
} dm_motor_tx_data_velocity;

/**
 * @brief 达妙电机常规源数据, EMIT控制报文
 *
 */
typedef struct __attribute__((packed))
{
    float control_angle = 0.0f;
    // 限定速度用, rad/s的100倍
    uint16_t control_omega = 0;
    // 限定电流用, 电流最大值的10000倍
    uint16_t control_current = 0;
} dm_motor_tx_data_emit;

/**
 * @brief 达妙电机经过处理的数据, 传统模式有效
 *
 */
typedef struct
{
    dm_motor_control_status control_Status = Motor_DM_Control_Status_Disable;
    float now_angle;        // 单圈角度值, rad
    float now_omega;
    float now_torque;
    float now_mos_temperature;
    float now_rotor_temperature;

    float total_angle;      // 总角度值, rad
    uint32_t pre_encoder;
    int32_t total_round;

} dm_motor_processed_data_normal;

/**
 * @brief 达妙电机经过处理的数据, 一拖四模式有效
 *
 */
typedef struct
{
    // 输出轴状态
    dm_motor_control_status status;             // 电机状态, 0x00: 失能, 0x01: 使能
    float now_angle;            // 输出轴当前总角度, rad
    float single_angle;         // 输出轴当前单圈角度, rad
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
} dm_motor_processed_data_1_to_4;

/**
 * @brief Reusable, 达妙电机, 传统模式
 * 没有零点, 可在上位机调零点
 * 初始化的角度, 角速度, 扭矩等参数是J4310电机默认值
 *
 */
class dm_motor_normal_class
{

public:

    void init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__can_rx_id = 0x00, const uint8_t &__can_tx_id = 0x01, const dm_motor_control_method &__dm_motor_control_method = Motor_DM_Control_Method_Normal_MIT, const float &__angle_max = 12.5f, const float &__omega_max = 25.0f, const float &__torque_max = 10.0f, const float &__current_max = 10.261194f);

    // 获取电机设定参数最大值
    inline float get_angle_max(void) const;

    inline float get_omega_max() const;

    inline float get_torque_max() const;

    inline float get_current_max() const;

    // 获取电机状态
    inline dm_motor_status get_status() const;

    // 获取电机控制状态, 传统模式有效
    inline dm_motor_control_status get_control_status() const;

    // 获取电机输出轴状态和数据
    inline float get_now_angle() const;

    inline float get_total_angle() const;

    inline float get_now_omega() const;

    inline float get_now_torque() const;

    inline float get_now_mos_temperature() const;

    inline float get_now_rotor_temperature() const;

    // 获取电机控制方式和控制参数
    inline dm_motor_control_method get_control_method() const;

    inline float get_control_angle() const;

    inline float get_control_omega() const;

    inline float get_control_torque() const;

    inline float get_control_current() const;

    inline float get_k_p() const;

    inline float get_k_d() const;
    
    // 设置电机的控制方式和控制参数
    inline void set_control_angle(const float &__control_angle);

    inline void set_control_omega(const float &__control_omega);

    inline void set_control_torque(const float &__control_torque);

    inline void set_control_current(const float &__control_current);

    inline void set_k_p(const float &__k_p);

    inline void set_k_d(const float &__k_d);

    // 计算和执行电机解算和控制任务
    void can_rx_callback();

    void can_send_clear_error() const;

    void can_send_enter() const;

    void can_send_exit() const;

    void can_send_save_zero() const;

    void alive_check();

    void calculate_callback();

private:

    // 绑定的CAN
    volatile FDCAN_RxFrame_TypeDef *can_rx_frame;

    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t can_rx_id;

    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t can_tx_id;

    // 最大位置, 与上位机控制幅值PMAX保持一致
    float angle_max;
    // 最大速度, 与上位机控制幅值VMAX保持一致
    float omega_max;
    // 最大扭矩, 与上位机控制幅值TMAX保持一致
    float torque_max;
    // 最大电流, 与上位机串口中上电打印电流保持一致, EMIT模式需要
    float current_max;

    // 常量

    // 内部变量

    // 当前时刻的电机接收flag
    uint32_t flag = 0;
    // 前一时刻的电机接收flag
    uint32_t pre_flag = 0;
    // 首帧反馈标志, 复位/上电后首帧建立编码器基准, 防止误计圈
    bool first_fb_flag = true;

    // 发送缓冲区
    uint8_t tx_data[8];

    // 读变量

    // 电机状态
    dm_motor_status motor_status = Motor_DM_Status_Disable;
    // 电机对外接口信息
    dm_motor_processed_data_normal rx_data;

    // 写变量

    // 读写变量

    // 电机控制方式
    dm_motor_control_method motor_control_method = Motor_DM_Control_Method_Normal_MIT;

    // 角度, rad, 目标角度
    float control_angle = 0.0f;
    // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
    float control_omega = 0.0f;
    // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
    float control_torque = 0.0f;
    // 电流, A, EMIT模式是限幅, 其余模式无效
    float control_current = 0.0f;
    // K_P, 0~500, MIT模式有效
    float k_p = 0.0f;
    // K_D, 0~5, MIT模式有效
    float k_d = 0.0f;

    // 内部函数

    void data_process();

    void output();
};

// 一拖四模式类, 适用于达妙电机一拖四模式
class dm_motor_1_to_4_class
{
public:
    // PID角度环控制
    pid_class pid_angle;
    // PID角速度环控制
    pid_class pid_omega;

    void init(const FDCAN_HandleTypeDef *hcan, const dm_motor_id_1_to_4 &__can_rx_id, const dm_motor_control_method &__dm_motor_control_method = Motor_DM_Control_Method_1_To_4_Angle, const int32_t &__encoder_offset = 0, const float &__nearest_angle = 0.0f, const float &__gearbox_rate = 3519.0f / 187.0f);

    // 获取电机输出轴状态和数据
    inline dm_motor_status get_status() const;

    inline float get_now_angle() const;

    inline float get_single_angle() const;

    inline float get_now_omega() const;

    inline float get_now_torque() const;

    inline float get_now_rotor_temperature() const;

    inline float get_now_power() const;

    // 获取电机转子状态和数据
    inline float get_rotor_angle() const;

    inline float get_rotor_omega() const;

    inline float get_rotor_current() const;

    // 获取电机控制方式和控制参数
    inline dm_motor_control_method get_control_method() const;

    inline float get_target_angle() const;

    inline float get_target_omega() const;

    inline float get_target_torque() const;

    inline float get_feedforward_omega() const;

    inline float get_feedforward_torque() const;

    // 设置电机的控制方式和控制参数
    inline void set_control_method(const dm_motor_control_method &__dm_motor_control_method);

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
    // 收数据绑定的CAN ID, 达妙系列0x301~0x308
    uint8_t can_rx_id;
    // 发送缓存区
    uint8_t *tx_data;
    // 编码器偏移
    int32_t encoder_offset = 0;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float nearest_angle;
    // 减速比, 默认带减速箱
    float gearbox_rate;

    // 常量

    // 功率计算系数
    const float POWER_K_0 = 0.0f;
    const float POWER_K_1 = 0.0f;
    const float POWER_K_2 = 0.0f;
    const float POWER_A = 0.0f;

    // 一圈编码器刻度
    const uint16_t Encoder_Num_Per_Round = 8192;

    // 扭矩电流常数, 由于电机手册没给, 则以额定扭矩除以额定电流计算
    const float Current_To_Torque = 3.5f / 20.5f;
    // 扭矩电流到输出刻度的转化系数
    const float Current_To_Out = 16384.0f / 3.5f;
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
    dm_motor_status motor_status = Motor_DM_Status_Disable;
    // 电机对外接口信息
    dm_motor_processed_data_1_to_4 rx_data;

    // 写变量

    // 读写变量

    // 电机控制方式
    dm_motor_control_method motor_control_method = Motor_DM_Control_Method_1_To_4_Angle;
    // 目标的角度
    float target_angle = 0.0f;
    // 目标的速度, rad/s
    float target_omega = 0.0f;
    // 目标的扭矩, Nm
    float target_torque = 0.0f;
    // 前馈的速度, rad/s
    float feedforward_omega = 0.0f;
    // 前馈的扭矩, Nm
    float feedforward_torque = 0.0f;

    // 外部处理后的传感器数据, 用于外部PID闭环计算, 其数据来源于滤波或处理后的传感器数据
    
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

/*------------------------------------------------------正常控制模式----------------------------------------------------------------*/

/**
 * @brief 获取角度最大值
 *
 * @return float 角度最大值
 */
inline float dm_motor_normal_class::get_angle_max(void) const
{
    return angle_max;
}

/**
 * @brief 获取角速度最大值
 *
 * @return float 角速度最大值
 */
inline float dm_motor_normal_class::get_omega_max() const
{
    return omega_max;
}

/**
 * @brief 获取扭矩最大值
 *
 * @return float 扭矩最大值
 */
inline float dm_motor_normal_class::get_torque_max() const
{
    return torque_max;
}

/**
 * @brief 获取电流最大值
 *
 * @return float 电流最大值
 */
inline float dm_motor_normal_class::get_current_max() const
{
    return current_max;
}

/**
 * @brief 获取电机状态
 *
 * @return dm_motor_status 电机状态
 */
inline dm_motor_status dm_motor_normal_class::get_status() const
{
    return motor_status;
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度
 */
inline float dm_motor_normal_class::get_now_angle() const
{
    return rx_data.now_angle;
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float dm_motor_normal_class::get_now_omega() const
{
    return rx_data.now_omega;
}

/**
 * @brief 获取当前总角度
 *
 * @return float 当前总角度
 */
inline float dm_motor_normal_class::get_total_angle() const
{
    return rx_data.total_angle;
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
inline float dm_motor_normal_class::get_now_torque() const
{
    return rx_data.now_torque;
}

/**
 * @brief 获取当前MOS温度
 *
 * @return float 当前MOS温度
 */
inline float dm_motor_normal_class::get_now_mos_temperature() const
{
    return rx_data.now_mos_temperature;
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
inline float dm_motor_normal_class::get_now_rotor_temperature() const
{
    return rx_data.now_rotor_temperature;
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_Motor_DM_Control_Method 电机控制方式
 */
inline dm_motor_control_method dm_motor_normal_class::get_control_method() const
{
    return motor_control_method;
}

/**
 * @brief 获取角度, rad, 目标角度
 *
 * @return float 角度, rad, 目标角度
 */
inline float dm_motor_normal_class::get_control_angle() const
{
    return control_angle;
}

/**
 * @brief 获取角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @return float 角速度, rad/s
 */
inline float dm_motor_normal_class::get_control_omega() const
{
    return control_omega;
}

/**
 * @brief 获取扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @return float 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline float dm_motor_normal_class::get_control_torque() const
{
    return control_torque;
}

/**
 * @brief 获取电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @return float 电流, A
 */
inline float dm_motor_normal_class::get_control_current() const
{
    return control_current;
}

/**
 * @brief 获取K_P, 0~500, MIT模式有效
 *
 * @return float K_P, 0~500
 */
inline float dm_motor_normal_class::get_k_p() const
{
    return k_p;
}

/**
 * @brief 设定角度, rad, 目标角度
 *
 * @param __Control_Angle 角度, rad, 目标角度
 */
inline void dm_motor_normal_class::set_control_angle(const float &__control_angle)
{
    control_angle = __control_angle;
}

/**
 * @brief 设定角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 *
 * @param __Control_Omega 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
 */
inline void dm_motor_normal_class::set_control_omega(const float &__control_omega)
{
    control_omega = __control_omega;
}

/**
 * @brief 设定扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 *
 * @param __Control_Torque 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
 */
inline void dm_motor_normal_class::set_control_torque(const float &__control_torque)
{
    control_torque = __control_torque;
}

/**
 * @brief 设定电流, A, EMIT模式是限幅, 其余模式无效
 *
 * @param __Control_Current 电流, A, EMIT模式是限幅, 其余模式无效
 */
inline void dm_motor_normal_class::set_control_current(const float &__control_current)
{
    control_current = __control_current;
}

/**
 * @brief 设定K_P, 0~500, MIT模式有效
 *
 * @param __K_P K_P, 0~500, MIT模式有效
 */
inline void dm_motor_normal_class::set_k_p(const float &__k_p)
{
    k_p = __k_p;
}

/**
 * @brief 设定K_D, 0~5, MIT模式有效
 *
 * @param __K_D K_D, 0~5, MIT模式有效
 */
inline void dm_motor_normal_class::set_k_d(const float &__k_d)
{
    k_d = __k_d;
}

/*------------------------------------------------------一拖四控制模式----------------------------------------------------------------*/

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
inline dm_motor_status dm_motor_1_to_4_class::get_status() const
{
    return motor_status;
}

/**
 * @brief 获取当前角度
 *
 * @return float 当前角度, rad
 */
inline float dm_motor_1_to_4_class::get_now_angle() const
{
    return rx_data.now_angle;
}

/**
 * @brief 获取当前单圈角度
 * @retval float: 单圈角度, rad
 */
inline float dm_motor_1_to_4_class::get_single_angle() const
{
    return rx_data.single_angle;
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float dm_motor_1_to_4_class::get_now_omega() const
{
    return rx_data.now_omega;
}

/**
 * @brief 获取当前扭矩
 *
 * @return float 当前扭矩
 */
inline float dm_motor_1_to_4_class::get_now_torque() const
{
    return rx_data.now_torque;
}

/**
 * @brief 获取当前转子温度
 *
 * @return float 当前转子温度
 */
inline float dm_motor_1_to_4_class::get_now_rotor_temperature() const
{
    return rx_data.now_temperature;
}

/**
 * @brief 获取当前功率
 *
 * @return float 当前功率
 */
inline float dm_motor_1_to_4_class::get_now_power() const
{
    return rx_data.now_power;
}

/**
 * @brief 获取电机转子当前总角度
 *
 * @return float 电机转子当前总角度, rad
 */
inline float dm_motor_1_to_4_class::get_rotor_angle() const
{
    return rx_data.rotor_angle;
}

/**
 * @brief 获取电机转子当前角速度
 *
 * @return float 电机转子当前角速度, rad/s
 */
inline float dm_motor_1_to_4_class::get_rotor_omega() const
{
    return rx_data.rotor_omega;
}

/**
 * @brief 获取电机转子原始输出电流
 *
 * @return float 电机转子原始输出电流, A
 */
inline float dm_motor_1_to_4_class::get_rotor_current() const
{
    return rx_data.rotor_current;
}

/**
 * @brief 获取当前电机控制方式
 *
 * @return dm_motor_control_method 控制方法
 */
inline dm_motor_control_method dm_motor_1_to_4_class::get_control_method() const
{
    return motor_control_method;
}

/**
 * @brief 获取目标的角度
 *
 * @return float 目标的角度
 */
inline float dm_motor_1_to_4_class::get_target_angle() const
{
    return target_angle;
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
inline float dm_motor_1_to_4_class::get_target_omega() const
{
    return target_omega;
}

/**
 * @brief 获取目标的扭矩, Nm
 *
 * @return float 目标的扭矩, Nm
 */
inline float dm_motor_1_to_4_class::get_target_torque() const
{
    return target_torque;
}

/**
 * @brief 获取前馈的速度, rad/s
 *
 * @return float 前馈的速度, rad/s
 */
inline float dm_motor_1_to_4_class::get_feedforward_omega() const
{
    return feedforward_omega;
}

/**
 * @brief 获取前馈的扭矩, Nm
 *
 * @return float 前馈的扭矩, Nm
 */
inline float dm_motor_1_to_4_class::get_feedforward_torque() const
{
    return feedforward_torque;
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DM_Motor_Control_Method 电机控制方式
 */
inline void dm_motor_1_to_4_class::set_control_method(const dm_motor_control_method &__dm_motor_control_method)
{
    motor_control_method = __dm_motor_control_method;
}

/**
 * @brief 设定目标的角度
 *
 * @param __target_angle 目标的角度
 */
inline void dm_motor_1_to_4_class::set_target_angle(const float &__target_angle)
{
    target_angle = __target_angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __target_omega 目标的速度, rad/s
 */
inline void dm_motor_1_to_4_class::set_target_omega(const float &__target_omega)
{
    target_omega = __target_omega;
}

/**
 * @brief 设定目标的扭矩, Nm
 *
 * @param __target_torque 目标的扭矩, Nm
 */
inline void dm_motor_1_to_4_class::set_target_torque(const float &__target_torque)
{
    target_torque = __target_torque;
}

/**
 * @brief 设定前馈的速度, rad/s
 *
 * @param __feedforward_omega 前馈的速度, rad/s
 */
inline void dm_motor_1_to_4_class::set_feedforward_omega(const float &__feedforward_omega)
{
    feedforward_omega = __feedforward_omega;
}

/**
 * @brief 设定前馈的扭矩, Nm
 *
 * @param __feedforward_torque 前馈的扭矩, Nm
 */
inline void dm_motor_1_to_4_class::set_feedforward_torque(const float &__feedforward_torque)
{
    feedforward_torque = __feedforward_torque;
}

/**
 * @brief 设定滤波后的传感器反馈角度
 *
 * @param __filter_angle 滤波后的传感器反馈角度
 */
inline void dm_motor_1_to_4_class::set_filter_angle(const float &__filter_angle)
{
    filter_angle = __filter_angle;
}

/**
 * @brief 设定滤波后的传感器反馈速度
 *
 * @param __filter_omega 滤波后的传感器反馈速度
 */
inline void dm_motor_1_to_4_class::set_filter_omega(const float &__filter_omega)
{
    filter_omega = __filter_omega;
}

/**
 * @brief 设定滤波后的传感器反馈扭矩
 *
 * @param __filter_torque 滤波后的传感器反馈扭矩
 */
inline void dm_motor_1_to_4_class::set_filter_torque(const float &__filter_torque)
{
    filter_torque = __filter_torque;
}
