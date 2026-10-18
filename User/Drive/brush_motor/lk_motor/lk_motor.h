#pragma once

#include "bsp_can.h"
#include "dji_motor_new.h"
#include "pid.h"
#include "basic_math.h"
#include <cstdint>

#define LK_Normal_Can_Offset 0x140      // LK电机正常模式CAN ID偏移量, 0x140 + can_rx_id

/** @brief LK电机连接状态 */
typedef enum
{
    Motor_LK_Status_Disable = 0,
    Motor_LK_Status_Enable = 1,
} lk_motor_status;

/** @brief LK电机错误标志位 (0x9A/0x9B 响应 DATA[7]) */
typedef enum
{
    Motor_LK_Control_Status_Normal = 0x00,                  // 0000 0000 正常
    Motor_LK_Control_Status_UnderVoltage = 0x01,            // 0000 0001 欠压保护
    Motor_LK_Control_Status_OverVoltage = 0x02,             // 0000 0010 过压保护
    Motor_LK_Control_Status_Driver_OverTemperature = 0x04,  // 0000 0100 驱动过温
    Motor_LK_Control_Status_Motor_OverTemperature = 0x08,   // 0000 1000 电机过温
    Motor_LK_Control_Status_OverCurrent = 0x10,             // 0001 0000 过流保护
    Motor_LK_Control_Status_ShortCircuit = 0x20,            // 0010 0000 电机短路
    Motor_LK_Control_Status_Block = 0x40,                   // 0100 0000 电机堵转
    Motor_LK_Control_Status_Signal_Lost = 0x80,             // 1000 0000 通信丢失
} lk_motor_control_status;

/** @brief LK电机控制方式 */
typedef enum
{
    // 详细控制模式 (对齐协议)
    Motor_LK_Control_Method_Normal_Open_Loop = 0,           // 开环控制模式, (该命令仅在MS电机上实现),目前未添加其实现
    Motor_LK_Control_Method_Normal_Torque,                  // 扭矩控制
    Motor_LK_Control_Method_Normal_Omega,                   // 角速度控制

    Motor_LK_Control_Method_Normal_Multi_Angle,             // 多圈角度控制
    Motor_LK_Control_Method_Normal_Multi_Angle_Omega,       // 多圈角度+角速度控制

    Motor_LK_Control_Method_Normal_Single_Angle,            // 单圈角度控制
    Motor_LK_Control_Method_Normal_Single_Angle_Omega,      // 单圈角度+角速度控制

    Motor_LK_Control_Method_Normal_Incremental_Angle,       // 增量角度控制
    Motor_LK_Control_Method_Normal_Incremental_Angle_Omega, // 增量角度+角速度控制

    Motor_LK_Control_Method_1_To_4,                         // 1拖4模式 (连续反馈, MCU侧PID)
} lk_motor_control_method;

/** @brief LK电机协议命令码*/
typedef enum
{
    Motor_LK_Edict_Motor_Close                    = 0x80, // 电机关闭，清除电机转动圈数及之前接收的控制指令，LED 由常亮转为慢闪。此时电机仍然可以回复控制命令，但不会执行动作。
    Motor_LK_Edict_Motor_Open                     = 0x88, // 电机运行，电机从关闭状态切换到开启状态，LED 由慢闪转为常亮。此时电机可以接收控制命令并执行动作。
    Motor_LK_Edict_Motor_Disable                  = 0x81, // 电机停止，但不清除电机运行状态。再次发送控制指令即可控制电机动作。
    Motor_LK_Edict_Brake_Control                  = 0x8C, // 抱闸器控制，或读取当前抱闸器的状态
    Motor_LK_Edict_Read_Status_Error              = 0x9A, // 读取状态1，电机的温度、电压和错误状态标志
    Motor_LK_Edict_Clear_Error_Flag               = 0x9B, // 清除错误状态
    Motor_LK_Edict_Read_Parameter                 = 0x9C, // 读取状态2，读取电机参数
    Motor_LK_Edict_Read_Phase_Current             = 0x9D, // 读取状态3，读取电机相电流

    Motor_LK_Edict_Normal_Open_Loop               = 0xA0, // 开环控制 (仅MS)
    Motor_LK_Edict_Normal_Torque                  = 0xA1, // 转矩闭环 (MF/MH/MG)
    Motor_LK_Edict_Normal_Omega                   = 0xA2, // 角速度闭环
    Motor_LK_Edict_Normal_Multi_Angle             = 0xA3, // 多圈位置闭环1，多圈角度闭环控制命令
    Motor_LK_Edict_Normal_Multi_Angle_Omega       = 0xA4, // 多圈位置闭环2，多圈角度+角速度闭环控制命令
    Motor_LK_Edict_Normal_Single_Angle            = 0xA5, // 单圈位置闭环1，单圈角度闭环控制命令
    Motor_LK_Edict_Normal_Single_Angle_Omega      = 0xA6, // 单圈位置闭环2，单圈角度+角速度闭环控制命令
    Motor_LK_Edict_Normal_Incremental_Angle       = 0xA7, // 增量位置闭环1，增量角度闭环控制命令
    Motor_LK_Edict_Normal_Incremental_Angle_Omega = 0xA8, // 增量位置闭环2，增量角度+角速度闭环控制命令

    Motor_LK_Edict_Read_Control_Parameter         = 0xC0, // 读取控制参数
    Motor_LK_Edict_Write_Control_Parameter        = 0xC1, // 写入控制参数
    Motor_LK_Edict_Read_Encoder_Parameter         = 0x90, // 读取编码器
    Motor_LK_Edict_Calibration_Encoder_Parameter  = 0x18, // 校准编码器
    Motor_LK_Edict_Set_Motor_Point_ROM            = 0x19, // 设零点(ROM)
    Motor_LK_Edict_Read_Multi_Angle_Parameter     = 0x92, // 读取多圈角度
    Motor_LK_Edict_Read_Single_Angle_Parameter    = 0x94, // 读取单圈角度
    Motor_LK_Edict_Set_Motor_Point_RAM            = 0x95, // 设零点(RAM)
    Motor_LK_Edict_Read_Motor_Set_Parameter       = 0x40, // 读取设定参数
    Motor_LK_Edict_Write_Motor_Set_Parameter      = 0x42, // 写入设定参数
    Motor_LK_Edict_Save_Motor_Set_Parameter       = 0x44, // 保存设定参数(ROM)，保存后需要重新上电或发送重启命令
    Motor_LK_Edict_Motor_Restart                  = 0x07, // 重启电机
} lk_motor_sign_edict;

/**
 * @brief LK电机源数据 (CAN报文原始数据, 小端序)
 * @note 数据格式需根据具体LK电机型号的通信协议进行调整
 *       读取电机状态 2 命令，读取当前电机的温度、电机转矩电流（MF、MG） /电机输出功率（MS）、转速、编码器位置。
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte;               // 命令字节, 0x9C
    uint8_t temperature;                // 电机温度值 [1℃/LSB]
    int16_t torque_current;             // 电机转矩电流值 [66/4096 A]
    int16_t omega;                      // 电机角速度值 [1dps/LSB]
    uint16_t encoder;                   // 电机编码器值 [0-65535]
} lk_motor_source_data;

/**
 * @brief LK电机状态错误响应帧 (0x9A、0x9B)
 * @note 协议 V2.362: DATA[0]=命令字节, DATA[1]=温度, DATA[2:3]=电压, DATA[4:5]=电流, DATA[6]=电机状态, DATA[7]=错误标志
 *       读取电机状态 1 和错误标志命令
 *       清除电机错误标志命令
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte;               // DATA[0]: 命令字节 0x9A
    int8_t temperature;                 // DATA[1]: 电机温度, int8_t, 单位 1℃/LSB
    int16_t voltage_reverse;            // DATA[2:3]: 母线电压, int16_t, 单位 0.01V/LSB (小端序)
    int16_t current_reverse;            // DATA[4:5]: 母线电流, int16_t, 单位 0.01A/LSB (小端序)
    uint8_t motor_state;                // DATA[6]: 电机状态 (0x00=开启, 0x10=关闭)
    uint8_t error_state;                // DATA[7]: 错误状态标志位
} lk_motor_status_error_data_t;

/**
 * @brief LK电机状态3 响应帧 (0x9D 读取三相电流)
 * @note 协议 V2.362: MS电机无相电流采样，该命令在MS电机上无效
 *       读取电机状态 3 命令
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte;               // DATA[0]: 命令字节 0x9D
    int8_t temperature;                 // DATA[1]: 电机温度, 1℃/LSB
    int16_t iA_reverse;                 // DATA[2:3]: A相电流, int16_t (小端序), MG:(66/4096 A) / LSB   MF:(33/4096 A) / LSB
    int16_t iB_reverse;                 // DATA[4:5]: B相电流, int16_t (小端序), MG:(66/4096 A) / LSB   MF:(33/4096 A) / LSB
    int16_t iC_reverse;                 // DATA[6:7]: C相电流, int16_t (小端序), MG:(66/4096 A) / LSB   MF:(33/4096 A) / LSB
} lk_motor_phase_current_data_t;

/**
 * @brief 领控电机常规源数据, 转矩闭环控制报文(MF、 MH、 MG)
 * @note 转矩范围: -2048~ 2048, MG：-33A~33A  MF: -16.5A~16.5A
 *       转矩值 = torque * 66/4096 A (MG)
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Torque;    // 转矩闭环控制命令字节, 0xA0
    uint16_t reserved1;                     // 保留2字节
    uint8_t reserved2;                      // 保留1字节
    int16_t torque;                         // 电机转矩值 [66/4096 A]
    uint16_t reserved3;                     // 保留2字节
} lk_motor_tx_data_torque;

/**
 * @brief 领控电机常规源数据, 角速度闭环控制报文(MF、 MH、 MG、 MS)
 * @note 转矩范围: -2048~ 2048, MG：-33A~33A  MF: -16.5A~16.5A
 *       转矩值 = torque * 66/4096 A (MG)
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Omega;    // 角速度闭环控制命令字节, 0xA2
    uint8_t reserved;                       // 保留1字节
    int16_t limit_torque;                   // 限制力矩值
    int32_t omega;                          // 电机角速度值 [0.01°/LSB]
} lk_motor_tx_data_omega;

/**
 * @brief 领控电机常规源数据, 多圈角度闭环控制报文(MF、 MH、 MG、 MS)
 * @note 角度分辨率: 0.01°/LSB
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Multi_Angle;    // 多圈角度闭环控制命令字节, 0xA3
    uint16_t reserved1;                     // 保留2字节
    uint8_t reserved2;                      // 保留1字节
    int32_t angle;                          // 电机多圈角度值 [0.01°/LSB], 36000-360°
} lk_motor_tx_data_multi_angle;

/**
 * @brief 领控电机常规源数据, 多圈角度+角速度闭环控制报文(MF、 MH、 MG、 MS)
 * @note 角度分辨率: 0.01°/LSB
 *       限制角速度分辨率: 1°/LSB
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Multi_Angle_Omega;    // 多圈角度+角速度闭环控制命令字节, 0xA4
    uint8_t reserved;                       // 保留1字节
    uint16_t limit_omega;                   // 限制角速度值 [1.0°/LSB]
    int32_t angle;                          // 电机多圈角度值 [0.01°/LSB], 36000-360°
} lk_motor_tx_data_multi_angle_omega;

/**
 * @brief 领控电机常规源数据, 单圈角度闭环控制报文(MF、 MH、 MG、 MS)
 * @note 角度分辨率: 0.01°/LSB
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Single_Angle;    // 单圈角度闭环控制命令字节, 0xA5
    uint8_t spin_direction;                 // 旋转方向
    uint16_t reserved;                      // 保留2字节
    uint32_t angle;                          // 电机单圈角度值 [0.01°/LSB], 36000-360°
} lk_motor_tx_data_single_angle;

/**
 * @brief 领控电机常规源数据, 单圈角度+角速度闭环控制报文(MF、 MH、 MG、 MS)
 * @note 角度分辨率: 0.01°/LSB
 *       限制角速度分辨率: 1°/LSB   
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Single_Angle_Omega;    // 单圈角度+角速度闭环控制命令字节, 0xA6
    uint8_t spin_direction;                 // 旋转方向
    uint16_t limit_omega;                   // 限制角速度值 [1.0°/LSB]
    uint32_t angle;                          // 电机单圈角度值 [0.01°/LSB], 36000-360°
} lk_motor_tx_data_single_angle_omega;

/**
 * @brief 领控电机常规源数据, 增量角度闭环控制报文(MF、 MH、 MG、 MS)
 * @note 角度分辨率: 0.01°/LSB
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Incremental_Angle;    // 增量角度闭环控制命令字节, 0xA6
    uint16_t reserved1;                     // 保留2字节
    uint8_t reserved2;                      // 保留1字节
    int32_t angle;                          // 电机单圈角度值 [0.01°/LSB], 36000-360°
} lk_motor_tx_data_incremental_angle;

/**
 * @brief 领控电机常规源数据, 增量角度+角速度闭环控制报文(MF、 MH、 MG、 MS)
 * @note 角度分辨率: 0.01°/LSB
 *       限制角速度分辨率: 1°/LSB   
 */
typedef struct __attribute__((packed))
{
    uint8_t command_byte = Motor_LK_Edict_Normal_Incremental_Angle_Omega;    // 增量角度+角速度闭环控制命令字节, 0xA6
    uint16_t reserved;                      // 保留2字节
    uint8_t limit_omega;                    // 限制角速度值 [1.0°/LSB]
    int32_t angle;                          // 电机单圈角度值 [0.01°/LSB], 36000-360°
} lk_motor_tx_data_incremental_angle_omega;

/**
 * @brief 领控电机经过处理的数据, 传统模式有效
 *
 */
typedef struct
{
    // 输出轴状态
    float now_angle;            // 输出轴角度, rad
    float single_angle;          // 单圈角度, rad

    float now_omega;            // 输出轴角速度, rad/s
    float now_torque;           // 输出轴扭矩, Nm
    float now_temperature;      // 温度, ℃

    uint32_t pre_encoder;       // 上一时刻编码器
    int32_t total_encoder;      // 累计编码器
    int32_t total_round;        // 累计圈数
} lk_motor_processed_data_normal;

/**
 * @brief LK无刷电机驱动类 (1拖4模式)
 * @note 对齐 DJI/DM 1拖4类设计
 *       支持扭矩/角速度/角度/MIT四种控制模式
 *       内部集成双环PID (角度环→角速度环→扭矩输出)
 *       支持前馈控制和传感器滤波数据外部闭环
 */
class lk_motor_normal_class
{
public:
    void init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__can_tx_rx_id, const lk_motor_control_method &__motor_lk_control_method = Motor_LK_Control_Method_Normal_Torque, const int32_t &__encoder_offset = 0, const float &__nearest_angle = 0.0f, const float &__gearbox_rate = 6.0f);

    // 获取电机状态和数据
    inline lk_motor_status get_status() const;

    inline float get_now_angle() const;

    inline float get_single_angle() const;

    inline float get_now_omega() const;

    inline float get_now_torque() const;

    inline float get_now_temperature() const;

    // 获取电机控制方式和控制参数
    inline lk_motor_control_method get_control_method() const;

    inline float get_control_angle() const;

    inline float get_control_omega() const;

    inline float get_control_torque() const;

    // 设置电机的控制方式和控制参数
    inline void set_control_method(const lk_motor_control_method &__motor_lk_control_method);

    inline void set_control_angle(const float &__control_angle);

    inline void set_control_omega(const float &__control_omega);

    inline void set_control_torque(const float &__control_torque);

    // 命令模式: 电机启停与状态管理
    void can_send_enter() const;

    void can_send_exit() const;

    void can_send_clear_error() const;

    void can_send_set_zero_ram() const;

    void can_send_get_status() const;

    // 计算和执行电机解算和控制任务
    void can_rx_callback();

    void alive_check();

    void calculate_callback();         // 使用电机反馈的原始数据，进行PID闭环控制

private:
    // 初始化变量

    // 绑定的CAN
    volatile FDCAN_RxFrame_TypeDef *can_rx_frame;

    // 绑定的CAN ID,领控电机的发送ID与接收ID相同。
    uint16_t can_tx_rx_id;

    // 编码器偏移
    int32_t encoder_offset = 0;
    // 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
    float nearest_angle = 0.0f;
    // 减速比
    float gearbox_rate = 6.0f;      // 电机反馈的数据是减速后输出轴的数据，因此不用除减速比，直接使用即可。

    // 常量

    // 一圈编码器刻度 (16位编码器为0-65535)
    const uint32_t Encoder_Num_Per_Round = 65536;
    // 电流扭矩转换系数 (Nm/A)
    const float Current_To_Torque = 0.16f;
    // 电流值到输出刻度的转化系数 (A → 输出刻度)
    const float Current_To_Out = 33.0f / 2048.0f;
    // 最大输出刻度
    const float Torque_Out_Max = 2048.0f;

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

    // 电机连接状态
    lk_motor_status motor_status = Motor_LK_Status_Disable;
    // 电机对外接口信息
    lk_motor_processed_data_normal rx_data;
    // 电机当前状态
    lk_motor_control_status control_status = Motor_LK_Control_Status_Normal;

    // 读写变量

    // 电机控制方式
    lk_motor_control_method control_method = Motor_LK_Control_Method_Normal_Torque;

    // 目标的角度rad, 0.01degree/LSB, 角度模式下有效
    float control_angle = 0.0f;
    // 目标的速度rad/s, 0.01dps/LSB, 速度模式下有效
    float control_omega = 0.0f;
    // 目标的扭矩Nm, -2048~2048, 转矩模式下有效
    float control_torque = 0.0f;
    // 目标的扭矩限制Nm, -2048~2048, 转矩模式下有效
    float torque_limit = 0.0f;
    // 目标的速度限制rad/s, 角速度模式下有效
    float omega_limit = 0.0f;

    // 内部函数

    void data_process();

    void error_process();

    void output();
};

/*------------------------------------------------------内联函数实现----------------------------------------------------------------*/

/**
 * @brief 获取电机连接状态
 *
 * @return lk_motor_status 电机连接状态
 */
inline lk_motor_status lk_motor_normal_class::get_status() const
{
    return motor_status;
}

/**
 * @brief 获取输出轴当前角度
 *
 * @return float 当前角度, rad
 */
inline float lk_motor_normal_class::get_now_angle() const
{
    return rx_data.now_angle;
}

/**
 * @brief 获取输出轴当前单圈角度
 * @retval float: 单圈角度, rad
 */
inline float lk_motor_normal_class::get_single_angle() const
{
    return rx_data.single_angle;
}

/**
 * @brief 获取输出轴当前角速度
 *
 * @return float 当前角速度, rad/s
 */
inline float lk_motor_normal_class::get_now_omega() const
{
    return rx_data.now_omega;
}

/**
 * @brief 获取输出轴当前扭矩
 *
 * @return float 当前扭矩, Nm
 */
inline float lk_motor_normal_class::get_now_torque() const
{
    return rx_data.now_torque;
}

/**
 * @brief 获取电机当前温度
 *
 * @return float 当前温度, ℃
 */
inline float lk_motor_normal_class::get_now_temperature() const
{
    return rx_data.now_temperature;
}

/**
 * @brief 获取当前电机控制方式
 *
 * @return lk_motor_control_method 控制方法
 */
inline lk_motor_control_method lk_motor_normal_class::get_control_method() const
{
    return control_method;
}

/**
 * @brief 获取目标角度
 *
 * @return float 目标角度, rad
 */
inline float lk_motor_normal_class::get_control_angle() const
{
    return control_angle;
}

/**
 * @brief 获取目标角速度
 *
 * @return float 目标角速度, rad/s
 */
inline float lk_motor_normal_class::get_control_omega() const
{
    return control_omega;
}

/**
 * @brief 获取目标扭矩
 *
 * @return float 目标扭矩, Nm
 */
inline float lk_motor_normal_class::get_control_torque() const
{
    return control_torque;
}

/**
 * @brief 设定电机控制方式
 *
 * @param __motor_lk_control_method 电机控制方式
 */
inline void lk_motor_normal_class::set_control_method(const lk_motor_control_method &__motor_lk_control_method)
{
    control_method = __motor_lk_control_method;
}

/**
 * @brief 设定目标角度
 *
 * @param __target_angle 目标角度, rad
 */
inline void lk_motor_normal_class::set_control_angle(const float &__control_angle)
{
    control_angle = __control_angle;
}

/**
 * @brief 设定目标角速度
 *
 * @param __target_omega 目标角速度, rad/s
 */
inline void lk_motor_normal_class::set_control_omega(const float &__control_omega)
{
    control_omega = __control_omega;
}

/**
 * @brief 设定目标扭矩
 *
 * @param __target_torque 目标扭矩, Nm
 */
inline void lk_motor_normal_class::set_control_torque(const float &__control_torque)
{
    control_torque = __control_torque;
}
