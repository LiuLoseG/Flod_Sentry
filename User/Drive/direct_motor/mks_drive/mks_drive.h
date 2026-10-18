#pragma once

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "user_lib.h"
#include "crc8.h"

// 电机角度控制单位
#ifndef MKS_ANGLE_MATCH_UNIT
#define MKS_ANGLE_MATCH_UNIT 0 // 0: 角度单位为度; 1: 角度单位为弧度; 2: 角度单位为编码器值；3：脉冲数值
#endif

// 电机反馈的位置信息模式
#ifndef MKS_ENCODER_FEEDBACK_MODE
#define MKS_ENCODER_FEEDBACK_MODE 0 // 0: 进位制多圈编码器值; 1: 累加制多圈编码器值(int48_t); 2: 原始累加多圈编码器值(int48_t)
#endif

// 电机位置角度误差
#ifndef MKS_POSITION_ANGLE_ERROR_MODE
#define MKS_POSITION_ANGLE_ERROR_MODE 0 // 0: 不启动位置角度误差功能，电机不返回位置角度误差参数，1: 启动位置角度误差功能，电机将定时返回位置角度误差参数; 
#endif

// 电机使能状态
#ifndef MKS_ENABLE_CHECK_MODE
#define MKS_ENABLE_CHECK_MODE 0 // 0: 不开启电机使能状态检测; 1: 开启电机使能状态检测
#endif

// 电机最大速度值RPM
#ifndef MKS_MAX_SPEED_RPM
#define MKS_MAX_SPEED_RPM 3000.0f // 电机最大速度值，单位rpm
#endif

// 电机编码器值
#define MKS_ENCODER_RESOLUTION 0x4000 // 电机编码器分辨率，单位为每圈编码器计数值

// CRC_CHECK_MODE
// #define MKS_CRC_CHECK_CODE 0xFF // CRC校验码，具体数值根据电机数据手册中CRC校验方式设置

#define MKS_MAX_PULSE_COUNT 0xFFFFFF // 电机位置控制模式下，位置脉冲计数器的最大值，根据电机数据手册设置

#define MKS_PULSE_CYCLE 0xC8 // 位置控制模式下，电机位置脉冲计数器的周期，根据电机数据手册设置

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    // 脉冲控制
    CR_OPEN = 0,        // 脉冲接口开环模式，不需要编码器就能运行，工作电流固定
    CR_CLOSE = 1,       // 脉冲接口闭环模式，编码器闭环可防止丢步，工作电流固定
    CR_vFOC = 2,        // 脉冲接口 FOC 模式，编码器闭环， 工作电流随负载自动调节

    // 串行控制
    SR_OPEN = 3,        // 串行接口开环模式，不需要编码器就能运行，工作电流固定
    SR_CLOSE = 4,       // 串行接口闭环模式，编码器闭环可防止丢步，工作电流固定
    SR_vFOC = 5,        // 串行接口 FOC 模式，编码器闭环， 工作电流随负载自动调节
} mks_control_mode_e;

typedef enum
{
    SPEED_CTRL_MODE = 0,    // 速度控制模式
    POSITION_CTRL_MODE = 1, // 位置控制模式
    DATA_READ_MODE = 2,     // 只读数据模式
} mks_execute_mode_e;

typedef enum
{
    CAN_COMMUNIACTION_MODE = 0,   // CAN通信模式，使用CAN总线发送控制命令
    UART_COMMUNIACTION_MODE = 1,  // UART通信模式，使用UART串口发送控制命令
} mks_communication_mode_e;

typedef enum
{

/*-------------------------只读参数读取指令(5.1)-------------------------------*/
    READ_CARRY_ENCODER_COMMAND = 0x30,          // 读取进位制多圈编码器值 (5.1.1)
    READ_ACC_ENCODER_COMMAND = 0x31,            // 读取累加制多圈编码器值 (5.1.2)
    READ_SPEED_COMMAND = 0x32,              // 读取电机实时转速指令 (5.1.3)
    READ_PULSE_COUNT_COMMAND = 0x33,        // 读取接收脉冲数指令 (5.1.4)
    READ_ORIGINAL_ACC_ENCODER_COMMAND = 0x35,   // 读取原始累加多圈编码器值(int48_t),正转一圈，多圈编码器值 +0x4000; 逆转一圈，多圈编码器值 -0x4000; (5.1.5)
    READ_POSITION_ANGLE_ERROR_COMMAND = 0x39,   // 读取位置角度误差, 指令控制的位置角度减去电机的实时角度位置得到的差值 (5.1.6)
    ENABLE_BUS_COMMAND = 0x3A,   // 驱动板的使能状态，配合总线控制使用 (5.1.7)
    ZERO_RESET_COMMAND = 0x3B,   // 读取电机回零状态 (5.1.8)
    AUTO_RETURN_PARAM_COMMAND = 0x01,   // 电机自动定时返回某个只读参数，上位机不需要频繁发出对应的查询指令 (5.1.9)
    SINGLE_CONFIGURATION_COMMAND = 0x00,   // 读取单个配置参数, 只写配置参数写入后，使用该指令，可以读出查看 (5.1.10)
    HARDWARE_FIRMWARE_COMMAND = 0x40,   // 读取硬件版本和固件版本指令 (5.1.11)
    SET_CAN_ID_COMMAND = 0x42,    // 设置CAN ID指令 (5.1.12)

/*--------------------只写参数设置指令(对应数据手册5.2)--------------------------*/
    CALIBRATION_COMMAND = 0x80,  // 电机校准指令 (5.2.1)
    SET_WORK_MODE_COMMAND = 0x82,   // 设置电机工作模式指令 (5.2.2)
    SET_WORK_CURRENT_COMMAND = 0x83,    // 设置电机工作电流指令 (5.2.3)
    SET_CURRENT_PERCENT_COMMAND = 0x9B,     // 设置电机工作电流百分比指令 (5.2.4)
    SET_MICSTEP_COMMAND = 0x84,    // 设置电机细分数指令 (5.2.5)
    SET_PIN_EFFECTIVE_LEVEL_COMMAND = 0x85,    // 设置电机控制引脚有效电平指令 (5.2.6)
    SET_MOTOR_ROTATION_DIRECTION_COMMAND = 0x86,    // 设置电机旋转方向指令 (5.2.7)
    SET_AUTO_BLACK_SCREEN_COMMAND = 0x87,    // 设置自动黑屏时间指令 (5.2.8)
    SET_INTERPOLATION_COMMAND = 0x89,    // 设置细分插补功能指令 (5.2.9)

    SET_CAN_BAUD_RATE_COMMAND = 0x8A,    // 设置CAN波特率指令 (5.2.10)
    SET_CAN_ADDRESS_COMMAND = 0x8B,    // 设置CAN从机地址指令 (5.2.11)
    SET_CAN_ASK_RESPONSE_COMMAND = 0x8C,    // 设置CAN应答方式指令 (5.2.12)
    SET_CAN_CHECK_SUM_COMMAND = 0x4C,    // 设置CAN校验方式指令 (5.2.13)
    SET_BUTTON_LOCK_COMMAND = 0x8F,    // 设置按键锁定指令 (5.2.14)
    SET_GROUP_ADDRESS_COMMAND = 0x8D,    // 设置分组地址指令 (5.2.15)
    SET_HEARTBEAT_PROTECT_COMMAND = 0x98,    // 设置心跳保护指令 (5.2.16)
    SET_LOCATION_ARRIVAL_THRESHOLD_COMMAND = 0x95,    // 设置位置到达阈值指令 (5.2.17)

/*--------------------PID参数设置指令(对应数据手册5.3)--------------------------*/
    SET_VFOC_PID_COMMAND = 0x96,    // 设置电机PID参数指令 (5.3.1)
    SET_CLOSE_PID_COMMAND = 0x97,    // 设置闭环PID参数指令 (5.3.2)

/*--------------------堵转保护和超载保护指令(对应数据手册5.4)--------------------------*/
    SET_CURRENT_OVERLOAD_PROTECT_COMMAND = 0x88,    // 设置电流过载保护指令 (5.4.1)
    SET_POSITION_OVERLOAD_PROTECT_COMMAND = 0x9D,    //  设置位置超差保护参数 (5.4.2)
    READ_LOCK_ROTOR_STATUS_COMMAND = 0x3E,      // 读取电机堵转状态 (5.4.3)
    RELEASE_LOCK_ROTOR_COMMAND = 0x3D,          // 解除电机堵转状态 (5.4.4)

/*--------------------恢复出厂设置指令(对应数据手册5.5)--------------------------*/
    RECOVERY_PARAMENTER_COMMAND = 0x3F,    // 恢复出厂设置指令 (5.5.1)
    RESET_RESTART_MOTOR_COMMAND = 0x41,    // 复位重启电机指令 (5.5.2)

/*--------------------ID端口控制指令(对应数据手册6)--------------------------*/
    READ_GPIO_STATUS_COMMAND = 0x34,    // 读取GPIO状态指令 (6.1)
    READ_PORT_INPUT_OUTPUT__MODE_COMMAND = 0x9F,    // 读取端口输入输出状态指令 (6.2)
    SET_PORT_WEITE_DATA_COMMAND = 0x36,    // 设置端口输出数据指令 (6.3)
    SET_PORT_PULSE_FREQUENCY_DIVISION_COMMAND = 0x99,    // 设置端口脉冲频率分频指令 (6.4)

/*--------------------设置电机回零相关参数指令(对应数据手册7.3)--------------------------*/
    SET_PORT_MAP_COMMAND = 0x9E,    // 设置端口映射指令 (7.3.1)
    SET_PARAMETER_DIRECT_SPEED_COMMAND = 0x90,    // 设置回零方向、速度等参数 (7.3.2)
    SET_ZERO_TORQUE_ORIGIN_COMMAND = 0x94,    // 设置零转矩、原点偏移量指令 (7.3.3)
    SET_SINGLE_TRUN_HOME_COMMAND = 0x9A,    // 设置单圈回零指令 (7.3.4)
    SET_ZERO_POINT_ANGLE_COMMAND = 0x92,    // 设置零点角度指令 (7.3.5)
    EXECUTE_ZERO_RETURN_COMMAND = 0x91,    // 执行回零指令 (7.3.6)

/*--------------------总线控制通用指令(对应数据手册9.2)--------------------------*/
    READ_OPERATE_COMMAND = 0xF1,   // 读取电机运行状态 (9.2.1)
    SET_ENABLE_COMMAND = 0xF3,    // 设置电机使能状态 (9.2.2)
    SET_STOP_COMMAND = 0xF4,      // 设置电机停止状态 (9.2.3)

/*--------------------速度模式指令(对应数据手册10.1)--------------------------*/
    SPEED_COMMAND = 0xF6,     // 速度模式控制

/*--------------------上电自运行指令(对应数据手册10.3)--------------------------*/
    SET_AUTO_RUN_COMMAND = 0xFF,  // 上电自运行控制 

/*--------------------位置模式指令(对应数据手册11)--------------------------*/
    POSITION_PULSE_COMMAND = 0xFD,  // 位置模式控制，脉冲相对运行指令(11.1.1)
    POSITION_ABS_COMMAND = 0xFE,    // 位置模式控制，绝对位置运行指令(11.2.1)
    POSITION_REF_AXIS_COMMAND = 0xF4,   // 位置模式控制，坐标相对运行指令(11.3.1)
    POSITION_ABS_AXIS_COMMAND = 0xF5,    // 位置模式控制，坐标绝对位置运行指令(11.4.1)

/*--------------------多电机同步控制指令(对应数据手册12.3)--------------------------*/
    MULTI_SYNC_COMMAND = 0x4A,              // 多电机同步控制功能指令 (12.3.1)
    MULTI_SYNC_EXECUTE_COMMAND = 0x4B,      // 多电机同步控制执行指令, 接收到此命令后电机将执行之前发送的多电机同步控制指令 (12.3.2)

} mks_control_code_e;   // 控制功能码

typedef struct
{
    int16_t rotate_speed;         // 电机转速值(rpm,每分钟转多少圈)
    // 电机位置反馈
#if MKS_ENCODER_FEEDBACK_MODE == 0      // 进位制编码器反馈模式
    int32_t rotate_count;               // 电机转动圈数
    uint16_t encoder;             // 当前编码器值
#elif MKS_ENCODER_FEEDBACK_MODE == 1    // 累加制编码器值反馈模式
    int64_t encoder_value;       
#endif

    // 电机位置角度误差反馈
#if MKS_POSITION_ANGLE_ERROR_MODE == 1
    float position_angle_error;   // 位置角度误差值 (度)
#endif

    // 电机使能状态反馈
#if MKS_ENABLE_CHECK_MODE == 1
    bool enable_status;          
#endif

} mks_original_encoder_t;   // 电机原始编码器数据结构体

typedef struct
{
    mks_original_encoder_t original_encoder;   // 电机原始编码器数据

#if MKS_ANGLE_MATCH_UNIT == 0
    float angle_speed;          // 电机角速度值 (度/s)
    float single_round_angle;   // 单圈角度值 (度)
    float multi_round_angle;    // 多圈角度值 (度)
    
#else 
    float angular_speed;        // 电机角速度值 (rad/s)
    float single_round_rad;     // 单圈角度值 (弧度)
    float multi_round_rad;      // 多圈角度值 (弧度)
#endif

} mks_process_feedback_t;   // 电机反馈数据结构体

typedef struct
{
    mks_communication_mode_e communication_mode;   // 电机通信模式, 串口/CAN
    mks_control_mode_e control_mode;   // 电机控制模式，开环/闭环/FOC
    mks_execute_mode_e execute_mode;   // 电机函数执行模式， 速度模式/位置模式

    float target_angle;    // 目标位置角度值，单位根据MKS_ANGLE_MATCH_UNIT宏定义设置

    float speed_conversion_unit;    // 速度控制单位转换，0: rpm; 1: deg/s RPM_TO_ANGLE_SEC; 2: rad/s RPM_TO_RAD_SEC
    float target_speed;    // 目标速度值，单位rpm
    uint8_t target_acceleration;  // 目标加速度值，单位rpm/s
} mks_motor_control_t;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;   // CAN句柄指针，电机通信使用CAN总线时需要提供
    uint8_t motor_id;   // 电机ID
    uint16_t motor_niche; // 电机细分数
    mks_control_code_e control_code;   // 电机控制功能码，具体功能码定义参考mks_control_code_e枚举类型
    mks_process_feedback_t process_feedback;   // 电机反馈数据
    mks_motor_control_t motor_control;   // 电机控制参数
} mks_motor_drive_t;

void mks_motor_init(FDCAN_HandleTypeDef* hfdcan, mks_motor_drive_t *p_motor, uint8_t id, mks_communication_mode_e communication_mode, mks_control_mode_e control_mode, mks_execute_mode_e execute_mode, uint16_t motor_niche);
uint8_t mks_motor_set_work_mode(FDCAN_HandleTypeDef* hfdcan, mks_motor_drive_t *p_motor, mks_control_mode_e control_mode);
uint8_t mks_motor_speed_ctrl(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, mks_communication_mode_e comm_mode, float speed_unit_vel, float speed_vel, uint8_t acc_vel);
uint8_t mks_motor_position_ctrl(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, mks_control_code_e control_code, mks_communication_mode_e comm_mode, float speed_unit_vel, uint16_t motor_niche, float angle_vel, float speed_vel, uint8_t acc_vel);
void mks_motor_read_parameter(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, mks_communication_mode_e comm_mode, mks_control_code_e code_command);
void mks_motor_original_feedback(mks_motor_drive_t *p_motor, uint8_t *rx_data);
void mks_motor_ctrl_send(FDCAN_HandleTypeDef *hfdcan, mks_motor_drive_t *p_motor);
void mks_motor_fddata(mks_motor_drive_t *p_motor);