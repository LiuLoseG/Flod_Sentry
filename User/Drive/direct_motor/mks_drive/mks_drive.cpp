#include "mks_drive.h"
#include <cstdint>

#define MKS_MOTOR_NUM 10

mks_motor_drive_t *mks_drive[MKS_MOTOR_NUM];

//局部静态变量定义当前电机注册实例序号
static uint16_t MKS_MOTORS_USED_NUM = 0;

uint8_t mks_motor_set_work_mode(FDCAN_HandleTypeDef* hfdcan, mks_motor_drive_t *p_motor, mks_control_mode_e control_mode);

/**
 * @brief 初始化电机
 * @param hfdcan CAN句柄
 * @param p_motor 电机初始化结构体指针
 * @param id 电机ID
 * @param communication_mode 通信模式
 * @param control_mode 控制模式
 * @param execute_mode 执行模式
 * @param motor_niche 电机细分数
 * @retval 无
 * @details 初始化电机参数并将其添加到电机数组中
 */
void mks_motor_init(FDCAN_HandleTypeDef* hfdcan, mks_motor_drive_t *p_motor, uint8_t id, mks_communication_mode_e communication_mode, mks_control_mode_e control_mode, mks_execute_mode_e execute_mode, uint16_t motor_niche)
{
    p_motor->hfdcan = hfdcan;
    p_motor->motor_id = id;
    p_motor->motor_niche = motor_niche;
    p_motor->motor_control.communication_mode = communication_mode;
    p_motor->motor_control.control_mode = control_mode;
    p_motor->motor_control.execute_mode = execute_mode; // 设置电机执行模式

    p_motor->motor_control.speed_conversion_unit = 1.0f; // 默认速度控制单位为rpm,360.0f/60.0f=6.0f(角度/s)或2*3.1415926f/60.0f=0.104719755f(弧度/s)，根据需要调整

    mks_drive[MKS_MOTORS_USED_NUM++] = p_motor;  //将电机添加到电机数组中

    if (MKS_MOTORS_USED_NUM > MKS_MOTOR_NUM)
    {
        MKS_MOTORS_USED_NUM = MKS_MOTOR_NUM;
    }

    // mks_motor_set_work_mode(hfdcan, p_motor, control_mode);
}

/**
 * @brief 设置电机工作模式
 * @param hfdcan CAN句柄
 * @param p_motor 电机初始化结构体指针
 * @param control_mode 控制模式
    * @retval 发送结果，0表示成功，非0表示失败
    * @details 根据电机控制模式和通信模式构建控制命令数据，并通过CAN或UART发送给电机
 */
uint8_t mks_motor_set_work_mode(FDCAN_HandleTypeDef* hfdcan, mks_motor_drive_t *p_motor, mks_control_mode_e control_mode)
{
    uint8_t command_data[3]; // 定义一个8字节的数组来存储控制命令数据

    p_motor->motor_control.control_mode = control_mode;

    command_data[0] = (uint8_t)SET_WORK_MODE_COMMAND;   // 将控制模式转换为字节并存储在命令数据的第一个字节
    command_data[1] = (uint8_t)control_mode;            // 将控制模式转换为字节并存储在命令数据的第二个字节
    command_data[2] = (uint8_t)crc8_check_sum(p_motor->motor_id, command_data, 2, CRC_CHECK_CODE); // 计算CRC校验码并存储在命令数据的第三个字节  

    // 发送CAN指令设置电机控制模式和执行模式
    if (p_motor->motor_control.communication_mode == CAN_COMMUNIACTION_MODE)
    {
        return (bsp_fdcan_send(hfdcan, p_motor->motor_id, command_data, 3)); // 发送3字节的命令数据
    }
    else
    {
        // 发送UART指令设置电机控制模式和执行模式
        return 0; // UART发送函数返回值，具体实现根据UART发送函数定义进行修改
    }
}   

/**
    * @brief 设置电机速度控制参数并发送控制命令
    * @param hfdcan CAN句柄
    * @param p_motor 电机初始化结构体指针
    * @param speed_vel 目标速度值，单位rpm
    * @param acc_vel 目标加速度值，单位rpm/s
        * @retval 发送结果，0表示成功，非0表示失败
        * @details 根据电机控制模式和通信模式构建控制命令数据，并通过CAN或UART发送给电机
 */
uint8_t mks_motor_speed_ctrl(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, mks_communication_mode_e comm_mode, float speed_unit_vel, float speed_vel, uint8_t acc_vel)
{
    uint8_t data[5];
    float f_max_vel;
    uint16_t rpm_buf;

    data[0] = (uint8_t)SPEED_COMMAND; // 控制功能码

    f_max_vel = speed_unit_vel * MKS_MAX_SPEED_RPM; // 根据速度控制单位转换最大速度值

    rpm_buf =  float_abs_to_bit16(speed_vel, f_max_vel, MKS_MAX_SPEED_RPM); // 将目标速度绝对值映射到12位范围(0~4095)
    
    data[1] = (uint8_t)((speed_vel < 0) ? 0x80 : 0x00) | ((rpm_buf >> 8) & 0x0F); // bit7=方向, bit3~0=速度高4位
    data[2] = (uint8_t)(rpm_buf & 0xFF);                                  // 速度低8位
    data[3] = acc_vel; // 加速度值
    data[4] = (uint8_t)crc8_check_sum(motor_id, data, 4, CRC_CHECK_CODE); // CRC校验码

    if (comm_mode == CAN_COMMUNIACTION_MODE)
    {
        return bsp_fdcan_send(hfdcan, motor_id, data, 5); // 发送5字节的命令数据
    }
    else
    {
        // 发送UART指令设置电机速度和加速度
        return 0; // UART发送函数返回值，具体实现根据UART发送函数定义进行修改
    }
}

/**
 * @brief 设置电机相对位置控制参数并发送控制命令
 * @param hfdcan CAN句柄
 * @param comm_mode 通信模式
 * @param speed_unit_vel 速度控制单位
 * @param target_angle 目标角度值
 * @param target_speed 目标速度值
 * @param acc_vel 目标加速度值
    * @retval 发送结果，0表示成功，非0表示失败
    * @details 根据电机控制模式和通信模式构建控制命令数据，并通过CAN或UART发送给电机
     * 位置控制命令数据构建和发送逻辑
     * 根据电机控制模式和通信模式构建控制命令数据，并通过CAN或UART发送给电机
 */
uint8_t mks_motor_position_ctrl(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, mks_control_code_e control_code, mks_communication_mode_e comm_mode, float speed_unit_vel, uint16_t motor_niche, float angle_vel, float speed_vel, uint8_t acc_vel)
{
    uint8_t data[8];
    float f_max_speed;      // 根据速度控制单位转换最大速度值
    float f_max_position;   // 根据角度控制单位转换最大位置值

    float f_step_position;  // 每个脉冲对应的控制单位值
    uint16_t rpm_buf;       // 目标速度映射后的数值
    uint16_t angle_buf;     // 目标角度映射后的数值

    f_max_speed = speed_unit_vel * MKS_MAX_SPEED_RPM; // 根据速度控制单位转换最大速度值

#if MKS_ANGLE_MATCH_UNIT == 0
    f_step_position = 360.0f / (float)(motor_niche * MKS_PULSE_CYCLE); // 根据角度控制单位转换最大位置值，单位为度
#elif MKS_ANGLE_MATCH_UNIT == 1
    f_step_position = 2.0f * PI / (float)(motor_niche * MKS_PULSE_CYCLE); // 根据角度控制单位转换最大位置值，单位为弧度
#elif MKS_ANGLE_MATCH_UNIT == 2
    f_step_position = MKS_ENCODER_RESOLUTION / (float)(motor_niche * MKS_PULSE_CYCLE); // 根据角度控制单位转换最大位置值，单位为编码器值
#else
    f_step_position = 1.0f; // 根据角度控制单位转换最大位置值，单位为脉冲数值
#endif

    f_max_position = f_step_position * (float)MKS_MAX_PULSE_COUNT; // 根据角度控制单位转换最大位置值

    rpm_buf =  float_abs_to_bit16(speed_vel, f_max_speed, MKS_MAX_SPEED_RPM); // 将目标速度绝对值映射到12位范围(0~4095)
    angle_buf = float_abs_to_bit32(angle_vel, f_max_position, MKS_MAX_PULSE_COUNT); // 将目标角度绝对值映射到24位范围(0~16777215)

    data[0] = (uint8_t)control_code; // 控制功能码
    data[1] = (uint8_t)((angle_vel < 0) ? 0x80 : 0x00) | ((rpm_buf >> 8) & 0x0F); // bit7=方向, bit3~0=速度高4位
    data[2] = (uint8_t)(rpm_buf & 0xFF);                                  // 速度低8位
    data[3] = (uint8_t)acc_vel; // 加速度值
    data[4] = (uint8_t)((angle_buf >> 16) & 0xFF); // 目标角度高8位
    data[5] = (uint8_t)((angle_buf >> 8) & 0xFF);  // 目标角度中8位
    data[6] = (uint8_t)(angle_buf & 0xFF); // 目标角度低8位
    data[7] = (uint8_t)crc8_check_sum(motor_id, data, 7, CRC_CHECK_CODE); // CRC校验码，计算前7字节数据的CRC8校验值

    if (comm_mode == CAN_COMMUNIACTION_MODE)
    {
        return bsp_fdcan_send(hfdcan, motor_id, data, 8); // 发送8字节的命令数据
    }
    else
    {
        // 发送UART指令设置电机速度和加速度
        return 0; // UART发送函数返回值，具体实现根据UART发送函数定义进行修改
    }
    // 位置控制命令数据构建和发送逻辑
    // 根据电机控制模式和通信模式构建控制命令数据，并通过CAN或UART发送给电机
}

/**
 * @brief 读取电机编码器值
 * @param hfdcan CAN句柄
 * @param motor_id 电机ID
 * @param comm_mode 通信模式
 * @param read_command 读取命令
 * @retval 无
 * @details 根据通信模式构建读取编码器值的命令数据，并通过CAN或UART发送给电机
 * 读取编码器值命令数据构建和发送逻辑
 * 根据通信模式构建读取编码器值的命令数据，并通过CAN或UART发送给电机
 */
void mks_motor_read_parameter(FDCAN_HandleTypeDef* hfdcan, uint8_t motor_id, mks_communication_mode_e comm_mode, mks_control_code_e code_command)
{
    uint8_t data[2];

    data[0] = (uint8_t)code_command; // 读取编码器值的功能码
    data[1] = (uint8_t)crc8_check_sum(motor_id, data, 1, CRC_CHECK_CODE); // CRC校验码，计算前1字节数据的CRC8校验值

    if (comm_mode == CAN_COMMUNIACTION_MODE)
    {
        bsp_fdcan_send(hfdcan, motor_id, data, 2); // 发送2字节的命令数据
    }
    else
    {
        // 发送UART指令读取电机编码器值
    }
}

/**
 * @brief 发送电机控制命令
 * @param hfdcan CAN句柄
 * @param p_motor 电机初始化结构体指针
 * @retval 无
 * @details 根据电机执行模式发送相应的控制命令
 */
void mks_motor_ctrl_send(FDCAN_HandleTypeDef *hfdcan, mks_motor_drive_t *p_motor)
{
    switch (p_motor->motor_control.execute_mode)
    {
        case SPEED_CTRL_MODE:
            mks_motor_speed_ctrl(hfdcan, p_motor->motor_id, p_motor->motor_control.communication_mode, p_motor->motor_control.speed_conversion_unit, p_motor->motor_control.target_speed, p_motor->motor_control.target_acceleration);
            break;
            
        case POSITION_CTRL_MODE:
            mks_motor_position_ctrl(hfdcan, p_motor->motor_id, p_motor->control_code, p_motor->motor_control.communication_mode, p_motor->motor_control.speed_conversion_unit, p_motor->motor_niche, p_motor->motor_control.target_angle, p_motor->motor_control.target_speed, p_motor->motor_control.target_acceleration);
            break;
        default:
            // 处理无效的执行模式
            break;
    }
}

/**
 * @brief 解析电机反馈数据
 * @param p_motor 电机初始化结构体指针
 * @retval 无
 * @details 根据电机执行模式解析反馈数据并更新电机状态
 */
void mks_motor_fddata(mks_motor_drive_t *p_motor)
{
    // 解析电机反馈数据，更新p_motor->process_feedback等参数
#if MKS_ANGLE_MATCH_UNIT == 0
    if (p_motor->motor_control.execute_mode == SPEED_CTRL_MODE)
    {
        p_motor->process_feedback.angle_speed = p_motor->process_feedback.original_encoder.rotate_speed * RPM_TO_ANGLE_SEC; // 转换为度/s;
    }

    else if (p_motor->motor_control.execute_mode == POSITION_CTRL_MODE)
    {
        p_motor->process_feedback.single_round_angle = (float)p_motor->process_feedback.original_encoder.encoder / (float)MKS_ENCODER_RESOLUTION * 360.0f; // 单圈角度值，单位为度
        p_motor->process_feedback.multi_round_angle = p_motor->process_feedback.single_round_angle + (float)p_motor->process_feedback.original_encoder.rotate_count * 360.0f; // 多圈角度值，单位为度
    }

#elif MKS_ANGLE_MATCH_UNIT == 1
    if (p_motor->motor_control.execute_mode == SPEED_CTRL_MODE)
    {
        p_motor->process_feedback.angular_speed = p_motor->process_feedback.original_encoder.rotate_speed * RPM_TO_RAD_SEC; // 转换为rad/s;
    }

    else if (p_motor->motor_control.execute_mode == POSITION_CTRL_MODE)
    {
        p_motor->process_feedback.single_round_rad = (float)p_motor->process_feedback.original_encoder.encoder / (float)MKS_ENCODER_RESOLUTION * 2.0f * PI; // 单圈角度值，单位为弧度
        p_motor->process_feedback.multi_round_rad = p_motor->process_feedback.single_round_rad + (float)p_motor->process_feedback.original_encoder.rotate_count * 2.0f * PI; // 多圈角度值，单位为弧度
    }

#endif

}

/**
 * @brief 解析电机原始反馈数据
 * @param p_motor 电机初始化结构体指针
 * @param rx_data 接收到的反馈数据
 * @retval 无
 * @details 根据功能码解析反馈数据并更新电机状态
 */
void mks_motor_original_feedback(mks_motor_drive_t *p_motor, uint8_t *rx_data)
{
    switch (rx_data[0]) // 根据功能码判断反馈数据类型
    {

    #if MKS_ENCODER_FEEDBACK_MODE == 0      // 进位制编码器反馈模式
        case READ_CARRY_ENCODER_COMMAND:
            // 解析编码器反馈数据，更新p_motor->process_feedback.original_encoder.encoder等参数
            p_motor->process_feedback.original_encoder.rotate_count = (int32_t)(rx_data[1] << 24) | (rx_data[2] << 16) | rx_data[3] << 8 | rx_data[4];
            p_motor->process_feedback.original_encoder.encoder = (uint16_t)((rx_data[5] << 8) | rx_data[6]);
            break;

    #elif MKS_ENCODER_FEEDBACK_MODE == 1    // 累加制编码器值反馈模式
        case READ_ACC_ENCODER_COMMAND:
            // 解析累加制编码器反馈数据，更新p_motor->process_feedback.original_encoder.encoder等参数
            p_motor->process_feedback.original_encoder.encoder_value = (int64_t)(rx_data[1] << 40) | (rx_data[2] << 32) | (rx_data[3] << 24) | (rx_data[4] << 16) | (rx_data[5] << 8) | rx_data[6];
                break;
    #elif MKS_ENCODER_FEEDBACK_MODE == 2    // 原始累加多圈编码器值反馈模式
            case READ_ORIGINAL_ACC_ENCODER_COMMAND:    //  读取原始累加制编码器值的功能码
                // 解析原始累加制编码器反馈数据，更新p_motor->process_feedback.original_encoder.encoder_value等参数
                p_motor->process_feedback.original_encoder.encoder_value = (int64_t)(rx_data[1] << 40) | (rx_data[2] << 32) | (rx_data[3] << 24) | (rx_data[4] << 16) | (rx_data[5] << 8) | rx_data[6];
                break;
    #endif

        case READ_SPEED_COMMAND:    //  读取速度值的功能码
            // 解析速度反馈数据，更新p_motor->process_feedback.speed等参数
            p_motor->process_feedback.original_encoder.rotate_speed = (int16_t)((rx_data[1] << 8) | rx_data[2]);
            break;

        // 其他功能码的反馈数据解析逻辑
        default:
            // 处理未知功能码的反馈数据
            break;
    }
}