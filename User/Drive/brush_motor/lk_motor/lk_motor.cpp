#include "lk_motor.h"
#include "basic_math.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "user_lib.h"
#include <cmath>
#include <cstdint>

// LK电机命令报文, 使用命令响应模式 (CAN ID = 0x140 + motor_id)
// 电机运行 (打开)
uint8_t lk_motor_can_enter_message[8] = {Motor_LK_Edict_Motor_Open, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 电机停止 (失能, 不清除运行状态)
uint8_t lk_motor_can_exit_message[8] = {Motor_LK_Edict_Motor_Close, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 清除错误标志
uint8_t lk_motor_can_clear_error_message[8] = {Motor_LK_Edict_Clear_Error_Flag, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 设置当前零点 (RAM, 断电丢失)
uint8_t lk_motor_can_set_zero_ram_message[8] = {Motor_LK_Edict_Set_Motor_Point_RAM, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 获取电机当前状态
uint8_t lk_motor_can_get_status_message[8] = {Motor_LK_Edict_Read_Status_Error, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __can_rx_id 电机CAN ID (1~8), 对应CAN地址 0x200 + ID
 * @param __motor_lk_control_method 电机控制方式, 默认1拖4模式
 * @param __encoder_offset 编码器偏移, 默认0
 * @param __nearest_angle 就近转位的单次最大旋转角度, 0表示不启用
 * @param __gearbox_rate 减速比, 默认6.0
 */
void lk_motor_normal_class::init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__can_tx_rx_id, const lk_motor_control_method &__motor_lk_control_method, const int32_t &__encoder_offset, const float &__nearest_angle, const float &__gearbox_rate)
{
    // 绑定的CAN总线
    if (hcan == nullptr)
    {
        return;
    }

    else if (hcan->Instance == FDCAN1)
    {
        can_rx_frame = &fdcan1_rxframe;
    }

    else if (hcan->Instance == FDCAN2)
    {
        can_rx_frame = &fdcan2_rxframe;
    }

    else if (hcan->Instance == FDCAN3)
    {
        can_rx_frame = &fdcan3_rxframe;
    }

    can_tx_rx_id = __can_tx_rx_id + LK_Normal_Can_Offset;

    control_method = __motor_lk_control_method;
    encoder_offset = __encoder_offset;
    nearest_angle = __nearest_angle;
    gearbox_rate = __gearbox_rate;
    first_fb_flag = true;   // 复位/上电后首帧建立编码器基准, 防止误计圈
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void lk_motor_normal_class::can_rx_callback()
{
    // 滑动窗口, 判断电机是否在线
    flag += 1;

    // 复位后 CAN 中断可能先于 init() 触发, 此时 can_rx_frame 尚未绑定, 忽略早期帧, 防止读取空指针垃圾数据
    if (can_rx_frame == nullptr)
    {
        return;
    }

    uint8_t tmp_command = can_rx_frame->data[0];

    if (tmp_command == Motor_LK_Edict_Read_Status_Error)
    {
        error_process();
    }

    else if (tmp_command == Motor_LK_Edict_Read_Parameter || tmp_command == Motor_LK_Edict_Normal_Open_Loop || 
             tmp_command == Motor_LK_Edict_Normal_Torque || tmp_command == Motor_LK_Edict_Normal_Omega ||
             tmp_command == Motor_LK_Edict_Normal_Multi_Angle || tmp_command == Motor_LK_Edict_Normal_Multi_Angle_Omega ||
             tmp_command == Motor_LK_Edict_Normal_Single_Angle || tmp_command == Motor_LK_Edict_Normal_Single_Angle_Omega || 
             tmp_command == Motor_LK_Edict_Normal_Incremental_Angle || tmp_command == Motor_LK_Edict_Normal_Incremental_Angle_Omega)
    {
        data_process();
    }

    else
    {
        // 其他命令, 暂不处理
    }
}

/**
 * @brief 发送电机使能命令 (0x88 电机运行)
 */
void lk_motor_normal_class::can_send_enter() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, lk_motor_can_enter_message, 8);
}

/**
 * @brief 发送电机失能命令 (0x81 电机停止)
 */
void lk_motor_normal_class::can_send_exit() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, lk_motor_can_exit_message, 8);
}

/**
 * @brief 发送清除错误命令 (0x9B)
 */
void lk_motor_normal_class::can_send_clear_error() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, lk_motor_can_clear_error_message, 8);
}

/**
 * @brief 发送设置当前零点命令 (0x95, RAM)
 */
void lk_motor_normal_class::can_send_set_zero_ram() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, lk_motor_can_set_zero_ram_message, 8);
}

void lk_motor_normal_class::can_send_get_status() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, lk_motor_can_get_status_message, 8);
}

/**
 * @brief 电机存活检测
 * @note 定期检测电机是否存活, 检测周期取决于电机掉线时长
 */
void lk_motor_normal_class::alive_check()
{
    // 复位后 CAN 中断可能先于 init() 触发, 或 init() 因句柄为空提前返回, 此时总线未绑定:
    // 直接返回, 避免下方 can_send_enter() 解引用空指针
    if (can_rx_frame == nullptr)
    {
        return;
    }

    // 判断该时间段内是否接收过电机数据
    if (flag == pre_flag)
    {
        // 电机断开连接
        motor_status = Motor_LK_Status_Disable;
    }

    else
    {
        // 电机保持连接
        motor_status = Motor_LK_Status_Enable;
    }

    pre_flag = flag;

    // 如果电机断开连接, 则尝试重新使能电机
    if (motor_status == Motor_LK_Status_Disable)
    {
        can_send_clear_error();
    }
}

/**
 * @brief 电机传感器滤波后数据计算回调函数, 计算周期取决于滤波反馈数据周期
 *
 */
void lk_motor_normal_class::calculate_callback()
{
    if (motor_status == Motor_LK_Status_Enable)
    {
        basic_math_constrain(&control_torque, -Torque_Out_Max, Torque_Out_Max);
        basic_math_constrain(&torque_limit, -Torque_Out_Max, Torque_Out_Max);
        output();
    }

    else if (motor_status == Motor_LK_Status_Disable)
    {
        // 电机断开连接, 尝试重新使能电机
        can_send_clear_error();
    }
}

/**
 * @brief 数据处理过程
 * @note Status 2 (0x9C) 格式)，数据为小端序, 无需字节序转换
 */
void lk_motor_normal_class::data_process()
{
    static int32_t delta_encoder;
    static uint16_t tmp_encoder;
    static int16_t tmp_omega, tmp_torque_current;

    // 将CAN原始数据映射为Status 2 (0x9C) 响应帧格式
    lk_motor_source_data *tmp_buffer = (lk_motor_source_data *)can_rx_frame->data;

    // 提取各字段 (小端序, 无需字节序转换)
    tmp_torque_current = tmp_buffer->torque_current;
    tmp_omega = tmp_buffer->omega;
    tmp_encoder = tmp_buffer->encoder;

    // 首次接收: 跳过圈数判断, 以当前绝对位置为基准, 防止复位/上电时误计圈
    if (first_fb_flag)
    {
        first_fb_flag = false;
        rx_data.pre_encoder = tmp_encoder;
        rx_data.total_round = 0;    // 复位后多圈信息已丢失, 从当前位置重新开始计数
    }

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - rx_data.pre_encoder;

    if (delta_encoder < -(int32_t)(Encoder_Num_Per_Round / 2))
    {
        // 正方向转过了一圈
        rx_data.total_round += 1;
    }

    else if (delta_encoder > (int32_t)(Encoder_Num_Per_Round / 2))
    {
        // 反方向转过了一圈
        rx_data.total_round -= 1;
    }

    rx_data.total_encoder = rx_data.total_round * Encoder_Num_Per_Round + tmp_encoder + encoder_offset;
    
    // 计算电机本身信息
    rx_data.now_angle = (float)(rx_data.total_encoder) / (Encoder_Num_Per_Round) * 2.0f * PI;       // 输出轴角度, rad
    rx_data.single_angle = (float)(tmp_encoder) / (Encoder_Num_Per_Round) * 2.0f * PI;              // 输出轴单圈角度, rad
    rx_data.now_omega = (float)(tmp_omega) * BASIC_MATH_DEG_TO_RAD / (float)gearbox_rate;                                // 输出轴角速度, rad/s
    rx_data.now_torque = (float)(tmp_torque_current) * Current_To_Out * Current_To_Torque;          // 输出轴扭矩, Nm
    rx_data.now_temperature = (float)(tmp_buffer->temperature);                                     // 电机温度, ℃

    // NaN/inf 运行时安全钳位: 防止 Ozone Timeline 统计被污染
    if (rx_data.now_angle != rx_data.now_angle || rx_data.now_angle > 1e10f || rx_data.now_angle < -1e10f)
        rx_data.now_angle = 0.0f;
    if(rx_data.single_angle != rx_data.single_angle || rx_data.single_angle > 1e10f || rx_data.single_angle < -1e10f)
        rx_data.single_angle = 0.0f;
    if (rx_data.now_omega != rx_data.now_omega || rx_data.now_omega > 1e10f || rx_data.now_omega < -1e10f)
        rx_data.now_omega = 0.0f;
    if (rx_data.now_torque != rx_data.now_torque || rx_data.now_torque > 1e10f || rx_data.now_torque < -1e10f)
        rx_data.now_torque = 0.0f;
    if (rx_data.now_temperature != rx_data.now_temperature || rx_data.now_temperature > 1e10f || rx_data.now_temperature < -1e10f)
        rx_data.now_temperature = 0.0f;

    // 存储预备信息
    rx_data.pre_encoder = tmp_encoder;
}



/**
 * @brief 电机PID计算控制，使用电机反馈的原始数据进行PID闭环控制
 *
 */
void lk_motor_normal_class::error_process()
{
    uint8_t tmp_error_state;
    lk_motor_status_error_data_t *tmp_buffer = (lk_motor_status_error_data_t *)can_rx_frame->data;
    tmp_error_state = tmp_buffer->error_state;

    // 解析错误标志位 —— 更新到 control_status 成员中
    control_status = Motor_LK_Control_Status_Normal;  // 先清零

    if (tmp_error_state & Motor_LK_Control_Status_UnderVoltage)
        control_status = Motor_LK_Control_Status_UnderVoltage;

    if (tmp_error_state & Motor_LK_Control_Status_OverVoltage)
        control_status = Motor_LK_Control_Status_OverVoltage;

    if (tmp_error_state & Motor_LK_Control_Status_Driver_OverTemperature)
        control_status = Motor_LK_Control_Status_Driver_OverTemperature;

    if (tmp_error_state & Motor_LK_Control_Status_Motor_OverTemperature)
        control_status = Motor_LK_Control_Status_Motor_OverTemperature;

    if (tmp_error_state & Motor_LK_Control_Status_OverCurrent)
        control_status = Motor_LK_Control_Status_OverCurrent;

    if (tmp_error_state & Motor_LK_Control_Status_ShortCircuit)
        control_status = Motor_LK_Control_Status_ShortCircuit;

    if (tmp_error_state & Motor_LK_Control_Status_Block)
        control_status = Motor_LK_Control_Status_Block;

    if (tmp_error_state & Motor_LK_Control_Status_Signal_Lost)
        control_status = Motor_LK_Control_Status_Signal_Lost;
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 * @note 连续反馈模式: 输出为int16_t写入共享CAN缓冲区
 *       命令模式: 输出为完整8字节命令报文发送到 CAN总线
 */
void lk_motor_normal_class::output()
{
    switch (control_method)
    {
        case Motor_LK_Control_Method_Normal_Torque:
        {
            // 转矩闭环控制报文 (0xA1)
            lk_motor_tx_data_torque *tmp_buffer = (lk_motor_tx_data_torque *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Torque;
            tmp_buffer->torque = (int16_t)(control_torque / Current_To_Torque / Current_To_Out);  // 输出轴的转矩-反馈的电流值: Nm → A
            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, tx_data, 8);
            break;
        }

        case Motor_LK_Control_Method_Normal_Omega:
        {
            // 角速度闭环控制报文 (0xA2)
            lk_motor_tx_data_omega *tmp_buffer = (lk_motor_tx_data_omega *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Omega;
            tmp_buffer->limit_torque = (int16_t)(torque_limit / Current_To_Torque / Current_To_Out); // 输出轴的转矩限制: Nm → A
            // 角速度: rad/s → 0.01dps (0.01°/s)
            tmp_buffer->omega = (int32_t)(control_omega / BASIC_MATH_DEG_TO_RAD * 100.0f);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, tx_data, 8);
            break;
        }

        case Motor_LK_Control_Method_Normal_Multi_Angle:
        {
            // 多圈角度闭环控制报文 (0xA3)
            lk_motor_tx_data_multi_angle *tmp_buffer = (lk_motor_tx_data_multi_angle *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Multi_Angle;
            // 角度: rad → 0.01°
            tmp_buffer->angle = (int32_t)(control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, tx_data, 8);
            break;
        }

        case Motor_LK_Control_Method_Normal_Multi_Angle_Omega:
        {
            uint16_t tmp_omega;
            // 角速度限制: rad/s → 1.0dps (1.0°/s)
            tmp_omega = fabsf(omega_limit) / BASIC_MATH_DEG_TO_RAD; 

            // 多圈角度+角速度闭环控制报文 (0xA4)
            lk_motor_tx_data_multi_angle_omega *tmp_buffer = (lk_motor_tx_data_multi_angle_omega *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Multi_Angle_Omega;
            tmp_buffer->limit_omega = (uint16_t)(tmp_omega); 
            tmp_buffer->angle = (int32_t)(control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, tx_data, 8);
            break;
        }

        case Motor_LK_Control_Method_Normal_Single_Angle:
        {
            // 单圈角度闭环控制报文 (0xA5)
            uint8_t tmp_spin;   // 旋转方向, 0: 顺时针, 1: 逆时针
            uint32_t tmp_angle;

            if (control_angle >= 0.0f)
            {
                tmp_spin = 0x00;
                tmp_angle = (uint32_t)(control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);
            }

            else
            {
                tmp_spin = 0x01;
                tmp_angle = (uint32_t)(-control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);
            }

            lk_motor_tx_data_single_angle *tmp_buffer = (lk_motor_tx_data_single_angle *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Single_Angle;
            tmp_buffer->spin_direction = tmp_spin;
            tmp_buffer->angle = (uint32_t)(tmp_angle);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, (uint8_t *)&tmp_buffer, 8);
            break;
        }

        case Motor_LK_Control_Method_Normal_Single_Angle_Omega:
        {
            // 单圈角度+角速度闭环控制报文 (0xA6)
            uint8_t tmp_spin;   // 旋转方向, 0: 顺时针, 1: 逆时针
            uint16_t tmp_omega;
            uint32_t tmp_angle; 

            if (control_angle >= 0.0f)
            {
                tmp_spin = 0x00;
                tmp_angle = (uint32_t)(control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);
            }

            else
            {
                tmp_spin = 0x01;
                tmp_angle = (uint32_t)(-control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);
            }

            tmp_omega = fabsf(omega_limit) / BASIC_MATH_DEG_TO_RAD;

            lk_motor_tx_data_single_angle_omega *tmp_buffer = (lk_motor_tx_data_single_angle_omega *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Single_Angle_Omega;
            tmp_buffer->spin_direction = tmp_spin;
            tmp_buffer->limit_omega = (uint16_t)(tmp_omega);
            tmp_buffer->angle = (uint32_t)(tmp_angle);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, (uint8_t *)&tmp_buffer, 8);
            break;
        }

        case Motor_LK_Control_Method_Normal_Incremental_Angle:
        {
            // 增量角度闭环控制报文 (0xA7)
            lk_motor_tx_data_incremental_angle *tmp_buffer = (lk_motor_tx_data_incremental_angle *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Incremental_Angle;
            tmp_buffer->angle = (int32_t)(control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, (uint8_t *)&tmp_buffer, 8);
            break;
        }

        case Motor_LK_Control_Method_Normal_Incremental_Angle_Omega:
        {
            // 增量角度+角速度闭环控制报文 (0xA8)
            uint16_t tmp_omega;

            tmp_omega = fabsf(omega_limit) / BASIC_MATH_DEG_TO_RAD;

            lk_motor_tx_data_incremental_angle_omega *tmp_buffer = (lk_motor_tx_data_incremental_angle_omega *)tx_data;
            tmp_buffer->command_byte = Motor_LK_Edict_Normal_Incremental_Angle_Omega;
            tmp_buffer->limit_omega = (uint8_t)(tmp_omega);
            tmp_buffer->angle = (int32_t)(control_angle / BASIC_MATH_DEG_TO_RAD * 100.0f);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_rx_id, (uint8_t *)&tmp_buffer, 8);
            break;
        }

        default:
        {
            break;
        }
    }
}

