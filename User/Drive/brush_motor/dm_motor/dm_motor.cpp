#include "dm_motor.h"
#include "basic_math.h"
#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"
#include "user_lib.h"
#include <cstdint>

// 清除电机错误信息, 传统模式有效
uint8_t dm_motor_can_clear_error_message[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};

// 使能电机, 传统模式有效
uint8_t dm_motor_can_enter_message[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};

// 失能电机, 传统模式有效
uint8_t dm_motor_can_exit_message[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};

// 保存当前电机位置为零点, 传统模式有效
uint8_t dm_motor_can_save_zero_message[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

/**
 * @brief 分配CAN发送缓冲区, 一拖四模式有效
 *
 * @param hcan CAN编号
 * @param __can_id CAN ID
 * @param __dm_motor_driver_version 大疆驱动版本, 当且仅当当前被分配电机为6020, 且是电流驱动新版本时选2023, 否则都是default
 * @return uint8_t* 缓冲区指针
*/
static uint8_t *allocate_can_tx_data(const FDCAN_HandleTypeDef *hcan, const dm_motor_id_1_to_4 &__can_rx_id_1_to_4)
{
    uint8_t *tmp_tx_data_ptr;

    // 选定hfdcan1进行电机通讯
    if (hcan == &hfdcan1)
    {
        switch (__can_rx_id_1_to_4)
        {
            case (Motor_DM_ID_0x201):
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[0]);
                break;
            }

            case (Motor_DM_ID_0x202):
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[2]);
                break;
            }

            case (Motor_DM_ID_0x203):
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[4]);
                break;
            }

            case (Motor_DM_ID_0x204):
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[6]);
                break;
            }

            case (Motor_DM_ID_0x205):
            {
                tmp_tx_data_ptr = &(can1_0x1ff_tx_data[0]);
                break;
            }

            case (Motor_DM_ID_0x206):
            {
                tmp_tx_data_ptr = &(can1_0x1ff_tx_data[2]);
                break;
            }

            case (Motor_DM_ID_0x207):
            {
                tmp_tx_data_ptr = &(can1_0x1ff_tx_data[4]);
                break;
            }

            case (Motor_DM_ID_0x208):
            {
                tmp_tx_data_ptr = &(can1_0x1ff_tx_data[6]);
                break;
            }
        }
    }

    // 选定hfdcan2进行电机通讯
    else if (hcan == &hfdcan2)
    {
        switch (__can_rx_id_1_to_4)
        {
            case (Motor_DM_ID_0x201):
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[0]);
                break;
            }

            case (Motor_DM_ID_0x202):
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[2]);
                break;
            }

            case (Motor_DM_ID_0x203):
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[4]);
                break;
            }

            case (Motor_DM_ID_0x204):
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[6]);
                break;
            }

            case (Motor_DM_ID_0x205):
            {
                tmp_tx_data_ptr = &(can2_0x1ff_tx_data[0]);
                break;
            }

            case (Motor_DM_ID_0x206):
            {
                tmp_tx_data_ptr = &(can2_0x1ff_tx_data[2]);
                break;
            }

            case (Motor_DM_ID_0x207):
            {
                tmp_tx_data_ptr = &(can2_0x1ff_tx_data[4]);
                break;
            }

            case (Motor_DM_ID_0x208):
            {
                tmp_tx_data_ptr = &(can2_0x1ff_tx_data[6]);
                break;
            }
        }
    }

    // 选定hfdcan3进行电机通讯
    else if (hcan == &hfdcan3)
    {
        switch (__can_rx_id_1_to_4)
        {
            case (Motor_DM_ID_0x201):
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[0]);
                break;
            }

            case (Motor_DM_ID_0x202):
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[2]);
                break;
            }

            case (Motor_DM_ID_0x203):
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[4]);
                break;
            }

            case (Motor_DM_ID_0x204):
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[6]);
                break;
            }

            case (Motor_DM_ID_0x205):
            {
                tmp_tx_data_ptr = &(can3_0x1ff_tx_data[0]);
                break;
            }

            case (Motor_DM_ID_0x206):
            {
                tmp_tx_data_ptr = &(can3_0x1ff_tx_data[2]);
                break;
            }

            case (Motor_DM_ID_0x207):
            {
                tmp_tx_data_ptr = &(can3_0x1ff_tx_data[4]);
                break;
            }

            case (Motor_DM_ID_0x208):
            {
                tmp_tx_data_ptr = &(can3_0x1ff_tx_data[6]);
                break;
            }
        }
    }

    // 未选中正确的fdcan通讯接口
    else
    {
        tmp_tx_data_ptr = nullptr;
    }

    return (tmp_tx_data_ptr);
}

/**
 * @brief 估计功率值
 *
 * @param k_0 电机建模系数
 * @param k_1 电机建模系数
 * @param k_2 电机建模系数
 * @param a 电机建模系数
 * @param torque 减速箱前扭矩, Nm
 * @param omega 减速箱前角速度, rad/s
 * @return
 */
static float power_calculate(float k_0, float k_1, float k_2, float a, float torque, float omega)
{
    return (k_0 * torque * omega + k_1 * omega * omega + k_2 * torque * torque + a);
}

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __can_rx_id 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __can_tx_id 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __motor_dm_control_method 电机控制方式
 * @param __angle_max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __omega_max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __torque_max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 * @param __current_max 最大电流, 与上位机串口中上电打印电流保持一致, EMIT模式需要
 */
void dm_motor_normal_class::init(const FDCAN_HandleTypeDef *hcan, const uint8_t &__can_rx_id, const uint8_t &__can_tx_id, const dm_motor_control_method &__dm_motor_control_method, const float &__angle_max, const float &__omega_max, const float &__torque_max, const float &__current_max)
{
    // 绑定的CAN总线·
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

    can_rx_id = __can_rx_id;

    // 接收ID偏移
    switch (__dm_motor_control_method)
    {
        case Motor_DM_Control_Method_Normal_MIT:
        {
            can_tx_id = __can_tx_id;
            break;
        }

        case Motor_DM_Control_Method_Normal_Angle_Omega:
        {
            can_tx_id = __can_tx_id + 0x100;
            break;
        }

        case Motor_DM_Control_Method_Normal_Omega:
        {
            can_tx_id = __can_tx_id + 0x200;
            break;
        }

        case Motor_DM_Control_Method_Normal_EMIT:
        {
            can_tx_id = __can_tx_id + 0x300;
            break;
        }

        default:
        {
            break;
        }
    }

    motor_control_method = __dm_motor_control_method;
    angle_max = __angle_max;
    omega_max = __omega_max;
    torque_max = __torque_max;
    current_max = __current_max;
    first_fb_flag = true;   // 复位/上电后首帧建立编码器基准, 防止误计圈
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void dm_motor_normal_class::can_rx_callback()
{
    // 滑动窗口, 判断电机是否在线
    flag += 1;

    // 复位后 CAN 中断可能先于 init() 触发, 此时 can_rx_frame 尚未绑定, 忽略早期帧, 防止读取空指针垃圾数据
    if (can_rx_frame == nullptr)
    {
        return;
    }

    data_process();
}

/**
 * @brief 发送清除错误信息
 *
 */
void dm_motor_normal_class::can_send_clear_error() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_id & 0x7ff, dm_motor_can_clear_error_message, 8);
}

/**
 * @brief 发送使能电机
 *
 */
void dm_motor_normal_class::can_send_enter() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_id & 0x7ff, dm_motor_can_enter_message, 8);
}

/**
 * @brief 发送失能电机
 *
 */
void dm_motor_normal_class::can_send_exit() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_id & 0x7ff, dm_motor_can_exit_message, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 */
void dm_motor_normal_class::can_send_save_zero() const
{
    bsp_fdcan_send(can_rx_frame->hcan, can_tx_id & 0x7ff, dm_motor_can_save_zero_message, 8);
}

/**
  * @brief 电机存活检测
  * @note 定期检测电机是否存活, 检测周期取决于电机掉线时长
 */
void dm_motor_normal_class::alive_check()
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
        motor_status = Motor_DM_Status_Disable;
    }
    else
    {
        // 电机保持连接
        motor_status = Motor_DM_Status_Enable;
    }

    pre_flag = flag;

    // 如果电机断开连接, 则尝试重新使能电机
    if (motor_status == Motor_DM_Status_Disable)
    {
        can_send_enter();
    }
}

/**
 * @brief 电机控制回调函数, 控制周期取决于任务周期, 传统模式有效
 *
 */
void dm_motor_normal_class::calculate_callback()
{
    if (rx_data.control_Status == Motor_DM_Control_Status_Enable)
    {
        basic_math_constrain(&control_angle, -angle_max, angle_max);
        basic_math_constrain(&control_omega, -omega_max, omega_max);
        basic_math_constrain(&control_torque, -torque_max, torque_max);
        basic_math_constrain(&control_current, -current_max, current_max);

        basic_math_constrain(&k_p, 0.0f, 500.0f);
        basic_math_constrain(&k_d, 0.0f, 5.0f);

        output();
    }

    else if (rx_data.control_Status == Motor_DM_Control_Status_Disable)
    {
        // 电机未使能, 发送失能报文
        can_send_enter();
    }

    else
    {
        // 电机状态异常, 发送清除错误报文
        can_send_clear_error();
    }
}

/**
    @brief DM电机反馈数据报文处理
 */
void dm_motor_normal_class::data_process()
{
    static uint16_t tmp_encoder, tmp_omega, tmp_torque;
    static int32_t delta_encoder;

    dm_motor_rx_data_normal *tmp_buffer = (dm_motor_rx_data_normal *)can_rx_frame->data;

    if (tmp_buffer->can_id != (can_rx_id & 0x0f))
    {
        return;
    }

    // 处理大小端数据，大小端转换
    basic_math_endian_swap_16((void *)(&tmp_buffer->angle_reverse), &tmp_encoder);
    tmp_omega = (uint16_t)((tmp_buffer->omega_11_4 << 4) | (tmp_buffer->omega_3_0_torque_11_8 >> 4));
    tmp_torque = (uint16_t)(((tmp_buffer->omega_3_0_torque_11_8 & 0x0f) << 8) | (tmp_buffer->torque_7_0));

    rx_data.control_Status = static_cast<dm_motor_control_status>(tmp_buffer->control_status_enum);

    // 首次接收: 跳过圈数判断, 以当前绝对位置为基准, 防止复位/上电时误计圈
    if (first_fb_flag)
    {
        first_fb_flag = false;
        rx_data.pre_encoder = tmp_encoder;
        rx_data.total_round = 0;    // 复位后多圈信息已丢失, 从当前位置重新开始计数
    }

    // 计算电机总角度和总圈数
    delta_encoder = tmp_encoder - rx_data.pre_encoder;

    if (delta_encoder < -(1 << 15))
    {
        // 电机正向旋转一圈
        rx_data.total_round += 1;
    }

    else if (delta_encoder > (1 << 15))
    {
        // 电机反向旋转一圈
        rx_data.total_round -= 1;
    }

    rx_data.total_angle = (float)(rx_data.total_round * (1 << 16) + tmp_encoder - ((1 << 15) - 1)) / (float)((1 << 16) - 1) * angle_max * 2.0f; // 计算电机总角度, 以编码器计数为单位

    // 计算电机本身信息
    rx_data.now_angle = basic_math_int_to_float(tmp_encoder, 0x7fff, (1 << 16) - 1, 0, angle_max);
    rx_data.now_omega = basic_math_int_to_float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, omega_max);
    rx_data.now_torque = basic_math_int_to_float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, torque_max);
    rx_data.now_mos_temperature = tmp_buffer->mos_temperature;      // MOS管温度, °C
    rx_data.now_rotor_temperature = tmp_buffer->rotor_temperature;  // 转子温度, °C

    // NaN/inf 运行时安全钳位: 防止 Ozone Timeline 统计被污染
    if (rx_data.now_angle != rx_data.now_angle || rx_data.now_angle > 1e10f || rx_data.now_angle < -1e10f)
        rx_data.now_angle = 0.0f;
    if (rx_data.now_omega != rx_data.now_omega || rx_data.now_omega > 1e10f || rx_data.now_omega < -1e10f)
        rx_data.now_omega = 0.0f;
    if (rx_data.now_torque != rx_data.now_torque || rx_data.now_torque > 1e10f || rx_data.now_torque < -1e10f)
        rx_data.now_torque = 0.0f;

    // 保存当前编码器值为前一时刻编码器值
    rx_data.pre_encoder = tmp_encoder;

}

/**
 * @brief 输出控制数据
 */
void dm_motor_normal_class::output()
{
    switch (motor_control_method)
    {
        case Motor_DM_Control_Method_Normal_MIT:
        {
            dm_motor_tx_data_mit *tmp_buffer = (dm_motor_tx_data_mit *)tx_data;

            uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

            tmp_angle = basic_math_float_to_int(control_angle, 0, angle_max, 0x7fff, (1 << 16) - 1);
            tmp_omega = basic_math_float_to_int(control_omega, 0, omega_max, 0x7ff, (1 << 12) - 1);
            tmp_torque = basic_math_float_to_int(control_torque, 0, torque_max, 0x7ff, (1 << 12) - 1);
            tmp_k_p = basic_math_float_to_int(k_p, 0, 500.0f, 0, (1 << 12) - 1);
            tmp_k_d = basic_math_float_to_int(k_d, 0, 5.0f, 0, (1 << 12) - 1);
            // tmp_angle = float_to_uint(control_angle, -angle_max, angle_max, 16);
            // tmp_omega = float_to_uint(control_omega, -omega_max, omega_max, 12);
            // tmp_k_p = float_to_uint(k_p, 0, 500.0f, 12);
            // tmp_k_d = float_to_uint(k_d, 0, 5.0f, 12);
            // tmp_torque = float_to_uint(control_torque, -torque_max, torque_max , 12);

            tmp_buffer->control_angle_reverse = basic_math_endian_swap_16(&tmp_angle, nullptr);

            tmp_buffer->control_omega_11_4 = (uint8_t)(tmp_omega >> 4);
            tmp_buffer->control_omega_3_0_k_p_11_8 = (uint8_t)(((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8));
            tmp_buffer->k_p_7_0 = (uint8_t)(tmp_k_p & 0xff);
            tmp_buffer->k_d_11_4 = (uint8_t)(tmp_k_d >> 4);

            tmp_buffer->k_d_3_0_control_torque_11_8 = (uint8_t)(((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8));
            tmp_buffer->control_torque_7_0 = (uint8_t)(tmp_torque & 0xff);

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_id, tx_data, 8);
            break;
        }

        case Motor_DM_Control_Method_Normal_Angle_Omega:
        {
            dm_motor_tx_data_position_velocity *tmp_buffer = (dm_motor_tx_data_position_velocity *)tx_data;

            tmp_buffer->control_angle = control_angle;
            tmp_buffer->control_omega = control_omega;

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_id, tx_data, 8);
            break;
        }

        case Motor_DM_Control_Method_Normal_Omega:
        {
            dm_motor_tx_data_velocity *tmp_buffer = (dm_motor_tx_data_velocity *)tx_data;

            tmp_buffer->control_omega = control_omega;

            bsp_fdcan_send(can_rx_frame->hcan, can_tx_id, tx_data, 4);
            break;
        }

        case Motor_DM_Control_Method_Normal_EMIT:
        {
            dm_motor_tx_data_emit *tmp_buffer = (dm_motor_tx_data_emit *)tx_data;

            tmp_buffer->control_angle = control_angle;
            tmp_buffer->control_omega = (uint16_t)(control_omega * 100.0f);
            tmp_buffer->control_current = (uint16_t)(control_current / current_max * 10000.0f);


            bsp_fdcan_send(can_rx_frame->hcan, can_tx_id, tx_data, 8);
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * @brief 电机初始化
 * 
 * @param hcan 绑定的CAN总线
 * @param __can_rx_id 绑定的CAN ID
 * @param __dm_motor_control_method 电机控制方式, 默认角度
 * @param __encoder_offset 编码器偏移, 默认0
 * @param __nearest_angle 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
 * @param __gearbox_rate 减速比, 默认10.0f
 */
void dm_motor_1_to_4_class::init(const FDCAN_HandleTypeDef *hcan, const dm_motor_id_1_to_4 &__can_rx_id, const dm_motor_control_method &__dm_motor_control_method, const int32_t &__encoder_offset, const float &__nearest_angle, const float &__gearbox_rate)
{
    // 绑定的CAN总线·
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

    can_rx_id = __can_rx_id;

    motor_control_method = __dm_motor_control_method;
    encoder_offset = __encoder_offset;
    nearest_angle = __nearest_angle;
    gearbox_rate = __gearbox_rate;
    first_fb_flag = true;   // 复位/上电后首帧建立编码器基准, 防止误计圈
    tx_data = allocate_can_tx_data(hcan, __can_rx_id);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void dm_motor_1_to_4_class::can_rx_callback()
{
    // 滑动窗口, 判断电机是否在线
    flag += 1;

    // 复位后 CAN 中断可能先于 init() 触发, 此时 can_rx_frame 尚未绑定, 忽略早期帧, 防止读取空指针垃圾数据
    if (can_rx_frame == nullptr)
    {
        return;
    }

    data_process();
}

/**
  * @brief 电机存活检测
  * @note 定期检测电机是否存活, 检测周期取决于电机掉线时长
 */
void dm_motor_1_to_4_class::alive_check()
{
    // 判断该时间段内是否接收过电机数据
    if (flag == pre_flag)
    {
        // 电机断开连接
        motor_status = Motor_DM_Status_Disable;
    }
    else
    {
        // 电机保持连接
        motor_status = Motor_DM_Status_Enable;
    }

    pre_flag = flag;
}

/**
 * @brief 电机原始传感器反馈数据计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void dm_motor_1_to_4_class::internal_calculate_callback()
{
    pid_internal_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque / gearbox_rate * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_torque = 0.0f;
    feedforward_omega = 0.0f;
}

/**
 * @brief 电机传感器滤波后数据计算回调函数, 计算周期取决于滤波反馈数据周期
 *
 */
void dm_motor_1_to_4_class::external_calculate_callback()
{
    pid_external_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque / gearbox_rate * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_torque = 0.0f;
    feedforward_omega = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 */
void dm_motor_1_to_4_class::data_process()
{
    // 数据处理过程
    static int16_t delta_encoder;
    static uint16_t tmp_encoder;
    static int16_t tmp_omega, tmp_current;

    dm_motor_rx_data_1_to_4 *tmp_buffer = (dm_motor_rx_data_1_to_4 *)can_rx_frame->data;

        // 处理大小端数据，大小端转换
    basic_math_endian_swap_16((void *)(&tmp_buffer->encoder_reverse), (void *)&tmp_encoder);
    basic_math_endian_swap_16((void *)(&tmp_buffer->omega_reverse), (void *)&tmp_omega);
    basic_math_endian_swap_16((void *)(&tmp_buffer->current_reverse), (void *)&tmp_current);

    // 首次接收: 跳过圈数判断, 以当前绝对位置为基准, 防止复位/上电时误计圈
    if (first_fb_flag)
    {
        first_fb_flag = false;
        rx_data.pre_encoder = tmp_encoder;
        rx_data.total_round = 0;    // 复位后多圈信息已丢失, 从当前位置重新开始计数
    }

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - rx_data.pre_encoder;

    if (delta_encoder < -(Encoder_Num_Per_Round / 2))
    {
        // 正方向转过了一圈
        rx_data.total_round += 1;
    }

    else if (delta_encoder > (Encoder_Num_Per_Round / 2))
    {
        // 反方向转过了一圈
        rx_data.total_round -= 1;
    }

    rx_data.total_encoder = rx_data.total_round * Encoder_Num_Per_Round + tmp_encoder + encoder_offset;

    // 计算电机本身信息

    rx_data.status = static_cast<dm_motor_control_status>(tmp_buffer->motor_status);; // 电机状态, 0x00: 失能, 0x01: 使能

    // 电机转子原始数据
    rx_data.rotor_angle = (float)(rx_data.total_encoder) / (float)(Encoder_Num_Per_Round) * 2.0f * PI; // 转子角度, rad
    rx_data.rotor_omega = (float)(tmp_omega) * BASIC_MATH_RPM_TO_RADPS;                 // 转子角速度, rad/s
    rx_data.rotor_current = (float)(tmp_current);                                       // 转子电流, A

    // 电机输出轴转换数据
    rx_data.now_angle = rx_data.rotor_angle;                                            // 减速箱减速前电机角度, rad
    rx_data.single_angle = (float)(tmp_encoder) / (float)(Encoder_Num_Per_Round) * 2.0f * PI;    // 电机单圈角度, rad
    rx_data.now_omega = rx_data.rotor_omega;                                            // 电机角速度, rad/s
    rx_data.now_torque = rx_data.rotor_current / Current_To_Out * Current_To_Torque;    // 电机扭矩, Nm
    rx_data.now_temperature = (float)(tmp_buffer->temperature);                         // 电机温度
    rx_data.now_power = power_calculate(POWER_K_0, POWER_K_1, POWER_K_2, POWER_A, rx_data.now_torque, rx_data.now_omega); // 电机功率, W

    // NaN/inf 运行时安全钳位: 防止 Ozone Timeline 统计被污染
    if (rx_data.now_angle != rx_data.now_angle || rx_data.now_angle > 1e10f || rx_data.now_angle < -1e10f)
        rx_data.now_angle = 0.0f;
    if (rx_data.single_angle != rx_data.single_angle || rx_data.single_angle > 1e10f || rx_data.single_angle < -1e10f)
        rx_data.single_angle = 0.0f;
    if (rx_data.now_omega != rx_data.now_omega || rx_data.now_omega > 1e10f || rx_data.now_omega < -1e10f)
        rx_data.now_omega = 0.0f;
    if (rx_data.now_torque != rx_data.now_torque || rx_data.now_torque > 1e10f || rx_data.now_torque < -1e10f)
        rx_data.now_torque = 0.0f;
    if (rx_data.now_power != rx_data.now_power || rx_data.now_power > 1e10f || rx_data.now_power < -1e10f)
        rx_data.now_power = 0.0f;

    // 存储预备信息
    rx_data.pre_encoder = tmp_encoder;
}

/**
 * @brief 电机PID计算控制，使用电机反馈的原始数据进行PID闭环控制
 *
 */
void dm_motor_1_to_4_class::pid_internal_calculate()
{
    switch (motor_control_method)
    {
        case Motor_DM_Control_Method_1_To_4_Current:
        {
            break;
        }

        case Motor_DM_Control_Method_1_To_4_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(rx_data.now_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case Motor_DM_Control_Method_1_To_4_Angle:
        {
            pid_angle.set_target(target_angle);
            pid_angle.set_now(rx_data.now_angle);
            pid_angle.pid_calculate_callback();

            target_omega = pid_angle.get_out();

            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(rx_data.now_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * @brief 电机PID计算控制，使用电机滤波后的数据进行PID闭环控制
 *
 */
void dm_motor_1_to_4_class::pid_external_calculate()
{
    switch (motor_control_method)
    {
        case Motor_DM_Control_Method_1_To_4_Current:
        {
            break;
        }

        case Motor_DM_Control_Method_1_To_4_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(filter_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case Motor_DM_Control_Method_1_To_4_Angle:
        {
            pid_angle.set_target(target_angle);
            pid_angle.set_now(filter_angle);
            pid_angle.pid_calculate_callback();

            target_omega = pid_angle.get_out();

            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(filter_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        default:
        {
            target_torque = 0.0f;
            break;
        }
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 */
void dm_motor_1_to_4_class::output()
{
    *(int16_t *)tx_data = (int16_t)(out);
}
