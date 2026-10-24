#include "dji_motor_new.h"

static uint8_t *allocate_can_tx_data(const FDCAN_HandleTypeDef *hcan, const dji_motor_id_e &__can_rx_id, const dji_motor_gm6020_driver_version_e &__dji_motor_driver_version = Motor_DJI_GM6020_Driver_Version_Default)
{
    uint8_t *tmp_tx_data_ptr = nullptr;

    if(hcan == &hfdcan1)
    {
        switch (__can_rx_id)
        {
            case Motor_DJI_ID_0x201:
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[0]);
                break;
            }

            case Motor_DJI_ID_0x202:
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[2]);
                break;
            }

            case Motor_DJI_ID_0x203:
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[4]);
                break;
            }

            case Motor_DJI_ID_0x204:
            {
                tmp_tx_data_ptr = &(can1_0x200_tx_data[6]);
                break;
            }

            case Motor_DJI_ID_0x205:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can1_0x1ff_tx_data[0]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can1_0x1fe_tx_data[0]);
                }

                break;
            }

            case Motor_DJI_ID_0x206:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can1_0x1ff_tx_data[2]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can1_0x1fe_tx_data[2]);
                }

                break;
            }

            case Motor_DJI_ID_0x207:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can1_0x1ff_tx_data[4]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can1_0x1fe_tx_data[4]);
                }

                break;
            }

            case Motor_DJI_ID_0x208:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can1_0x1ff_tx_data[6]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can1_0x1fe_tx_data[6]);
                }

                break;
            }

            case Motor_DJI_ID_0x209:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can1_0x2ff_tx_data[0]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can1_0x2fe_tx_data[0]);
                }

                break;
            }

            case Motor_DJI_ID_0x20A:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can1_0x2ff_tx_data[2]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can1_0x2fe_tx_data[2]);
                }

                break;
            }

            case Motor_DJI_ID_0x20B:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can1_0x2ff_tx_data[4]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can1_0x2fe_tx_data[4]);
                }

                break;
            }
        }
    }

    else if(hcan == &hfdcan2)
    {
        switch (__can_rx_id)
        {
            case Motor_DJI_ID_0x201:
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[0]);
                break;
            }

            case Motor_DJI_ID_0x202:
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[2]);
                break;
            }

            case Motor_DJI_ID_0x203:
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[4]);
                break;
            }

            case Motor_DJI_ID_0x204:
            {
                tmp_tx_data_ptr = &(can2_0x200_tx_data[6]);
                break;
            }

            case Motor_DJI_ID_0x205:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can2_0x1ff_tx_data[0]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can2_0x1fe_tx_data[0]);
                }

                break;
            }

            case Motor_DJI_ID_0x206:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can2_0x1ff_tx_data[2]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can2_0x1fe_tx_data[2]);
                }

                break;
            }

            case Motor_DJI_ID_0x207:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can2_0x1ff_tx_data[4]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can2_0x1fe_tx_data[4]);
                }

                break;
            }

            case Motor_DJI_ID_0x208:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can2_0x1ff_tx_data[6]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can2_0x1fe_tx_data[6]);
                }

                break;
            }

            case Motor_DJI_ID_0x209:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can2_0x2ff_tx_data[0]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can2_0x2fe_tx_data[0]);
                }

                break;
            }

            case Motor_DJI_ID_0x20A:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can2_0x2ff_tx_data[2]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can2_0x2fe_tx_data[2]);
                }

                break;
            }

            case Motor_DJI_ID_0x20B:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can2_0x2ff_tx_data[4]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can2_0x2fe_tx_data[4]);
                }

                break;
            }
        }
    }

    else if(hcan == &hfdcan3)
    {
        switch (__can_rx_id)
        {
            case Motor_DJI_ID_0x201:
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[0]);
                break;
            }

            case Motor_DJI_ID_0x202:
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[2]);
                break;
            }

            case Motor_DJI_ID_0x203:
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[4]);
                break;
            }

            case Motor_DJI_ID_0x204:
            {
                tmp_tx_data_ptr = &(can3_0x200_tx_data[6]);
                break;
            }

            case Motor_DJI_ID_0x205:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can3_0x1ff_tx_data[0]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can3_0x1fe_tx_data[0]);
                }

                break;
            }

            case Motor_DJI_ID_0x206:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can3_0x1ff_tx_data[2]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can3_0x1fe_tx_data[2]);
                }

                break;
            }

            case Motor_DJI_ID_0x207:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can3_0x1ff_tx_data[4]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can3_0x1fe_tx_data[4]);
                }

                break;
            }

            case Motor_DJI_ID_0x208:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can3_0x1ff_tx_data[6]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can3_0x1fe_tx_data[6]);
                }

                break;
            }

            case Motor_DJI_ID_0x209:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can3_0x2ff_tx_data[0]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can3_0x2fe_tx_data[0]);
                }

                break;
            }

            case Motor_DJI_ID_0x20A:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can3_0x2ff_tx_data[2]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can3_0x2fe_tx_data[2]);
                }

                break;
            }

            case Motor_DJI_ID_0x20B:
            {
                if (__dji_motor_driver_version == Motor_DJI_GM6020_Driver_Version_Default)
                {
                    tmp_tx_data_ptr = &(can3_0x2ff_tx_data[4]);
                }
                else
                {
                    tmp_tx_data_ptr = &(can3_0x2fe_tx_data[4]);
                }

                break;
            }
        }
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

/*------------------------------------------------------DJI_GM6020----------------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param __can_rx_id 绑定的CAN ID
 * @param __motor_dji_control_method 电机控制方式, 默认角度
 * @param __encoder_offset 编码器偏移, 默认0
 * @param __nearest_angle 就近转位的单次最大旋转角度, 其数值一般为圆周的整数倍或纯小数倍, 且纯小数倍时可均分圆周, 0表示不启用就近转位
 */
void dji_gm6020_motor_class::init(const FDCAN_HandleTypeDef *hcan, const dji_motor_id_e &__can_rx_id, const dji_motor_control_method_e &__motor_dji_control_method, const int32_t &__encoder_offset, const float &__nearest_angle)
{
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
    control_method = __motor_dji_control_method;
    encoder_offset = __encoder_offset;
    nearest_angle = __nearest_angle;
    first_fb_flag = true;   // 复位/上电后首帧建立编码器基准, 防止误计圈
    tx_data = allocate_can_tx_data(hcan, __can_rx_id, Motor_DJI_GM6020_Driver_Version_6020);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void dji_gm6020_motor_class::can_rx_callback()
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
void dji_gm6020_motor_class::alive_check()
{
    // 判断该时间段内是否接收过电机数据
    if (flag == pre_flag)
    {
        // 电机断开连接
        motor_status = Motor_DJI_Status_Disable;
        pid_angle.set_integral_error(0.0f);
        pid_omega.set_integral_error(0.0f);
    }
    else
    {
        // 电机连接正常
        motor_status = Motor_DJI_Status_Enable;
    }
    pre_flag = flag;
}

/**
 * @brief 电机原始传感器反馈数据计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void dji_gm6020_motor_class::internal_calculate_callback()
{
    pid_internal_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_omega = 0.0f;
    feedforward_torque = 0.0f;
}

/**
 * @brief 电机传感器滤波后数据计算回调函数, 计算周期取决于滤波反馈数据周期
 *
 */
void dji_gm6020_motor_class::external_calculate_callback()
{
    pid_external_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_omega = 0.0f;
    feedforward_torque = 0.0f;
}

/**
    @brief DJI_6020电机反馈数据报文处理
 */
void dji_gm6020_motor_class::data_process()
{
    static int16_t delta_encoder;
    static uint16_t tmp_encoder;
    static int16_t tmp_omega, tmp_current;

    dji_motor_source_data_t *tmp_buffer = (dji_motor_source_data_t *)can_rx_frame->data;

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

    // 电机转子原始数据
    rx_data.rotor_angle = (float)(rx_data.total_encoder) / (float)(Encoder_Num_Per_Round) * 2.0f * PI; // 转子角度, rad
    rx_data.rotor_omega = (float)(tmp_omega) * BASIC_MATH_RPM_TO_RADPS;                     // 转子角速度, rad/s
    rx_data.rotor_current = (float)(tmp_current);     // 转子电流, A

    // 电机输出轴转换数据
    rx_data.now_angle = rx_data.rotor_angle; // 减速箱减速前电机角度, rad
    // 电机单圈角度, rad, 归一化到 (-PI, PI]
    rx_data.single_angle = basic_math_loop_constrain((float)tmp_encoder / (float)(Encoder_Num_Per_Round) * 2.0f * PI, -PI, PI);
    rx_data.now_omega = rx_data.rotor_omega;                               // 电机角速度, rad/s
    rx_data.now_torque = rx_data.rotor_current / Current_To_Out * Current_To_Torque; // 电机扭矩, Nm
    rx_data.now_temperature = (float)(tmp_buffer->temperature); // 电机温度
    rx_data.now_power = power_calculate(POWER_K_0, POWER_K_1, POWER_K_2, POWER_A, rx_data.now_torque, rx_data.now_omega); // 电机功率, W

    // NaN/inf 运行时安全钳位: 防止 Ozone Timeline 统计被污染
    if (rx_data.now_angle != rx_data.now_angle) rx_data.now_angle = 0.0f;
    if (rx_data.single_angle != rx_data.single_angle) rx_data.single_angle = 0.0f;
    if (rx_data.now_omega != rx_data.now_omega) rx_data.now_omega = 0.0f;
    if (rx_data.now_torque != rx_data.now_torque) rx_data.now_torque = 0.0f;
    if (rx_data.now_power != rx_data.now_power) rx_data.now_power = 0.0f;

    // 存储预备信息
    rx_data.pre_encoder = tmp_encoder;
}

/**
 * @brief 电机PID计算控制，使用电机反馈的原始数据进行PID闭环控制
 *
 */
void dji_gm6020_motor_class::pid_internal_calculate()
{
   switch (control_method)
   {
        case DJI_Motor_Control_Method_Torque:
        {
            break;
        }

        case DJI_Motor_Control_Method_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(rx_data.now_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case DJI_Motor_Control_Method_Angle:
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
            target_torque = 0.0f;
            break;
        }
   }
}

/**
 * @brief 电机PID计算控制，使用电机滤波后的数据进行PID闭环控制
 *
 */
void dji_gm6020_motor_class::pid_external_calculate()
{
   switch (control_method)
   {
        case DJI_Motor_Control_Method_Torque:
        {
            break;
        }

        case DJI_Motor_Control_Method_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(filter_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case DJI_Motor_Control_Method_Angle:
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
void dji_gm6020_motor_class::output()
{
    tx_data[0] = (uint8_t)((int16_t)(out) >> 8);
    tx_data[1] = (uint8_t)((int16_t)(out) & 0x00ff);
}

/*------------------------------------------------------DJI_C610_GM2006----------------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __can_rx_id CAN ID
 * @param __motor_dji_control_method 电机控制方式, 默认角度
 * @param __gearbox_rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __current_max 最大电流
 */
void dji_c610_motor_class::init(const FDCAN_HandleTypeDef *hcan, const dji_motor_id_e &__can_rx_id, const dji_motor_control_method_e &__motor_dji_control_method, const float &__gearbox_rate)
{
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
    control_method = __motor_dji_control_method;
    gearbox_rate = __gearbox_rate;
    first_fb_flag = true;   // 复位/上电后首帧建立编码器基准, 防止误计圈
    tx_data = allocate_can_tx_data(hcan, __can_rx_id);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void dji_c610_motor_class::can_rx_callback()
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
void dji_c610_motor_class::alive_check()
{
    // 判断该时间段内是否接收过电机数据
    if (flag == pre_flag)
    {
        // 电机断开连接
        motor_status = Motor_DJI_Status_Disable;
        pid_angle.set_integral_error(0.0f);
        pid_omega.set_integral_error(0.0f);
    }
    else
    {
        // 电机连接正常
        motor_status = Motor_DJI_Status_Enable;
    }
    pre_flag = flag;
}

/**
 * @brief 电机原始传感器反馈数据计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void dji_c610_motor_class::internal_calculate_callback()
{
    pid_internal_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque / gearbox_rate * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_omega = 0.0f;
    feedforward_torque = 0.0f;
}

/**
 * @brief 电机传感器滤波后数据计算回调函数, 计算周期取决于滤波反馈数据周期
 *
 */
void dji_c610_motor_class::external_calculate_callback()
{
    pid_external_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque / gearbox_rate * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_omega = 0.0f;
    feedforward_torque = 0.0f;
}

/**
    @brief DJI_2006电机反馈数据报文处理
 */
void dji_c610_motor_class::data_process()
{
    static int16_t delta_encoder;
    static uint16_t tmp_encoder;
    static int16_t tmp_omega, tmp_current;

    dji_motor_source_data_t *tmp_buffer = (dji_motor_source_data_t *)can_rx_frame->data;

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

    rx_data.total_encoder = rx_data.total_round * Encoder_Num_Per_Round + tmp_encoder;

    // 计算电机本身信息

    // 电机转子反馈数据信息
    rx_data.rotor_angle = (float)(rx_data.total_encoder) / (float)(Encoder_Num_Per_Round) * 2.0f * PI; // 转子角度, rad
    rx_data.rotor_omega = (float)(tmp_omega) * BASIC_MATH_RPM_TO_RADPS;                     // 转子角速度, rad/s
    rx_data.rotor_current = (float)(tmp_current);                                           // 转子电流, A

    // 电机输出轴转换数据
    rx_data.now_angle = rx_data.rotor_angle / gearbox_rate; // 减速箱减速前电机角度, rad
    // 电机单圈角度, rad, 归一化到 (-PI/gearbox_rate, PI/gearbox_rate]
    rx_data.single_angle = basic_math_loop_constrain((float)tmp_encoder / (float)(Encoder_Num_Per_Round) / gearbox_rate * 2.0f * PI, -PI / gearbox_rate, PI / gearbox_rate);
    rx_data.now_omega = rx_data.rotor_omega / gearbox_rate;                               // 电机角速度, rad/s
    rx_data.now_torque = rx_data.rotor_current / Current_To_Out * Current_To_Torque * gearbox_rate; // 电机扭矩, Nm
    rx_data.now_temperature = (float) (tmp_buffer->temperature); // 电机温度

    // NaN/inf 运行时安全钳位: 防止 Ozone Timeline 统计被污染
    if (rx_data.now_angle != rx_data.now_angle || rx_data.now_angle > 1e10f || rx_data.now_angle < -1e10f)
        rx_data.now_angle = 0.0f;
    if (rx_data.single_angle != rx_data.single_angle || rx_data.single_angle > 1e10f || rx_data.single_angle < -1e10f)
        rx_data.single_angle = 0.0f;
    if (rx_data.now_omega != rx_data.now_omega || rx_data.now_omega > 1e10f || rx_data.now_omega < -1e10f)
        rx_data.now_omega = 0.0f;
    if (rx_data.now_torque != rx_data.now_torque || rx_data.now_torque > 1e10f || rx_data.now_torque < -1e10f)
        rx_data.now_torque = 0.0f;

    // 存储预备信息
    rx_data.pre_encoder = tmp_encoder;
}

/**
 * @brief 电机PID计算控制，使用电机反馈的原始数据进行PID闭环控制
 *
 */
void dji_c610_motor_class::pid_internal_calculate()
{
   switch (control_method)
   {
        case DJI_Motor_Control_Method_Torque:
        {
            break;
        }

        case DJI_Motor_Control_Method_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(rx_data.now_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case DJI_Motor_Control_Method_Angle:
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
            target_torque = 0.0f;
            break;
        }
   }
}

/**
 * @brief 电机PID计算控制，使用电机滤波后的数据进行PID闭环控制
 *
 */
void dji_c610_motor_class::pid_external_calculate()
{
   switch (control_method)
   {
        case DJI_Motor_Control_Method_Torque:
        {
            break;
        }

        case DJI_Motor_Control_Method_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(filter_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case DJI_Motor_Control_Method_Angle:
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
void dji_c610_motor_class::output()
{
    tx_data[0] = (uint8_t)((int16_t)(out) >> 8);
    tx_data[1] = (uint8_t)((int16_t)(out) & 0x00ff);
}

/*------------------------------------------------------DJI_C620_GM3508----------------------------------------------------------------*/

/**
 * @brief 电机初始化
 *
 * @param hcan CAN编号
 * @param __can_rx_id CAN ID
 * @param __motor_dji_control_method 电机控制方式, 默认角度
 * @param __gearbox_rate 减速箱减速比, 默认为原装减速箱, 如拆去减速箱则该值设为1
 * @param __current_max 最大电流
 */
void dji_c620_motor_class::init(const FDCAN_HandleTypeDef *hcan, const dji_motor_id_e &__can_rx_id, const dji_motor_control_method_e &__motor_dji_control_method, const float &__gearbox_rate)
{
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
    control_method = __motor_dji_control_method;
    gearbox_rate = __gearbox_rate;
    first_fb_flag = true;   // 复位/上电后首帧建立编码器基准, 防止误计圈
    tx_data = allocate_can_tx_data(hcan, __can_rx_id);
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void dji_c620_motor_class::can_rx_callback()
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
void dji_c620_motor_class::alive_check()
{
    // 判断该时间段内是否接收过电机数据
    if (flag == pre_flag)
    {
        // 电机断开连接
        motor_status = Motor_DJI_Status_Disable;
        pid_angle.set_integral_error(0.0f);
        pid_omega.set_integral_error(0.0f);
    }
    else
    {
        // 电机连接正常
        motor_status = Motor_DJI_Status_Enable;
    }
    pre_flag = flag;
}

/**
 * @brief 电机原始传感器反馈数据计算回调函数, 计算周期取决于电机反馈周期
 *
 */
void dji_c620_motor_class::internal_calculate_callback()
{
    pid_internal_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque / gearbox_rate * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_omega = 0.0f;
    feedforward_torque = 0.0f;
}

/**
 * @brief 电机传感器滤波后数据计算回调函数, 计算周期取决于滤波反馈数据周期
 *
 */
void dji_c620_motor_class::external_calculate_callback()
{
    pid_external_calculate();

    out = (target_torque + feedforward_torque) / Current_To_Torque / gearbox_rate * Current_To_Out;
    basic_math_constrain(&out, -Out_Max, Out_Max);

    output();

    feedforward_omega = 0.0f;
    feedforward_torque = 0.0f;
}

/**
    @brief DJI_3508电机反馈数据报文处理
 */
void dji_c620_motor_class::data_process()
{
    static int16_t delta_encoder;
    static uint16_t tmp_encoder;
    static int16_t tmp_omega, tmp_current;

    dji_motor_source_data_t *tmp_buffer = (dji_motor_source_data_t *)can_rx_frame->data;

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

    rx_data.total_encoder = rx_data.total_round * Encoder_Num_Per_Round + tmp_encoder;

    // 计算电机本身信息

    // 电机转子反馈数据信息
    rx_data.rotor_angle = (float)(rx_data.total_encoder) / (float)(Encoder_Num_Per_Round) * 2.0f * PI; // 转子角度, rad
    rx_data.rotor_omega = (float)(tmp_omega) * BASIC_MATH_RPM_TO_RADPS;                     // 转子角速度, rad/s
    rx_data.rotor_current = (float)(tmp_current);                                           // 转子电流, A

    // 电机输出轴转换数据
    rx_data.now_angle = rx_data.rotor_angle / gearbox_rate; // 减速箱减速前电机角度, rad
    // 电机单圈角度, rad, 归一化到 (-PI/gearbox_rate, PI/gearbox_rate]
    rx_data.single_angle = basic_math_loop_constrain((float)tmp_encoder / (float)(Encoder_Num_Per_Round) / (float)(gearbox_rate) * 2.0f * PI, -PI / (float)(gearbox_rate), PI / (float)(gearbox_rate));
    rx_data.now_omega = rx_data.rotor_omega / gearbox_rate;                               // 电机角速度, rad/s
    rx_data.now_torque = (float)(tmp_current) / Current_To_Out * Current_To_Torque * gearbox_rate; // 电机扭矩, Nm
    rx_data.now_temperature = (float)(tmp_buffer->temperature); // 电机温度
    rx_data.now_power = power_calculate(POWER_K_0, POWER_K_1, POWER_K_2, POWER_A, rx_data.now_torque / gearbox_rate, rx_data.now_omega * gearbox_rate); // 电机功率, W

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
void dji_c620_motor_class::pid_internal_calculate()
{
   switch (control_method)
   {
        case DJI_Motor_Control_Method_Torque:
        {
            break;
        }

        case DJI_Motor_Control_Method_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(rx_data.now_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case DJI_Motor_Control_Method_Angle:
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
            target_torque = 0.0f;
            break;
        }
   }
}

/**
 * @brief 电机PID计算控制，使用电机滤波后的数据进行PID闭环控制
 *
 */
void dji_c620_motor_class::pid_external_calculate()
{
   switch (control_method)
   {
        case DJI_Motor_Control_Method_Torque:
        {
            break;
        }

        case DJI_Motor_Control_Method_Omega:
        {
            pid_omega.set_target(target_omega + feedforward_omega);
            pid_omega.set_now(filter_omega);
            pid_omega.pid_calculate_callback();

            target_torque = pid_omega.get_out();

            break;
        }

        case DJI_Motor_Control_Method_Angle:
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
void dji_c620_motor_class::output()
{
    tx_data[0] = (uint8_t)((int16_t)(out) >> 8);
    tx_data[1] = (uint8_t)((int16_t)(out) & 0x00ff);
}