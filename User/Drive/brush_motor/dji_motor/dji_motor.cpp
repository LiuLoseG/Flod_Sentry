#include "dji_motor.h"

#define DJI_MOTOR_NUM 10

dji_motor_t *dji_motor[DJI_MOTOR_NUM];

//局部静态变量定义当前电机注册实例序号
static uint16_t DJI_MOTORS_USED_NUM = 0;

/**
 * @brief DJI电机反馈数据处理函数
 * @note  解析接收到的电机反馈数据，并更新电机参数
 */

void dji_motor_init(dji_motor_t *p_motor, uint16_t can_id, dji_motor_type_e motor_type, dji_motor_identifier_e identifier)
{
    p_motor->identifier_id = can_id + identifier; // 电机id
    p_motor->type = motor_type;
    p_motor->first_fb_flag = true; // 首帧反馈标志置位，等待第一帧反馈作为基准

    if (p_motor->type == DJI_6020) 
    {
        p_motor->current_to_torque_ratio = DJI_6020_CURRENT_OUT;
        p_motor->motor_control_max = DJI_6020_3508_CURRENT;
    }

    else if (p_motor->type == DJI_3508)
    {
        p_motor->current_to_torque_ratio = DJI_3508_CURRENT_OUT;
        p_motor->motor_control_max = DJI_6020_3508_CURRENT;
    }

    else if (p_motor->type == DJI_2006)
    {
        p_motor->current_to_torque_ratio = DJI_2006_CURRENT_OUT;
        p_motor->motor_control_max = DJI_2006_CURRENT;
    }

    dji_motor[DJI_MOTORS_USED_NUM++] = p_motor;  //将电机添加到电机数组中

    if (DJI_MOTORS_USED_NUM > DJI_MOTOR_NUM)
    {
        DJI_MOTORS_USED_NUM = DJI_MOTOR_NUM;
    }
}

/**
 * @brief:      	dji_motor_fbdata: 处理电机反馈数据函数
 * @param[in]:   motor: 指向dji_motor_t结构体的指针，包含电机信息
 * @param[in]:   rx_data: 指向接收到的CAN数据的指针
 * @retval:     	void
 * @details:    	解析接收到的CAN数据，并更新电机反馈参数
 */

void dji_motor_fbdata(dji_motor_t* motor, uint8_t *rx_data)
{
    motor->rx_fdpara.last_ecd = motor->rx_fdpara.encoder;
    motor->rx_fdpara.encoder = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    motor->rx_fdpara.single_round_angle = DJI_ECD_TO_ANGLE * (float)(motor->rx_fdpara.encoder);
    motor->rx_fdpara.single_round_rad = DJI_ECD_TO_RAD * (float)(motor->rx_fdpara.encoder);

    motor->rx_fdpara.rotate_speed = (float)((int16_t)(rx_data[2] << 8 | rx_data[3]));
    motor->rx_fdpara.angular_speed = RPM_TO_RAD_SEC * ((float)motor->rx_fdpara.rotate_speed); // 转换为rad/s
    motor->rx_fdpara.angle_speed = RPM_TO_ANGLE_SEC * ((float)motor->rx_fdpara.rotate_speed); // 转换为度/s

    motor->rx_fdpara.electric = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    motor->rx_fdpara.temperature = (uint8_t)(rx_data[6]);

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°
    // 首帧反馈只记录基准值，跳过计圈：上电时 last_ecd/encoder 均为 0，
    // 直接比较会使 encoder - 0 超过 ±4096 而被误判为越圈
    if (motor->first_fb_flag)
    {
        motor->first_fb_flag = false;
        motor->rx_fdpara.last_ecd = motor->rx_fdpara.encoder;
    }
    else if(motor->rx_fdpara.encoder - motor->rx_fdpara.last_ecd > 4096)
    {
        motor->rx_fdpara.total_round--;
    }
    else if(motor->rx_fdpara.encoder - motor->rx_fdpara.last_ecd < -4096)
    {
        motor->rx_fdpara.total_round++;
    }

    motor->rx_fdpara.total_angle = motor->rx_fdpara.total_round * 360.0f + motor->rx_fdpara.single_round_angle;
    motor->rx_fdpara.total_rad = motor->rx_fdpara.total_round * 2.0f * PI + motor->rx_fdpara.single_round_rad;
}

/**
 * @brief:      	dji_motor_filter_fbdata: 过滤电机反馈数据函数
 * @param[in]:   motor: 指向dji_motor_t结构体的指针，包含电机信息
 * @param[in]:   filter_type: 过滤类型
 * @param[in]:   filter_value: 过滤值
 * @retval:     	void
 * @details:    	根据提供的参数过滤电机反馈数据，并更新电机滤波后的反馈参数
 */
void dji_motor_filter_fbdata(dji_motor_t* motor, dji_motor_filter_type_e filter_type, float filter_value)
{
    switch (filter_type)
    {
    case RPM_FILTER:
        motor->filter_para.filter_rotate_speed = filter_value;
        motor->filter_para.filter_angle_speed = RPM_TO_ANGLE_SEC * filter_value; // 转换为度/s
        motor->filter_para.filter_angular_speed = RPM_TO_RAD_SEC * filter_value; // rad/s
        break;

    case RAD_FILTER:

        motor->filter_para.filter_rotate_speed = RAD_TO_RPM * filter_value; // 转换为rpm
        motor->filter_para.filter_angular_speed = filter_value;
        motor->filter_para.filter_angle_speed = RAD_TO_DEGREE * filter_value; // 转换为度/s
        break;

    case ANGLE_FILTER:
        motor->filter_para.filter_rotate_speed = ANGLE_TO_RPM * filter_value; // 转换为rpm
        motor->filter_para.filter_angular_speed = DEGREE_TO_RAD * filter_value; // 转换为rad/s
        motor->filter_para.filter_angle_speed = filter_value;
        break;

    default:
        break;
    }
}

// /**
//  * @brief:      	dji_motor_ctrl: 发送电机控制命令函数
//  * @param[in]:   hfdcan: 指向FDCAN_HandleTypeDef结构体的指针，指定CAN总线
//  * @param[in]:   identifier: CAN ID标识符
//  * @param[in]:   value_1: 控制参数1
//  * @param[in]:   value_2: 控制参数2
//  * @param[in]:   value_3: 控制参数3
//  * @param[in]:   value_4: 控制参数4（仅用于特定ID）
//  * @retval:     	void
//  * @details:    	根据提供的参数构建CAN数据帧，并通过指定的CAN总线发送控制命令
//  */

// void dji_motor_ctrl(FDCAN_HandleTypeDef* hfdcan, dji_motor_identifier_e identifier, int16_t value_1, int16_t value_2, int16_t value_3, int16_t value_4)
// {
//     static uint8_t tx_data[8];

//     tx_data[0] = (uint8_t)(value_1 >> 8);
//     tx_data[1] = (uint8_t)(value_1 & 0x00FF);
//     tx_data[2] = (uint8_t)(value_2 >> 8);
//     tx_data[3] = (uint8_t)(value_2 & 0x00FF);
//     tx_data[4] = (uint8_t)(value_3 >> 8);
//     tx_data[5] = (uint8_t)(value_3 & 0x00FF);

//     //can发送ID标识符为0x2ff的数据，data[6]和data[7]为NULL,参考GM6020电机手册
//     if (identifier == DJI_L3508_L2006_FB_ID || identifier == DJI_L6020V_H3508_H2006_ID)
//     {
//         tx_data[6] = (uint8_t)(value_4 >> 8); 
//         tx_data[7] = (uint8_t)(value_4 & 0x00FF);
//     }
//     bsp_fdcan_send(hfdcan, identifier, tx_data, 8);
// }

/**
 * @brief:      	dji_motor_ctrl: 发送电机控制命令函数
 * @param[in]:   hfdcan: 指向FDCAN_HandleTypeDef结构体的指针，指定CAN总线
 * @param[in]:   motor: 指向dji_motor_t结构体的指针，指定电机
 * @param[in]:   value_1: 控制参数1
 * @param[in]:   value_2: 控制参数2
 * @param[in]:   value_3: 控制参数3
 * @param[in]:   value_4: 控制参数4（仅用于特定ID）
 * @retval:     	void
 * @details:    	根据提供的参数构建CAN数据帧，并通过指定的CAN总线发送控制命令
 */

void dji_motor_ctrl(FDCAN_HandleTypeDef* hfdcan, dji_motor_t *motor, dji_motor_identifier_e identifier, float value_1, float value_2, float value_3, float value_4)
{
    static uint8_t tx_data[8];  // 发送数据缓存

    static float control_data[4];  // 转化后的电机控制参数
    control_data[0] = basic_math_constrain(value_1 * motor->current_to_torque_ratio, -motor->motor_control_max, motor->motor_control_max);
    control_data[1] = basic_math_constrain(value_2 * motor->current_to_torque_ratio, -motor->motor_control_max, motor->motor_control_max);
    control_data[2] = basic_math_constrain(value_3 * motor->current_to_torque_ratio, -motor->motor_control_max, motor->motor_control_max);
    control_data[3] = basic_math_constrain(value_4 * motor->current_to_torque_ratio, -motor->motor_control_max, motor->motor_control_max);

    tx_data[0] = (uint8_t)((int16_t)control_data[0] >> 8);
    tx_data[1] = (uint8_t)((int16_t)control_data[0] & 0x00FF);
    tx_data[2] = (uint8_t)((int16_t)control_data[1] >> 8);
    tx_data[3] = (uint8_t)((int16_t)control_data[1] & 0x00FF);
    tx_data[4] = (uint8_t)((int16_t)control_data[2] >> 8);
    tx_data[5] = (uint8_t)((int16_t)control_data[2] & 0x00FF);

    //can发送ID标识符为0x2ff的数据，data[6]和data[7]为NULL,参考GM6020电机手册
    if (identifier == DJI_L3508_L2006_FB_ID || identifier == DJI_L6020V_H3508_H2006_ID)
    {
        tx_data[6] = (uint8_t)((int16_t)control_data[3] >> 8); 
        tx_data[7] = (uint8_t)((int16_t)control_data[3] & 0x00FF);
    }
    bsp_fdcan_send(hfdcan, identifier, tx_data, 8);
}
