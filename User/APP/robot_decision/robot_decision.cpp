/**
 * @file robot_decision.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief 
 * @date 2026-08-21
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */

#include "fdcan.h"
#include "freeRTOS.h"
#include "cmsis_os2.h"

#include "robot_decision.h"
#include "can_distribution.h"

// 调用can板间通讯类的接收数据解析函数
static void can_comm_board_rx_client(const volatile FDCAN_RxFrame_TypeDef *frame)
{
    (void)frame;
    robot_decision.can_board_communication.rx_packet();
}

/*--------------------------机器人决策相关数据--------------------------------*/
// 发射机构用于决策的源数据
shoot_decision_source_t shoot_decision_source;  

// 云台控制用于决策的源数据
gimbal_decision_source_t gimbal_decision_source;  

// 机器人决策对象
robot_decision_class robot_decision;  

/**
 * @brief 机器人决策初始化
 * @note 绑定遥控器对象与各状态机的决策对象指针
 */
void robot_decision_init(void)
{
    // 绑定遥控器对象指针
    robot_decision.crsf = &crsf;
    robot_decision.hi12_imu = &hi12_data;

    // 绑定各状态机的决策对象指针
    robot_decision.heat_fsm.decision = &robot_decision;
    robot_decision.fire_fsm.decision = &robot_decision;

    // 状态机初始化
    robot_decision.heat_fsm.init(Robot_Fsm_Heat_Status_Stop);
    robot_decision.fire_fsm.init(Robot_Fsm_Fire_Status_Stop);

    // 初始化MCU到PC的视觉数据传输类
    robot_decision.mcu_to_pc_aim.init();
    robot_decision.pc_to_mcu_aim.init();
    robot_decision.can_board_communication.init(&hfdcan3, 0x16, 0x6, 500);

    // 注册can板间通讯接收回调函数
    can_distribution_register(3, 0x16, can_comm_board_rx_client);  
}

/*--------------------------机器人热量检测状态机--------------------------------*/

/**
 * @brief 热量检测状态机计算回调
 * @note 重写基类虚函数, 先调用基类保持 count_time 计数, 再补充融合逻辑
 */
void robot_heat_fsm_class::fsm_calculate_callback()
{
    status[now_status_serial].count_time++;

    switch (now_status_serial)
    {
        case Robot_Fsm_Heat_Status_Stop:     // 停止状态

        {
            // 停止状态逻辑
            if (shoot_decision_source.fric_r_omega <= decision->Fric_Omega_Threshold)
            {
                // 摩擦轮电机转速超过阈值---> 进入准备状态
                set_status(Robot_Fsm_Heat_Status_Ready);
            }
            break;
        }

        case Robot_Fsm_Heat_Status_Ready:    // 准备状态

        {
            // 准备状态逻辑
            if (shoot_decision_source.fric_r_torque <= decision->Fric_Torque_Threshold)
            {
                // 电机扭矩超过阈值---> 进入怀疑状态
                set_status(Robot_Fsm_Heat_Status_Suspect);   // 发射嫌疑状态
            }

            if (shoot_decision_source.shoot_now_mode == 0) // 发射机构失能
            {
                // 发射机构失能---> 进入停止状态
                set_status(Robot_Fsm_Heat_Status_Stop);   // 停止状态
            } 

            break;
        }

        case Robot_Fsm_Heat_Status_Suspect:  // 怀疑状态(由于消除抖动，防止一发弹丸被多次重复计数)

        {
            // 怀疑状态逻辑
            if (status[now_status_serial].count_time >= decision->Fric_Torque_Confirm_Time)
            {
                // 电机扭矩持续时间超过阈值---> 进入确认状态
                set_status(Robot_Fsm_Heat_Status_Confirm);   // 发射确认状态
            }

            break;
        }

        case Robot_Fsm_Heat_Status_Confirm:  // 确认状态

        {
            // 确认状态逻辑
            now_heat += Single_Heat_Increase;  // 累加热量值
            total_ammunition_num++;  // 累加发射弹量
            set_status(Robot_Fsm_Heat_Status_Ready);   // 进入准备状态
            break;
        }
    }
    
    basic_math_constrain(&now_heat, 0.0f, decision->heat_maximum);  // 限幅热量值

    if (now_heat > 0.0f)
    {
        // 热量值自然衰减
        now_heat -= decision->heat_cd / 1000.0f;  
    }

    else
    {
        now_heat = 0.0f;
    }
}

/*--------------------------机器人单发/连发选择状态机--------------------------------*/

/**
 * @brief 机器人开火选择状态机计算回调
 */
void robot_fire_fsm_class::fsm_calculate_callback()
{
    status[now_status_serial].count_time++;

    switch (get_now_status_serial())
    {
        case Robot_Fsm_Fire_Status_Stop:     // 停止状态
        {
            if (decision->crsf->get_left_nlock() == CRSF_SWITCH_FREE_PRESS)
            {
                now_fire_status = Robot_Fsm_Fire_Status_Press;  // 停机状态---> 进入按下状态
                // 停机状态---> 进入按下状态
                set_status(Robot_Fsm_Fire_Status_Press);
            }

            break;
        }

        case Robot_Fsm_Fire_Status_Free:     // 空闲状态
        {
            if (decision->crsf->get_left_nlock() == CRSF_SWITCH_FREE_PRESS)
            {
                now_fire_status = Robot_Fsm_Fire_Status_Press;  // 空闲状态---> 进入按下状态
                // 空闲状态---> 进入按下状态
                set_status(Robot_Fsm_Fire_Status_Press);
            }

            break;
        }

        case Robot_Fsm_Fire_Status_Press:     // 按下状态
        {
            if (decision->crsf->get_left_nlock() == CRSF_SWITCH_PRESS_FREE)
            {
                now_fire_status = Robot_Fsm_Fire_Status_Single;  // 按下状态---> 进入单击状态
                // 按下->松开，单击状态
                set_status(Robot_Fsm_Fire_Status_Single);
            }

            else if(status[now_status_serial].count_time >= Repeat_Fire_Time_Threshold)
            {
                now_fire_status = Robot_Fsm_Fire_Status_Repeat;  // 按下状态---> 进入连发状态
                // 按下->按下，连发状态
                set_status(Robot_Fsm_Fire_Status_Repeat);
            }

            break;
        }

        case Robot_Fsm_Fire_Status_Single:     // 单发状态
        {
            now_fire_status = Robot_Fsm_Fire_Status_Free;  // 单发状态结束，进入空闲状态
            set_status(Robot_Fsm_Fire_Status_Free);  // 单发状态结束，进入空闲状态

            break;
        }

        case Robot_Fsm_Fire_Status_Repeat:     // 连发状态
        {
            now_fire_status = Robot_Fsm_Fire_Status_Repeat;  // 连发状态

            if (decision->crsf->get_left_nlock() == CRSF_SWITCH_PRESS_FREE)
            {
                now_fire_status = Robot_Fsm_Fire_Status_Free;  // 连发状态结束，进入空闲状态
                // 连发状态结束，进入空闲状态
                set_status(Robot_Fsm_Fire_Status_Free);
            }

            break;
        }
    }
}

/*--------------------------机器人传输数据(USB/CAN)更新(PC/底盘)--------------------------------*/

/**
 * @brief 机器人数据传输更新
 */
void robot_data_transmission_update(void)
{
    uint8_t temp_mode;
    float temp_vx, temp_vy, temp_vz, temp_yaw;

    // 视觉数据更新
    robot_decision.mcu_to_pc_aim.enemy_color_data_update(2);
    robot_decision.mcu_to_pc_aim.bullet_data_update(20.0f, robot_decision.heat_fsm.get_totol_ammunition_num());
    // robot_decision.mcu_to_pc_aim.imu_data_update(robot_decision.hi12_imu->quat_w(), robot_decision.hi12_imu->quat_x(), robot_decision.hi12_imu->quat_y(), robot_decision.hi12_imu->quat_z(), 
    //                                             robot_decision.hi12_imu->yaw() * BASIC_MATH_DEG_TO_RAD, robot_decision.hi12_imu->gyr_z() * BASIC_MATH_DEG_TO_RAD, 
    //                                             robot_decision.hi12_imu->roll()* BASIC_MATH_DEG_TO_RAD, robot_decision.hi12_imu->gyr_y() * BASIC_MATH_DEG_TO_RAD);

    robot_decision.mcu_to_pc_aim.imu_data_update(robot_decision.hi12_imu->quat_w(), robot_decision.hi12_imu->quat_x(), robot_decision.hi12_imu->quat_y(), robot_decision.hi12_imu->quat_z(), 
                                                robot_decision.hi12_imu->yaw() * BASIC_MATH_DEG_TO_RAD, gimbal_decision_source.vision_gyr_z, 
                                                robot_decision.hi12_imu->roll()* BASIC_MATH_DEG_TO_RAD, robot_decision.hi12_imu->gyr_y() * BASIC_MATH_DEG_TO_RAD);

    // 板间通讯数据更新
    switch(robot_decision.crsf->get_left_toggle())
    {
        case CRSF_SWITCH_TRIG_MID_UP:               // 中拨→上拨过渡
        case CRSF_SWITCH_UP:                        // 左拨动开关上拨
        {
            temp_mode = 0x20;   // 机器人模式: 自动模式
            break;
        }

        case CRSF_SWITCH_TRIG_UP_MID:               // 上拨→中拨过渡
        case CRSF_SWITCH_TRIG_DOWN_MID:             // 下拨→中拨过渡
        case CRSF_SWITCH_MID:                       // 左拨动开关中拨
        {
            temp_mode = 0x10;   // 机器人模式: 手动模式
            break;
        }

        case CRSF_SWITCH_TRIG_MID_DOWN:             // 中拨→下拨过渡
        case CRSF_SWITCH_DOWN:                      // 左拨动开关下拨
        {
            temp_mode = 0x00;   // 机器人模式: 无力模式
            break;
        }

        default:
        {
            temp_mode = 0x00;   // 机器人模式: 无力模式
            break;
        }
    }

    switch (robot_decision.crsf->get_right_toggle()) 
    {
        case CRSF_SWITCH_TRIG_MID_UP:               // 中拨→上拨过渡
        case CRSF_SWITCH_UP:                        // 右拨动开关上拨
        {
            temp_mode |= 0x02;   // 机器人模式: 小陀螺模式
            break;
        }

        case CRSF_SWITCH_TRIG_UP_MID:               // 上拨→中拨过渡
        case CRSF_SWITCH_TRIG_DOWN_MID:             // 下拨→中拨过渡
        case CRSF_SWITCH_MID:                       // 右拨动开关中拨
        {
            temp_mode |= 0x01;   // 机器人模式: 跟随模式
            break;
        }

        case CRSF_SWITCH_TRIG_MID_DOWN:             // 中拨→下拨过渡
        case CRSF_SWITCH_DOWN:                      // 右拨动开关下拨
        {
            temp_mode |= 0x00;   // 机器人模式: 无力模式
            break;
        }

        default:
        {
            temp_mode |= 0x00;   // 机器人模式: 无力模式
            break;
        }
    }

    temp_vx = robot_decision.crsf->get_right_x() * CHASSIS_MAX_MOVE_SPEED;  // 前进/后退速度
    temp_vy = robot_decision.crsf->get_right_y() * CHASSIS_MAX_MOVE_SPEED;  // 左移/右移速度
    temp_vz = robot_decision.crsf->get_wheel() * CHASSIS_MAX_ROTATE_SPEED;  // 旋转速度

    // 云台偏航角度
    if (gimbal_decision_source.small_yaw_angle == 0.0f || gimbal_decision_source.big_yaw_angle == 0.0f)
    {
        temp_yaw = 0.0f;
    }
    
    else
    {
        temp_yaw = basic_math_loop_constrain(gimbal_decision_source.big_yaw_angle + (gimbal_decision_source.small_yaw_angle - YAW_SMALL_MOTOR_MID_ANGLE), -PI, PI);
    }

#if YAW_RAD_MIDDLE_THAN_ZERO == 0   // 云台初始化yaw轴中值位置 <= 0.0f

    if(temp_yaw > YAW_MOTOR_MID_ANGLE && temp_yaw < (YAW_MOTOR_MID_ANGLE + PI))
    {
        robot_decision.set_gimbal_for_chassis_offset(temp_yaw - YAW_MOTOR_MID_ANGLE);
    }

    else if(temp_yaw > YAW_MOTOR_MID_ANGLE + PI)
    {
        robot_decision.set_gimbal_for_chassis_offset(temp_yaw - YAW_MOTOR_MID_ANGLE - 2.0f * PI);
    }
    
    else
    {
        robot_decision.set_gimbal_for_chassis_offset(temp_yaw - YAW_MOTOR_MID_ANGLE);
    }


#else  // 云台初始化yaw轴中值位置 > 0.0f

    if (temp_yaw > YAW_MOTOR_MID_ANGLE)
    {
        robot_decision.set_gimbal_for_chassis_offset(temp_yaw - YAW_MOTOR_MID_ANGLE);
    }

    else if (temp_yaw <= YAW_MOTOR_MID_ANGLE && temp_yaw >= (YAW_MOTOR_MID_ANGLE - PI))
    {
        robot_decision.set_gimbal_for_chassis_offset(temp_yaw - YAW_MOTOR_MID_ANGLE);
    }

    else
    {
        robot_decision.set_gimbal_for_chassis_offset(temp_yaw - YAW_MOTOR_MID_ANGLE + 2.0f * PI);
    }

#endif

    robot_decision.can_board_communication.set_control_mode(temp_mode);
    robot_decision.can_board_communication.set_vx_control(temp_vx);
    robot_decision.can_board_communication.set_vy_control(temp_vy);
    robot_decision.can_board_communication.set_vz_control(temp_vz);
    robot_decision.can_board_communication.set_rotation_matrix_yaw_control(robot_decision.get_gimbal_for_chassis_offset());

    robot_decision.mcu_to_pc_aim.transmit_data();
    robot_decision.can_board_communication.transmit_data();
}

/*--------------------------机器人传输数据解析--------------------------------*/

/**
 * @brief 机器人通讯数据解析中心
 * @note 解析USB接收的协议帧, 按CMD_ID分发到对应协议解包
 * @note 帧协议: | SOF(0x5A) | LEN | CMD_ID(小端2B) | CRC8(帧头) | DATA | CRC16(整包) |
 * @note 与usb_callback同型, 可注册给 bsp_usb_init(robot_data_transfer_analyze)
 * @param buffer USB接收到的原始数据
 * @param length 数据长度
 */
void robot_data_transfer_analyze(uint8_t *buffer, uint16_t length)
{
    if (buffer == nullptr || length == 0)
    {
        return;
    }

    // 协议常量
    const uint8_t  PC_SOF         = 0x5Au;   // 帧头
    const uint8_t  PC_HEADER_SIZE = 5u;      // SOF + LEN + CMD_ID(2B) + CRC8
    const uint8_t  PC_CRC16_SIZE  = 2u;      // 整包CRC16
    const uint16_t PC_FRAME_MAX   = (uint16_t)(PC_HEADER_SIZE + 255u + PC_CRC16_SIZE); // 单帧最大长度

    // 静态帧解析缓冲(处理跨包/半包)
    static uint8_t  parse_buf[5u + 255u + 2u];
    static uint16_t parse_len = 0;

    uint16_t idx = 0;
    while (idx < length)
    {
        // 1. 追加到解析缓冲(缓冲满仍无完整帧则丢弃重来)
        if (parse_len >= PC_FRAME_MAX)
        {
            parse_len = 0;
        }
        uint16_t space = (uint16_t)(PC_FRAME_MAX - parse_len);
        uint16_t n = ((length - idx) < space) ? (length - idx) : space;
        memcpy(&parse_buf[parse_len], &buffer[idx], n);
        parse_len += n;
        idx += n;

        // 2. 扫描并取出所有完整帧
        uint16_t offset = 0;
        while (offset < parse_len)
        {
            // 寻找帧头
            while (offset < parse_len && parse_buf[offset] != PC_SOF)
            {
                offset++;
            }
            if (offset >= parse_len)
            {
                break; // 未找到帧头
            }
            if (parse_len - offset < PC_HEADER_SIZE)
            {
                break; // 帧头不完整, 等待更多数据
            }
            // 帧头CRC8校验(SOF + LEN + CMD_ID)
            if (verify_crc8_check_sum(&parse_buf[offset], PC_HEADER_SIZE) == 0)
            {
                offset++; // 帧头校验失败, 重新找帧头
                continue;
            }
            uint8_t  len       = parse_buf[offset + 1];
            uint16_t frame_len = (uint16_t)(PC_HEADER_SIZE + len + PC_CRC16_SIZE);
            if (parse_len - offset < frame_len)
            {
                break; // 整帧不完整, 等待更多数据
            }
            // 整包CRC16校验
            if (verify_crc16_check_sum(&parse_buf[offset], frame_len) == 0)
            {
                offset++; // 整包校验失败, 重新找帧头
                continue;
            }

            // 3. 解析CMD_ID(小端)并按CMD_ID分发到对应协议解包
            uint16_t cmd_id = (uint16_t)(parse_buf[offset + 2]) | ((uint16_t)(parse_buf[offset + 3]) << 8);
            if (cmd_id == robot_decision.pc_to_mcu_aim.get_cmd_id())
            {
                // 自瞄视觉协议: 解包数据段(帧头之后)
                robot_decision.pc_to_mcu_aim.data_unpack(&parse_buf[offset + PC_HEADER_SIZE]);
            }
            // 后续新增协议在此追加 if/else 分支

            offset += frame_len; // 消费该帧
        }

        // 4. 压缩剩余未处理数据
        if (offset > 0)
        {
            uint16_t remaining = parse_len - offset;
            if (remaining > 0)
            {
                memmove(parse_buf, &parse_buf[offset], remaining);
            }
            parse_len = remaining;
        }
    }
}

/**
 * @brief 决策中心任务
 * @note 该任务主要用于机器人决策状态机的计算与更新
 * @param argument 
 */
extern "C" void decision_center_task(void *argument)
{
    // 机器人决策对象初始化
    robot_decision_init();
    osDelay(100);
    for(;;)
    {
        robot_decision.heat_fsm.fsm_calculate_callback();  // 机器人热量检测状态机计算
        robot_decision.fire_fsm.fsm_calculate_callback();  // 机器人开火选择状态机计算
        robot_data_transmission_update();  // MCU到PC的视觉数据传输更新
        robot_decision.can_board_communication.rx_unpack();  // CAN板间通讯数据解析
        osDelay(1);  // 延时1ms
    }
}

