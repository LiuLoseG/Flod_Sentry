#include "freertos.h"
#include "cmsis_os2.h"
#include "remote_center.h"

crsf_class crsf;

void crsf_class::data_process(const crsf_channels_t *channels)
{
    if (channels == nullptr)
    {
        return;
    }

    //使用局部变量，临时存储遥控器通道数据，防止按键检测中途，UART中断修改遥控器数据时，导致两次读取数据时域混乱。
    uint16_t part_ch[10] = {0};

    for (uint8_t i = 0; i < 10; i++)
    {
        if (channels->ch[i] == 0)
        {
            return;  // 通道数据为0，视为无效帧，舍弃整帧数据
        }
        part_ch[i] = channels->ch[i];
    }

    /*------------------------------拨杆和滚轮通道映射------------------------------------*/
    switch_event.left_x = float(basic_math_constrain(part_ch[0] - rocker_offset, -rocker_scale, rocker_scale)) / rocker_scale;
    // 采用右手坐标系，对通道的y值取反
    switch_event.left_y = float(basic_math_constrain(-part_ch[1] + rocker_offset, -rocker_scale, rocker_scale)) / rocker_scale;
    switch_event.right_x = float(basic_math_constrain(part_ch[2] - rocker_offset, -rocker_scale, rocker_scale)) / rocker_scale;

    // 采用右手坐标系，对通道的y值取反
    switch_event.right_y = float(basic_math_constrain(-part_ch[3] + rocker_offset, -rocker_scale, rocker_scale)) / rocker_scale;
    // 采用右手坐标系，对左右方向拨杆通道的y值取反
    switch_event.wheel = float(basic_math_constrain(-part_ch[6] + rocker_offset, -rocker_scale, rocker_scale)) / rocker_scale;

    // 处理拨动开关和按动开关状态
    /*------------------------------右拨动开关状态检测--------------------------------------*/
    if (part_ch[5] == CRSF_REMOTE_CONTROL_MAX) 
    {
        switch_state.right_toggle = CRSF_UP;
    }

    else if (part_ch[5] == CRSF_REMOTE_CONTROL_MID) 
    {
        switch_state.right_toggle = CRSF_MID;
    }

    else if (part_ch[5] == CRSF_REMOTE_CONTROL_MIN) 
    {
        switch_state.right_toggle = CRSF_DOWN;
    
    }

    /*------------------------------左拨动开关状态检测--------------------------------------*/

    if (part_ch[8] == CRSF_REMOTE_CONTROL_MAX) 
    {
        switch_state.left_toggle = CRSF_UP;
    }

    else if (part_ch[8] == CRSF_REMOTE_CONTROL_MID) 
    {
        switch_state.left_toggle = CRSF_MID;
    }

    else if (part_ch[8] == CRSF_REMOTE_CONTROL_MIN) 
    {
        switch_state.left_toggle = CRSF_DOWN;
    
    }

    /*------------------------------右自锁开关状态检测--------------------------------------*/
   if (part_ch[4] == CRSF_REMOTE_CONTROL_MAX) 
    {
        switch_state.right_lock = CRSF_UP;
    }

    else if (part_ch[4] == CRSF_REMOTE_CONTROL_MIN) 
    {
        switch_state.right_lock = CRSF_DOWN;
    }

    /*------------------------------左自锁开关状态检测--------------------------------------*/
    if (part_ch[7] == CRSF_REMOTE_CONTROL_MAX) 
    {
        switch_state.left_lock = CRSF_UP;
    }

    else if (part_ch[7] == CRSF_REMOTE_CONTROL_MIN) 
    {
        switch_state.left_lock = CRSF_DOWN;
    }
    /*------------------------------左非自锁开关状态检测--------------------------------------*/

    if (part_ch[9] == CRSF_REMOTE_CONTROL_MAX) 
    {
        switch_state.left_nlock = CRSF_UP;
    }

    else if (part_ch[9] == CRSF_REMOTE_CONTROL_MIN) 
    {
        switch_state.left_nlock = CRSF_DOWN;
    }

    // 更新上一次的通道数据
}

void crsf_class::judge_toggle_switch(crsf_toggle_e *toggle_switch, uint8_t status, uint8_t *pre_status)
{
    switch (*pre_status) 
    {
        case CRSF_UP:
            switch (status) 
            {
                case CRSF_UP:
                    *toggle_switch = CRSF_SWITCH_UP;            // 上档保持不变
                    break;
                case CRSF_MID:
                    *toggle_switch = CRSF_SWITCH_TRIG_UP_MID;   // 上档切换至中档
                    break;
                case CRSF_DOWN:
                    *toggle_switch = CRSF_SWITCH_TRIG_MID_DOWN; // 中档切换至下档
                    break;
                default:
                    break;
            }
            break;

        case CRSF_MID:
            switch (status)
            {
                case CRSF_UP:
                    *toggle_switch = CRSF_SWITCH_TRIG_MID_UP;   // 中档切换至上档
                    break;
                case CRSF_MID:
                    *toggle_switch = CRSF_SWITCH_MID;           // 中档保持不变
                    break;
                case CRSF_DOWN:
                    *toggle_switch = CRSF_SWITCH_TRIG_MID_DOWN; // 中档切换至下档
                    break;
                default:
                    break;
            }
            break;

        case CRSF_DOWN:
            switch (status)
            {
                case CRSF_UP:
                    *toggle_switch = CRSF_SWITCH_TRIG_MID_UP;   // 中档切换至上档
                    break;
                case CRSF_MID:
                    *toggle_switch = CRSF_SWITCH_TRIG_DOWN_MID; // 下档切换至中档
                    break;
                case CRSF_DOWN:
                    *toggle_switch = CRSF_SWITCH_DOWN;          // 下档保持不变
                    break;
                default:
                    break;
            }
            break;
    }
}

void crsf_class::judge_push_switch(crsf_push_e *push_switch, uint8_t status, uint8_t *pre_status)
{
    switch (*pre_status) 
    {
        case CRSF_UP:
            switch (status) 
            {
                case CRSF_UP:
                    *push_switch = CRSF_SWITCH_FREE;           // 松开保持不变
                    break;
                case CRSF_DOWN:
                    *push_switch = CRSF_SWITCH_PRESS_FREE;      // 按下后松开
                    break;
                default:
                    break;
            }
            break;

        case CRSF_DOWN:
            switch (status)
            {
                case CRSF_UP:
                    *push_switch = CRSF_SWITCH_FREE_PRESS;      // 松开后按下
                    break;
                case CRSF_DOWN:
                    *push_switch = CRSF_SWITCH_PRESS;            // 按下保持不变
                    break;
                default:
                    break;
            }
            break;
    }
}

/**
 * @brief 检查遥控器是否在线
 * @note 检查遥控器是否在线。
 */
void crsf_class::crsf_alive_check()
{
    if (rc_channels.link_status == pre_flag)
    {
        connection_status = CRSF_STATUS_DISCONNECTED; // 遥控器未连接
    }
    
    else
    {
        connection_status = CRSF_STATUS_CONNECTED; // 遥控器已连接
    }

    pre_flag = rc_channels.link_status; // 更新上一次的标志位
}

void crsf_class::crsf_calculate_callback()
{
    data_process(&rc_channels);
    judge_toggle_switch(&switch_event.left_toggle, switch_state.left_toggle, &pre_state.left_toggle);
    judge_toggle_switch(&switch_event.right_toggle, switch_state.right_toggle, &pre_state.right_toggle);
    judge_push_switch(&switch_event.left_lock, switch_state.left_lock, &pre_state.left_lock);
    judge_push_switch(&switch_event.right_lock, switch_state.right_lock, &pre_state.right_lock);
    judge_push_switch(&switch_event.left_nlock, switch_state.left_nlock, &pre_state.left_nlock);

    memcpy(&pre_state, &switch_state, sizeof(crsf_switch_state_t));
}

/**
 * @brief 遥控器任务函数
 * @param argument 任务参数
 */
extern "C" void remote_control_task(void *argument)
{
    osDelay(1000); // 延时1秒，等待系统初始化完成
    for(;;)
    {
        crsf.crsf_calculate_callback();
        osDelay(1);
    }
}