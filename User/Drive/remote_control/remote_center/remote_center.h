#pragma once

#include "bsp_uart.h"
#include "crsf.h"
#include "stm32h7xx_hal_uart.h"
#include "basic_math.h"

//拨动开关状态定义,不从0开始定义是为了防止与默认值冲突，导致误判

#define CRSF_UP      (1)         //拨动开关上档
#define CRSF_MID     (2)         //拨动开关中档
#define CRSF_DOWN    (3)         //拨动开关下档

//拨动开关事件枚举
typedef enum
{
    CRSF_SWITCH_UP = 1,                //拨动开关上档
    CRSF_SWITCH_TRIG_UP_MID,           //拨动开关上档切换至中档
    CRSF_SWITCH_TRIG_MID_UP,           //拨动开关中档切换至上档
    CRSF_SWITCH_MID,                   //拨动开关中档
    CRSF_SWITCH_TRIG_MID_DOWN,         //拨动开关中档切换至下档
    CRSF_SWITCH_TRIG_DOWN_MID,         //拨动开关下档切换至中档
    CRSF_SWITCH_DOWN,                  //拨动开关下档   
}crsf_toggle_e;

//按动开关事件枚举
typedef enum
{
    CRSF_SWITCH_PRESS = 1,              //开关按下
    CRSF_SWITCH_PRESS_FREE,             //开关按下后松开
    CRSF_SWITCH_FREE_PRESS,             //开关松开后按下
    CRSF_SWITCH_FREE,                   //开关松开，机械0位置
}crsf_push_e;

typedef struct
{
    float right_x = 0.0f;           //右摇杆X轴 [-1, 1]
    float right_y = 0.0f;           //右摇杆Y轴 [-1, 1]
    float left_x = 0.0f;            //左摇杆X轴 [-1, 1]
    float left_y = 0.0f;            //左摇杆Y轴 [-1, 1]
    float wheel = 0.0f;             //滚轮 [-1, 1]

    crsf_push_e left_lock;          //左自锁开关事件
    crsf_push_e right_lock;         //右自锁开关事件

    crsf_push_e left_nlock;         //左非锁开关事件

    crsf_toggle_e left_toggle;      //左拨动开关事件
    crsf_toggle_e right_toggle;     //右拨动开关事件
} crsf_data_t;

typedef struct
{
    uint8_t left_lock;              //左自锁开关状态
    uint8_t right_lock;             //右自锁开关状态

    uint8_t left_nlock;             //左非锁开关状态

    uint8_t left_toggle;            //左拨动开关状态
    uint8_t right_toggle;           //右拨动开关状态
} crsf_switch_state_t;

typedef enum
{
    CRSF_STATUS_DISCONNECTED = 0,   //遥控器未连接
    CRSF_STATUS_CONNECTED,          //遥控器已连接
} crsf_link_status_e;

class crsf_class
{
    public:

        inline crsf_link_status_e get_connection_status();      // 获取遥控器连接状态

        inline float get_right_x();                             // 获取右摇杆X轴值
        inline float get_right_y();                             // 获取右摇杆Y轴值
        inline float get_left_x();                              // 获取左摇杆X轴值
        inline float get_left_y();                              // 获取左摇杆Y轴值
        inline float get_wheel();                               // 获取滚轮值

        inline crsf_push_e get_left_lock();                     // 获取左自锁开关状态
        inline crsf_push_e get_right_lock();                    // 获取右自锁开关状态

        inline crsf_push_e get_left_nlock();                    // 获取左非自锁开关状态

        inline crsf_toggle_e get_left_toggle();                 // 获取左拨动开关状态
        inline crsf_toggle_e get_right_toggle();                // 获取右拨动开关状态

        void crsf_alive_check();                                // 检查遥控器是否在线
        void crsf_calculate_callback();                         // 计算遥控器数据并更新状态

    protected:

        crsf_switch_state_t switch_state;   //遥控器开关状态结构体
        crsf_switch_state_t pre_state;      //上一次遥控器开关状态

        crsf_data_t switch_event;           //遥控器状态事件

        float rocker_offset = 992.0f; //摇杆偏移量，用于校准摇杆中立位置
        float rocker_scale = 800.0f;  //摇杆总刻度 

        uint32_t pre_flag = 0; //上一次的标志位，用于判断遥控器数据帧是否发生变化

        crsf_link_status_e connection_status = CRSF_STATUS_DISCONNECTED; //遥控器连接状态

        void data_process(const crsf_channels_t *channels); //处理遥控器数据

        void judge_toggle_switch(crsf_toggle_e *toggle_switch, uint8_t status, uint8_t *pre_status);
        void judge_push_switch(crsf_push_e *push_switch, uint8_t status, uint8_t *pre_status);
};

extern crsf_class crsf; // 遥控器对象实例

/**
 * @brief 获取遥控器连接状态
 * @return 遥控器连接状态
 * @note 获取遥控器连接状态。
 */
inline crsf_link_status_e crsf_class::get_connection_status()
{
    return (connection_status);
}

/**
 * @brief 获取右摇杆X轴值
 * @return 右摇杆X轴值 [-1, 1]
 */
inline float crsf_class::get_right_x()
{
    return (switch_event.right_x);
}

/**
 * @brief 获取右摇杆Y轴值
 * @return 右摇杆Y轴值 [-1, 1]
 */
inline float crsf_class::get_right_y()
{
    return (switch_event.right_y);
}


/**
 * @brief 获取左摇杆X轴值
 * @return 左摇杆X轴值 [-1, 1]
 */
inline float crsf_class::get_left_x()
{
    return (switch_event.left_x);
}

/**
 * @brief 获取左摇杆Y轴值
 * @return 左摇杆Y轴值 [-1, 1]
 */
inline float crsf_class::get_left_y()
{
    return (switch_event.left_y);
}

/**
 * @brief 获取滚轮值
 * @return 滚轮值 [-1, 1]
 */
inline float crsf_class::get_wheel()
{
    return (switch_event.wheel);
}

/**
 * @brief 获取左自锁开关状态
 * @return 左自锁开关状态
 */
inline crsf_push_e crsf_class::get_left_lock()
{
    return (switch_event.left_lock);
}

/**
 * @brief 获取右自锁开关状态
 * @return 右自锁开关状态
 */
inline crsf_push_e crsf_class::get_right_lock()
{
    return (switch_event.right_lock);
}

/**
 * @brief 获取左非锁开关状态
 * @return 左非锁开关状态
 */
inline crsf_push_e crsf_class::get_left_nlock()
{
    return (switch_event.left_nlock);
}

/**
 * @brief 获取左拨动开关状态
 * @return 左拨动开关状态
 */
inline crsf_toggle_e crsf_class::get_left_toggle()
{
    return (switch_event.left_toggle);
}

/**
 * @brief 获取右拨动开关状态
 * @return 右拨动开关状态
 */
inline crsf_toggle_e crsf_class::get_right_toggle()
{
    return (switch_event.right_toggle);
}

