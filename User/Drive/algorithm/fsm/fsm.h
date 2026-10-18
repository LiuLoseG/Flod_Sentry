/**
 * @file fsm.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 有限自动机
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-15 1.1 修改为模板类更易于适配
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "basic_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 状态所处的阶段
 *
 */
enum Enum_Status_Stage
{
    Status_Stage_DISABLE = 0,
    Status_Stage_ENABLE,
};

/**
 * @brief 状态结构体
 *
 */
struct struct_status
{
    Enum_Status_Stage status_stage;
    uint32_t count_time;
};

/**
 * @brief Reusable, 有限自动机核心, 一般有时间需求的则采用有限自动机
 * 使用时请继承->声明友元后使用
 *
 */
template<uint8_t status_max = 10>
class fsm_class
{
public:
    struct_status status[status_max];

    void init(const uint8_t &__now_status_serial = 0);

    inline uint8_t get_now_status_serial() const;

    inline void set_status(const uint8_t &next_status_serial);

    // 使用虚函数在基类中定义一个空的状态转移函数, 在子类中重写这个函数以实现不同的状态转移逻辑
    virtual void fsm_calculate_callback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 读变量

    // FSM当前状态
    uint8_t now_status_serial = 0;

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 状态机初始化
 *
 * @tparam status_max 状态机最大状态数量
 * @param __now_status_serial 当前指定状态机初始编号
 */
template<uint8_t status_max>
void fsm_class<status_max>::init(const uint8_t &__now_status_serial)
{
    now_status_serial = __now_status_serial;

    // 所有状态全刷0
    for (int i = 0; i < status_max; i++)
    {
        status[i].status_stage = Status_Stage_DISABLE;
        status[i].count_time = 0;
    }

    // 使能初始状态
    status[__now_status_serial].status_stage = Status_Stage_ENABLE;
}

/**
 * @brief 定时器处理函数, 计算周期与模型有关
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 * @tparam status_max 状态机最大状态数量
 */
template<uint8_t status_max>
void fsm_class<status_max>::fsm_calculate_callback()
{
    status[now_status_serial].count_time++;

    // 自己接着编写状态转移函数
}

/**
 * @brief 获取FSM当前状态
 *
 * @tparam status_max 状态机最大状态数量
 * @return uint8_t FSM当前状态
 */
template<uint8_t status_max>
inline uint8_t fsm_class<status_max>::get_now_status_serial() const
{
    return (now_status_serial);
}

/**
 * @brief 设置状态改变
 *
 * @tparam status_max 状态机最大状态数量
 * @param next_status_serial 下一个状态
 */
template<uint8_t status_max>
inline void fsm_class<status_max>::set_status(const uint8_t &next_status_serial)
{
    // 失能当前状态, 计数器清零
    status[now_status_serial].status_stage = Status_Stage_DISABLE;
    status[now_status_serial].count_time = 0;

    // 转到下一个状态
    status[next_status_serial].status_stage = Status_Stage_ENABLE;
    now_status_serial = next_status_serial;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/