/**
 * @file alg_queue.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief 静态内存队列结构
 * @version 0.1
 * @date 2024-07-15 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2024
 *
 */

#pragma once

/* Includes ------------------------------------------------------------------*/

#include "basic_math.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, 循环队列本体
 *
 */
template<typename type, uint32_t max_size = 200>
class class_queue
{
public:
    void init();

    inline uint32_t get_length() const;

    inline type get_front() const;

    inline type get_rear() const;

    inline void push(const type &__data);

    inline type pop();

    inline void clear();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量

    // 队列本体
    type queue[max_size];

    // 队首元素位置
    uint32_t front = 0;
    // 队尾元素位置
    uint32_t rear = 0;

    // 读变量

    // 队列长度
    uint32_t length = 0;

    // 写变量

    // 读写变量

    // 内部函数
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化
 *
 * @tparam Type 类型
 * @tparam Max_Size 队列长度最大值
 */
template<typename type, uint32_t max_size>
void class_queue<type, max_size>::init()
{
    front = 0;
    rear = 0;
    length = 0;
}

/**
* @brief 获取队列长度
 *
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @return 队首元素
*/
template<typename type, uint32_t max_size>
inline uint32_t class_queue<type, max_size>::get_length() const
{
    return (length);
}

/**
* @brief 获取队首元素
 *
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @return 队首元素
*/
template<typename type, uint32_t max_size>
inline type class_queue<type, max_size>::get_front() const
{
    return (queue[front]);
}

/**
 * @brief 获取队尾元素
 *
 * @tparam Type 类型
 * @tparam Max_Size 队列长度最大值
 * @return 队尾元素
 */
template<typename type, uint32_t max_size>
inline type class_queue<type, max_size>::get_rear() const
{
    return (queue[rear]);
}

/**
* @brief 送入队列
*
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @param __Data 传入值
*/
template<typename type, uint32_t max_size>
inline void class_queue<type, max_size>::push(const type &__data)
{
    if (length == max_size)
    {
        return;
    }
    else
    {
        if (length == 0)
        {
            front = 0;
            rear = 0;
        }
        else
        {
            rear = (rear + 1) % max_size;
        }

        queue[rear] = __data;
        length++;
    }
}

/**
* @brief 弹出队列
*
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
* @return 弹出值
*/
template<typename type, uint32_t max_size>
inline type class_queue<type, max_size>::pop()
{
    type temp = queue[front];

    if (length == 0)
    {
    }
    else
    {
        front = (front + 1) % max_size;
        length--;
    }

    return (temp);
}

/**
* @brief 清空队列
*
* @tparam Type 类型
* @tparam Max_Size 队列长度最大值
*/
template<typename type, uint32_t max_size>
inline void class_queue<type, max_size>::clear()
{
    front = 0;
    rear = 0;
    length = 0;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/