/**
 * @file regular_key.cpp
 * @author HNAU-Robot (19038200162@163.com)
 * @brief 常规按键操作状态检测驱动
 * @date 2026-07-16
 * @version 0.1
 * @copyright HNAU-Robot (c) 2026
 */
#include "regular_key.h"

#include <stdint.h>

#include "stdio.h"

#define KEY_DEBOUNCE_MS 50      // 按键消抖时间，单位：ms
#define KEY_IDLE_TIMEOUT_MS 200 // 空闲超时时间，单位：ms
#define KEY_LONGPRESS_MS 1500   // 长按时间，单位：ms

static void do_key_down(key_callback_t *scan) {if (scan->key_down_func) scan->key_down_func();}
static void do_key_up(key_callback_t *scan) {if (scan->key_up_func) scan->key_up_func();}
static void do_key_click(key_callback_t *scan) {if (scan->key_click_func) scan->key_click_func();}
static void do_key_longpress(key_callback_t *scan) {if (scan->key_longpress_func) scan->key_longpress_func();}
static void do_key_double_click(key_callback_t *scan) {if (scan->key_double_click_func) scan->key_double_click_func();}
static void do_key_idle(key_callback_t *scan) {if (scan->key_idle_func) scan->key_idle_func();}

static const key_state_item_t key_state_table[] = 
{
                        // 当前状态                       事件                              下一个状态                      状态处理函数
    {KEY_STATE_IDLE,            KEY_EVENT_PRESS,        KEY_DOWN_DEBOUNCE,          NULL},
    {KEY_DOWN_DEBOUNCE,         KEY_EVENT_TIMEOUT,      KEY_STATE_DOWN,             do_key_down},           // 按键消抖完成，进入按下状态，执行按键按下回调函数
    {KEY_DOWN_DEBOUNCE,         KEY_EVENT_RELEASE,      KEY_STATE_IDLE,             NULL}, 

    {KEY_STATE_DOWN,            KEY_EVENT_RELEASE,      KEY_UP_DEBOUNCE,            NULL},
    {KEY_STATE_DOWN,            KEY_EVENT_LONGPRESSED,  KEY_STATE_LONGPRESSED,      do_key_longpress},      // 按键长按，进入长按状态，执行按键长按回调函数

    {KEY_STATE_LONGPRESSED,     KEY_EVENT_RELEASE,      KEY_LOONGPRESS_DEBOUNCE,    NULL},
    {KEY_LOONGPRESS_DEBOUNCE,   KEY_EVENT_TIMEOUT,      KEY_STATE_IDLE,             do_key_idle},           // 长按抬起消抖完成，进入空闲状态，执行按键空闲回调函数
    {KEY_LOONGPRESS_DEBOUNCE,   KEY_EVENT_PRESS,        KEY_STATE_LONGPRESSED,      NULL},

    {KEY_UP_DEBOUNCE,           KEY_EVENT_TIMEOUT,      KEY_STATE_UP,               do_key_up},             // 按键抬起消抖完成，进入抬起状态，执行按键抬起回调函数
    {KEY_UP_DEBOUNCE,           KEY_EVENT_PRESS,        KEY_STATE_DOWN,             NULL},     

    {KEY_STATE_UP,              KEY_EVENT_IDLE,         KEY_STATE_IDLE,             do_key_click},          // 按键空闲，执行按键单击回调函数
    {KEY_STATE_UP,              KEY_EVENT_PRESS,        KEY_DOUBLE_DOWN_DEBOUNCE,   NULL},

    {KEY_DOUBLE_DOWN_DEBOUNCE,  KEY_EVENT_TIMEOUT,      KEY_DOUBLE_DOWN,            NULL},
    {KEY_DOUBLE_DOWN_DEBOUNCE,  KEY_EVENT_RELEASE,      KEY_STATE_UP,               NULL},

    {KEY_DOUBLE_DOWN,           KEY_EVENT_RELEASE,      KEY_DOUBLE_UP_DEBOUNCE,     NULL},
    {KEY_DOUBLE_DOWN,           KEY_EVENT_LONGPRESSED,  KEY_STATE_LONGPRESSED,      do_key_longpress},      // 按键长按，进入长按状态，执行按键长按回调函数

    {KEY_DOUBLE_UP_DEBOUNCE,    KEY_EVENT_TIMEOUT,      KEY_STATE_IDLE,             do_key_double_click},   // 双击按键抬起消抖完成，进入空闲状态，执行按键双击回调函数
    {KEY_DOUBLE_UP_DEBOUNCE,    KEY_EVENT_PRESS,        KEY_DOUBLE_DOWN,            NULL},
};

#define KEY_STATE_TABLE_SIZE (sizeof(key_state_table)/sizeof(key_state_table[0]))

void key_state_scan(key_callback_t *scan)
{
    uint8_t key_input = scan->key_input_func();

    uint32_t current_time = scan->time_input_func();

    key_event_t event = KEY_EVENT_NONE;

    switch (scan->state)
    {
        //由空闲到第一次按下
    case KEY_STATE_IDLE:
        if (key_input == 0)
        {
            event = KEY_EVENT_PRESS;
            scan->last_time = current_time;
        }
        break;
        //按下消抖阶段
    case KEY_DOWN_DEBOUNCE:
        if (key_input == 1)
        {
            event = KEY_EVENT_RELEASE;
        }

        else if ((current_time - scan->last_time) > KEY_DEBOUNCE_MS)
        {
            event = KEY_EVENT_TIMEOUT;
            scan->last_time = current_time;
        }
        break;
        //按下状态
    case KEY_STATE_DOWN:
        if (key_input == 1)
        {
            event = KEY_EVENT_RELEASE;
            scan->last_time = current_time;
        }
        else if ((current_time - scan->last_time) > KEY_LONGPRESS_MS)
        {
            event = KEY_EVENT_LONGPRESSED;
        }
        break;
        //长按状态
    case KEY_STATE_LONGPRESSED:
        if (key_input == 1)
        {
            event = KEY_EVENT_RELEASE;
            scan->last_time = current_time;
        }
        break;
    //长按抬起消抖阶段
    case KEY_LOONGPRESS_DEBOUNCE:
        if (key_input == 0)
        {
            event = KEY_EVENT_PRESS;
        }
        else if ((current_time - scan->last_time) > KEY_DEBOUNCE_MS)
        {
            event = KEY_EVENT_TIMEOUT;
            scan->last_time = current_time;
        }
        break;
        //抬起消抖阶段
    case KEY_UP_DEBOUNCE:
        if (key_input == 0)
        {
            event = KEY_EVENT_PRESS;
        }
        else if ((current_time - scan->last_time) > KEY_DEBOUNCE_MS)
        {
            event = KEY_EVENT_TIMEOUT;
        }
        break;
        //抬起状态
    case KEY_STATE_UP:
        if (key_input == 0)
        {
            event = KEY_EVENT_PRESS;
            scan->last_time = current_time;
        }
        else if ((current_time - scan->last_time) > KEY_IDLE_TIMEOUT_MS)
        {
            event = KEY_EVENT_IDLE;
        }
        break;
        //双击按下消抖阶段
    case KEY_DOUBLE_DOWN_DEBOUNCE:
        if (key_input == 1)
        {
            event = KEY_EVENT_RELEASE;
        }
        else if ((current_time - scan->last_time) > KEY_DEBOUNCE_MS)
        {
            event = KEY_EVENT_TIMEOUT;
            scan->last_time = current_time;
        }
        break;
        //双击按下状态
    case KEY_DOUBLE_DOWN:
        if (key_input == 1)
        {
            event = KEY_EVENT_RELEASE;
            scan->last_time = current_time;
        }
        else if ((current_time - scan->last_time) > KEY_LONGPRESS_MS)
        {
            event = KEY_EVENT_LONGPRESSED;
            scan->last_time = current_time;
        }
        break;
        //双击抬起消抖阶段
    case KEY_DOUBLE_UP_DEBOUNCE:
        if (key_input == 0)
        {
            event = KEY_EVENT_PRESS;
        }
        else if ((current_time - scan->last_time) > KEY_DEBOUNCE_MS)
        {
            event = KEY_EVENT_TIMEOUT;
            scan->last_time = current_time;
        }
        break;
    
    default:
        break;
    }

    // 查找状态转换表，进行状态转换
    for (uint16_t i = 0; i < KEY_STATE_TABLE_SIZE - 1; i++)
    {
        if (key_state_table[i].now_state == scan->state &&
            key_state_table[i].event == event)
        {
            scan->state = key_state_table[i].next_state;
            if (key_state_table[i].state_func)
            {
                key_state_table[i].state_func(scan);
            }
            break;
        }
    }
    
}

/*
    * @brief  按键初始化函数
    * @param  scan: 指向按键回调函数结构体指针
    * @param  key_up_func: 按键释放回调函数指针
    * @param  key_down_func: 按键按下回调函数指针
    * @param  key_click_func: 按键单击回调函数指针
    * @param  key_longpress_func: 按键长按回调函数指针
    * @param  key_double_click_func: 按键双击回调函数指针
    * @param  key_idle_func: 按键空闲回调函数指针
    * @param  key_input_func: 按键输入检测函数指针，返回按键状态，高低电平
    * @param  time_input_func: 时间获取函数指针，返回当前时间，单位：ms
    * @retval None
*/

void key_init(  key_callback_t *scan,
                    void (* key_up_func)(void),
                    void (* key_down_func)(void),
                    void (* key_click_func)(void),
                    void (* key_longpress_func)(void),
                    void (* key_double_click_func)(void),
                    void (* key_idle_func)(void),
                    uint8_t (* key_input_func)(void),
                    uint32_t (* time_input_func)(void))
{
    scan->state = KEY_STATE_IDLE;
    scan->last_time = 0;

    scan->key_up_func = key_up_func;
    scan->key_down_func = key_down_func;

    scan->key_click_func = key_click_func;
    scan->key_longpress_func = key_longpress_func;
    scan->key_double_click_func = key_double_click_func;

    scan->key_idle_func = key_idle_func;

    scan->key_input_func = key_input_func;
    scan->time_input_func = time_input_func;
}

#undef KEY_DEBOUNCE_MS
#undef KEY_IDLE_TIMEOUT_MS
#undef KEY_LONGPRESS_MS

#undef KEY_STATE_TABLE_SIZE
