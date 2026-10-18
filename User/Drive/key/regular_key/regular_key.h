#pragma once

#include <stdint.h>

typedef enum 
{
    KEY_STATE_IDLE = 0,             // 空闲状态
    KEY_DOWN_DEBOUNCE,              // 按下消抖状态

    KEY_STATE_DOWN,                 // 按下状态
    KEY_UP_DEBOUNCE,                // 抬起消抖状态

    KEY_STATE_LONGPRESSED,          // 长按状态
    KEY_LOONGPRESS_DEBOUNCE,        // 长按抬起消抖状态

    KEY_STATE_UP,                   // 抬起状态
    KEY_DOUBLE_DOWN_DEBOUNCE,       // 双击按下消抖状态
    KEY_DOUBLE_DOWN,                // 双击按下状态
    KEY_DOUBLE_UP_DEBOUNCE          // 双击抬起消抖状态

} key_state_t;// 按键状态枚举

typedef enum 
{
    KEY_EVENT_NONE = 0,             // 无事件
    KEY_EVENT_PRESS,                // 按键按下事件
    KEY_EVENT_RELEASE,              // 按键释放事件
    KEY_EVENT_TIMEOUT,              // 按键超时事件
    KEY_EVENT_LONGPRESSED,          // 长按事件
    KEY_EVENT_DOUBLECLICK,          // 双击事件

    KEY_EVENT_IDLE

} key_event_t;// 按键事件枚举

typedef struct 
{
    key_state_t state;
    uint32_t last_time;

    void (* key_up_func)(void);             // 按键抬起回调函数指针
    void (* key_down_func)(void);           // 按键按下回调函数指针

    void (* key_click_func)(void);          // 按键单击回调函数指针
    void (* key_longpress_func)(void);      // 按键长按回调函数指针
    void (* key_double_click_func)(void);   // 按键双击回调函数指针

    void (* key_idle_func)(void);           // 按键长按后释放空闲回调函数指针

    uint8_t (* key_input_func)(void);       // 按键输入函数指针
    uint32_t (* time_input_func)(void);     // 时间输入函数指针

} key_callback_t;// 按键回调函数结构体;


typedef struct 
{
    key_state_t now_state;        // 当前按键状态
    key_event_t event;            // 当前按键事件
    key_state_t next_state;       // 下一个按键状态
    void (*state_func)(key_callback_t *scan);   // 状态处理函数指针
} key_state_item_t;// 按键状态机结构体

void key_state_scan(key_callback_t *scan);

void key_init(  key_callback_t *scan,
                    void (* key_up_func)(void),
                    void (* key_down_func)(void),
                    void (* key_click_func)(void),
                    void (* key_longpress_func)(void),
                    void (* key_double_click_func)(void),
                    void (* key_idle_func)(void),
                    uint8_t (* key_input_func)(void),
                    uint32_t (* time_input_func)(void));

