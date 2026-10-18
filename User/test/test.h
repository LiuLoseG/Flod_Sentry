#pragma once

#include "bsp_uart.h"
#include "bsp_can.h"
#include "dji_motor.h"
#include "frequency_filter.h"

typedef struct
{
    float motor_speed; // 电机速度
    float motor_current; // 电机电流
    float motor_temperature; // 电机温度

    dji_motor_t motor; // 指向电机结构体的指针

}motor_fibpara_t;

typedef struct __menu_page_t{
    uint8_t menu_name[16];
    struct __menu_page_t *next;
    struct __menu_page_t *prev;
    struct __menu_page_t *child; // 子菜单
    struct __menu_page_t *parent; // 父菜单
    
    uint8_t task_id;
} menu_page_t;
