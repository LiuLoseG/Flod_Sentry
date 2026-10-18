#pragma once

#include "bsp_uart.h"
#include <cstdint>

#define TFMINI_UART huart1  //tfmini传感器使用的UART句柄
#define TFMINI_DATA_SIZE 9
#define TFMINI_Header 0X59            // 数据帧头

typedef struct
{
    uart_manger_object *tfmini_uart_manager; //UART管理对象

    float distance;
    uint16_t strength;
    float temp;
} tfmini_data_t;