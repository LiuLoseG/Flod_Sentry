#pragma once

#include "bsp_uart.h"

#define OPS9_UART huart1  //ops_9传感器使用的UART句柄
#define OPS9_DATA_SIZE 28U

#define OPS9_FIRST_HEADER 0x0D
#define OPS9_SECOND_HEADER 0x0A

#define OPS9_FIRST_TAIL 0x0A
#define OPS9_SECOND_TAIL 0x0D

typedef struct
{
    float yaw;              // 航向角
    float pitch;            // 俯仰角
    float roll;             // 翻滚角

    float x;                // X坐标
    float y;                // Y坐标

    float yaw_angular;      // 航向角速度

} ops_9_location_message_t;

void ops_9_updata_x_axes(UART_HandleTypeDef *huart, float now_x);
void ops_9_updata_y_axes(UART_HandleTypeDef *huart, float now_y);
void ops_9_updata_yaw(UART_HandleTypeDef *huart, float now_yaw);
void ops_9_data_analyse(uint8_t *buf, uint16_t len);
