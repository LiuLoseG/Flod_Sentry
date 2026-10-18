#pragma once

#include "bsp_uart.h"

#define MSPM0_IMU_UART huart10          // MSPM0 IMU使用的UART句柄

#define MSPM0_IMU_DATA_SIZE 5           // MSPM0 IMU一帧数据包的长度

#define MSPM0_IMU_Header 0X5A            // 数据帧头

#define MSPM0_IMU_ACCELEROMETER 0XAA    // 角速度数据帧标识

#define MSPM0_IMU_ANGLE 0XBB            // 角度数据帧标识

// MSPM0 IMU寄存器操作枚举
typedef enum
{
    MSPM0_IMU_Register_Save,                    // 保存指令
    MSPM0_IMU_Register_Restart,                 // 重启指令
    MSPM0_IMU_Register_Reset,                   // 恢复出厂设置指令
    MSPM0_IMU_Register_Angle_Zero,              // Z轴角度归零指令
    MSPM0_IMU_Register_Zero_Offset,             // 自动获取零偏指令

    MSPM0_IMU_Register_Yaw_Calibration,         // Z轴标定指令

    MSPM0_IMU_Register_Baud_Rate,               // 设置串口波特率指令
    MSPM0_IMU_Register_Output_Frequency,        // 设置输出频率指令
} mspm0_imu_register_e;

// MSPM0 IMU数据结构体
typedef struct
{
    uart_manger_object *mspm0_uart_manager; // MSPM0 IMU使用的UART管理对象
    float angular;      // 角速度
    float angle;        // 角度
} mspm0_imu_t;

extern volatile mspm0_imu_t mspm0_data; // MSPM0 IMU数据结构体实例，使用volatile防止中断中调用本结构体导致数据错乱

void mspm0_imu_data_analysis(uint8_t *buf, uint16_t len);