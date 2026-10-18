#include "bsp_mcu.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_adc.h"
#include "crsf.h"
#include "hi12_imu.h"
#include "mspm0_imu.h"
#include "uart_vofa.h"
#include "usart.h"
#include <cstddef>

#include "robot_decision.h"

bool init_finished = false;

/**
 * @brief MCU外设初始化函数
 * @param None
 * @retval None
 */

extern "C" void bsp_mcu_init(void)
{
    bsp_dwt_init(550); // 初始化DWT，参数为CPU频率（MHz）
    // wit_data.init(&uart3_manager); // 初始化WIT陀螺仪数据结构，传入UART管理器
    // bsp_uart_init(&huart3, wit_gyro_uart_callback); // 初始化UART3，使用中断模式
    bsp_uart_init(&huart7, crsf_rx_idle_callback); // 初始化UART7，无回调函数

    h12_imu_init(&uart10_manager); // 初始化HI12 IMU数据结构，传入UART管理器
    bsp_uart_init(&huart10, hi12_gyro_uart_callback); // 初始化UART10，使用中断模式
    
    // bsp_uart_init(&huart1, NULL); // 初始化VOFA UART，使用中断模式
    // mspm0_imu_init(&mspm0_data, &uart10_manager); // 初始化MSPM0 IMU数据结构，传入UART管理器
    // bsp_uart_init(&huart10, mspm0_imu_data_analysis); // 初始化UART10，无回调函数
    // bsp_uart_init(&huart1, nullptr); // 初始化UART1，无回调函数
    bsp_usb_init(robot_data_transfer_analyze); // 初始化USB，注册回调函数
    bsp_fdcan_init_all(); // 初始化所有FDCAN实例
    bsp_adc_init(); // 初始化ADC
    init_finished = true;
}
