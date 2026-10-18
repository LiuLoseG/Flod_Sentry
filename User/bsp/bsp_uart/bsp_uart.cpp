/**
 * @file bsp_uart.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照RoboWalker麻神代码改写的UART通信初始化与配置流程
 * @version 1.0
 * @date 2026-05-18 1.0 适配达妙MC02板、F103C8T6芯片，使用HAL库进行UART通信配置和数据传输

 * @note 目前使用串口DMA空闲中断处理串口数据，采用乒乓缓冲机制提高数据接收效率，适用于需要高频率数据传输的应用场景。
        但是数据处理回调函数在中断中处理，会增加中断处理时间，后续可以考虑将数据处理放在主循环中，通过信号量或消息队列通知主循环进行数据处理，以减少中断处理时间，提高系统响应能力。
        或者将缓存模式改为循环缓冲模式，减少切换缓冲区的开销，同时在主循环中定期检查接收缓冲区的数据状态进行处理，进一步优化系统性能。
 *
 * @copyright liu_balance (c) 2026
 *
 */

#include "bsp_uart.h"
#include "stm32h723xx.h"
#include <cstddef>

uart_manger_object uart1_manager __attribute__((section(".DMA_RAM_D1"), aligned(32))) = {NULL};
uart_manger_object uart2_manager __attribute__((section(".DMA_RAM_D1"), aligned(32))) = {NULL};
uart_manger_object uart3_manager __attribute__((section(".DMA_RAM_D1"), aligned(32))) = {NULL};
uart_manger_object uart4_manager __attribute__((section(".DMA_RAM_D1"), aligned(32))) = {NULL};
uart_manger_object uart5_manager __attribute__((section(".DMA_RAM_D1"), aligned(32))) = {NULL};
uart_manger_object uart7_manager __attribute__((section(".DMA_RAM_D1"), aligned(32))) = {NULL};
// uart_manger_object uart7_manager = {NULL};

uart_manger_object uart10_manager __attribute__((section(".DMA_RAM_D1"), aligned(32))) = {NULL};

static void bsp_uart_prepare_manager(uart_manger_object &manager, UART_HandleTypeDef *huart, uart_callback callback)
{
    // 清零管理器结构体，并初始化相关成员变量
    memset(&manager, 0, sizeof(manager));
    manager.uart_handler = huart;
    manager.callback_function = callback;
    manager.rx_buffer_active = manager.rx_buffer_0;
    manager.rx_buffer_ready = manager.rx_buffer_1;
}

/**
 * @brief UART初始化函数
 * @param huart UART句柄
 * @param callback 回调函数
 * @retval None
 */
void bsp_uart_init(UART_HandleTypeDef *huart, uart_callback callback)
{
    if (huart->Instance == USART1)
    {
        bsp_uart_prepare_manager(uart1_manager, huart, callback);

        // 启动UART接收中断
        HAL_UARTEx_ReceiveToIdle_DMA(uart1_manager.uart_handler, uart1_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == USART2)
    {
        bsp_uart_prepare_manager(uart2_manager, huart, callback);

        // 启动UART接收中断
        HAL_UARTEx_ReceiveToIdle_DMA(uart2_manager.uart_handler, uart2_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == USART3)
    {
        bsp_uart_prepare_manager(uart3_manager, huart, callback);

        // 启动UART接收中断
        HAL_UARTEx_ReceiveToIdle_DMA(uart3_manager.uart_handler, uart3_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == UART4)
    {
        bsp_uart_prepare_manager(uart4_manager, huart, callback);

        // 启动UART接收中断
        HAL_UARTEx_ReceiveToIdle_DMA(uart4_manager.uart_handler, uart4_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == UART5)
    {
        bsp_uart_prepare_manager(uart5_manager, huart, callback);

        // 启动UART接收中断
        HAL_UARTEx_ReceiveToIdle_DMA(uart5_manager.uart_handler, uart5_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == UART7)
    {
        bsp_uart_prepare_manager(uart7_manager, huart, callback);

        // 启动UART接收中断
        HAL_UARTEx_ReceiveToIdle_DMA(uart7_manager.uart_handler, uart7_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == USART10)
    {
        bsp_uart_prepare_manager(uart10_manager, huart, callback);

        // 启动UART接收中断
        HAL_UARTEx_ReceiveToIdle_DMA(uart10_manager.uart_handler, uart10_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }
}

/**
 * @brief 通信异常时重启UART接收
 * @param huart UART句柄
 */

void bsp_uart_deinit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_manager.rx_buffer_active = uart1_manager.rx_buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == USART2)
    {
        uart2_manager.rx_buffer_active = uart2_manager.rx_buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart2_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == USART3)
    {
        uart3_manager.rx_buffer_active = uart3_manager.rx_buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == UART4)
    {
        uart4_manager.rx_buffer_active = uart4_manager.rx_buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart4_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == UART5)
    {
        uart5_manager.rx_buffer_active = uart5_manager.rx_buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart5_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == UART7)
    {
        uart7_manager.rx_buffer_active = uart7_manager.rx_buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart7_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }

    else if (huart->Instance == USART10)
    {
        uart10_manager.rx_buffer_active = uart10_manager.rx_buffer_0;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart10_manager.rx_buffer_active, UART_BUFFER_SIZE);
    }
}

/**
 * @brief UART数据发送函数
 * @param huart UART句柄
 * @param data 待发送数据指针
 * @param length 待发送数据长度
 * @return 0表示成功，1表示失败
 */

uint8_t bsp_uart_transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    return (HAL_UART_Transmit_DMA(huart, data, length) == HAL_OK) ? 0 : 1;
}

/**
 * @brief UART接收完成回调函数
 * @param huart UART句柄
 */

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 判断程序初始化完成
    if (!init_finished) 
    {
        // 重启接收
        if (huart->Instance == USART1) 
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_manager.rx_buffer_active, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == USART2)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart2_manager.rx_buffer_active, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == USART3)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_manager.rx_buffer_active, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == UART4)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart4_manager.rx_buffer_active, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == UART5)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart5_manager.rx_buffer_active, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == UART7)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart7_manager.rx_buffer_active, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == USART10)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, uart10_manager.rx_buffer_active, UART_BUFFER_SIZE);
        }
        return;
    }

    // 选择回调函数
    if (huart->Instance == USART1)
    {
        uart1_manager.rx_buffer_ready = uart1_manager.rx_buffer_active;

        // 切换缓冲区
        if (uart1_manager.rx_buffer_active == uart1_manager.rx_buffer_0) 
        {
            uart1_manager.rx_buffer_active = uart1_manager.rx_buffer_1;
        }
        else 
        {
            uart1_manager.rx_buffer_active = uart1_manager.rx_buffer_0;
        }

        // 记录接收时间戳
        uart1_manager.rx_time_stamp = bsp_dwt_get_timeline_ms();

        // 重启接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_manager.rx_buffer_active, UART_BUFFER_SIZE);

        // 调用回调函数处理接收到的数据
        if (uart1_manager.callback_function != nullptr) 
        {
            uart1_manager.callback_function(uart1_manager.rx_buffer_ready, Size);
        }
    }
    else if (huart->Instance == USART2)
    {
        uart2_manager.rx_buffer_ready = uart2_manager.rx_buffer_active;
        // 切换缓冲区
        if (uart2_manager.rx_buffer_active == uart2_manager.rx_buffer_0) 
        {
            uart2_manager.rx_buffer_active = uart2_manager.rx_buffer_1;
        }
        else 
        {
            uart2_manager.rx_buffer_active = uart2_manager.rx_buffer_0;
        }

        // 记录接收时间戳
        uart2_manager.rx_time_stamp = bsp_dwt_get_timeline_ms();

        // 重启接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart2_manager.rx_buffer_active, UART_BUFFER_SIZE);

        // 调用回调函数处理接收到的数据
        if (uart2_manager.callback_function != nullptr) 
        {
            uart2_manager.callback_function(uart2_manager.rx_buffer_ready, Size);
        }
    }
    else if (huart->Instance == USART3)
    {
        uart3_manager.rx_buffer_ready = uart3_manager.rx_buffer_active;
        if (uart3_manager.rx_buffer_active == uart3_manager.rx_buffer_0) 
        {
            uart3_manager.rx_buffer_active = uart3_manager.rx_buffer_1;
        }
        else 
        {
            uart3_manager.rx_buffer_active = uart3_manager.rx_buffer_0;
        }

        uart3_manager.rx_time_stamp = bsp_dwt_get_timeline_ms();

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart3_manager.rx_buffer_active, UART_BUFFER_SIZE);

        if (uart3_manager.callback_function != nullptr) 
        {
            uart3_manager.callback_function(uart3_manager.rx_buffer_ready, Size);
        }
    }
    else if (huart->Instance == UART4)
    {
        uart4_manager.rx_buffer_ready = uart4_manager.rx_buffer_active;
        if (uart4_manager.rx_buffer_active == uart4_manager.rx_buffer_0) 
        {
            uart4_manager.rx_buffer_active = uart4_manager.rx_buffer_1;
        }
        else 
        {
            uart4_manager.rx_buffer_active = uart4_manager.rx_buffer_0;
        }

        uart4_manager.rx_time_stamp = bsp_dwt_get_timeline_ms();

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart4_manager.rx_buffer_active, UART_BUFFER_SIZE);

        if (uart4_manager.callback_function != nullptr) 
        {
            uart4_manager.callback_function(uart4_manager.rx_buffer_ready, Size);
        }
    }
    else if (huart->Instance == UART5)
    {
        uart5_manager.rx_buffer_ready = uart5_manager.rx_buffer_active;
        if (uart5_manager.rx_buffer_active == uart5_manager.rx_buffer_0) 
        {
            uart5_manager.rx_buffer_active = uart5_manager.rx_buffer_1;
        }
        else 
        {
            uart5_manager.rx_buffer_active = uart5_manager.rx_buffer_0;
        }

        uart5_manager.rx_time_stamp = bsp_dwt_get_timeline_ms();

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart5_manager.rx_buffer_active, UART_BUFFER_SIZE);

        if (uart5_manager.callback_function != nullptr) 
        {
            uart5_manager.callback_function(uart5_manager.rx_buffer_ready, Size);
        }
    }
    else if (huart->Instance == UART7)
    {
        uart7_manager.rx_buffer_ready = uart7_manager.rx_buffer_active;
        if (uart7_manager.rx_buffer_active == uart7_manager.rx_buffer_0) 
        {
            uart7_manager.rx_buffer_active = uart7_manager.rx_buffer_1;
        }
        else 
        {
            uart7_manager.rx_buffer_active = uart7_manager.rx_buffer_0;
        }

        uart7_manager.rx_time_stamp = bsp_dwt_get_timeline_ms();

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart7_manager.rx_buffer_active, UART_BUFFER_SIZE);

        if (uart7_manager.callback_function != nullptr) 
        {
            uart7_manager.callback_function(uart7_manager.rx_buffer_ready, Size);
        }
    }
    else if (huart->Instance == USART10)
    {
        uart10_manager.rx_buffer_ready = uart10_manager.rx_buffer_active;
        if (uart10_manager.rx_buffer_active == uart10_manager.rx_buffer_0) 
        {
            uart10_manager.rx_buffer_active = uart10_manager.rx_buffer_1;
        }
        else 
        {
            uart10_manager.rx_buffer_active = uart10_manager.rx_buffer_0;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart10_manager.rx_buffer_active, UART_BUFFER_SIZE);

        uart10_manager.rx_time_stamp = bsp_dwt_get_timeline_ms();

        if (uart10_manager.callback_function != nullptr) 
        {
            uart10_manager.callback_function(uart10_manager.rx_buffer_ready, Size);
        }
    }

}

/**
 * @brief UART错误回调函数
 * @param huart UART句柄
 */

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_deinit(huart);
}
