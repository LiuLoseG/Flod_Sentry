/**
 * @file drv_usb.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的USB通信初始化与配置流程
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/**
 * 由于Cube中 usbd_cdc_if.c 文件为C, 而我们用的是C++, 所以需将该文件扩展名改为cpp
 * static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len) 函数为实际接收回调函数, 在该文件中
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

struct_usb_manage_object usb0_manage_object = {nullptr};

// USB设备句柄, 由 usbd_cdc_if.c 中定义
extern USBD_HandleTypeDef hUsbDeviceHS;
// USB接收缓冲区, 由 usbd_cdc_if.c 中定义(注意大小写: UserRxBufferHS)
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param Callback_Function 处理回调函数
 */
void bsp_usb_init(usb_callback callback_function)
{
    usb0_manage_object.callback_function = callback_function;

    usb0_manage_object.rx_buffer_active = UserRxBufferHS;
}

/**
 * @brief 发送数据帧
 *
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
uint8_t bsp_usb_transmit(uint8_t *data, uint16_t length)
{
    return (CDC_Transmit_HS(data, length) == USBD_OK) ? 0 : 1;
}

/**
 * @brief 自己写的USB通信下一轮接收开启前回调函数, 非HAL库回调函数
 *
 * @param Size 接收数据长度
 */
void bsp_usb_receive_callback(uint16_t size)
{
    if (!init_finished)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, usb0_manage_object.rx_buffer_active);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return;
    }

    // 自设双缓冲USB
    usb0_manage_object.rx_buffer_ready = usb0_manage_object.rx_buffer_active;
    if (usb0_manage_object.rx_buffer_active == usb0_manage_object.rx_buffer_0)
    {
        usb0_manage_object.rx_buffer_active = usb0_manage_object.rx_buffer_1;
    }
    else
    {
        usb0_manage_object.rx_buffer_active = usb0_manage_object.rx_buffer_0;
    }

    usb0_manage_object.rx_time_stamp = bsp_dwt_get_timeline_ms();

    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, usb0_manage_object.rx_buffer_active);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);

    if (usb0_manage_object.callback_function != nullptr)
    {
        usb0_manage_object.callback_function(usb0_manage_object.rx_buffer_ready, size);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/