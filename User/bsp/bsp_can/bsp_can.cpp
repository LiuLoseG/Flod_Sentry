/**
  ******************************************************************************
  * @file    bsp_can.c
  * @author  MCD Application Team
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
**/
#include "bsp_can.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include <string.h>

/*  告诉编译器，该对象可能被程序以外的途径改变（中断、DMA）防止编译优化，
    如果不加volatile，在后续中断读取和修改该结构体的时候，可能会导致读到陈旧或不一致的数据。 */
volatile FDCAN_RxFrame_TypeDef fdcan1_rxframe;
volatile FDCAN_RxFrame_TypeDef fdcan2_rxframe;
volatile FDCAN_RxFrame_TypeDef fdcan3_rxframe;

// CAN通信发送缓冲区

// 电机共享区

// CAN1
uint8_t can1_0x1fe_tx_data[8];
uint8_t can1_0x1ff_tx_data[8];
uint8_t can1_0x200_tx_data[8];
uint8_t can1_0x2fe_tx_data[8];
uint8_t can1_0x2ff_tx_data[8];
uint8_t can1_0x3fe_tx_data[8];
uint8_t can1_0x4fe_tx_data[8];

// CAN2
uint8_t can2_0x1fe_tx_data[8];
uint8_t can2_0x1ff_tx_data[8];
uint8_t can2_0x200_tx_data[8];
uint8_t can2_0x2fe_tx_data[8];
uint8_t can2_0x2ff_tx_data[8];
uint8_t can2_0x3fe_tx_data[8];
uint8_t can2_0x4fe_tx_data[8];

// CAN3
uint8_t can3_0x1fe_tx_data[8];
uint8_t can3_0x1ff_tx_data[8];
uint8_t can3_0x200_tx_data[8];
uint8_t can3_0x2fe_tx_data[8];
uint8_t can3_0x2ff_tx_data[8];
uint8_t can3_0x3fe_tx_data[8];
uint8_t can3_0x4fe_tx_data[8];

// 接收错误的 ISR 安全计数器，主循环可读取并记录或进行处理. 
volatile uint32_t fdcan_rx_error_count = 0;

static void fdcan_handle_init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) return;

    else if (hfdcan->Instance == FDCAN1)
    {
        fdcan1_rxframe.hcan = hfdcan;
    }

    else if (hfdcan->Instance == FDCAN2)
    {
        fdcan2_rxframe.hcan = hfdcan;
    }

    else if (hfdcan->Instance == FDCAN3)
    {
        fdcan3_rxframe.hcan = hfdcan;
    }
}

//classic CAN 接收数据长度(0~8)解码函数,兼容后续的FDCAN配置
static uint8_t fdcan_dlc_to_bytes(uint32_t dataLengthField)
{
    
    // HAL使用FDCAN_DLC_BYTES_x宏对DLC进行编码。将这些值映射到对应的字节数。 
    switch (dataLengthField)
    {
        //虽然FDCAN_DLC_BYTES_0..8的宏值就是0..8。
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        //兼容FDCAN的DLC配置
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return dataLengthField;
        //对于后续配置FDCAN来说，由于FDCAN_DLC_BYTES_12对于的宏值为0x00000009U不是连续的，因此用DLC校验来处理数据
    }
}

/* 安全接收辅助函数：从FIFO0读取一帧报文至提供的volatile类型rx_frame中。
返回接收到的字节数（出现错误或长度为0时返回0）。 */
uint8_t fdcan_receive_data(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo, volatile FDCAN_RxFrame_TypeDef *rx_frame)
{
    if (hfdcan == NULL || rx_frame == NULL) return 0;

    FDCAN_RxHeaderTypeDef hdr;
    uint8_t tmp[8]; // classic CAN 最大单次发送 8 bytes 

    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &hdr, tmp) != HAL_OK)//使用FIO0接收消息
    {
        // 在ISR中：避免记录日志或调用Error_Handler()。因此增加一个ISR安全的计数器，可在主循环中处理 
        fdcan_rx_error_count++;
        return 0;
    }

    uint8_t len = fdcan_dlc_to_bytes(hdr.DataLength);
    
    rx_frame->rx_len = len;

    // 先保存 header（包含 DLC/ID 等）——对 volatile 目标逐字段赋值，避免调用非-volatile 的赋值运算符
    rx_frame->header.Identifier = hdr.Identifier;
    rx_frame->header.IdType = hdr.IdType;
    rx_frame->header.RxFrameType = hdr.RxFrameType;
    rx_frame->header.DataLength = hdr.DataLength;
    rx_frame->header.ErrorStateIndicator = hdr.ErrorStateIndicator;
    rx_frame->header.BitRateSwitch = hdr.BitRateSwitch;
    rx_frame->header.FDFormat = hdr.FDFormat;
    rx_frame->header.RxTimestamp = hdr.RxTimestamp;
    rx_frame->header.FilterIndex = hdr.FilterIndex;
    rx_frame->header.IsFilterMatchingFrame = hdr.IsFilterMatchingFrame;

    for (uint8_t i = 0; i < len; ++i)
    {
        rx_frame->data[i] = tmp[i];
    }
    /* 清零未使用字节，避免使用方读取到旧数据 */
    for (uint8_t i = len; i < 8; ++i)
    {
        rx_frame->data[i] = 0;
    }

    return len;
}

/** 
  * @brief  FDCAN初始化函数
  * @param  hfdcan: 指向FDCAN句柄的指针
  * @note   此函数配置过滤器、全局过滤器、激活中断并启动FDCAN
  *         
  * @retval None
  */

static void fdcan_init_instance(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t fifo_signal = 0;                          //FIFO信号选择标志

    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;                      // 仅过滤标准ID
    filter.FilterIndex = 0;                                 // 使用过滤器0
    filter.FilterType = FDCAN_FILTER_MASK;                  // 使用掩码过滤器

    if((hfdcan == &hfdcan1) || (hfdcan == &hfdcan3))
    {
        fifo_signal = FDCAN_SIGNAL_FIFO0;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;          // 将匹配的消息存入FIFO0
    }

    else if(hfdcan == &hfdcan2)
    {
        fifo_signal = FDCAN_SIGNAL_FIFO1;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;          // 将匹配的消息存入FIFO1
    }

    else
    {
        LOG_WARNING("[bsp_can] CAN Filter Config Error: Unknown hfdcan");
        Error_Handler();
    }

    /* 设置过滤器ID和掩码以接收所有消息 */
    filter.FilterID1 = 0x00000000;                          // 过滤器ID，接收所有ID
    filter.FilterID2 = 0x00000000;                          // 掩码，接收所有ID

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)  //配置过滤器
    {
        LOG_WARNING("[bsp_can] CAN Filter Config Error");
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)//配置全局过滤器
    {
        LOG_WARNING("[bsp_can] CAN Global Filter Config Error");
        Error_Handler();
    }

    if (fifo_signal == FDCAN_SIGNAL_FIFO0)//选择FIFO0
    {
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0) != HAL_OK)//激活接收中断和错误状态中断
        {
            LOG_WARNING("[bsp_can] CAN FIFO0 Notification Config Error");
            Error_Handler();
        }
    }

    else if (fifo_signal == FDCAN_SIGNAL_FIFO1)//选择FIFO1
    {
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0) != HAL_OK)//激活接收中断和错误状态中断
        {
            LOG_WARNING("[bsp_can] CAN FIFO1 Notification Config Error");
            Error_Handler();
        }
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)                  //启动FDCAN
    {
        LOG_WARNING("[bsp_can] CAN Start Error");
        Error_Handler();
    }
    fdcan_handle_init(hfdcan); // 初始化接收帧结构体的句柄指针
}

/** 
  * @brief  初始化所有FDCAN实例
  * @retval None
  */

void bsp_fdcan_init_all(void)
{
    fdcan_init_instance(&hfdcan1);
    fdcan_init_instance(&hfdcan2);
    fdcan_init_instance(&hfdcan3);
}

/** 
  * @brief  CAN发送函数
  * @param  hfdcan: 指向FDCAN句柄的指针
  * @param  id: 标准ID，范围0~0x7FF
  * @param  data: 指向发送数据的指针
  * @param  len: 发送数据的长度，范围0~8
  * @retval uint8_t: CAN发送状态
  *                      - CAN_SUCCESS: 发送成功
  *                     - CAN_LINE_BUSY: 总线忙，发送失败
  */

uint8_t bsp_fdcan_send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    static uint32_t can_busy_count = 0; // 统计总线忙碌次数

    FDCAN_TxHeaderTypeDef TxHeader;
    // 检测传入can句柄是否有效 
    if (hfdcan == NULL)
    {
        LOG_ERROR("[bsp_can] CAN Send Error: hfdcan is NULL");
        return CAN_LINE_BUSY;
    }

    if (len > 8)
    {
        // 经典CAN不支持的长度 
        LOG_ERROR("[bsp_can] CAN Send Error: len > 8");
        return CAN_LINE_BUSY;
    }

    // 若数据长度为0（无有效载荷），允许数据指针为空；当数据长度大于0时，必须提供非空数据指针。 
    if (len > 0 && data == NULL)
    {
        LOG_ERROR("[bsp_can] CAN Send Error: data is NULL");
        return CAN_LINE_BUSY;
    }

    // 配置帧头 - 标准ID必须符合11位格式，数据长度必须符合经典CAN的限制。
    TxHeader.Identifier = id & 0x7FFU;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;

    // 将字节长度映射到HAL FDCAN数据长度代码枚举值, 经典CAN仅支持0~8字节，因此直接映射到对应的枚举值。对于后续配置FDCAN来说，超过8字节的长度会被HAL_FDCAN_AddMessageToTxFifoQ函数内部校验并返回错误。
    switch (len)
    {
        case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
        case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;

        default:
            LOG_ERROR("[bsp_can] Classics CAN DataLength Error: Invalid len");
            return CAN_LINE_BUSY;
    }

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0x00;

   // 非阻塞发送: 若Tx FIFO已满, 立即返回, 避免阻塞破坏1ms控制周期
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
    {
        LOG_WARNING("[bsp_can] Fdcan Tx FiFo full,add can message failed!. Total busy count:%d", can_busy_count);
        can_busy_count++;
        return CAN_LINE_BUSY;
    }
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
    {
        // 传输请求出错
        LOG_ERROR("[bsp_can] CAN Send Error: CAN bus BUS! Total busy count:%d", can_busy_count);
        can_busy_count++;
        return CAN_LINE_BUSY;
    }

    return CAN_SUCCESS;
}

/**
 * @brief 一拖四CAN发送函数, 适用于一拖四电机的CAN发送
 * @param hfdcan 绑定的CAN总线
 * @param can_tx_id 绑定的CAN ID
 * @param tx_data 发送数据缓冲区
 * @param frequency 发送频率, 可选值为1000Hz、500Hz、250Hz(默认任务频率为1000Hz),此处频率是相较于任务频率的分频发送
 * @note  该函数仅在一拖四电机模式下使用，其他模式下请勿调用

 */
void bsp_can_1_to_4_transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t can_tx_id, uint8_t *tx_data, Can_transmit_frequency_t frequency)
{
    // 每个调用点只绑定一个频率，使用静态 tick 实现分频
    static uint16_t tick = 0;

    // 1000Hz: 每次调用都发送
    if (frequency == Can_Transmit_Frequency_Div1)
    {
        bsp_fdcan_send(hfdcan, can_tx_id, tx_data, 8);
        return;
    }

    tick++;

    // 500Hz: 每2次调用发送1次（tick 为偶数时发送）
    if (frequency == Can_Transmit_Frequency_Div2 && (tick % 2 == 0))
    {
        bsp_fdcan_send(hfdcan, can_tx_id, tx_data, 8);
        return;
    }
    
    // 250Hz: 每4次调用发送1次（tick 为4的倍数时发送）
    else if (frequency == Can_Transmit_Frequency_Div4 && (tick % 4 == 0))
    {
        bsp_fdcan_send(hfdcan, can_tx_id, tx_data, 8);
        return;
    }
}

/** 
  * @brief  FDCAN 接收回调函数弱定义
  * @note   重定义此函数以处理接收到的 FDCAN 消息
  * @param  rx_frame: 指向接收到的 FDCAN 帧的指针
  */

 //can1接收回调函数
__weak void fdcan1_rx_callback(const volatile FDCAN_RxFrame_TypeDef *rx_frame)
{
    (void)rx_frame;
}

//can2接收回调函数
__weak void fdcan2_rx_callback(const volatile FDCAN_RxFrame_TypeDef *rx_frame)
{
    (void)rx_frame;
}

//can3接收回调函数
__weak void fdcan3_rx_callback(const volatile FDCAN_RxFrame_TypeDef *rx_frame)
{
    (void)rx_frame;
}

/*
  * @brief  Rx FIFO 0 中断回调函数
  * @param  hfdcan: 指向包含指定 FDCAN 配置信息的 FDCAN_HandleTypeDef 结构体的指针 
  * @param  RxFifo0ITs: FDCAN Rx
*/
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)  // 检查是否有新消息
    {
        if(hfdcan == &hfdcan1)
        {
            fdcan_receive_data(&hfdcan1, FDCAN_RX_FIFO0, &fdcan1_rxframe);
            fdcan1_rxframe.rx_time_stamp = bsp_dwt_get_timeline_us(); //记录接收时间戳，单位微秒
            fdcan1_rx_callback(&fdcan1_rxframe);//调用自定义的中断处理函数，处理接收到的消息
        }
        if(hfdcan == &hfdcan3)
        {
            fdcan_receive_data(&hfdcan3, FDCAN_RX_FIFO0, &fdcan3_rxframe);
            fdcan3_rxframe.rx_time_stamp = bsp_dwt_get_timeline_us(); //记录接收时间戳，单位微秒
            fdcan3_rx_callback(&fdcan3_rxframe);//调用自定义的中断处理函数，处理接收到的消息
        }
    }
} 

/*
  * @brief  Rx FIFO 1 中断回调函数
  * @param  hfdcan: 指向包含指定 FDCAN 配置信息的 FDCAN_HandleTypeDef 结构体的指针 
  * @param  RxFifo1ITs: FDCAN Rx
*/
extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)  // 检查是否有新消息
    {
        if(hfdcan == &hfdcan2)
        {
            fdcan_receive_data(&hfdcan2, FDCAN_RX_FIFO1, &fdcan2_rxframe);
            fdcan2_rxframe.rx_time_stamp = bsp_dwt_get_timeline_us(); //记录接收时间戳，单位微秒
            fdcan2_rx_callback(&fdcan2_rxframe);//调用自定义的中断处理函数，处理接收到的消息
        }
    }
}

/**
 * @brief CAN总线离线完整恢复
 * @note  BUS_OFF后需要完整重新初始化过滤器和中断通知, 仅Stop/Start无法恢复收发功能
 */
static void fdcan_bus_off_recovery(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) return;

    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = 0x00000000;
    filter.FilterID2 = 0x00000000;

    // 1. 停止CAN
    HAL_FDCAN_Stop(hfdcan);

    // 2. 重新配置过滤器
    if ((hfdcan == &hfdcan1) || (hfdcan == &hfdcan3))
    {
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    }
    else if (hfdcan == &hfdcan2)
    {
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    }
    else
    {
        LOG_WARNING("[bsp_can] BusOff Recovery: Unknown hfdcan");
        return;
    }

    HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                  FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // 3. 重新激活中断通知
    if ((hfdcan == &hfdcan1) || (hfdcan == &hfdcan3))
    {
        HAL_FDCAN_ActivateNotification(hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
            FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0);
    }
    else if (hfdcan == &hfdcan2)
    {
        HAL_FDCAN_ActivateNotification(hfdcan,
            FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_BUS_OFF |
            FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0);
    }

    // 4. 启动CAN
    HAL_FDCAN_Start(hfdcan);

    // 5. 重新绑定接收帧句柄
    fdcan_handle_init(hfdcan);

    LOG_INFO("[bsp_can] BusOff Recovery complete for %p", (void*)hfdcan);
}

/**
 * @brief HAL库CAN错误中断
 *
 * @param hfdcan CAN编号
 * @param ErrorStatusITs 错误状态
 */
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    LOG_ERROR("[bsp_can] CAN Error Status Callback: hfdcan=%p, ErrorStatusITs=0x%08X", (void*)hfdcan, ErrorStatusITs);
    
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET)
    {
        // CAN总线离线, 完整恢复过滤器和中断
        fdcan_bus_off_recovery(hfdcan);
    }

    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != RESET)
    {
        // Error Passive: 仅记录日志, 不做 Stop/Start 恢复.
        //
        // 注意: 旧代码在此处调用 fdcan_bus_off_recovery() (Stop + Start),
        // 这会导致 CAN 控制器完全停止工作 ~0.4~0.8s, 造成:
        //   - GM6020 反馈帧丢失 → small_yaw_angle 冻结
        //   - DM MIT 控制帧发不出 → 电机失去扭矩 → pitch 被重力拉下坠
        //
        // 实际上 Error Passive 状态下节点仍可正常收发数据,
        // 只是发送前等待 8 个额外隐性位 (约 10µs@1Mbps).
        // CAN 协议会自动降低错误计数器, 总线无需人为干预.
        LOG_WARNING("[bsp_can] CAN Error Passive (ignored, auto-recover)");
    }

    // Error Warning 只做记录, 不需要干预
}


/*
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)  // 检查是否有新消息
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, (FDCAN_RxHeaderTypeDef *)&FDCAN1_RxFrame.Header, (uint8_t *)FDCAN1_RxFrame.Data) != HAL_OK) // 从接收FIFO0获取消息
        {
            // 在ISR中：避免记录日志或调用Error_Handler()。因此增加一个ISR安全的计数器，可在主循环中处理
            fdcan_rx_error_count++;
            return;
        }

            //调用自定义的中断回调函数. 
        fdcan_rx_callback(&fdcan1_rxframe);//调用自定义的中断处理函数，处理接收到的消息
    }
}
*/



