#pragma once

#include "fdcan.h"
#include "stm32H7xx_hal.h"
#include "main.h"

#define CAN_SUCCESS     1
#define CAN_LINE_BUSY   0

#define FDCAN_SIGNAL_FIFO0    0
#define FDCAN_SIGNAL_FIFO1    1

// CAN总线发送频率枚举，适用于一拖四电机的CAN发送
typedef enum
{
    Can_Transmit_Frequency_Div1 = 0,
    Can_Transmit_Frequency_Div2,
    Can_Transmit_Frequency_Div4,
} Can_transmit_frequency_t;

typedef struct 
{
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint8_t rx_len;

    // 接收时间戳
    uint64_t rx_time_stamp;

}FDCAN_RxFrame_TypeDef;

extern volatile FDCAN_RxFrame_TypeDef fdcan1_rxframe;
extern volatile FDCAN_RxFrame_TypeDef fdcan2_rxframe;
extern volatile FDCAN_RxFrame_TypeDef fdcan3_rxframe;

extern uint8_t can1_0x1fe_tx_data[];    // (GM6020电流控制1-4)
extern uint8_t can1_0x1ff_tx_data[];    // (GM6020电压控制1-4)、(GM2006电流控制5-8)、(GM3508电流控制5-8)、(DM3519电流控制5-8)
extern uint8_t can1_0x200_tx_data[];    // (GM2006电流控制1-4)、(GM3508电流控制1-4)、(DM3519电流控制1-4)
extern uint8_t can1_0x2fe_tx_data[];    // (GM6020电流控制5-7)
extern uint8_t can1_0x2ff_tx_data[];    // (GM6020电压控制5-7)

extern uint8_t can2_0x1fe_tx_data[];    // (GM6020电流控制1-4)
extern uint8_t can2_0x1ff_tx_data[];    // (GM6020电压控制1-4)、(GM2006电流控制5-8)、(GM3508电流控制5-8)、(DM3519电流控制5-8)
extern uint8_t can2_0x200_tx_data[];    // (GM2006电流控制1-4)、(GM3508电流控制1-4)、(DM3519电流控制1-4)
extern uint8_t can2_0x2fe_tx_data[];    // (GM6020电流控制5-7)
extern uint8_t can2_0x2ff_tx_data[];    // (GM6020电压控制5-7)

extern uint8_t can3_0x1fe_tx_data[];    // (GM6020电流控制1-4)
extern uint8_t can3_0x1ff_tx_data[];    // (GM6020电压控制1-4)、(GM2006电流控制5-8)、(GM3508电流控制5-8)、(DM3519电流控制5-8)
extern uint8_t can3_0x200_tx_data[];    // (GM2006电流控制1-4)、(GM3508电流控制1-4)、(DM3519电流控制1-4)
extern uint8_t can3_0x2fe_tx_data[];    // (GM6020电流控制5-7)
extern uint8_t can3_0x2ff_tx_data[];    // (GM6020电压控制5-7)

void bsp_fdcan_init_all(void);
uint8_t bsp_fdcan_send(FDCAN_HandleTypeDef *hfdcan, uint16_t id,uint8_t *data, uint32_t len);
void bsp_can_1_to_4_transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t can_tx_id, uint8_t *tx_data, Can_transmit_frequency_t frequency);
