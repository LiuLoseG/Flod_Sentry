#pragma once

#include "stdint.h"
#include "bsp_uart.h"

#define VOFA_UART huart1

#ifdef __cplusplus
extern "C" {
#endif

//将float数据转换为字节数组
#define byte0(dw_temp)     (*(char *)(&dw_temp))
#define byte1(dw_temp)     (*((char *)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char *)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char *)(&dw_temp) + 3))

void uart_vofa_start(void);
void uart_vofa_transmit(uint8_t *buf, uint16_t len);
void uart_vofa_send_data(uint8_t num, float data);
void uart_vofa_send_frame_end_data(void);
void uart_vofa_send_damo(void);

void uart_vofa_data_process(uint8_t *buf, uint16_t len);
uint8_t uart_vofa_get_rx_flag(void);
float uart_vofa_rx_packet_data_analyse(void);

uint8_t uart_vofa_get_id_flag(void);

void uart_vofa_data_rx(void);

#ifdef __cplusplus
}
#endif


