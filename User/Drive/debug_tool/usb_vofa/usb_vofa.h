#ifndef VOFA_H
#define VOFA_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

//将float数据转换为字节数组
#define byte0(dw_temp)     (*(char *)(&dw_temp))
#define byte1(dw_temp)     (*((char *)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char *)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char *)(&dw_temp) + 3))

void usb_vofa_start(void);
void usb_vofa_transmit(uint8_t *buf, uint16_t len);
void usb_vofa_send_data(uint8_t num,float data);
void usb_vofa_send_frame_end_data(void);
void usb_vofa_send_damo(void);

void usb_vofa_data_process(uint8_t* Buf, const uint32_t *Len);
uint8_t usb_vofa_get_rx_flag(void);
float usb_vofa_rx_packet_data_analyse(void);

uint8_t usb_vofa_get_id_flag(void);

void usb_vofa_data_rx(void);

#ifdef __cplusplus
}
#endif

#endif // VOFA_H

