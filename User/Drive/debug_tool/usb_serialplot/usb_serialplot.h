#pragma once
#include <stdint.h>

#define word0(rw_temp)     (*(char*)(&rw_temp))
#define word1(rw_temp)     (*((char*)(&rw_temp) + 1))
#define word2(rw_temp)     (*((char*)(&rw_temp) + 2))
#define word3(rw_temp)     (*((char*)(&rw_temp) + 3))

void serialplot_start(void);
void serialplot_transmit(uint8_t* buf, uint16_t len);
void serialplot_send_start_data(void);
void serialplot_send_data(uint8_t num, float data);
void serialplot_send_frame_end_data(void);
void serialplot_damo(void);
