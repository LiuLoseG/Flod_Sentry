#include "usb_serialplot.h"

#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

#include "math.h"

#define MAX_BUFFER_SERIALPLOT_SIZE 1024

//extern float v1;

uint8_t send_serialplot_buf[MAX_BUFFER_SERIALPLOT_SIZE];
uint16_t serialplot_cnt = 0;

// SerialPlot 启动函数,将该函数添加到任务或主循环中即可发送数据
void serialplot_start(void)
{
    serialplot_damo();  
}

//通过USB发送数据
void serialplot_transmit(uint8_t* buf, uint16_t len)
{
    CDC_Transmit_HS((uint8_t *)buf, len);
}

//发送帧起始标志
void serialplot_send_start_data(void)
{
    send_serialplot_buf[serialplot_cnt++] = 0xAB;
}

//发送单个数据
void serialplot_send_data(uint8_t num, float data) 
{
    send_serialplot_buf[serialplot_cnt++] = word0(data);
    send_serialplot_buf[serialplot_cnt++] = word1(data);
    send_serialplot_buf[serialplot_cnt++] = word2(data);
    send_serialplot_buf[serialplot_cnt++] = word3(data);
}

//发送帧结束标志
void serialplot_send_frame_end_data(void)
{
    serialplot_transmit((uint8_t *)send_serialplot_buf, serialplot_cnt);
    serialplot_cnt = 0;
}

//发送数据
void serialplot_damo(void)
{
    static float scnt = 0.0f;

	scnt += 0.01f;

	if(scnt >= 360.0f)
		scnt = 0.0f;

	float v1 = scnt;
    serialplot_send_start_data();       //帧起始标志
    serialplot_send_data(0, v1);        //发送(float类型)数据v1
    serialplot_send_frame_end_data();   //帧结束,打包数据发送。
}
