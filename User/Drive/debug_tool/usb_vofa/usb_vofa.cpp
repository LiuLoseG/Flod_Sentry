#include "usb_vofa.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "test.h"

#define MAX_BUFFER_VOFA_SIZE 516

//usb接收数据相关变量,接收vofa发送的数据
static volatile uint8_t usb_vofa_rx_data;               //当前处理的数据(一位一位处理，防止丢包和错位)
static volatile uint8_t usb_vofa_rx_flag;               //接收标志，1为接收到完整数据包

static volatile uint8_t usb_vofa_id_flag;               //数据ID标志，1为kp，2为ki，3为kd，

static const uint8_t usb_vofa_rx_packet_len = 100;      //接收数据包长度
static uint8_t usb_vofa_rx_packet[usb_vofa_rx_packet_len];                 //处理后的接收数据包

static uint8_t usb_vofa_data_bit_num = 0;               //数据位计数器

//外部声明要打印的变量

static uint16_t usb_vofa_cnt = 0;

extern float wave_out;
extern float vofa_out;
//发送缓冲区
static uint8_t usb_vofa_send_buffer[MAX_BUFFER_VOFA_SIZE];

/*---------------------------------------------usb_vofa发送数据处理------------------------------------------------*/

//开始发送数据
void usb_vofa_start(void)
{
    usb_vofa_send_damo();
}
//通过USB发送数据
void usb_vofa_transmit(uint8_t *buf, uint16_t len)
{
    CDC_Transmit_HS((uint8_t *)buf, len);
}
//发送单个数据
void usb_vofa_send_data(uint8_t num,float data)
{
    usb_vofa_send_buffer[usb_vofa_cnt++] = byte0(data);
    usb_vofa_send_buffer[usb_vofa_cnt++] = byte1(data);
    usb_vofa_send_buffer[usb_vofa_cnt++] = byte2(data);
    usb_vofa_send_buffer[usb_vofa_cnt++] = byte3(data);
}
//发送帧结束标志
void usb_vofa_send_frame_end_data(void)
{
    usb_vofa_send_buffer[usb_vofa_cnt++] = 0x00;
    usb_vofa_send_buffer[usb_vofa_cnt++] = 0x00;
    usb_vofa_send_buffer[usb_vofa_cnt++] = 0x80;
    usb_vofa_send_buffer[usb_vofa_cnt++] = 0x7f;

    usb_vofa_transmit((uint8_t *)usb_vofa_send_buffer, usb_vofa_cnt);
    usb_vofa_cnt = 0;//每发送完一帧数据，清零计数器
}
//发送damo数据
void usb_vofa_send_damo(void)
{
	usb_vofa_send_data(0, wave_out);
	// usb_vofa_send_data(1, roll);
	// usb_vofa_send_data(2, yaw);
	// usb_vofa_send_data(3, pitch_mahony);
	// usb_vofa_send_data(4, roll_mahony);
	// usb_vofa_send_data(5, yaw_mahony);
	// usb_vofa_send_data(6, temp);
    
    usb_vofa_send_frame_end_data();
}

/*---------------------------------------------usb_vofa接收数据处理------------------------------------------------*/

/**
 * @brief  数据解析函数
 * @param  Buf: 接收数据缓冲区
 * @param  Len: 接收数据长度指针
 * @retval 将USB中接收回调函数中的数据进行拆包解析
 */

void usb_vofa_data_process(uint8_t* Buf, const uint32_t *Len)
{
    static uint8_t rx_state = 0;            //状态机状态变量
    static uint8_t prx_packet = 0;          //接收数据包计数器

    if (Buf == NULL || Len == 0) {return;}      
    if (*Len == 0) {return;}

    for (uint32_t i = 0; i < *Len; i++)
    {
        usb_vofa_rx_data = Buf[i];

        switch (rx_state)
        {
            case 0:                                 //等待帧头 '#'
                if (usb_vofa_rx_data == 0x23)
                {
                    prx_packet = 0;
                    rx_state = 1;
                }
                break;

            case 1:                                 //等待 'P'
                if (usb_vofa_rx_data == 0x50) rx_state = 2;
                else if (usb_vofa_rx_data == 0x23) { prx_packet = 0; rx_state = 1; }
                else rx_state = 0;
                break;

            case 2:                                 //读取 ID 字符 
                if (usb_vofa_rx_data >= 0x30 && usb_vofa_rx_data <= 0x39)
                {
                    usb_vofa_id_flag = usb_vofa_rx_data - 0x30;  //ASCII '0' = 0x30 ~ '9' = 0x39
                    rx_state = 3; 
                }
                else//ID错误，重置状态机
                {
                    prx_packet = 0;
                    rx_state = 0;
                }
                break;

            case 3:                                 //等待 '='  
                if (usb_vofa_rx_data == 0x3D) { prx_packet = 0; rx_state = 4; }
                else { prx_packet = 0; rx_state = 0; }
                break;

            case 4:                                 //接收数据直到 '!'
                if (usb_vofa_rx_data == 0x21)//数据结束符 '!'，一包数据接收完成
                {
                    usb_vofa_data_bit_num = prx_packet;
                    prx_packet = 0;
                    rx_state = 0;
                    usb_vofa_rx_flag = 1;    //接收完成标志
                }
                else//接收数据打包
                {
                    usb_vofa_rx_packet[prx_packet++] = usb_vofa_rx_data;
                }
                break;

            default:                                //重置状态机
                prx_packet = 0;
                rx_state = 0;
                break;
        }
    }
}

/**
 * @brief  获取数据接收完成标志
 * @retval 返回rx_flag标志位
 */

uint8_t usb_vofa_get_rx_flag(void)
{
    if (usb_vofa_rx_flag == 1)
    {
        usb_vofa_rx_flag = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  获取数据接收完成标志
 * @retval 返回rx_flag标志位
 */

uint8_t usb_vofa_get_id_flag(void)
{
    return usb_vofa_id_flag;
}

/**
 * @brief 指数函数运算，用于处理小数点后的数据
 * @param x 需要处理的数据
 * @param n 指数
 * @note 本函数仅支持10的负整数次幂运算
 */

static float pow_invert(uint8_t x, uint8_t n)
{
    float result = x;
    while (n--)
    {
        result /= 10.0f;
    }
    return result;
}

/**
 * @brief  数据包处理函数
 * @retval 返回处理后的浮点数数据
 */

float usb_vofa_rx_packet_data_analyse(void)
{

    if (usb_vofa_data_bit_num == 0) { return 0.0f; }

    //每次进入此函数，下面定义的变量都会被重新初始化
    float data = 0.0f;
    uint8_t dot_flag = 0;
    uint8_t dot_after_num = 1;
    int8_t sign_flag = 1;

    for(uint8_t i = 0; i < usb_vofa_data_bit_num; i++)
    {
        if (usb_vofa_rx_packet[i] == 0x2D)   //处理负号'-'
        {
            if (i == 0){ sign_flag = -1; continue; }
            else { break; }         //中间出现'-'符号，数据错误，直接跳出循环
        }

        if (dot_flag == 0)  //处理小数点前的数据
        {
            if (usb_vofa_rx_packet[i] == 0x2E)
            {
                dot_flag = 1;
            }
            else
            {
                data = data * 10 + (usb_vofa_rx_packet[i] - 48);
            }
        }

        else                //处理小数点后的数据
        {
            data = data + pow_invert((usb_vofa_rx_packet[i] - 48), dot_after_num);
            dot_after_num++;
        }
    }

    return data * sign_flag;
}

void usb_vofa_data_rx(void)
{
    if (usb_vofa_get_rx_flag() == 1)
    {
        vofa_out = usb_vofa_rx_packet_data_analyse();
        // trigger_motor_pid[usb_vofa_get_id_flag() - 1] = usb_vofa_rx_packet_data_analyse();
        //yaw_motor_pid[usb_vofa_get_id_flag() - 1] = usb_vofa_rx_packet_data_analyse();
        //pitch_motor_pid[usb_vofa_get_id_flag() - 1] = usb_vofa_rx_packet_data_analyse();
    }
}
