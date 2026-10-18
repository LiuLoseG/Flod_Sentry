#include "crsf.h"

/**
 * @brief CRSF CRC8校验表
 */
static unsigned char crc8tab[256] = 
{
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crsf_send_buffer[64] = {CRSF_ADDRESS_FLIGHT_CONTROLLER};

crsf_boardcast_frame_t crsf_frame;

crsf_channels_t rc_channels;

// CRSF 解包FLIGHT_CONTROLLER
__STATIC_INLINE uint8_t  crsf_unpack_flight_controller(const crsf_boardcast_frame_t * frame)
{
    if (frame->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        const unsigned numOfChannels = 16;
        const unsigned srcBits = 11;
        const unsigned inputChannelMask = (1 << srcBits) - 1;

        rc_channels.link_status++;
        // code from BetaFlight rx/crsf.cpp / bitpacker_unpack
        uint8_t bitsMerged = 0;
        uint32_t readValue = 0;
        unsigned readByteIndex = 0;
        for (uint8_t n = 0; n < numOfChannels; n++)
        {
            while (bitsMerged < srcBits)
            {
                uint8_t readByte = frame->payload[readByteIndex++];
                readValue |= ((uint32_t) readByte) << bitsMerged;
                bitsMerged += 8;
            }
            rc_channels.ch[n] = (readValue & inputChannelMask);
            readValue >>= srcBits;
            bitsMerged -= srcBits;
        }

        return 1;
    }

    return 0;
}

/**
 * @brief 发送电池传感器数据
 * @param voltage 电压，V，四舍五入保留一位小数
 * @param current 电流，A，四舍五入保留一位小数
 * @param used_capacity 已用容量，mAh，整数
 * @param estimated_remaining_capacity 估计剩余容量，%，整数
 */
__STATIC_INLINE void crsf_send_battery_sensor(const float voltage, const float current, const uint32_t used_capacity, const uint8_t estimated_remaining_capacity)
{
    crsf_battery_sensor_t battery_data = {0};

    battery_data.voltage = REV(((voltage + .05f) * 10.f), 16);
    battery_data.current = REV(((current + .05f) * 10.f), 16);
    battery_data.used_capacity = REV(used_capacity, 24);
    battery_data.estimated_remaining_capacity = estimated_remaining_capacity;

    send_crsf_packet(CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t *)&battery_data, sizeof(battery_data));
}

/**
 * @brief 发送GPS数据
 * @param latitude 纬度，度，七位小数
 * @param longitude 经度，度，七位小数
 * @param altitude 海拔，米，增加1000米后取整
 * @param groundspeed 地面速度，千米每小时，乘以10取整
 * @param heading 方向，度，乘以100取整
 * @param satellites 卫星数量，整数
 */
__STATIC_INLINE void crsf_send_GPS(const float latitude, const float longitude, const uint16_t altitude, const float groundspeed, const float heading, const uint16_t satellites)
{
    crsf_gps_t gps_data = {.latitude = REV((latitude * 1e7f), 32),
                           .longitude = REV((longitude * 1e7f), 32),
                           .groundspeed = REV((groundspeed * 10), 16),
                           .heading = REV((heading * 100), 16),
                           .altitude = REV(altitude + 1000, 16),
                           .satellites = satellites};

    send_crsf_packet(CRSF_FRAMETYPE_GPS, (uint8_t *)&gps_data, sizeof(gps_data));
}


//__STATIC_INLINE static inline,静态内联函数，编译器会尽可能地将函数调用替换为函数体代码，以减少函数调用的开销。

/**
 * @brief 计算CRSF数据包的CRC8校验值
 * @param ptr 指向数据的指针
 * @param len 数据长度
 * @return 计算得到的CRC8校验值
 */
__STATIC_INLINE uint8_t crsf_calculate_crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}

/**
 * @brief 检查CRSF接收消息的完整性
 * @param msg 指向消息的指针
 * @param len 消息长度
 * @return 1表示消息完整，0表示消息不完整
 */
__STATIC_INLINE uint8_t crsf_rx_msg_check(const uint8_t *msg, uint8_t len)
{
    if(len < 4 || len > 64)
        return 0;
    if(msg[1] != len - 2)
        return 0;
    if(crsf_calculate_crc8(msg + 2, len - 3) != msg[len - 1])
        return 0;
    return 1;
}

/**
 * @brief CRSF接收空闲回调函数
 * @param buf 指向接收缓冲区的指针
 * @param len 缓冲区长度
 */
void crsf_rx_idle_callback(uint8_t *buf, uint16_t len)
{
    if(!crsf_rx_msg_check(buf, len))
    {
        return;
    }

    crsf_frame.addr = buf[0];
    crsf_frame.length = buf[1];
    crsf_frame.type = buf[2];
    memcpy(crsf_frame.payload, &buf[3], crsf_frame.length - 2);
    crsf_frame.crc = buf[len - 1];

    switch (crsf_frame.addr)
    {
        case CRSF_ADDRESS_FLIGHT_CONTROLLER:
            switch (crsf_frame.type)
            {
                case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                    // 16ch 遥控
                    crsf_unpack_flight_controller(&crsf_frame);
                    break;
                
                default:
                    break;
            }
                break;
    
        default:
            break;
    }
}

void mcu_send_crsf_packet(const CRSF_FRAMETYPE_t frame_type, const uint8_t *payload, const uint8_t size_of_payload)
{
    crsf_send_buffer[1] = size_of_payload + 2;
    crsf_send_buffer[2] = frame_type;
    memcpy(&crsf_send_buffer[3], payload, size_of_payload);
    crsf_send_buffer[size_of_payload + (4 - 1)] = crsf_calculate_crc8(&crsf_send_buffer[2], size_of_payload + 1);
    bsp_uart_transmit(&CRSF_UART, crsf_send_buffer, size_of_payload + 4);
    /* 注意: 不在此处 memset 清空缓冲区, 因为 bsp_uart_transmit 使用 DMA 异步发送,
       立即清空会导致 DMA 发出全零数据, 造成遥控器端 CRC 校验失败, 遥控信号丢失。
       缓冲区在下次发送时会被完全覆盖, 无需清空。 */
}

// 发送函数指针
void (*send_crsf_packet)(const CRSF_FRAMETYPE_t, const uint8_t *, const uint8_t) = mcu_send_crsf_packet;
