#include "wit_imu.h"

volatile wit_gyro wit_data;	//使用volatile，防止中断中调用本结构体导致数据错乱

/**
 * @brief 解析WIT传感器数据
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void wit_gyro::wit_data_analysis(uint8_t *buf, uint16_t len) volatile
{
    if (buf == nullptr || len < WIT_DATA_SIZE) {
        return;
    }

    uint16_t offset = 0;
    while (offset + WIT_DATA_SIZE <= len) {
        if (buf[offset] != WIT_DATA_TYPE_PORT) {
            ++offset;
            continue;
        }

        uint8_t type = buf[offset + 1];
        if (type != WIT_DATA_TYPE_TIME && type != WIT_DATA_TYPE_ACC &&
            type != WIT_DATA_TYPE_GYRO && type != WIT_DATA_TYPE_ANGLE &&
            type != WIT_DATA_TYPE_MAG && type != WIT_DATA_TYPE_QUAT) {
            ++offset;
            continue;
        }

        const uint8_t *frame = &buf[offset];    // 编译器保护，防止意外写入

        switch (type)
        {
            case WIT_DATA_TYPE_TIME: //时间
                time.year = frame[2];
                time.month = frame[3];
                time.day = frame[4];
                time.hour = frame[5];
                time.minute = frame[6];
                time.second = frame[7];
                time.millisecond = (frame[9] << 8 | frame[8]);
                break;

            case WIT_DATA_TYPE_ACC: //加速度
                acc.accx = (int16_t)(frame[3] << 8 | frame[2]) / 32768.0f * 16.0f * 9.8f;
                acc.accy = (int16_t)(frame[5] << 8 | frame[4]) / 32768.0f * 16.0f * 9.8f;
                acc.accz = (int16_t)(frame[7] << 8 | frame[6]) / 32768.0f * 16.0f * 9.8f;
                acc.Temperature = (int16_t)(frame[9] << 8 | frame[8]) / 100.0f;
                break;

            case WIT_DATA_TYPE_GYRO: //角速度
                gyro.gyrox = (int16_t)(frame[3] << 8 | frame[2]) / 32768.0f * 2000.0f;
                gyro.gyroy = (int16_t)(frame[5] << 8 | frame[4]) / 32768.0f * 2000.0f;
                gyro.gyroz = (int16_t)(frame[7] << 8 | frame[6]) / 32768.0f * 2000.0f;
                gyro.Voltage = (int16_t)(frame[9] << 8 | frame[8]) / 100.0f;
                break;

            case WIT_DATA_TYPE_ANGLE: //欧拉角
                angel.roll = (int16_t)(frame[3] << 8 | frame[2]) / 32768.0f * 180.0f;
                angel.pitch = (int16_t)(frame[5] << 8 | frame[4]) / 32768.0f * 180.0f;
                angel.yaw = (int16_t)(frame[7] << 8 | frame[6]) / 32768.0f * 180.0f;
                angel.Version = (int16_t)(frame[9] << 8 | frame[8]);
                break;

            case WIT_DATA_TYPE_MAG: //磁力计
                mag.magx = (int16_t)(frame[3] << 8 | frame[2]);
                mag.magy = (int16_t)(frame[5] << 8 | frame[4]);
                mag.magz = (int16_t)(frame[7] << 8 | frame[6]);
                mag.Temperature = (int16_t)(frame[9] << 8 | frame[8]) / 100.0f;
                break;

            case WIT_DATA_TYPE_QUAT: //四元数
                quat.q0 = (int16_t)(frame[3] << 8 | frame[2]) / 32768.0f;
                quat.q1 = (int16_t)(frame[5] << 8 | frame[4]) / 32768.0f;
                quat.q2 = (int16_t)(frame[7] << 8 | frame[6]) / 32768.0f;
                quat.q3 = (int16_t)(frame[9] << 8 | frame[8]) / 32768.0f;
                break;

            default:
                break;
        }

        offset += WIT_DATA_SIZE;
    }
}

/**
 * @brief WIT传感器UART回调函数
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void wit_gyro_uart_callback(uint8_t *buf, uint16_t len)
{
    wit_data.wit_data_analysis(buf, len);
}
