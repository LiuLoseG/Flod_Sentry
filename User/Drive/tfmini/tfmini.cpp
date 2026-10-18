#include "tfmini.h"

volatile tfmini_data_t tfmini_data; //使用volatile，防止中断中调用本结构体导致数据错乱

void tfmini_data_analysis(uint8_t *buf, uint16_t len)
{
    if (buf == nullptr || len < TFMINI_DATA_SIZE)
    {
        return;
    }

    uint16_t offset = 0;
    while (offset + TFMINI_DATA_SIZE <= len)
    {
        // 检查数据帧头是否正确，确保数据帧的起始位置有效
        if (buf[offset] != TFMINI_Header)
        {
            ++offset;
            continue;
        }

        const uint8_t *frame = &buf[offset];    // 编译器保护，防止意外写入

        // 解析距离数据
        tfmini_data.distance = (float)(int16_t)(frame[1] << 8 | frame[2]);

        // 解析信号强度数据
        tfmini_data.strength = frame[3] << 8 | frame[4];

        // 解析温度数据
        tfmini_data.temp = (float)(int16_t)(frame[5] << 8 | frame[6]) / 8.0f - 256.0f;

        offset += TFMINI_DATA_SIZE;
    }
}