#include "ops_9.h"

volatile ops_9_location_message_t ops_9_data;

/**
 * @brief 拼接字符串
 * @param str_destination 目标字符串
 * @param str_source 源字符串
 * @param num 拼接字符数
 */
void ops_9_concatenation_string(char str_destination[], char str_source[], int num)
{
    int i = 0, j = 0;

    while (str_destination[i] != '\0') i++;

    for (j = 0; j < num; j++)
    {
        str_destination[i++] = str_source[j];
    }
}

/**
 * @brief 更新X轴数据
 * @param huart UART句柄
 * @param now_x 当前X轴数据
 */
void ops_9_updata_x_axes(UART_HandleTypeDef *huart, float now_x)
{
    char update_now_x[9] = "ACTX";

    static union
    {
        float x;
        char x_char[4];
    } new_set_value;

    new_set_value.x = now_x;
    ops_9_concatenation_string(update_now_x, new_set_value.x_char, 4);

    bsp_uart_transmit(huart, (uint8_t *)&update_now_x, 9);
}

/**
 * @brief 更新Y轴数据
 * @param huart UART句柄
 * @param now_y 当前Y轴数据
 */
void ops_9_updata_y_axes(UART_HandleTypeDef *huart, float now_y)
{
    char update_now_y[9] = "ACTY";

    static union
    {
        float y;
        char y_char[4];
    } new_set_value;

    new_set_value.y = now_y;
    ops_9_concatenation_string(update_now_y, new_set_value.y_char, 4);

    bsp_uart_transmit(huart, (uint8_t *)&update_now_y, 9);
}

/**
 * @brief 更新航向角数据
 * @param huart UART句柄
 * @param now_yaw 当前航向角数据
 */
void ops_9_updata_yaw(UART_HandleTypeDef *huart, float now_yaw)
{
    char update_now_yaw[9] = "ACTJ";

    static union
    {
        float yaw;
        char yaw_char[4];
    } new_set_value;

    new_set_value.yaw = now_yaw;
    ops_9_concatenation_string(update_now_yaw, new_set_value.yaw_char, 4);

    bsp_uart_transmit(huart, (uint8_t *)&update_now_yaw, 9);
}

/**
 * @brief 数据解析
 * @param buf 缓冲区
 * @param len 缓冲区长度
 */
void ops_9_data_analyse(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < OPS9_DATA_SIZE)
    {
        return;
    }

    uint16_t offset = 0;

    while (offset + OPS9_DATA_SIZE <= len)
    {
        if (buf[offset] != OPS9_FIRST_HEADER)   // 查找帧头 
        {
            ++offset;
            continue;
        }

        uint8_t type = buf[offset + 1];   // 查找帧头下一字节，是否为匹配第二位帧头

        if (type != OPS9_SECOND_HEADER)
        {
            offset += 2;   // 如果不匹配，跳过这两个字节继续查找
            continue;
        }

        const uint8_t *frame = &buf[offset];    // 找到完整帧头开始的位置，从后开始解析数据

        ops_9_data.yaw = frame[2] << 24 | frame[3] << 16 | frame[4] << 8 | frame[5];
        ops_9_data.pitch = frame[6] << 24 | frame[7] << 16 | frame[8] << 8 | frame[9];
        ops_9_data.roll = frame[10] << 24 | frame[11] << 16 | frame[12] << 8 | frame[13];
        ops_9_data.x = frame[14] << 24 | frame[15] << 16 | frame[16] << 8 | frame[17];
        ops_9_data.y = frame[18] << 24 | frame[19] << 16 | frame[20] << 8 | frame[21];
        ops_9_data.yaw_angular = frame[22] << 24 | frame[23] << 16 | frame[24] << 8 | frame[25];

        offset += OPS9_DATA_SIZE;   // 处理完一帧数据后，跳过这帧数据继续查找下一帧
    }
}