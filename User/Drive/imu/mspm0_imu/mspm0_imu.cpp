#include "mspm0_imu.h"

volatile mspm0_imu_t mspm0_data; // 使用volatile，防止中断中调用本结构体导致数据错乱

                                // KEY（解锁）
// uint8_t mspm0_imu_register_unlock[5] = {0x55, 0xAA, 0x13, 0x8E, 0x5F}; // MSPM0 IMU寄存器解锁命令
                                // 保存指令
// uint8_t mspm0_imu_register_save[5] = {0x55, 0xAA, 0x00, 0x00, 0x00};  // MSPM0 IMU寄存器数据保存指令
                                // 重启指令
// uint8_t mspm0_imu_register_restart[5] = {0x55, 0xAA, 0x00, 0xFF, 0x00};  // MSPM0 IMU寄存器重启指令
                                // 恢复出厂指令
// uint8_t mspm0_imu_register_reset[5] = {0x55, 0xAA, 0x00, 0x01, 0x00};  // MSPM0 IMU寄存器恢复出厂设置指令
                                // Yaw_Zero（Z 轴角度归零）
// uint8_t mspm0_imu_register_angle_zero[5] = {0x55, 0xAA, 0x15, 0x00, 0x00};  // MSPM0 IMU寄存器校准指令
                                // BIAS_CAL（自动获取零偏）
// uint8_t mspm0_imu_register_zero_offset[5] = {0x55, 0xAA, 0x0A, 0x01, 0x00};  // MSPM0 IMU寄存器自动获取零偏指令
                                // 读取 0A 寄存器状态
// uint8_t mspm0_imu_register_read_0a[5] = {0x55, 0xAA, 0x04, 0x0A, 0x00};  // MSPM0 IMU寄存器读取0A寄存器指令
                                // 获取零偏成功
// uint8_t mspm0_imu_register_return_0a[5] = {0x5A, 0xCC, 0x00, 0x00, 0x96};  // 设置零偏成功
                                // Scale_Factor（Z 轴标定）,开始标定指令
// uint8_t mspm0_imu_register_calibration_start[5] = {0x55, 0xAA, 0x0A, 0x03, 0x00};  // MSPM0 IMU寄存器开始校准指令
                                // Scale_Factor（Z 轴标定）,停止标定指令
// uint8_t mspm0_imu_register_calibration_stop[5] = {0x55, 0xAA, 0x00, 0x00, 0x00};  // MSPM0 IMU寄存器停止校准指令

/**
 * @brief 分析MSPM0 IMU数据
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void mspm0_imu_data_analysis(uint8_t *buf, uint16_t len)
{
    if (buf == nullptr || len < MSPM0_IMU_DATA_SIZE) 
    {
        return;
    }

    uint8_t offset = 0;

    while(offset + MSPM0_IMU_DATA_SIZE <= len)
    {
        // 检查数据帧头是否正确，确保数据帧的起始位置有效
        if (buf[offset] != MSPM0_IMU_Header)
        {
            ++offset;
            continue;
        }

        uint8_t type = buf[offset + 1];

        // 检查数据类型是否有效
        if (type != MSPM0_IMU_ACCELEROMETER && type != MSPM0_IMU_ANGLE)
        {
            ++offset;
            continue;
        }

        const uint8_t *frame = &buf[offset];
        
        // 校验和验证：检查数据帧的校验和是否正确，确保数据的完整性
        if (frame[4] != ((frame[0] + frame[1] + frame[2] + frame[3]) & 0xFF))
        {
            ++offset;
            continue;
        }

        // 解析数据帧，根据类型更新对应的IMU数据结构体成员
        switch (type)
        {
            case MSPM0_IMU_ACCELEROMETER: // 角速度数据
                mspm0_data.angular = (float)((int16_t)(frame[3] << 8 | frame[2]) / 32768.0f * 2000.0f);
                break;

            case MSPM0_IMU_ANGLE: // 角度数据
                mspm0_data.angle = (float)((int16_t)(frame[3] << 8 | frame[2]) / 32768.0f * 180.0f);
                break;

            default:
                break;
        }

        offset += MSPM0_IMU_DATA_SIZE;
    }

}
