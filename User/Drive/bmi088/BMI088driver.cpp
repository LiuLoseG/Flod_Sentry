/**
 * @file BMI088driver.cpp
 * @author Robin Liu (19038200162@163.com)
 * @brief BMI088 六轴 IMU 驱动(SPI): 初始化 + 一帧数据突发读取
 * @note  1) 加速度计与陀螺仪为两颗独立 die, 片选独立
 *        2) 加速度计 SPI 读时序: 地址|0x80 + 1 个哑字节 + 数据; 陀螺仪: 地址|0x80 + 数据
 *        3) 一帧数据只用 3 次突发传输(加速度 6B / 陀螺仪 8B / 温度 2B), 替代原来逐字节收发
 *           (约 17 次单字节 HAL 调用)的做法, 1kHz 采样下可省下数倍 SPI 时间
 *        4) 输出单位: 加速度 m/s^2, 角速度 rad/s, 温度 ℃
 * @date 2026-10-26
 * @version 0.2
 * @copyright HNAU-Robot (c) 2026
 */
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

#include <string.h>

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#if defined(BMI088_USE_SPI)
// 单次突发传输缓冲区大小: 地址(1) + 哑字节(1) + 数据(最大 8)
#define BMI088_SPI_FRAME_SIZE 12U

typedef enum
{
    BMI088_DEV_ACCEL = 0,
    BMI088_DEV_GYRO,
} bmi088_device_e;

static uint8_t bmi088_tx_frame[BMI088_SPI_FRAME_SIZE];
static uint8_t bmi088_rx_frame[BMI088_SPI_FRAME_SIZE];

/**
************************************************************************
* @brief:      	bmi088_cs_low / bmi088_cs_high
* @param:       dev - 目标(加速度计/陀螺仪)
* @retval:     	void
* @details:    	片选控制, 避免在读写函数里重复写 if/else
************************************************************************
**/
static void bmi088_cs_low(bmi088_device_e dev)
{
    if (dev == BMI088_DEV_ACCEL)
    {
        BMI088_ACCEL_NS_L();
    }
    else
    {
        BMI088_GYRO_NS_L();
    }
}

static void bmi088_cs_high(bmi088_device_e dev)
{
    if (dev == BMI088_DEV_ACCEL)
    {
        BMI088_ACCEL_NS_H();
    }
    else
    {
        BMI088_GYRO_NS_H();
    }
}

/**
************************************************************************
* @brief:      	bmi088_reg_read(dev, reg, buf, len)
* @param:       dev - 目标(加速度计/陀螺仪)
*               reg - 起始寄存器地址
*               buf - 数据缓冲区
*               len - 读取字节数
* @retval:     	uint8_t - 0 成功, 1 失败
* @details:    	单次片选内完成"地址+哑字节+len 字节数据"的突发读
************************************************************************
**/
static uint8_t bmi088_reg_read(bmi088_device_e dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    // 加速度计读操作在地址后需要 1 个哑字节, 陀螺仪不需要
    const uint16_t head = (dev == BMI088_DEV_ACCEL) ? 2U : 1U;
    const uint16_t total = (uint16_t)(head + len);

    if ((buf == 0) || (len == 0U) || (total > BMI088_SPI_FRAME_SIZE))
    {
        return 1U;
    }

    bmi088_tx_frame[0] = (uint8_t)(reg | 0x80U);
    for (uint16_t i = 1U; i < total; i++)
    {
        bmi088_tx_frame[i] = 0x55U;
    }

    bmi088_cs_low(dev);
    uint8_t status = BMI088_transmit_receive(bmi088_tx_frame, bmi088_rx_frame, total);
    bmi088_cs_high(dev);

    if (status != 0U)
    {
        return 1U;
    }

    memcpy(buf, &bmi088_rx_frame[head], len);
    return 0U;
}

/**
************************************************************************
* @brief:      	bmi088_reg_write(dev, reg, data)
* @param:       dev - 目标(加速度计/陀螺仪)
*               reg - 寄存器地址
*               data - 写入的数据
* @retval:     	uint8_t - 0 成功, 1 失败
* @details:    	单次片选内写入 1 个寄存器
************************************************************************
**/
static uint8_t bmi088_reg_write(bmi088_device_e dev, uint8_t reg, uint8_t data)
{
    bmi088_tx_frame[0] = (uint8_t)(reg & 0x7FU);
    bmi088_tx_frame[1] = data;

    bmi088_cs_low(dev);
    uint8_t status = BMI088_transmit_receive(bmi088_tx_frame, bmi088_rx_frame, 2U);
    bmi088_cs_high(dev);

    return status;
}
#endif

/**
************************************************************************
* @brief:      	write_BMI088_accel_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088加速度传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
/**
************************************************************************
* @brief:      	write_BMI088_gyro_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088陀螺仪传感器寄存器数据写入错误处理初始化
************************************************************************
**/
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
/**
************************************************************************
* @brief:      	BMI088_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088传感器初始化函数，包括GPIO和SPI初始化，以及加速度和陀螺仪的初始化
************************************************************************
**/
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}
/**
************************************************************************
* @brief:      	bmi088_accel_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088加速度传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    if (bmi088_reg_read(BMI088_DEV_ACCEL, BMI088_ACC_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (bmi088_reg_read(BMI088_DEV_ACCEL, BMI088_ACC_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    if (bmi088_reg_write(BMI088_DEV_ACCEL, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    if (bmi088_reg_read(BMI088_DEV_ACCEL, BMI088_ACC_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (bmi088_reg_read(BMI088_DEV_ACCEL, BMI088_ACC_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        if (bmi088_reg_write(BMI088_DEV_ACCEL, write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]) != 0U)
        {
            return BMI088_COM_ERROR;
        }
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (bmi088_reg_read(BMI088_DEV_ACCEL, write_BMI088_accel_reg_data_error[write_reg_num][0], &res, 1U) != 0U)
        {
            return BMI088_COM_ERROR;
        }
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:      	bmi088_gyro_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088陀螺仪传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    if (bmi088_reg_read(BMI088_DEV_GYRO, BMI088_GYRO_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (bmi088_reg_read(BMI088_DEV_GYRO, BMI088_GYRO_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    if (bmi088_reg_write(BMI088_DEV_GYRO, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    if (bmi088_reg_read(BMI088_DEV_GYRO, BMI088_GYRO_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (bmi088_reg_read(BMI088_DEV_GYRO, BMI088_GYRO_CHIP_ID, &res, 1U) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        if (bmi088_reg_write(BMI088_DEV_GYRO, write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]) != 0U)
        {
            return BMI088_COM_ERROR;
        }
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (bmi088_reg_read(BMI088_DEV_GYRO, write_BMI088_gyro_reg_data_error[write_reg_num][0], &res, 1U) != 0U)
        {
            return BMI088_COM_ERROR;
        }
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:      	BMI088_read_imu(float gyro[3], float accel[3])
* @param:       gyro  - 角速度数组 (x, y, z), 单位 rad/s
*               accel - 加速度数组 (x, y, z), 单位 m/s^2
* @retval:     	uint8_t - BMI088_NO_ERROR 表示成功
* @details:    	突发读取加速度 6B + 陀螺仪 8B(含 chip id 自检), 不含温度
* @note        先在局部缓冲区完成解析, 全部成功后才写回调用者缓冲区,
*              保证失败时输出与调用前完全一致(不会出现"加速度已更新、陀螺仪没更新"的半帧)
************************************************************************
**/
uint8_t BMI088_read_imu(float gyro[3], float accel[3])
{
    uint8_t buf[BMI088_GYRO_BURST_LEN] = {0};
    float accel_local[3] = {0.0f, 0.0f, 0.0f};
    float gyro_local[3] = {0.0f, 0.0f, 0.0f};
    int16_t raw = 0;
    uint16_t i = 0;

    if ((gyro == 0) || (accel == 0))
    {
        return BMI088_COM_ERROR;
    }

    // 加速度计 0x12 ~ 0x17
    if (bmi088_reg_read(BMI088_DEV_ACCEL, BMI088_ACCEL_XOUT_L, buf, BMI088_ACCEL_BURST_LEN) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    for (i = 0; i < 3U; i++)
    {
        raw = (int16_t)(((uint16_t)buf[2U * i + 1U] << 8) | buf[2U * i]);
        accel_local[i] = (float)raw * BMI088_ACCEL_SEN;
    }

    // 陀螺仪 0x00 ~ 0x07: 首字节为 chip id, 用于通信自检
    if (bmi088_reg_read(BMI088_DEV_GYRO, BMI088_GYRO_CHIP_ID, buf, BMI088_GYRO_BURST_LEN) != 0U)
    {
        return BMI088_COM_ERROR;
    }
    if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }
    for (i = 0; i < 3U; i++)
    {
        raw = (int16_t)(((uint16_t)buf[2U * i + 3U] << 8) | buf[2U * i + 2U]);
        gyro_local[i] = (float)raw * BMI088_GYRO_SEN;
    }

    for (i = 0; i < 3U; i++)
    {
        accel[i] = accel_local[i];
        gyro[i] = gyro_local[i];
    }

    return BMI088_NO_ERROR;
}

/**
************************************************************************
* @brief:      	BMI088_read_temperature(float *temperature)
* @param:       temperature - 芯片温度, 单位 ℃
* @retval:     	uint8_t - BMI088_NO_ERROR 表示成功
* @details:    	读取温度寄存器 0x22 ~ 0x23
* @note        温度是 11bit 有符号: TEMP_M 全 8 位 + TEMP_L 高 3 位,
*              0.125℃/LSB, 偏置 +23℃, 超过 BMI088_TEMP_SIGN_BIT 时减 2048 做符号扩展
************************************************************************
**/
uint8_t BMI088_read_temperature(float *temperature)
{
    uint8_t buf[BMI088_TEMP_BURST_LEN] = {0};
    int16_t raw = 0;

    if (temperature == 0)
    {
        return BMI088_COM_ERROR;
    }

    if (bmi088_reg_read(BMI088_DEV_ACCEL, BMI088_TEMP_M, buf, BMI088_TEMP_BURST_LEN) != 0U)
    {
        return BMI088_COM_ERROR;
    }

    raw = (int16_t)(((uint16_t)buf[0] << 3) | (uint16_t)(buf[1] >> 5));
    if (raw > (BMI088_TEMP_SIGN_BIT - 1))
    {
        raw -= (2 * BMI088_TEMP_SIGN_BIT);
    }

    *temperature = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    return BMI088_NO_ERROR;
}

/**
************************************************************************
* @brief:      	BMI088_read_status(float gyro[3], float accel[3], float *temperature)
* @param:       gyro - 角速度数组 (x, y, z), 单位 rad/s
*               accel - 加速度数组 (x, y, z), 单位 m/s^2
*               temperature - 芯片温度, 单位 ℃
* @retval:     	uint8_t - BMI088_NO_ERROR 表示成功
* @details:    	读取一帧完整数据: 加速度 + 角速度 + 温度(3 次突发事务)
************************************************************************
**/
uint8_t BMI088_read_status(float gyro[3], float accel[3], float *temperature)
{
    float gyro_local[3] = {0.0f, 0.0f, 0.0f};
    float accel_local[3] = {0.0f, 0.0f, 0.0f};
    float temperature_local = 0.0f;
    uint16_t i = 0;

    if ((gyro == 0) || (accel == 0) || (temperature == 0))
    {
        return BMI088_COM_ERROR;
    }

    uint8_t status = BMI088_read_imu(gyro_local, accel_local);
    if (status != BMI088_NO_ERROR)
    {
        return status;
    }

    status = BMI088_read_temperature(&temperature_local);
    if (status != BMI088_NO_ERROR)
    {
        return status;
    }

    for (i = 0; i < 3U; i++)
    {
        gyro[i] = gyro_local[i];
        accel[i] = accel_local[i];
    }
    *temperature = temperature_local;

    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:      	BMI088_read(float gyro[3], float accel[3], float *temperate)
* @param:       gyro - 陀螺仪数据数组 (x, y, z), 单位 rad/s
*               accel - 加速度计数据数组 (x, y, z), 单位 m/s^2
*               temperate - 温度数据指针, 单位 ℃
* @retval:     	void
* @details:    	兼容旧接口: 读失败时保留调用者缓冲区中的上一帧数据
************************************************************************
**/
void BMI088_read(volatile float gyro[3], volatile float accel[3], volatile float *temperate)
{
    float gyro_data[3] = {0.0f, 0.0f, 0.0f};
    float accel_data[3] = {0.0f, 0.0f, 0.0f};
    float temperature = 0.0f;
    uint16_t i = 0;

    if (BMI088_read_status(gyro_data, accel_data, &temperature) != BMI088_NO_ERROR)
    {
        return;
    }

    for (i = 0; i < 3U; i++)
    {
        gyro[i] = gyro_data[i];
        accel[i] = accel_data[i];
    }
    *temperate = temperature;
}
