#include "bsp_software_i2c.h"

/**
 * @brief 模拟I2C延时
 *
 */
static void i2c_Delay(void)
{
    volatile uint8_t i;

    for (i = 0; i < 5; i++)
        ;
}

/**
 * @brief 软件模拟I2C初始化
 *
 */
void i2c_gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pins : SCL_Pin SDA_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief CPU发起I2C总线启动信号
 *     _____
 * SDA      \_____________
 *     __________
 * SCL           \________
 */
void i2c_Start(void)
{
    i2c_sda_high();
    i2c_scl_high();
    i2c_Delay();
    i2c_sda_low();
    i2c_Delay();
    i2c_scl_low();
    i2c_Delay();
}

/**
 * @brief CPU发起I2C总线停止信号
 *                _______
 * SDA __________/
 *           ____________
 * SCL _____/
 */
void i2c_Stop(void)
{
    i2c_sda_low();
    i2c_scl_high();
    i2c_Delay();
    i2c_sda_high();
}

/**
 * @brief I2C 等待响应(CPU产生一个时钟，并读取器件的ACK应答信号)
 *
 * @return uint8_t
 */
uint8_t i2c_WaitAck(void)
{
    uint8_t re;
    i2c_sda_high();
    i2c_Delay();
    i2c_scl_high();
    i2c_Delay();
    if (i2c_read_sda())
    {
        re = 1;
    }
    else
    {
        re = 0;
    }
    i2c_scl_low();
    i2c_Delay();
    return re;
}

/**
 * @brief I2C 响应
 * @retval none
 * @author Mr.W
 * @date 2020-10-12
 */

/**
 * @brief CPU产生一个ACK信号
 *            ____
 * SCL ______/    \______
 *     ____         _____
 * SDA     \_______/
 */
void i2c_Ack(void)
{
    i2c_sda_low();
    i2c_Delay();
    i2c_scl_high();
    i2c_Delay();
    i2c_scl_low();
    i2c_Delay();
    i2c_sda_high();
}

/**
 * @brief I2C 不响应(CPU产生1个NACK信号)
 *            ____
 * SCL ______/    \______
 *     __________________
 * SDA
 */
void i2c_NAck(void)
{
    i2c_sda_high();
    i2c_Delay();
    i2c_scl_high();
    i2c_Delay();
    i2c_scl_low();
    i2c_Delay();
}

/**
 * @brief CPU向I2C总线设备发送8bit数据
 *
 * @param data
 */
void i2c_SendByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            i2c_sda_high();
        }
        else
        {
            i2c_sda_low();
        }

        i2c_Delay();
        i2c_scl_high();
        i2c_Delay();
        i2c_scl_low();
        if (i == 7)
        {
            i2c_sda_high();
        }
        data <<= 1;
        i2c_Delay();
    }
}

/**
 * @brief CPU从I2C总线设备读取8bit数据
 *
 * @return uint8_t
 */
uint8_t i2c_ReadByte(void)
{
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        i2c_scl_high();
        i2c_Delay();
        if (i2c_read_sda())
        {
            data++;
        }
        i2c_scl_low();
        i2c_Delay();
    }

    return data;
}

