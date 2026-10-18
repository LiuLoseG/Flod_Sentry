#pragma once

#include "stm32f1xx_hal.h"

#include "main.h"
#include "gpio.h"

#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN MPU6050_SCL_Pin
#define I2C_SDA_PIN MPU6050_SDA_Pin

/*
    * 软件I2C引脚操作宏定义
    * ODR寄存器可读可写：既能控制管脚为高电平，也能控制管脚为低电平。
    * BRR 只写寄存器：只能改变管脚状态为低电平
    * BRR 只写寄存器：只能改变管脚状态为高电平
    * IDR寄存器只读寄存器：只能读取管脚状态
*/

#define i2c_scl_high() I2C_SCL_PORT->BSRR = I2C_SCL_PIN /* SCL = 1 */
#define i2c_scl_low() I2C_SCL_PORT->BRR = I2C_SCL_PIN   /* SCL = 0 */

#define i2c_sda_high() I2C_SCL_PORT->BSRR = I2C_SDA_PIN /* SDA = 1 */
#define i2c_sda_low() I2C_SCL_PORT->BRR = I2C_SDA_PIN   /* SDA = 0 */

#define i2c_read_sda() ((I2C_SCL_PORT->IDR & I2C_SDA_PIN) != 0) /* 读SDA口线状态 */

void i2c_gpio_Init(void);
void i2c_Start(void);
void i2c_Stop(void);
uint8_t i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
void i2c_SendByte(uint8_t data);
uint8_t i2c_ReadByte(void);