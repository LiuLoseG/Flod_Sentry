#pragma once

#include "stdint.h"
// #include "i2c.h"

#define BSP_I2C_MEMADD_SIZE_8BIT (0x00000001U)
#define BSP_I2C_MEMADD_SIZE_16BIT (0x00000002U)

typedef enum
{
    I2C_MEM_MODE = 0,    // 向一个具有内部地址空间的 I2C 设备写入数据,允许指定数据将被写入设备的对应内部地址(阻塞模式)
    I2C_MAS_MODE = 1,    // 向一个 I2C 设备写入数据,不指定内部地址(阻塞模式)
    I2C_IT_MODE = 2,     // 向一个 I2C 设备写入数据,不指定内部地址(中断模式)
    I2C_DMA_MODE = 3,    // 向一个 I2C 设备写入数据,不指定内部地址(DMA 模式)
} i2c_work_mode;

typedef void (*i2c_callback)(uint8_t *data, uint16_t length);

typedef struct
{
    I2C_HandleTypeDef *hi2c;    // I2C句柄
    i2c_work_mode work_mode;    // 工作模式

    uint8_t drv_address;        // 设备地址
    uint8_t mem_address;        // 内部地址

    uint8_t *rx_buffer;         // 接收缓冲区
    uint8_t rx_length;          // 接收数据长度

    i2c_callback callback_function;   // 接收回调函数

} i2c_manger_object;

extern i2c_manger_object i2c1_managers;
extern i2c_manger_object i2c2_managers;

void bsp_i2c_init(I2C_HandleTypeDef *hi2c, i2c_work_mode work_mode, i2c_callback callback);
uint8_t bsp_i2c_block_write(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout);
uint8_t bsp_i2c_block_read(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout);
uint8_t bsp_i2c_interrupt_write(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size);
uint8_t bsp_i2c_interrupt_read(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size);
