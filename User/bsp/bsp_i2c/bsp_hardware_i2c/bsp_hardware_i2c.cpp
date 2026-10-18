/**
 * @file bsp_i2c.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照MyPHC代码改写的I2C通信初始化与配置流程
 * @version 1.0
 * @date 2026-05-19 1.0 适配达妙MC02板、F103C8T6芯片，使用HAL库进行I2C通信配置和数据传输

 * @note 目前使用I2C 阻塞方式传输I2C数据，后续可以考虑增加I2C中断和DMA模式的传输函数，以满足不同应用场景的需求。
        同时可以在I2C接收完成回调函数中处理接收到的数据，或者将数据处理放在主循环中，通过信号量或消息队列通知主循环进行数据处理，以提高系统响应能力和性能。
        
        后续考虑使用C++类封装I2C管理器，提供更灵活的接口和更好的代码组织结构，同时支持多实例管理和不同工作模式的配置，以适应更复杂的应用需求。
 * @copyright liu_balance (c) 2026
 *
 */
#include "bsp_hardware_i2c.h"
#include <cstdint>

i2c_manger_object i2c1_managers;
i2c_manger_object i2c2_managers;

/**
    * @brief 初始化I2C管理器
    * @param hi2c I2C句柄
    * @param work_mode 工作模式
    * @param callback 接收回调函数
 */

void bsp_i2c_init(I2C_HandleTypeDef *hi2c, i2c_work_mode work_mode, i2c_callback callback)
{
    if (hi2c->Instance == I2C1) {
        i2c1_managers.hi2c = hi2c;
        i2c1_managers.work_mode = work_mode;
        i2c1_managers.callback_function = callback;
    }
    
    else if (hi2c->Instance == I2C2) {
        i2c2_managers.hi2c = hi2c;
        i2c2_managers.work_mode = work_mode;
        i2c2_managers.callback_function = callback; 
    }   
}

/**
 * @brief 阻塞方式写入I2C数据
 * @param hi2c I2C句柄
 * @param dev_address 设备地址
 * @param mem_address 内部地址
 * @param mem_add_size 内部地址大小
 * @param data 数据缓冲区
 * @param size 数据长度
 * @param timeout 超时时间
 * @return HAL状态
 */

uint8_t bsp_i2c_block_write(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (hi2c == nullptr || data == nullptr) {
        return HAL_ERROR; // 参数检查，避免空指针访问
    }

    if (hi2c->Instance == I2C1) 
    {
        if (i2c1_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Write(hi2c, dev_address, mem_address, mem_add_size, data, size, timeout);
        }
        else if (i2c1_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Transmit(hi2c, dev_address, data, size, timeout);
        }
    }

    else if (hi2c->Instance == I2C2) {
        if (i2c2_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Write(hi2c, dev_address, mem_address, mem_add_size, data, size, timeout);
        }
        else if (i2c2_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Transmit(hi2c, dev_address, data, size, timeout);
        }
    }
    return HAL_ERROR;
}

/**
 * @brief 阻塞方式读取I2C数据
 * @param hi2c I2C句柄
 * @param dev_address 设备地址
 * @param mem_address 内部地址
 * @param mem_add_size 内部地址大小
 * @param data 数据缓冲区
 * @param size 数据长度
 * @param timeout 超时时间
 * @return HAL状态
 */

uint8_t bsp_i2c_block_read(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (hi2c == nullptr || data == nullptr) {
        return HAL_ERROR; // 参数检查，避免空指针访问
    }

    if (hi2c->Instance == I2C1) 
    {
        if (i2c1_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Read(hi2c, dev_address, mem_address, mem_add_size, data, size, timeout);
        }
        else if (i2c1_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Receive(hi2c, dev_address, data, size, timeout);
        }
    }

    else if (hi2c->Instance == I2C2) {
        if (i2c2_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Read(hi2c, dev_address, mem_address, mem_add_size, data, size, timeout);
        }
        else if (i2c2_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Receive(hi2c, dev_address, data, size, timeout);
        }
    }
    return HAL_ERROR;
}

/**
 * @brief 中断方式写入I2C数据
 * @param hi2c I2C句柄
 * @param dev_address 设备地址
 * @param mem_address 内部地址
 * @param mem_add_size 内部地址大小
 * @param data 数据缓冲区
 * @param size 数据长度
 * @return HAL状态
 */
uint8_t bsp_i2c_interrupt_write(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size)
{
    if (hi2c == nullptr || data == nullptr) {
        return HAL_ERROR; // 参数检查，避免空指针访问
    }

    if (hi2c->Instance == I2C1) 
    {
        if (i2c1_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Write_IT(hi2c, dev_address, mem_address, mem_add_size, data, size);
        }
        else if (i2c1_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Transmit_IT(hi2c, dev_address, data, size);
        }
    }

    else if (hi2c->Instance == I2C2) {
        if (i2c2_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Write_IT(hi2c, dev_address, mem_address, mem_add_size, data, size);
        }
        else if (i2c2_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Transmit_IT(hi2c, dev_address, data, size);
        }
    }
    return HAL_ERROR;
}

/**
 * @brief 中断方式读取I2C数据
 * @param hi2c I2C句柄
 * @param dev_address 设备地址
 * @param mem_address 内部地址
 * @param mem_add_size 内部地址大小
 * @param data 数据缓冲区
 * @param size 数据长度
 * @return HAL状态
 */
uint8_t bsp_i2c_interrupt_read(I2C_HandleTypeDef *hi2c, uint8_t dev_address, uint8_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size)
{
    if (hi2c == nullptr || data == nullptr) {
        return HAL_ERROR; // 参数检查，避免空指针访问
    }
    if (hi2c->Instance == I2C1) 
    {
        if (i2c1_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Read_IT(hi2c, dev_address, mem_address, mem_add_size, data, size);
        }
        else if (i2c1_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Receive_IT(hi2c, dev_address, data, size);
        }
    }

    else if (hi2c->Instance == I2C2) {
        if (i2c2_managers.work_mode == I2C_MEM_MODE) {
            return HAL_I2C_Mem_Read_IT(hi2c, dev_address, mem_address, mem_add_size, data, size);
        }
        else if (i2c2_managers.work_mode == I2C_MAS_MODE) {
            return HAL_I2C_Master_Receive_IT(hi2c, dev_address, data, size);
        }
    }
    return HAL_ERROR;
}
