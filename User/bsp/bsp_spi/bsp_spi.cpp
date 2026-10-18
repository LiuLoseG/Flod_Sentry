/**
 * @file bsp_spi.cpp
 * @author liuguobing
 * @brief SPI底层驱动实现配置流程
 * @version 1.0
 */

#include "bsp_spi.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include <cstdint>
#include <cstring>

spi_class_manager_object spi1_manager_object;
spi_class_manager_object spi2_manager_object;
spi_class_manager_object spi3_manager_object;
spi_class_manager_object spi4_manager_object;
spi_class_manager_object spi5_manager_object;
spi_class_manager_object spi6_manager_object;

static constexpr uint32_t SPI_BLOCKING_TIMEOUT_MS = 100U;

/**
 * @brief 根据SPI句柄获取对应的管理对象指针
 * @param hspi SPI句柄
 * @return 对应的管理对象指针，若无匹配则返回nullptr
 */

static spi_class_manager_object* spi_manager_object_pointer(SPI_HandleTypeDef *hspi)
{
    if (hspi == nullptr) { return nullptr; }

    if (hspi->Instance == SPI1) { return &spi1_manager_object; } 

    else if (hspi->Instance == SPI2) { return &spi2_manager_object; }

    else if (hspi->Instance == SPI3) { return &spi3_manager_object; }

    else if (hspi->Instance == SPI4) { return &spi4_manager_object; }

    else if (hspi->Instance == SPI5) { return &spi5_manager_object; }

    else if (hspi->Instance == SPI6) { return &spi6_manager_object; }

    else { return nullptr; } // 无匹配
}

static bool spi_cs_is_configured(const spi_class_manager_object *spi_manager_object)
{
    return (spi_manager_object != nullptr) && (spi_manager_object->get_spi_cs_gpio_port() != nullptr);
}

static GPIO_PinState spi_inactive_level(GPIO_PinState active_level)
{
    return (active_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void spi_set_cs_active(spi_class_manager_object *spi_manager_object)
{
    if (spi_cs_is_configured(spi_manager_object))
    {
        HAL_GPIO_WritePin(spi_manager_object->get_spi_cs_gpio_port(),
                          spi_manager_object->get_spi_cs_gpio_pin(),
                          spi_manager_object->get_spi_cs_activate_level());
    }
}

static void spi_set_cs_inactive(spi_class_manager_object *spi_manager_object)
{
    if (spi_cs_is_configured(spi_manager_object))
    {
        HAL_GPIO_WritePin(spi_manager_object->get_spi_cs_gpio_port(),
                          spi_manager_object->get_spi_cs_gpio_pin(),
                          spi_inactive_level(spi_manager_object->get_spi_cs_activate_level()));
    }
}

/**
 * @brief 初始化SPI
 * @param hspi SPI句柄
 * @param callback SPI回调函数
*/

void bsp_spi_init(SPI_HandleTypeDef *hspi, spi_call_back callback)
{
    if (hspi == nullptr) 
    {
        return; // 无效的SPI句柄，直接返回
    }

    spi_class_manager_object *spi_manager_object = spi_manager_object_pointer(hspi);
    if (spi_manager_object != nullptr)
    {
        spi_manager_object->set_spi_handle(hspi);
        spi_manager_object->set_spi_callback_function(callback);
    }
}

/**
 * @brief 发送数据
 *
 * @param hspi SPI编号
 * @param gpio_port 片选GPIO引脚编组
 * @param gpio_pin 片选GPIO引脚号
 * @param activate_level 片选GPIO引脚电平
 * @param tx_length 长度
 * @return uint8_t 执行状态 0: 成功, 1: 失败
 */

uint8_t bsp_spi_transmit_data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState activate_level, uint16_t tx_length)
{
    if ((hspi == nullptr) || (tx_length == 0U) || (tx_length > SPI_BUFFER_SIZE)) { return 1U; } // 无效参数，返回错误码
    spi_class_manager_object *spi_manager_object = spi_manager_object_pointer(hspi);
    if (spi_manager_object == nullptr) { return 1U; }

    spi_manager_object->set_spi_cs_gpio_port(gpio_port);
    spi_manager_object->set_spi_cs_gpio_pin(gpio_pin);
    spi_manager_object->set_spi_cs_activate_level(activate_level);
    spi_manager_object->set_spi_tx_length(tx_length);
    spi_manager_object->set_spi_rx_length(0); // 仅发送数据，接收长度设置为0
    if (spi_manager_object->get_spi_handler() == nullptr) { return 1U; }

    // 选定从设备的CS引脚，开始通信
    spi_set_cs_active(spi_manager_object);

    // 由于SPI6需要靠BDMA传输, 而BDMA只能访问SRAM4, 即外置SRAM, 因此只能使用阻塞传输
    if (hspi->Instance != SPI6) 
    {
        if (HAL_SPI_Transmit_DMA(spi_manager_object->get_spi_handler(), spi_manager_object->get_tx_buffer(), tx_length) != HAL_OK)
        {
            spi_set_cs_inactive(spi_manager_object);
            return 1U;
        }
        return 0U;
    }

    else 
    {
        if (HAL_SPI_Transmit(spi_manager_object->get_spi_handler(), spi_manager_object->get_tx_buffer(), tx_length, SPI_BLOCKING_TIMEOUT_MS) != HAL_OK)
        {
            spi_set_cs_inactive(spi_manager_object);
            return 1U;
        }
        spi_set_cs_inactive(spi_manager_object);
        return 0U;
    }
    
}

/**
 * @brief 交互数据帧
 * 
 * @param hspi SPI编号
 * @param gpio_port 片选GPIO引脚编组
 * @param gpio_pin 片选GPIO引脚号
 * @param activate_level 片选GPIO引脚电平
 * @param tx_length 发送数据长度
 * @param rx_length 接收数据长度
 * @return uint8_t 执行状态 0: 成功, 1: 失败
 */

uint8_t bsp_spi_transmit_receive_data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState activate_level, uint16_t tx_length, uint16_t rx_length)
{
    if ((hspi == nullptr) || (tx_length == 0U) || (rx_length == 0U)) { return 1U; }
    if ((tx_length > SPI_BUFFER_SIZE) || (rx_length > SPI_BUFFER_SIZE) || (static_cast<uint32_t>(tx_length) + static_cast<uint32_t>(rx_length) > SPI_BUFFER_SIZE))
    {
        return 1U;
    }

    spi_class_manager_object *spi_manager_object = spi_manager_object_pointer(hspi);
    if (spi_manager_object == nullptr) { return 1U; }

    spi_manager_object->set_spi_cs_gpio_port(gpio_port);
    spi_manager_object->set_spi_cs_gpio_pin(gpio_pin);
    spi_manager_object->set_spi_cs_activate_level(activate_level);
    spi_manager_object->set_spi_tx_length(tx_length);
    spi_manager_object->set_spi_rx_length(rx_length);
    if (spi_manager_object->get_spi_handler() == nullptr) { return 1U; }
    memset(spi_manager_object->get_tx_buffer() + tx_length, 0, rx_length); // 读阶段发送空字节提供时钟

    // 选定从设备的CS引脚，开始通信
    spi_set_cs_active(spi_manager_object);

    if (hspi->Instance != SPI6) 
    {
        if (HAL_SPI_TransmitReceive_DMA(spi_manager_object->get_spi_handler(), spi_manager_object->get_tx_buffer(), spi_manager_object->get_rx_buffer(), tx_length + rx_length) != HAL_OK)
        {
            spi_set_cs_inactive(spi_manager_object);
            return 1U;
        }
        return 0U;
    }

    else 
    {
        if (HAL_SPI_TransmitReceive(spi_manager_object->get_spi_handler(), spi_manager_object->get_tx_buffer(), spi_manager_object->get_rx_buffer(), tx_length + rx_length, SPI_BLOCKING_TIMEOUT_MS) != HAL_OK)
        {
            spi_set_cs_inactive(spi_manager_object);
            return 1U;
        }
        spi_set_cs_inactive(spi_manager_object);
        return 0U;
    }
}

/**
 * @brief HAL库SPI仅发送回调函数
 *
 * @param hspi SPI编号
 */

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_class_manager_object *spi_manager_object = spi_manager_object_pointer(hspi);
    spi_set_cs_inactive(spi_manager_object); // 拉高/拉低到非激活电平，结束通信
}

/**
 * @brief HAL库SPI全双工DMA中断
 *
 * @param hspi SPI编号
 */

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_class_manager_object *spi_manager_object = spi_manager_object_pointer(hspi);
    spi_set_cs_inactive(spi_manager_object); // 拉高/拉低到非激活电平，结束通信

    if (spi_manager_object != nullptr && spi_manager_object->get_spi_callback_function() != nullptr)
    {
        spi_manager_object->set_spi_callback_function(spi_manager_object->get_tx_buffer(), spi_manager_object->get_rx_buffer(), spi_manager_object->get_spi_tx_length(), spi_manager_object->get_spi_rx_length());
    }
}
