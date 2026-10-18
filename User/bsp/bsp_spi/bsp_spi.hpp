/**
 * @file drv_spi.hpp
 * @brief SPI驱动程序头文件
 * @version 2.0
 * @date 2026-04-10
 *
 * 基于USTC-RoboWalker改进的SPI通信驱动
 * 支持STM32H7系列和STM32F4系列
 */

#pragma once

#include "spi.h"
#include "stm32h723xx.h"
#include <stdint.h>

#define SPI_BUFFER_SIZE 512

using spi_call_back = void(*)(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t tx_len, uint16_t rx_len);

class spi_class_manager_object
{
    public:

        // 设置SPI通信相关参数
        void set_spi_handle(SPI_HandleTypeDef *spi_handle) { spi_handle_ = spi_handle; };

        void set_spi_cs_gpio_port(GPIO_TypeDef *gpio_port) { activate_gpio_ = gpio_port; };
        void set_spi_cs_gpio_pin(uint16_t gpio_pin) { activate_pin_ = gpio_pin; };
        void set_spi_cs_activate_level(GPIO_PinState level) { activate_level_ = level; };

        void set_spi_tx_length(uint16_t length) { tx_length_ = length; };
        void set_spi_rx_length(uint16_t length) { rx_length_ = length; };

        void set_spi_callback_function(spi_call_back callback) { callback_function_ = callback; }

        // 重载回调函数
        void set_spi_callback_function(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t tx_len, uint16_t rx_len)
        {
            if (callback_function_) callback_function_(tx_buf, rx_buf, tx_len, rx_len);
        }

        // 获取SPI配置参数
        SPI_HandleTypeDef* get_spi_handler(void) const { return spi_handle_; };

        GPIO_TypeDef* get_spi_cs_gpio_port(void) const { return activate_gpio_; };
        uint16_t get_spi_cs_gpio_pin(void) const { return activate_pin_; };
        GPIO_PinState get_spi_cs_activate_level(void) const { return activate_level_; };

        uint16_t get_spi_tx_length(void) const { return tx_length_; };
        uint16_t get_spi_rx_length(void) const { return rx_length_; };

        spi_call_back get_spi_callback_function(void) const { return callback_function_; };

        // 数据缓冲区访问
        uint8_t* get_tx_buffer(void) { return tx_buffer_; };
        uint8_t* get_rx_buffer(void) { return rx_buffer_; };

    private:

        // SPI 句柄和回调函数
        SPI_HandleTypeDef *spi_handle_ = nullptr;
        spi_call_back callback_function_ = nullptr;

        // SPI_CS引脚GPIO口和引脚号
        GPIO_TypeDef *activate_gpio_ = nullptr;
        uint16_t activate_pin_ = 0;
        GPIO_PinState activate_level_ = GPIO_PIN_RESET;

         // SPI通信数据缓冲区
        uint8_t tx_buffer_[SPI_BUFFER_SIZE];
        uint8_t rx_buffer_[SPI_BUFFER_SIZE];

        // SPI通信数据长度
        uint16_t tx_length_ = 0;
        uint16_t rx_length_ = 0;
};

extern spi_class_manager_object spi1_manager_object;
extern spi_class_manager_object spi2_manager_object;
extern spi_class_manager_object spi3_manager_object;
extern spi_class_manager_object spi4_manager_object;
extern spi_class_manager_object spi5_manager_object;
extern spi_class_manager_object spi6_manager_object;

void bsp_spi_init(SPI_HandleTypeDef *hspi, spi_call_back callback);

uint8_t bsp_spi_transmit_data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState activate_level, uint16_t tx_length);

uint8_t bsp_spi_transmit_receive_data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState activate_level, uint16_t tx_length, uint16_t rx_length);

