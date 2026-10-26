#pragma once

#include "bsp_spi.hpp"
#include "cmsis_os2.h"

class ws2812_color
{
    public:
        uint8_t red;
        uint8_t green;
        uint8_t blue;
};

class ws2812_class
{
    public:
        void color_init(const uint8_t & __red = 0, const uint8_t & __green = 0, const uint8_t & __blue = 0);

        inline void set_red(const uint8_t &__red);
        inline void set_green(const uint8_t &__green);
        inline void set_blue(const uint8_t &__blue);

        inline void set_rgb(const uint8_t &__red, const uint8_t &__green, const uint8_t &__blue);

        inline void set_color(const ws2812_color &color, const float &__brightness = 1.0f);

        bool ws2812_write_color(void);

        static constexpr uint16_t frame_bytes = 24U;
        static constexpr uint16_t total_bytes = frame_bytes + 1U;

    protected:

        // 绑定的SPI
        spi_class_manager_object *spi_manager_object_ = nullptr;

        // 灯珠的逻辑码
        const uint8_t LEVEL_0_ = 0x60;
        const uint8_t LEVEL_1_ = 0x78;

        // RGB颜色值
        // WS2812的RGB顺序是GRB, 为方便使用, 仍采用RGB顺序
        ws2812_color color_ = {0, 0, 0};

};

inline void ws2812_class::set_red(const uint8_t &__red)
{
    color_.red = __red;
    color_.green = 0;
    color_.blue = 0;
}

inline void ws2812_class::set_green(const uint8_t &__green)
{
    color_.red = 0;
    color_.green = __green;
    color_.blue = 0;
}

inline void ws2812_class::set_blue(const uint8_t &__blue)
{
    color_.red = 0;
    color_.green = 0;
    color_.blue = __blue;
}

inline void ws2812_class::set_rgb(const uint8_t &__red, const uint8_t &__green, const uint8_t &__blue)
{
    color_.red = __red;
    color_.green = __green;
    color_.blue = __blue;
}

inline void ws2812_class::set_color(const ws2812_color &color, const float &__brightness)
{
    color_.red = static_cast<uint8_t>(color.red * __brightness);
    color_.green = static_cast<uint8_t>(color.green * __brightness);
    color_.blue = static_cast<uint8_t>(color.blue * __brightness);
}