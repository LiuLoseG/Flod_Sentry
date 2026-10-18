#include "crc16.h"
#include <cstdint>

// x16 + x12 + x5 + 1
uint16_t modbus_tab16[256];
uint8_t modbus_tab16_inited = 0;

void crc16_init(void)
{
    for (uint16_t i = 0; i < 256; ++i)
    {
        uint16_t crc = 0, c = i;
        for (uint16_t j = 0; j < 8; ++j)
        {
            if ((crc ^ c) & 0x0001)
            {
                crc = (crc >> 1) ^ CRC_POLY_16;
            }
            else
            {
                crc = crc >> 1;
            }
            c = c >> 1;
        }
        modbus_tab16[i] = crc;
    }
    modbus_tab16_inited = 1;
}

uint16_t crc16_modbus_calc(uint8_t* data, uint32_t num_bytes)
{
    uint16_t crc = CRC_START_MODBUS;
    while (num_bytes--) crc = (crc >> 8) ^ modbus_tab16[(crc ^ *data++) & 0xff];
    return crc;
}

