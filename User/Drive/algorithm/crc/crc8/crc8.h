#pragma once

#include "stdint.h"

#define CRC_CHECK_CODE 0xFF

uint8_t crc8_check_sum(uint8_t first_byte, uint8_t *data, uint8_t len, uint8_t crc8_init);

