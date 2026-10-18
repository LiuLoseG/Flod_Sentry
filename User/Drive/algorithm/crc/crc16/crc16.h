#pragma once

// C/C++ 兼容保护,头文件在 C++ 编译时禁止函数名被改名（name mangling），以便 C 与 C++ 正常链接。
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define CRC_POLY_16 0X8408
#define CRC_START_MODBUS 0xFFFF

void crc16_init(void);
uint16_t crc16_modbus_calc(uint8_t* data, uint32_t num_bytes);

#ifdef __cplusplus
}
#endif
