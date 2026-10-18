#include "crc8.h"

/**
 * @brief 计算CRC8校验和
 * @param data 待计算的数据指针
 * @param length 数据长度
 * @param crc8_init 初始CRC8值
 * @return CRC8校验和
 */
uint8_t crc8_check_sum(uint8_t first_byte, uint8_t *data, uint8_t len, uint8_t crc8_init)
{
    uint32_t i;
	uint8_t check_sum;
	uint32_t sum = 0;
	
	for(i = 0; i < len; i++)
	{
		sum += data[i];
	}
	sum += first_byte;
	check_sum = sum & CRC_CHECK_CODE;
	return check_sum;
}
