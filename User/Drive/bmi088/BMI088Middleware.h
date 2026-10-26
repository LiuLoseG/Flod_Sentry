#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "stdint.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

// SPI 单次突发传输超时时间, 单位 ms
// 实际一帧最长 12 字节, 在 6.25MHz 下约 16us; 取 2ms 既留足 100 倍余量
#define BMI088_SPI_TIMEOUT_MS 2

#ifndef BMI088_SPI_PRESCALER
#define BMI088_SPI_PRESCALER SPI_BAUDRATEPRESCALER_16
#endif

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

/**
 * @brief  SPI 全双工突发收发(不操作片选, 片选由调用者控制)
 * @param  tx 发送缓冲区
 * @param  rx 接收缓冲区
 * @param  len 传输字节数
 * @retval 0: 成功 1: 失败(超时/错误)
 * @note   逐字节收发在一帧数据上会产生十余次函数调用与片选翻转, 突发传输可显著降低 CPU 占用
 */
extern uint8_t BMI088_transmit_receive(uint8_t *tx, uint8_t *rx, uint16_t len);

#elif defined(BMI088_USE_IIC)

#endif

#endif
