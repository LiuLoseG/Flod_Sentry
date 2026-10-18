#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include "main.h"

#define TIME_PRECISION_USE      6     // 高精度,小数点后6位

typedef enum
{
    PRECISION_LOWER     = 2,    // 超低精度,小数点后0位
    PRECISION_LOW       = 3,    // 低精度,小数点后2位
    PRECISION_MID       = 4,    // 中等精度,小数点
    PRECISION_MEDHIGH   = 5,    // 较高精度,小数点后4位
    PRECISION_HIGH      = 6     // 高精度,小数点后6位

}TIME_PRECISION;

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} dwt_time_t;

/**
 * @brief 该宏用于计算代码段执行时间,单位为秒/s,返回值为float类型
 *        首先需要创建一个float类型的变量,用于存储时间间隔
 *        计算得到的时间间隔同时还会通过RTT打印到日志终端,你也可以将你的dt变量添加到查看
 */
#define TIME_ELAPSE(dt, code)                    \
    do                                           \
    {                                            \
        float tstart = bsp_dwt_get_timeline_s();      \
        code;                                    \
        dt = bsp_dwt_get_timeline_s() - tstart;       \
        LOG_INFO("[DWT] " #dt " = %f s\r\n", dt); \
    } while (0)

/**
 * @brief 初始化DWT,传入参数为CPU频率,单位MHz
 *
 * @param CPU_Freq_mHz CPU频率,单位MHz
 */
void bsp_dwt_init(uint32_t CPU_Freq_mHz);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 *
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为秒/s
 */
float bsp_dwt_get_deltat32(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double bsp_dwt_get_deltat64(uint32_t *cnt_last);

/**
 * @brief 获取当前时间,单位为秒/s,即初始化后的时间
 *
 * @return float 时间轴
 */
float bsp_dwt_get_timeline_s(void);

/**
 * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
 *
 * @return float
 */
float bsp_dwt_get_timeline_ms(void);

/**
 * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
 *
 * @return uint64_t
 */
uint64_t bsp_dwt_get_timeline_us(void);

/**
 * @brief DWT延时函数,单位为秒/s
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
 *
 * @param Delay 延时时间,单位为秒/s
 */
void bsp_dwt_delay_s(float Delay);

/**
 * @brief DWT延时函数,单位为微秒/us
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
 *
 * @param Delay 延时时间,单位为微秒/us
 */
void bsp_dwt_delay_us(uint32_t delay_us);

/**
 * @brief DWT延时函数,单位为毫秒/ms
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
 *
 * @param Delay 延时时间,单位为毫秒/ms
 */
void bsp_dwt_delay_ms(float delay_ms);

/**
 * @brief DWT更新时间轴函数,会被三个timeline函数调用
 * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
 */
void bsp_dwt_systime_update(void);

#endif
