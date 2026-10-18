#include "bsp_dwt.h"

static dwt_time_t SysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;

/**
 * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 *
 * @todo 更好的方案是为dwt的时间更新单独设置一个任务?
 *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
 *
 */
static void bsp_dwt_cnt_update(void)
{
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker)
    {
        bit_locker = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < CYCCNT_LAST)
            CYCCNT_RountCount++;

        CYCCNT_LAST = DWT->CYCCNT;
        bit_locker = 0;
    }
}

/**
 * @brief 初始化DWT
 * @param CPU_Freq_mHz CPU频率,单位MHz
 */
void bsp_dwt_init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;

    bsp_dwt_cnt_update();
}

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,单精度浮点型
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为秒/s
 */
float bsp_dwt_get_deltat32(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    bsp_dwt_cnt_update();

    return dt;
}

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度浮点型
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double bsp_dwt_get_deltat64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    bsp_dwt_cnt_update();

    return dt;
}

/**
 * @brief 更新系统时间
    * @attention 该函数会被三个timeline函数调用,以保证时间轴的更新
    * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
 */
void bsp_dwt_systime_update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    bsp_dwt_cnt_update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

/**
    * @brief 获取当前时间,单位为秒/s,即初始化后的时间
    * @return float 时间（s）
 */
float bsp_dwt_get_timeline_s(void)
{
    bsp_dwt_systime_update();

    float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

    return DWT_Timelinef32;
}

/**
 * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
 * @return float 时间（ms）
 */
float bsp_dwt_get_timeline_ms(void)
{
    bsp_dwt_systime_update();

    float DWT_Timelinef32 = SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;

    return DWT_Timelinef32;
}

/**
 * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
 * @return uint64_t 时间（us）
 */
uint64_t bsp_dwt_get_timeline_us(void)
{
    bsp_dwt_systime_update();

    uint64_t DWT_Timelinef32 = SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;

    return DWT_Timelinef32;
}

/**
 * @brief 延时函数，以秒为单位
 * @param delay_s float延时时间（s）
 */

void bsp_dwt_delay_s(float delay_s)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait = delay_s;

    while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_FREQ_Hz);
}


/**
 * @brief 延时函数，以微秒为单位
 * @param delay_us uint32_t延时时间（us）
 */
void bsp_dwt_delay_us(uint32_t delay_us)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_us = delay_us * CPU_FREQ_Hz_us;
    while ((DWT->CYCCNT - tickstart) < wait_us);
}

/**
 * @brief 延时函数，以毫秒为单位
 * @param delay_ms float延时时间（ms）
 */
void bsp_dwt_delay_ms(float delay_ms)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_ms = (uint32_t)((float)delay_ms * CPU_FREQ_Hz_ms);
    while ((DWT->CYCCNT - tickstart) < wait_ms);
}
