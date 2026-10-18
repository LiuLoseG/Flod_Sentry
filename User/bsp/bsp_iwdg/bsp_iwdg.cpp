#include "bsp_iwdg.h"

/**
 * @brief 独立看门狗喂狗函数
 * @note  定期调用此函数，用以处理程序异常时的复位
 */

void iwdg_feed(void)
{
    // HAL_IWDG_Refresh(&hiwdg1);
}