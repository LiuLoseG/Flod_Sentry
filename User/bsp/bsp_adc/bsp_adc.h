#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "adc.h"

void bsp_adc_init(void);
float bsp_get_adc_power_voltage(void);
float bsp_get_adc_lcd_key_voltage(void);

#endif /* __BSP_ADC_H */
