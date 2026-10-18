/** 
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  LiuGuoBin
  * @version V1.0
  * @date    2025-11-09
  * @brief   ADC底层驱动
  ******************************************************************************
  * @attention
  *
  * 参考达妙官方ADC例程，进行编写。
  * 
  ******************************************************************************
  */

#include "bsp_adc.h"
#include "adc.h"

__attribute__((section(".DMA_RAM_D1"), aligned(32))) volatile uint16_t adc_val[2]; // ADC采样值数组，使用DMA传输，放在DMA可访问的RAM_D1区域

/**
  * @brief  ADC初始化
  * @param  None
  * @retval None
  */

void bsp_adc_init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,2);
}

/**
  * @brief  获取ADC通道电源电压值
  * @param  channel: ADC通道号
  * @retval 电压值，单位V
  */

float bsp_get_adc_power_voltage(void)
{
    float voltage;
    voltage = (adc_val[0]*3.3f/65535)*11.0f;
    return voltage;
}

/**
  * @brief  获取ADC通道LCD按键电压值
  * @param  channel: ADC通道号
  * @retval 电压值，单位V
  */

float bsp_get_adc_lcd_key_voltage(void)
{
    float voltage;
    voltage = adc_val[1] * 3.3f / 65535;
    return voltage;
}