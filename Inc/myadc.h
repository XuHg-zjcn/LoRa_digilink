#ifndef _MYADC_H_
#define _MYADC_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
#endif
