#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_adc.h"

extern osThreadId codec2Handle;
extern uint8_t codec2_flag;
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	//printf("HalfCplt,encoding=%d\r\n",encoding_flag);
	/*if(codec2_flag)
		osThreadResume(codec2Handle);*/
	osSignalSet(codec2Handle, 0x0001);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
	//printf("Cplt,encoding=%d\r\n",encoding_flag);
	/*if(codec2_flag)
		osThreadResume(codec2Handle);*/
	osSignalSet(codec2Handle, 0x0002);
}
