

#include "ad7676.h"
#include "no_os_alloc.h"
#include "stm32l4xx_hal.h"
#include "main.h"


extern Data_Collector* ad7676_data;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == AD_BUSY_Pin){
		ad7676_read_one_sample();
	}
}

ad7676_init(Data_Collector** ad7676_data)
{
	Data_Collector* init_data;

	init_data = (Data_Collector*)no_os_calloc(1, sizeof(*init_data));

	init_data->data_ptr = 0;
	init_data->data_ptr_max = 65535;

	*ad7676_data = init_data;
}

ad7676_acquire_data(Data_Collector* ad7676_data, uint16_t data)
{
	ad7676_data->data_buf[ad7676_data->data_ptr++] = data;
}

ad7676_read_one_sample()
{

//	(GPIOx->IDR & GPIO_Pin);
//	GPIO_TypeDef GPIOB, D0_GPIO_Port, D15_GPIO_Port
//	Pin PB3 reserved for SWD
	int16_t sample = (GPIOB->IDR & AD7676_GPIOB_MASK) | ((GPIOC->IDR & AD7676_GPIOC_MASK) << 15);
	ad7676_acquire_data(ad7676_data, sample);
	AD7676_CNVST_OFF;
}

ad7676_reset_data(Data_Collector* ad7676_data)
{
	for (uint32_t i=0; i<=ad7676_data->data_ptr_max; i++){
		ad7676_data->data_buf[i] = 0;
	}
	ad7676_data->data_ptr = 0;
}

ad7676_start_conversion()
{
	AD7676_CNVST_ON;
}
