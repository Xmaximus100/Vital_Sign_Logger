

#include "ad7676.h"
#include "no_os_alloc.h"
#include "stm32l4xx_hal.h"
#include "main.h"


extern data_Collector_TypeDef* ad7676_data;


void ad7676_init(data_Collector_TypeDef** ad7676_data)
{
	data_Collector_TypeDef* init_data;

	init_data = (data_Collector_TypeDef*)no_os_calloc(1, sizeof(*init_data));

	init_data->data_ptr = 0;
	init_data->data_ptr_max = 65535;

	*ad7676_data = init_data;
}

void ad7676_acquire_data(data_Collector_TypeDef* ad7676_data, int16_t sample)
{
	ad7676_data->data_buf[ad7676_data->data_ptr++] = sample;
}

void ad7676_read_one_sample()
{

//	(GPIOx->IDR & GPIO_Pin);
//	GPIO_TypeDef GPIOB, D0_GPIO_Port, D15_GPIO_Port
//	Pin PB3 reserved for SWD
	int16_t sample = (GPIOB->IDR & AD7676_GPIOB_MASK) | ((GPIOC->IDR & AD7676_GPIOC_MASK) << 15);
	ad7676_acquire_data(ad7676_data, sample);
	AD7676_CNVST_ON;
}

void ad7676_reset_data(data_Collector_TypeDef* ad7676_data)
{
	for (uint32_t i=0; i<=ad7676_data->data_ptr_max; i++){
		ad7676_data->data_buf[i] = 0;
	}
	ad7676_data->data_ptr = 0;
}

void ad7676_start_conversion()
{
	AD7676_CNVST_OFF;
}
