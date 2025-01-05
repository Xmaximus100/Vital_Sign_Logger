

#include "ad7676.h"
#include "no_os_alloc.h"
#include "spi.h"
#include "main.h"


data_Collector_TypeDef* ad7676_data;
bool collect_data = false;
uint16_t awaited_samples = 0;


void ad7676_init(data_Collector_TypeDef** ad7676_data)
{
	data_Collector_TypeDef* init_data;

	init_data = (data_Collector_TypeDef*)no_os_calloc(1, sizeof(*init_data));

	init_data->spi_desc = &hspi2;
	init_data->data_ptr = 0;
	init_data->data_ptr_max = 200;
	init_data->current_channel = 0;
	init_data->num_channels = 4;

	*ad7676_data = init_data;
}

void ad7676_spi_read(uint8_t* buf, uint8_t size){
	HAL_SPI_Receive(ad7676_data->spi_desc, buf, size, 0xFF);
}

int ad7676_calculate_output(int32_t sample){
	int sample_voltage = (sample*10*1000)/32768;
	return sample_voltage;  //assuming range is +/-10V and REF is internal 2,5V datasheet p.23
}

void ad7676_read_one_sample() //when BUSY goes down
{

//	(GPIOx->IDR & GPIO_Pin);
//	GPIO_TypeDef GPIOB, D0_GPIO_Port, D15_GPIO_Port
//	Pin PB3 reserved for SWD
//	int16_t sample = (GPIOB->IDR & AD7676_GPIOB_MASK) | ((GPIOC->IDR & AD7676_GPIOC_MASK) << 15);
	uint8_t buf[8];
	AD7676_CS_OFF;
	ad7676_spi_read(buf, 8);
	for(ad7676_data->current_channel=0; ad7676_data->current_channel<ad7676_data->num_channels; ad7676_data->current_channel++){
		ad7676_data->data_buf[ad7676_data->current_channel][ad7676_data->data_ptr] = buf[2*ad7676_data->current_channel+1]+(buf[2*ad7676_data->current_channel]<<8); //LSB first
	}
	AD7676_CS_ON;
//	ad7676_data->data_buf[ad7676_data->data_ptr++] = sample;
	ad7676_data->data_ptr = (ad7676_data->data_ptr+1)%ad7676_data->data_ptr_max;
}

void ad7676_read_samples(uint16_t samples){
	awaited_samples = samples;
	collect_data = true;
}

void ad7676_reset_data(data_Collector_TypeDef* ad7676_data)
{
	for(ad7676_data->current_channel=0; ad7676_data->current_channel<ad7676_data->num_channels; ad7676_data->current_channel++){
		for (uint32_t i=0; i<=ad7676_data->data_ptr_max; i++){
			ad7676_data->data_buf[ad7676_data->current_channel][i] = 0;
		}
	}
	ad7676_data->data_ptr = 0;
}

void ad7676_start_conversion()
{
	AD7676_CNVST_OFF;
	AD7676_CONVST_DELAY;
	AD7676_CNVST_ON;
}
