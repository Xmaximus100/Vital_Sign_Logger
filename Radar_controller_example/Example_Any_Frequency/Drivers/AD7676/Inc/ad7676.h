
#define AD7676_GPIOB_MASK	0xFFF3 //Fourth Pin is Reserved for SWD
#define AD7676_GPIOC_MASK	0x0001
#define AD7676_CNVST_ON		HAL_GPIO_WritePin(ADC_CNVST_GPIO_Port, ADC_CNVST_Pin, GPIO_PIN_SET)
#define AD7676_CNVST_OFF	HAL_GPIO_WritePin(ADC_CNVST_GPIO_Port, ADC_CNVST_Pin, GPIO_PIN_RESET)
#define AD7676_CS_ON		HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET)
#define AD7676_CS_OFF		HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET)
#define AD7676_CONVST_DELAY	for(uint8_t i=0; i<5; i++) __NOP() //12,5ns * 5 for 80MHz RCC clock


#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

typedef struct __data_Collector {
	SPI_HandleTypeDef* spi_desc;
	uint16_t data_buf[4][200];
	uint16_t data_ptr;
	uint16_t data_ptr_max;
	uint8_t current_channel;
	uint8_t num_channels;
} data_Collector_TypeDef;

void ad7676_init(data_Collector_TypeDef** ad7676_data);

void ad7676_spi_read(uint8_t* buf, uint8_t size);

int ad7676_calculate_output(int32_t sample);

void ad7676_read_samples(uint16_t samples);

void ad7676_read_one_sample(void);

void ad7676_start_conversion(void);

