
#define AD7676_GPIOB_MASK	0xFFF3 //Fourth Pin is Reserved for SWD
#define AD7676_GPIOC_MASK	0x0001
#define AD7676_CNVST_ON		HAL_GPIO_WritePin(AD_CNVST_GPIO_Port, AD_CNVST_Pin, GPIO_PIN_SET)
#define AD7676_CNVST_OFF	HAL_GPIO_WritePin(AD_CNVST_GPIO_Port, AD_CNVST_Pin, GPIO_PIN_RESET)


#include <stdint.h>

typedef struct __data_Collector {
	uint16_t data_buf[65535];
	uint16_t data_ptr;
	uint16_t data_ptr_max;
} data_Collector_TypeDef;

void ad7676_init(data_Collector_TypeDef** ad7676_data);

void ad7676_acquire_data(data_Collector_TypeDef* ad7676_data, int16_t sample);

void ad7676_read_samples(uint8_t* data, uint32_t samples);

void ad7676_read_one_sample(void);

void ad7676_start_conversion(void);

