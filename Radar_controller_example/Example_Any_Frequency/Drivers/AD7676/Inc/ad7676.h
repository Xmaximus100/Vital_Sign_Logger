
#define AD7676_GPIOB_MASK	0xFFF3 //Fourth Pin is Reserved for SWD
#define AD7676_GPIOC_MASK	0x0001
#define AD7676_CNVST_ON		HAL_GPIO_WritePin(ADC_CNVST_GPIO_Port, ADC_CNVST_Pin, GPIO_PIN_SET)
#define AD7676_CNVST_OFF	HAL_GPIO_WritePin(ADC_CNVST_GPIO_Port, ADC_CNVST_Pin, GPIO_PIN_RESET)
#define AD7676_RESET_ON		HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_SET)
#define AD7676_RESET_OFF	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_RESET)
#define AD7676_CS_ON		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin //HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET)
#define AD7676_CS_OFF		ADC_CS_GPIO_Port->BRR = (uint32_t)ADC_CS_Pin //HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET)
#define AD7676_CONVST_DELAY	for(uint8_t i=0; i<5; i++) __NOP() //12,5ns * 5 for 80MHz RCC clock
#define AD7676_RESET_DELAY for(uint8_t i=0; i<5; i++) __NOP() //12,5ns * 5 for 80MHz RCC clock


#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

typedef union __data_Buffer {
	uint64_t data;
	uint16_t channels[4];
} dataBuffer;

typedef struct __data_Collector {
	SPI_HandleTypeDef* spi_desc;
	uint16_t data_buf[10000][4];
	uint16_t dummy_tx_buffer[255];
	uint16_t tx_buffer[255];
	uint16_t rx_buffer[255];
	uint16_t data_ptr;
	uint16_t data_ptr_max;
	uint16_t rx_buffer_ptr;
	uint16_t rx_buffer_ptr_max;
	uint8_t current_channel;
	uint8_t num_channels;
	uint8_t range_select;
	uint16_t range_tab[3];
} data_Collector_TypeDef;

void ad7676_init(data_Collector_TypeDef** ad7676_data);

void ad7676_write_register(uint16_t* data, uint8_t length);

void ad7676_spi_read(uint8_t* buf, uint8_t size);

void ad7676_spi_read_raw(uint16_t* buf, uint16_t size);

void ad7676_spi_write_read_raw(uint16_t* write_buf, uint16_t* read_buf, const uint16_t size);

int ad7676_calculate_output(int32_t sample);

void ad7676_read_samples(uint32_t samples);

void ad7676_read_continuous(bool enable);

void ad7676_display_samples(uint32_t awaited_samples, uint32_t* received_samples, void (*displayFunction)(char* message));

void ad7676_send_samples(uint32_t awaited_samples, uint32_t* received_samples, UART_HandleTypeDef* huart2);

void ad7676_send_sample(UART_HandleTypeDef* huart, uint32_t* received_samples);

void ad7676_set_sampling_rate(uint16_t sampling_rate);

void ad7676_read_one_sample(void);

void ad7676_start_conversion(void);

void ad7676_reset(void);

//void DMA1_Channel4_IRQHandler(void);

