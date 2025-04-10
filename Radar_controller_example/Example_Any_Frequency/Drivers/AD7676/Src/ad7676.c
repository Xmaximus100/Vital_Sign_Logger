

#include "ad7676.h"
#include "no_os_alloc.h"
#include "spi.h"
#include "usart.h"
#include "main.h"
#include "tim.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include <string.h>

//#define SPI_FLAG_BSY (0x1UL << 7U)


data_Collector_TypeDef* ad7676_data;
bool collect_data = false;
bool continuous_mode = false;
uint32_t awaited_samples = 0;
uint16_t sampling_rate = 1000;
static uint64_t start_time, end_time, elapsed_time = 0;


static void ad7676_spi_configuration(){
//	SPI_CR1_BIDIMODE 0
//	SPI2->CR1  |= SPI_CR1_BIDIOE; //0
//	SPI2->CR1 |= SPI_CR1_CRCEN;
//	SPI2->CR1 |= SPI_CR1_RXONLY; //0
//	SPI_CR1_LSBFIRST 0
//	SPI2->CR1 |= SPI_CR1_SPE; //enable when ready
	SPI2->CR1 |= SPI_CR1_BR_1;//(SPI_CR1_BR_0 |  | SPI_CR1_BR_2); //ultimately leave 0
	SPI2->CR1 |= SPI_CR1_MSTR;
	SPI2->CR1 |= SPI_CR1_CPOL; //spi configuration CPOL 1 CPHA 0
//	SPI2->CR1 |= SPI_CR1_CPHA 0

//	SPI2->CR1 = SPI_CR1_CRCEN | SPI_CR1_RXONLY |
//			SPI_CR1_BR_2 | SPI_CR1_MSTR | SPI_CR1_CPOL;

//	SPI2->CR2 |= SPI_CR2_FRXTH 0
	SPI2->CR2 |= SPI_CR2_DS;
//	SPI2->CR2 |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2);
//	SPI2->CR2 |= SPI_CR2_RXNEIE; //enable when RXNE interrupt necessary
//	SPI2->CR2 |= SPI_CR2_NSSP; //no NSS pulse between data
	SPI2->CR2 |= SPI_CR2_SSOE; //master SS enabled
	SPI2->CR2 |= SPI_CR2_RXDMAEN; //DMA request is set with every RXNE flag

//	SPI2->CR2 = SPI_CR2_DS | SPI_CR2_SSOE |
//			SPI_CR2_RXDMAEN;
}

static void ad7676_dma_configuration(){
	DMA1_Channel4->CCR |= DMA_CCR_PL_1; //priority high
	DMA1_Channel4->CCR |= DMA_CCR_MSIZE_0; //mem size 16-bit
	DMA1_Channel4->CCR |= DMA_CCR_PSIZE_0; //periph size 16-bit
	DMA1_Channel4->CCR |= DMA_CCR_MINC; //mem increment
//	DMA1_Channel4->CCR |= DMA_CCR_PINC; //periph increment - we read spi register so its always the same
//	DMA1_Channel4->CCR |= DMA_CCR_DIR 0
	DMA1_Channel4->CCR |= DMA_CCR_TCIE; //transfer complete interrupt en
//	DMA1_Channel4->CCR |= DMA_CCR_EN; //TODO check if needed to set
	uint8_t num_channel = 4;
	uint8_t num_half_bytes = 4;
	DMA1_CSELR->CSELR &= ~(0xF << num_half_bytes*(num_channel-1));
	DMA1_CSELR->CSELR |= 1 << num_half_bytes*(num_channel-1);
	DMA1_Channel5->CCR |= DMA_CCR_PL_1; //priority high
	DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0; //mem size 16-bit
	DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0; //periph size 16-bit
	DMA1_Channel5->CCR |= DMA_CCR_MINC; //mem increment
//	DMA1_Channel4->CCR |= DMA_CCR_PINC; //periph increment - we read spi register so its always the same
	DMA1_Channel5->CCR |= DMA_CCR_DIR;
//	DMA1_Channel5->CCR |= DMA_CCR_TCIE; //transfer complete interrupt en
//	DMA1_Channel4->CCR |= DMA_CCR_EN; //TODO check if needed to set
	num_channel = 5;
	DMA1_CSELR->CSELR &= ~(0xF << num_half_bytes*(num_channel-1));
	DMA1_CSELR->CSELR |= 1 << num_half_bytes*(num_channel-1);
}

static void ad7676_clock_configuration(){
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(SPI2_IRQn);

	/* DMA interrupt init */
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
//	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
//	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

void ad7676_init(data_Collector_TypeDef** ad7676_data)
{
	data_Collector_TypeDef* init_data;

	init_data = (data_Collector_TypeDef*)no_os_calloc(1, sizeof(*init_data));

	init_data->spi_desc = &hspi2;
	init_data->data_ptr = 0;
	init_data->data_ptr_max = 10000;
	init_data->rx_buffer_ptr = 0;
	init_data->rx_buffer_ptr_max = 255;
	init_data->current_channel = 0;
	init_data->num_channels = 4;
	init_data->range_tab[0] = 2500;
	init_data->range_tab[1] = 5000;
	init_data->range_tab[2] = 10000;
	init_data->range_select = 2;
	for(uint16_t i=0; i<256; i++){
		init_data->dummy_tx_buffer[i] = 0x8000;
		init_data->rx_buffer[i] = 0x0000;
	}
	*ad7676_data = init_data;

	ad7676_clock_configuration();
	ad7676_spi_configuration();
	ad7676_dma_configuration();
	ad7676_reset();

}

int ad7676_calculate_output(int32_t sample){
	int sample_voltage = (sample*ad7676_data->range_tab[ad7676_data->range_select])/32768;
	return sample_voltage;  //assuming range is +/-10V and REF is internal 2,5V datasheet p.23
}

static void ad7676_dma_enable_stream(DMA_Channel_TypeDef* dma_channel, uint16_t data_size, uint32_t src_addr, uint32_t dst_addr){
	dma_channel->CNDTR = data_size;
	dma_channel->CPAR = src_addr;
	dma_channel->CMAR = dst_addr;
}


void ad7676_write_register(uint16_t* data, const uint8_t length)
{
//	uint8_t buffer_ptr = length;
	SPI2->CR2 &= ~(SPI_CR2_RXDMAEN);
	AD7676_CS_OFF;
//	SPI2->CR2 &= ~SPI_CR2_RXNEIE;
//	SPI2->CR1 &= ~(SPI_CR1_SPE);

//	if ((SPI2->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
//		SPI2->CR1 |= SPI_CR1_SPE;
//	while(buffer_ptr > 0){
//		if(SPI2->SR & SPI_FLAG_TXE){
//			SPI2->DR = data[length - buffer_ptr];
////			SPI2->DR = 0x0A;//data[length - buffer_ptr];
////			SPI2->DR = 0x0C;//data[length - buffer_ptr];
//			buffer_ptr -= 1;
//		}
//	}

	ad7676_dma_enable_stream(DMA1_Channel5, length, (uint32_t)&(SPI2->DR), (uint32_t)data);
	DMA1_Channel5->CCR |= DMA_CCR_TCIE;
	DMA1_Channel5->CCR |= DMA_CCR_EN;

	SPI2->CR1 |= SPI_CR1_SPE;
	SPI2->CR2 |= SPI_CR2_TXDMAEN;

	while(SPI2->SR & SPI_FLAG_FTLVL);
	while(SPI2->SR & SPI_FLAG_BSY);
	uint8_t temp = SPI2->DR;
			temp = SPI2->SR;
//	SPI2->CR2 |= SPI_CR2_RXNEIE;
}

void ad7676_spi_read_raw(uint16_t* buf, const uint16_t size){
	SPI2->CR1 &= ~(SPI_CR1_SPE);
	SPI2->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
	SPI2->CR2 &= ~(SPI_CR2_FRXTH);

	DMA1_Channel4->CCR &= ~(DMA_CCR_EN);
	DMA1->IFCR |= DMA_ISR_GIF4;
	ad7676_dma_enable_stream(DMA1_Channel4, size, (uint32_t)&(SPI2->DR), (uint32_t)buf);
	DMA1_Channel4->CCR |= DMA_CCR_TCIE;
	DMA1_Channel4->CCR |= DMA_CCR_EN; //DMA en

//	SPI2->CR1 |= SPI_CR1_SPE;
//
//    for (uint16_t i = 0; i < size; i++) {
//        // Czekaj aż rejestr TX będzie gotowy
//        while (!(SPI2->SR & SPI_FLAG_TXE));
//        // Wysyłamy bajt dummy (np. 0x00)
//        SPI2->DR = 0x00;
//    }
//
//    SPI2->CR2 |= SPI_CR2_RXDMAEN; //enable RX DMA interrupt

    uint8_t dummy_tx_buffer[size];  // Upewnij się, że bufor jest wystarczająco duży
    for (uint16_t i = 0; i < size; i++) {
        dummy_tx_buffer[i] = 0x8000;  // lub inna wartość, jeśli wymaga tego ADC
    }

    DMA1_Channel5->CCR &= ~(DMA_CCR_EN);
    // Wyczyść flagi przerwań dla kanału TX, jeśli to konieczne
    ad7676_dma_enable_stream(DMA1_Channel5, size, (uint32_t)&(SPI2->DR), (uint32_t)dummy_tx_buffer);
    DMA1_Channel5->CCR |= DMA_CCR_TCIE;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    // Włączenie SPI i aktywacja DMA dla obu kanałów
    SPI2->CR1 |= SPI_CR1_SPE;
    SPI2->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
}

void ad7676_spi_write_read_raw(uint16_t* write_buf, uint16_t* read_buf, const uint16_t size){
	AD7676_CS_OFF;
	SPI2->CR1 &= ~(SPI_CR1_SPE);
	SPI2->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
	SPI2->CR2 &= ~(SPI_CR2_FRXTH);

	DMA1_Channel4->CCR &= ~(DMA_CCR_EN);
	DMA1->IFCR |= DMA_ISR_GIF4;
	ad7676_dma_enable_stream(DMA1_Channel4, size, (uint32_t)&(SPI2->DR), (uint32_t)read_buf);
	DMA1_Channel4->CCR |= DMA_CCR_TCIE;
	DMA1_Channel4->CCR |= DMA_CCR_EN; //DMA en

//	SPI2->CR1 |= SPI_CR1_SPE;
//
//    for (uint16_t i = 0; i < size; i++) {
//        // Czekaj aż rejestr TX będzie gotowy
//        while (!(SPI2->SR & SPI_FLAG_TXE));
//        // Wysyłamy bajt dummy (np. 0x00)
//        SPI2->DR = 0x00;
//    }
//
//    SPI2->CR2 |= SPI_CR2_RXDMAEN; //enable RX DMA interrupt
	DMA1_Channel5->CCR &= ~(DMA_CCR_EN);
	memcpy(ad7676_data->tx_buffer, (write_buf == NULL)?ad7676_data->dummy_tx_buffer:write_buf, size * sizeof(ad7676_data->tx_buffer[0]));
	ad7676_dma_enable_stream(DMA1_Channel5, size, (uint32_t)&(SPI2->DR), (uint32_t)ad7676_data->tx_buffer);
    // Wyczyść flagi przerwań dla kanału TX, jeśli to konieczne
    DMA1_Channel5->CCR |= DMA_CCR_TCIE;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    // Włączenie SPI i aktywacja DMA dla obu kanałów );//
    SPI2->CR1 |= SPI_CR1_SPE;
    SPI2->CR2 |= (SPI_CR2_RXDMAEN| SPI_CR2_TXDMAEN);

//	for (uint16_t i = 0; i < size; i++) {
//		// Czekaj aż rejestr TX będzie gotowy
//		while (!(SPI2->SR & SPI_FLAG_TXE));
//		// Wysyłamy bajt dummy (np. 0x00)
//		SPI2->DR = 0x00;
//	}
}

void ad7676_read_one_sample() //when BUSY goes down
{

//	ad7676_spi_read_raw((uint16_t*)&ad7676_data->data_buf[ad7676_data->data_ptr], 4);
	ad7676_spi_write_read_raw(NULL, (uint16_t*)&ad7676_data->data_buf[ad7676_data->data_ptr], 4);
//	ad7676_data->data_ptr = (ad7676_data->data_ptr+1)%ad7676_data->data_ptr_max;
}

void ad7676_read_samples(uint32_t samples){
	awaited_samples = samples;
	collect_data = true;
	if (continuous_mode){
		HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);
	}
	else ad7676_start_conversion();
}

void ad7676_read_continuous(bool enable){
	continuous_mode = enable;
}

void ad7676_display_samples(uint32_t awaited_samples, uint32_t* received_samples, void (*displayFunction)(char* message)){
	char buffer[64];
	int v1, v2, v3, v4;
	uint32_t tmp_ptr = ad7676_data->data_ptr - awaited_samples;
	collect_data = false;
	*received_samples = 0;
	sprintf(buffer, "Collected samples:%ld\n\rCHANNEL1 CHANNEL2 CHANNEL3 CHANNEL4\n\r", awaited_samples);
	displayFunction(buffer);
	for(uint32_t i=0; i<awaited_samples; i++){
		v1 = ad7676_calculate_output(ad7676_data->data_buf[(tmp_ptr + i)%ad7676_data->data_ptr_max][0]);
		v2 = ad7676_calculate_output(ad7676_data->data_buf[(tmp_ptr + i)%ad7676_data->data_ptr_max][1]);
		v3 = ad7676_calculate_output(ad7676_data->data_buf[(tmp_ptr + i)%ad7676_data->data_ptr_max][2]);
		v4 = ad7676_calculate_output(ad7676_data->data_buf[(tmp_ptr + i)%ad7676_data->data_ptr_max][3]);
		sprintf(buffer, "%d.%dV %d.%dV %d.%dV %d.%dV\n\r",
				v1/1000,abs(v1%1000),
				v2/1000,abs(v2%1000),
				v3/1000,abs(v3%1000),
				v4/1000,abs(v4%1000)
				);
		displayFunction(buffer);
	}
}

void ad7676_send_samples(uint32_t awaited_samples, uint32_t* received_samples, UART_HandleTypeDef* huart){
	uint32_t tmp_ptr = ad7676_data->data_ptr - awaited_samples;
    collect_data = false;
    *received_samples = 0;

    for(uint32_t i = 0; i < awaited_samples; i++){
        uint8_t frame[11];
        memcpy(frame, &i, 3); //first 2 bytes for sample index
        memcpy(frame+3, &(ad7676_data->data_buf[(tmp_ptr + i) % ad7676_data->data_ptr_max]), 8); //bytes order frame[2] low_byte, frame[3] high_byte
        HAL_UART_Transmit(huart, frame, 11, 1000);
    }
}

void ad7676_send_sample(UART_HandleTypeDef* huart, uint32_t* received_samples){
	uint8_t frame[11];
	memcpy(frame, received_samples, 3); //first 2 bytes for sample index
	memcpy(frame+3, &(ad7676_data->data_buf[(ad7676_data->data_ptr) % ad7676_data->data_ptr_max]), 8); //bytes order frame[2] low_byte, frame[3] high_byte
//	if ((received_samples-1)%10000 == 0) uint8_t tmp = 0;
	HAL_UART_Transmit(huart, frame, 11, 1000);
}

void ad7676_set_sampling_rate(uint16_t sampling_rate){
	sampling_rate = sampling_rate;
}

void ad7676_reset_data(data_Collector_TypeDef* ad7676_data)
{
	for(ad7676_data->current_channel=0; ad7676_data->current_channel<ad7676_data->num_channels; ad7676_data->current_channel++){
		for (uint32_t i=0; i<=ad7676_data->data_ptr_max; i++){
//			ad7676_data->data_buf[ad7676_data->current_channel][i] = 0;
			ad7676_data->data_buf[i][ad7676_data->current_channel] = 0;
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

void ad7676_reset(){
	AD7676_RESET_ON;
	AD7676_RESET_DELAY;
	AD7676_RESET_OFF;
}

