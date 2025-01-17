

#include "ad7676.h"
#include "no_os_alloc.h"
#include "spi.h"
#include "main.h"
#include "tim.h"
#include "stm32l4xx_hal.h"
#include <string.h>


data_Collector_TypeDef* ad7676_data;
bool collect_data = false;
bool continuous_mode = false;
uint16_t awaited_samples = 0;
static uint64_t start_time, end_time, elapsed_time = 0;


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
//	HAL_SPI_Receive(ad7676_data->spi_desc, (uint8_t*)buf, size, 0xFF);
//	HAL_SPI_Receive_DMA(ad7676_data->spi_desc, (uint8_t*)buf, size);
	/* Process Locked */
	__HAL_LOCK(ad7676_data->spi_desc);

	/* Set the transaction information */
	ad7676_data->spi_desc->State       = HAL_SPI_STATE_BUSY_RX;
	ad7676_data->spi_desc->ErrorCode   = HAL_SPI_ERROR_NONE;
	ad7676_data->spi_desc->pRxBuffPtr  = buf;
	ad7676_data->spi_desc->RxXferSize  = size;
	ad7676_data->spi_desc->RxXferCount = size;

	/*Init field not used in handle to zero */
	ad7676_data->spi_desc->RxISR       = NULL;
	ad7676_data->spi_desc->TxISR       = NULL;
	ad7676_data->spi_desc->TxXferSize  = 0U;
	ad7676_data->spi_desc->TxXferCount = 0U;

	CLEAR_BIT(ad7676_data->spi_desc->Instance->CR2, SPI_CR2_LDMARX);
	if (ad7676_data->spi_desc->Init.DataSize > SPI_DATASIZE_8BIT)
	{
	/* Set RX Fifo threshold according the reception data length: 16bit */
	CLEAR_BIT(ad7676_data->spi_desc->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	}
	/* Set the SPI RxDMA Half transfer complete callback */
//	ad7676_data->spi_desc->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;

	/* Set the SPI Rx DMA transfer complete callback */
	ad7676_data->spi_desc->hdmarx->XferCpltCallback = HAL_SPI_RxCpltCallback(ad7676_data->spi_desc); //TODO Finish it HAL uses SPI_DMAReceiveCplt

	/* Set the DMA error callback */
//	ad7676_data->spi_desc->hdmarx->XferErrorCallback = SPI_DMAError;

	/* Set the DMA AbortCpltCallback */
//	ad7676_data->spi_desc->hdmarx->XferAbortCallback = NULL;

	/* Enable the Rx DMA Stream/Channel  */
	if (HAL_OK != HAL_DMA_Start_IT(ad7676_data->spi_desc->hdmarx, (uint32_t)&ad7676_data->spi_desc->Instance->DR, (uint32_t)ad7676_data->spi_desc->pRxBuffPtr,
			ad7676_data->spi_desc->RxXferCount))
	{
	/* Update SPI error code */
	SET_BIT(ad7676_data->spi_desc->ErrorCode, HAL_SPI_ERROR_DMA);
	/* Process Unlocked */
	__HAL_UNLOCK(ad7676_data->spi_desc);
//	return HAL_ERROR;
	}

	/* Check if the SPI is already enabled */
	if ((ad7676_data->spi_desc->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
	/* Enable SPI peripheral */
	__HAL_SPI_ENABLE(ad7676_data->spi_desc);
	}

	/* Process Unlocked */
	__HAL_UNLOCK(ad7676_data->spi_desc);

	/* Enable the SPI Error Interrupt Bit */
	__HAL_SPI_ENABLE_IT(ad7676_data->spi_desc, (SPI_IT_ERR));

	/* Enable Rx DMA Request */
	SET_BIT(ad7676_data->spi_desc->Instance->CR2, SPI_CR2_RXDMAEN);

//	return HAL_OK;
}

int ad7676_calculate_output(int32_t sample){
	int sample_voltage = (sample*10*1000)/32768;
	return sample_voltage;  //assuming range is +/-10V and REF is internal 2,5V datasheet p.23
}

void ad7676_read_one_sample(uint64_t* timer) //when BUSY goes down
{

//	(GPIOx->IDR & GPIO_Pin);
//	GPIO_TypeDef GPIOB, D0_GPIO_Port, D15_GPIO_Port
//	Pin PB3 reserved for SWD
//	int16_t sample = (GPIOB->IDR & AD7676_GPIOB_MASK) | ((GPIOC->IDR & AD7676_GPIOC_MASK) << 15);
//	start_time = __HAL_TIM_GET_COUNTER(&htim2);
//	uint16_t buf[4];
	AD7676_CS_OFF;
	ad7676_spi_read(&ad7676_data->data_buf[ad7676_data->data_ptr], 4);

	HAL_SPI_Receive_DMA(ad7676_data->spi_desc, (uint8_t*)&ad7676_data->data_buf[ad7676_data->data_ptr], 4);

//	HAL_SPI_Receive_IT(ad7676_data->spi_desc, (uint8_t*)&ad7676_data->data_buf[ad7676_data->data_ptr], 4);
//	for(ad7676_data->current_channel=0; ad7676_data->current_channel<ad7676_data->num_channels; ad7676_data->current_channel++){
//		//ad7676_data->data_buf[ad7676_data->current_channel][ad7676_data->data_ptr] = buf[2*ad7676_data->current_channel+1]+(buf[2*ad7676_data->current_channel]<<8); //MSB first
//		ad7676_data->data_buf[ad7676_data->data_ptr] = buf[ad7676_data->current_channel]; //MSB first
//	}
//	memcpy(&ad7676_data->data_buf[ad7676_data->data_ptr].data, buf, 8);
//	ad7676_data->data_buf[ad7676_data->data_ptr].data = (uint64_t)buf[0]<<48 + (uint64_t)buf[1]<<32 + (uint64_t)buf[2]<<16 + (uint64_t)buf[3];
//	AD7676_CS_ON;
//	ad7676_data->data_buf[ad7676_data->data_ptr++] = sample;
	ad7676_data->data_ptr = (ad7676_data->data_ptr+1)%ad7676_data->data_ptr_max;
//	end_time = __HAL_TIM_GET_COUNTER(&htim2);
//	elapsed_time = end_time - start_time;
//	*timer = elapsed_time;
}

void ad7676_read_samples(uint16_t samples){
	awaited_samples = samples;
	collect_data = true;
}

void ad7676_read_continuous(bool enable){
	continuous_mode = enable;
}

void ad7676_display_samples(uint16_t awaited_samples, uint16_t* received_samples, void (*displayFunction)(char* message)){
	char buffer[64];
	int v1, v2, v3, v4;
	uint16_t tmp_ptr = ad7676_data->data_ptr - awaited_samples;
	collect_data = false;
	*received_samples = 0;
	sprintf(buffer, "Collected samples:%d\n\rCHANNEL1 CHANNEL2 CHANNEL3 CHANNEL4\n\r", awaited_samples);
	displayFunction(buffer);
	for(uint16_t i=0; i<awaited_samples; i++){
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
