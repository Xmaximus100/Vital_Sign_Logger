

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
static uint64_t start_time, end_time, elapsed_time = 0;


static void ad7676_spi_configuration(){
//	SPI_CR1_BIDIMODE 0
//	SPI_CR1_BIDIOE 0
//	SPI2->CR1 |= SPI_CR1_CRCEN;
	SPI2->CR1 |= SPI_CR1_RXONLY;
//	SPI_CR1_LSBFIRST 0
//	SPI2->CR1 |= SPI_CR1_SPE; //enable when ready
	SPI2->CR1 |= SPI_CR1_BR_0; // | SPI_CR1_BR_1); //ultimately leave 0
	SPI2->CR1 |= SPI_CR1_MSTR;
	SPI2->CR1 |= SPI_CR1_CPOL; //spi configuration CPOL 1 CPHA 0
//	SPI2->CR1 |= SPI_CR1_CPHA 0

//	SPI2->CR1 = SPI_CR1_CRCEN | SPI_CR1_RXONLY |
//			SPI_CR1_BR_2 | SPI_CR1_MSTR | SPI_CR1_CPOL;

//	SPI2->CR2 |= SPI_CR2_FRXTH 0
	SPI2->CR2 |= SPI_CR2_DS;
//	SPI2->CR2 |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2);
	SPI2->CR2 |= SPI_CR2_RXNEIE; //enable when RXNE interrupt necessary
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
//	DMA1_Channel4->CCR |= DMA_CCR_PINC; //periph increment - we reads spi register so its always the same
//	DMA1_Channel4->CCR |= DMA_CCR_DIR 0
	DMA1_Channel4->CCR |= DMA_CCR_TCIE; //transfer complete interrupt en
//	DMA1_Channel4->CCR |= DMA_CCR_EN; //TODO check if needed to set
	uint8_t num_channel = 4;
	uint8_t num_half_bytes = 4;
	DMA1_CSELR->CSELR &= ~(0xF << num_half_bytes*(num_channel-1));
	DMA1_CSELR->CSELR |= 1 << num_half_bytes*(num_channel-1);
}

static void ad7676_clock_configuration(){
	__HAL_RCC_SPI2_CLK_ENABLE();

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2;
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
}


void ad7676_init(data_Collector_TypeDef** ad7676_data)
{
	data_Collector_TypeDef* init_data;

	init_data = (data_Collector_TypeDef*)no_os_calloc(1, sizeof(*init_data));

	init_data->spi_desc = &hspi2;
	init_data->data_ptr = 0;
	init_data->data_ptr_max = 500;
	init_data->current_channel = 0;
	init_data->num_channels = 4;

	*ad7676_data = init_data;

	ad7676_clock_configuration();
	ad7676_spi_configuration();
	ad7676_dma_configuration();
}

static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{

	/* Check the parameters */
	assert_param(IS_DMA_BUFFER_SIZE(DataLength));

	/* Process locked */
	__HAL_LOCK(hdma);

    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Disable the peripheral */
    __HAL_DMA_DISABLE(hdma);
	/* Clear all flags */
	hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));

	/* Configure DMA Channel data length */
	hdma->Instance->CNDTR = DataLength;

	/* Configure DMA Channel source address */
	hdma->Instance->CPAR = SrcAddress;

	/* Configure DMA Channel destination address */
	hdma->Instance->CMAR = DstAddress;

	__HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
	__HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_TE));

	__HAL_DMA_ENABLE(hdma);

}

static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
                                                       uint32_t Timeout, uint32_t Tickstart)
{
  __IO uint32_t count;
  uint32_t tmp_timeout;
  uint32_t tmp_tickstart;

  /* Adjust Timeout value  in case of end of transfer */
  tmp_timeout   = Timeout - (HAL_GetTick() - Tickstart);
  tmp_tickstart = HAL_GetTick();

  /* Calculate Timeout based on a software loop to avoid blocking issue if Systick is disabled */
  count = tmp_timeout * ((SystemCoreClock * 32U) >> 20U);

  while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State)
  {
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tmp_tickstart) >= tmp_timeout) || (tmp_timeout == 0U))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
           on both master and slave sides in order to resynchronize the master
           and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                                                     || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);
        }

        hspi->State = HAL_SPI_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        return HAL_TIMEOUT;
      }
      /* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
      if (count == 0U)
      {
        tmp_timeout = 0U;
      }
      count--;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef SPI_WaitFifoStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Fifo, uint32_t State,
                                                       uint32_t Timeout, uint32_t Tickstart)
{
  __IO uint32_t count;
  uint32_t tmp_timeout;
  uint32_t tmp_tickstart;
  __IO const uint8_t  *ptmpreg8;
  __IO uint8_t  tmpreg8 = 0;

  /* Adjust Timeout value  in case of end of transfer */
  tmp_timeout = Timeout - (HAL_GetTick() - Tickstart);
  tmp_tickstart = HAL_GetTick();

  /* Initialize the 8bit temporary pointer */
  ptmpreg8 = (__IO uint8_t *)&hspi->Instance->DR;

  /* Calculate Timeout based on a software loop to avoid blocking issue if Systick is disabled */
  count = tmp_timeout * ((SystemCoreClock * 35U) >> 20U);

  while ((hspi->Instance->SR & Fifo) != State)
  {
    if ((Fifo == SPI_SR_FRLVL) && (State == SPI_FRLVL_EMPTY))
    {
      /* Flush Data Register by a blank read */
      tmpreg8 = *ptmpreg8;
      /* To avoid GCC warning */
      UNUSED(tmpreg8);
    }

    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - tmp_tickstart) >= tmp_timeout) || (tmp_timeout == 0U))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
           on both master and slave sides in order to resynchronize the master
           and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                                                     || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);
        }

        hspi->State = HAL_SPI_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        return HAL_TIMEOUT;
      }
      /* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
      if (count == 0U)
      {
        tmp_timeout = 0U;
      }
      count--;
    }
  }

  return HAL_OK;
}

static void DMA_callback(DMA_HandleTypeDef *hdma){
	SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);
	uint32_t tickstart;

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	/* DMA Normal Mode */
	if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
	{
		/* Disable ERR interrupt */
		__HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

		/* Normal case */
		CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

		__HAL_SPI_DISABLE(hspi);
		/* Check the end of the transaction */
		SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, 100U, tickstart);
		SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, 100U, tickstart);
		//	if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, 100U, tickstart) != HAL_OK)
		//	{
		//	  hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
		//	}
		//	if (SPI_WaitFifoStateUntilTimeout(hspi, SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, 100U, tickstart) != HAL_OK)
		//	{
		//	  SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
		//	  return HAL_TIMEOUT;
		//	}

		hspi->RxXferCount = 0U;
		hspi->State = HAL_SPI_STATE_READY;

	}
	HAL_SPI_RxCpltCallback(ad7676_data->spi_desc);
}

static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);

  /* Stop the disable DMA transfer on SPI side */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
  hspi->State = HAL_SPI_STATE_READY;
  /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->ErrorCallback(hspi);
#else
  HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
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

    __HAL_SPI_DISABLE(ad7676_data->spi_desc);
    SPI_1LINE_RX(ad7676_data->spi_desc);

	CLEAR_BIT(ad7676_data->spi_desc->Instance->CR2, SPI_CR2_LDMARX);
	/* Set RX Fifo threshold according the reception data length: 16bit */
	CLEAR_BIT(ad7676_data->spi_desc->Instance->CR2, SPI_RXFIFO_THRESHOLD);
	/* Set the SPI RxDMA Half transfer complete callback */
//	ad7676_data->spi_desc->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;

	/* Set the SPI Rx DMA transfer complete callback */
	ad7676_data->spi_desc->hdmarx->XferCpltCallback = DMA_callback;//HAL_SPI_RxCpltCallback(ad7676_data->spi_desc); //TODO Finish it HAL uses SPI_DMAReceiveCplt
	ad7676_data->spi_desc->hdmarx->XferErrorCallback = SPI_DMAError;

	/* Set the DMA error callback */
//	ad7676_data->spi_desc->hdmarx->XferErrorCallback = SPI_DMAError;

	/* Set the DMA AbortCpltCallback */
//	ad7676_data->spi_desc->hdmarx->XferAbortCallback = NULL;

	/* Enable the Rx DMA Stream/Channel  */
	DMA_SetConfig(ad7676_data->spi_desc->hdmarx, (uint32_t)&ad7676_data->spi_desc->Instance->DR, (uint32_t)ad7676_data->spi_desc->pRxBuffPtr, ad7676_data->spi_desc->RxXferCount);
//	if (HAL_OK != HAL_DMA_Start_IT(ad7676_data->spi_desc->hdmarx, (uint32_t)&ad7676_data->spi_desc->Instance->DR, (uint32_t)ad7676_data->spi_desc->pRxBuffPtr,
//			ad7676_data->spi_desc->RxXferCount))
//	{
//	/* Update SPI error code */
//	SET_BIT(ad7676_data->spi_desc->ErrorCode, HAL_SPI_ERROR_DMA);
//	/* Process Unlocked */
//	__HAL_UNLOCK(ad7676_data->spi_desc);
////	return HAL_ERROR;
//	}

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

static void ad7676_spi_transaction(){
	while(SPI2->SR & SPI_SR_BSY); //check if SPI is busy
//	SPI2->SR & SPI_SR_RXNE //check if RX buffer is empty
//	SPI2->DR //SPI FIFO pointer
}

static void ad7676_dma_enable_stream(uint16_t data_size, uint32_t src_addr, uint32_t dst_addr){
	DMA1_Channel4->CNDTR = data_size;
	DMA1_Channel4->CPAR = src_addr;
	DMA1_Channel4->CMAR = dst_addr;
}

//void DMA1_Channel4_IRQHandler(void) //Remember to comment out this line in stm32l4xx_it.c row 170
//{
//	if(DMA1->ISR & DMA_ISR_TCIF4){
//
//		SPI2->CR2 &= ~(SPI_CR2_RXDMAEN);
//		SPI2->CR1 &= ~(SPI_CR1_SPE);
//
//		DMA1->IFCR |= DMA_IFCR_CTCIF4; // clear interrupt
//	} //do sth if DMA transfer complete is raised
//}

void ad7676_spi_read_raw(uint8_t* buf, uint16_t size){
	SPI2->CR1 &= ~(SPI_CR1_SPE);
	SPI2->CR2 &= ~(SPI_CR2_RXDMAEN);
	SPI2->CR2 &= ~(SPI_CR2_FRXTH);

	DMA1_Channel4->CCR &= ~(DMA_CCR_EN);
	DMA1->IFCR |= DMA_ISR_GIF4;
	ad7676_dma_enable_stream(size, (uint32_t)&(SPI2->DR), (uint32_t)buf);
	DMA1_Channel4->CCR |= DMA_CCR_TCIE;
	DMA1_Channel4->CCR |= DMA_CCR_EN; //DMA en

	SPI2->CR1 |= SPI_CR1_SPE;
	SPI2->CR2 |= SPI_CR2_RXDMAEN; //enable RX DMA interrupt
}

void ad7676_read_one_sample() //when BUSY goes down
{

//	(GPIOx->IDR & GPIO_Pin);
//	GPIO_TypeDef GPIOB, D0_GPIO_Port, D15_GPIO_Port
//	Pin PB3 reserved for SWD
//	int16_t sample = (GPIOB->IDR & AD7676_GPIOB_MASK) | ((GPIOC->IDR & AD7676_GPIOC_MASK) << 15);
//	start_time = __HAL_TIM_GET_COUNTER(&htim2);
//	uint16_t buf[4];
	AD7676_CS_OFF;
//	ADC_CS_GPIO_Port->BRR = (uint32_t)ADC_CS_Pin;
	ad7676_spi_read_raw((uint8_t*)&ad7676_data->data_buf[ad7676_data->data_ptr], 4);

//	HAL_SPI_Receive_DMA(ad7676_data->spi_desc, (uint8_t*)&ad7676_data->data_buf[ad7676_data->data_ptr], 4);

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

void ad7676_read_samples(uint32_t samples){
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

void ad7676_send_samples(uint16_t awaited_samples, uint16_t* received_samples, UART_HandleTypeDef* huart){
    uint16_t tmp_ptr = ad7676_data->data_ptr - awaited_samples;
    collect_data = false;
    *received_samples = 0;

    for(uint16_t i = 0; i < awaited_samples; i++){
        uint8_t frame[10]; // 8 bytes for data + 1 for the null terminator

        // Copy 8 bytes directly from the data buffer
        // Make sure to specify the number of bytes (4 channels x 2 bytes)
        memcpy(frame, &i, 2);
        memcpy(frame+2, &(ad7676_data->data_buf[(tmp_ptr + i) % ad7676_data->data_ptr_max]), 8);
//        frame[10] = '\n'; // Null-terminate the string (optional if displayFunction expects a C-string)
        HAL_UART_Transmit(huart, frame, 10, 1000);
    }

    // Send termination frame
//    displayFunction("END\n\r");
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
