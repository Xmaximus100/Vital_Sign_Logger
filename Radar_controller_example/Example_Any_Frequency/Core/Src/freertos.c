/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "delay.h"
#include "ad7676.h"
#include "adf5355.h"
#include "basic_example.h"
#include "ring_buffer.h"
#include "parser.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern data_Collector_TypeDef* ad7676_data;
RingBuffer buffer;
uint8_t receive_tmp[32];
uint8_t received_lines = 0;
uint32_t received_samples = 0;
bool busy_dropped = false;
uint64_t end_time, elapsed_time = 0;
extern uint64_t start_time;
extern bool raw_data;
extern uint32_t awaited_samples;
extern bool collect_data;
extern bool continuous_mode;
uint32_t test_iter = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for adc_handler */
osThreadId_t adc_handlerHandle;
const osThreadAttr_t adc_handler_attributes = {
  .name = "adc_handler",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for at_cmds_handler */
osThreadId_t at_cmds_handlerHandle;
const osThreadAttr_t at_cmds_handler_attributes = {
  .name = "at_cmds_handler",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartADC(void *argument);
void StartATCmds(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of adc_handler */
  adc_handlerHandle = osThreadNew(StartADC, NULL, &adc_handler_attributes);

  /* creation of at_cmds_handler */
  at_cmds_handlerHandle = osThreadNew(StartATCmds, NULL, &at_cmds_handler_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint8_t tmp_buf[20];
	uint8_t len;
  /* Infinite loop */
	for(;;)
	{
//		len = sprintf(tmp_buf, "TestDMA\n\r");
//		HAL_UART_Transmit_DMA(&huart2, tmp_buf, len); //To prevent receiving constant interrupts after sending
														//simply i
		osDelay(10);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartADC */
/**
* @brief Function implementing the adc_handler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC */
void StartADC(void *argument)
{
  /* USER CODE BEGIN StartADC */
	ad7676_init(&ad7676_data);
	HAL_TIM_Base_Start(&htim2);
  /* Infinite loop */
//	ad7676_read_samples(10000);
//	ad7676_start_conversion();
	for(;;)
	{
	//	  UARTLog("Hello World\n\r");
		osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever); //TODO prepare collect_data flag
		HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_4);
		end_time = __HAL_TIM_GET_COUNTER(&htim2);
		elapsed_time = end_time - start_time;
		collect_data = false;
		received_samples = 0;
		uint32_t base_freq = 80000000;
//		uint32_t read_freq = base_freq/read_time;
		uint32_t collect_freq = (base_freq*awaited_samples)/elapsed_time;
		char buffer[50];
		if(continuous_mode) continue;
		if(raw_data){
			ad7676_send_samples(awaited_samples, &received_samples, &huart2);
			raw_data = false;
		}
		else{
			ad7676_display_samples(awaited_samples, &received_samples, UARTLog);
			sprintf(buffer, "ADC Collect Freq: %d", (int)collect_freq);
			UARTLog(buffer);
		}
	}
  /* USER CODE END StartADC */
}

/* USER CODE BEGIN Header_StartATCmds */
/**
* @brief Function implementing the at_cmds_handler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartATCmds */
void StartATCmds(void *argument)
{
  /* USER CODE BEGIN StartATCmds */
	uint8_t received_data[32];
	//	HAL_UART_Receive_IT(&huart2, &receive_tmp, 1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_tmp, 32);
	UARTLog("Send any request\n\r");
  /* Infinite loop */
	for(;;)
	{
		osThreadFlagsWait(0x01, osFlagsNoClear, osWaitForever);
		if(received_lines > 0){
		  ParserTakeLine(&buffer, received_data);
		  ParserParse((char*)received_data);
		  received_lines--;
		}
		else osThreadFlagsClear(0x01);
	}
  /* USER CODE END StartATCmds */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Prevent unused argument(s) compilation warning */
	if(huart->Instance == USART2){
		if(RB_OK == WriteToBuffer(&buffer, receive_tmp, Size)){
			if(receive_tmp[Size-1] == ENDLINE){
				received_lines++;
				osThreadFlagsSet(at_cmds_handlerHandle, 0x01);
			}
		}
		else FlushBuffer(&buffer);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_tmp, 32);
	}

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART2){
//		if(RB_OK == WriteToBuffer(&buffer, receive_tmp, 1)){
//			if(receive_tmp == ENDLINE){
//				received_lines++;
//			}
//		}
//		HAL_UART_Receive_IT(&huart2, &receive_tmp, 1);
//	}
//}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim->Instance == TIM4){
		ad7676_start_conversion();
	}

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	if (hspi->Instance == SPI2){
		AD7676_CS_ON;
		if (received_samples < awaited_samples){
			ad7676_start_conversion();
			received_samples++;
		}
		else {
			osThreadFlagsSet(adc_handlerHandle, 0x01);
		}
//		osThreadFlagsSet(adc_handlerHandle, received_samples++);
//		ad7676_start_conversion();
	}
}

void SPI2_IRQHandler(void){

	SPI2->CR1 &= ~SPI_CR1_SPE;
	while((SPI2->CR1 & SPI_CR1_SPE) != 0);
//	while((SPI2->SR & SPI_SR_FRLVL) != 0);
	__NVIC_ClearPendingIRQ(SPI2_IRQn);
	SPI2->CR1 |= SPI_CR1_SPE;
}

void DMA1_Channel4_IRQHandler(void) //Remember to comment out this line in stm32l4xx_it.c row 170
{
	if(DMA1->ISR & DMA_ISR_TCIF4){

		SPI2->CR2 &= ~(SPI_CR2_RXDMAEN);
		DMA1->IFCR |= DMA_IFCR_CTCIF4; // clear interrupt
		__NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
		while(SPI2->SR & SPI_FLAG_FTLVL){
			uint8_t temp = SPI2->DR;
					temp = SPI2->SR;
		};
		SPI2->CR1 &= ~(SPI_CR1_SPE);
		while((SPI2->SR & SPI_SR_BSY) != 0)
		SPI2->CR2 &= ~SPI_CR2_RXNEIE;
		AD7676_CS_ON;
		if (!collect_data) return;
		else if (received_samples++ < awaited_samples){
			if (continuous_mode){
				ad7676_send_sample(&huart2, &received_samples);
			}
			else ad7676_start_conversion();
			ad7676_data->data_ptr = (ad7676_data->data_ptr + 1) % ad7676_data->data_ptr_max;
//			continue;
		}
		else osThreadFlagsSet(adc_handlerHandle, 0x01);
	} //do sth if DMA transfer complete is raised
}

void DMA1_Channel5_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF5) { // Sprawdzamy flagę zakończenia transferu dla kanału TX
        DMA1->IFCR |= DMA_IFCR_CTCIF5;  // Czyścimy flagę przerwania
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;  // Wyłączamy TX DMA
        // Tutaj możesz dodać dodatkowe działania, np. ustawienie CS do stanu wysokiego
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
	if (hspi->Instance == SPI2){
		while(1) __NOP();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  if (huart->Instance == USART2){

  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ADC_BUSY_Pin){
		if (collect_data) ad7676_read_one_sample();
		else HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_4);
//		osThreadFlagsSet(adc_handlerHandle, 0x01);
//		if(busy_dropped == false)
//		busy_dropped = true;
	}
}

/* USER CODE END Application */

