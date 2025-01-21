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
uint16_t received_samples = 0;
bool busy_dropped = false;
uint64_t end_time, elapsed_time = 0;
extern uint64_t start_time;
extern uint16_t awaited_samples;
extern bool collect_data;
extern bool continuous_mode;

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
/* Definitions for pll_handler */
osThreadId_t pll_handlerHandle;
const osThreadAttr_t pll_handler_attributes = {
  .name = "pll_handler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for at_cmds_handler */
osThreadId_t at_cmds_handlerHandle;
const osThreadAttr_t at_cmds_handler_attributes = {
  .name = "at_cmds_handler",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartADC(void *argument);
void StartPLL(void *argument);
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

  /* creation of pll_handler */
  pll_handlerHandle = osThreadNew(StartPLL, NULL, &pll_handler_attributes);

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
	for(;;)
	{
	//	  UARTLog("Hello World\n\r");
		osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever); //TODO prepare collect_data flag
		end_time = __HAL_TIM_GET_COUNTER(&htim2);
		elapsed_time = end_time - start_time;
		collect_data = false;
		received_samples = 0;
		uint32_t base_freq = 80000000;
//		uint32_t read_freq = base_freq/read_time;
		uint32_t collect_freq = (base_freq*awaited_samples)/elapsed_time;
		char buffer[50];
		ad7676_display_samples(awaited_samples, &received_samples, UARTLog);
		sprintf(buffer, "ADC Collect Freq: %d", (int)collect_freq);
		UARTLog(buffer);
	}
  /* USER CODE END StartADC */
}

/* USER CODE BEGIN Header_StartPLL */
/**
* @brief Function implementing the pll_handler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPLL */
void StartPLL(void *argument)
{
  /* USER CODE BEGIN StartPLL */

//	ADF5355_Param_Init();
//	basic_example_main(&hadf5355);
  /* Infinite loop */
	for(;;)
	{
//		UARTLog("Hello World\n\r");
		osDelay(10);
	}
  /* USER CODE END StartPLL */
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
	if (GPIO_Pin == ADC_BUSY_Pin && collect_data){
		ad7676_read_one_sample();
//		osThreadFlagsSet(adc_handlerHandle, 0x01);
//		if(busy_dropped == false)
//		busy_dropped = true;
	}
}
/* USER CODE END Application */

