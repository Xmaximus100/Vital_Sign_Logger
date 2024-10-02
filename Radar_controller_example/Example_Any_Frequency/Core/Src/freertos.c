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
#include "delay.h"
#include "ad7676.h"
#include "adf5355.h"
#include "basic_example.h"
#include "ring_buffer.h"
#include "parser.h"
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
struct adf5355_init_param hadf5355;
Data_Collector* ad7676_data;
RingBuffer buffer;
uint8_t receive_tmp;
uint8_t received_lines = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adc_handler */
osThreadId_t adc_handlerHandle;
const osThreadAttr_t adc_handler_attributes = {
  .name = "adc_handler",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pll_handler */
osThreadId_t pll_handlerHandle;
const osThreadAttr_t pll_handler_attributes = {
  .name = "pll_handler",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void ADF5355_Param_Init(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartADC(void *argument);
void StartPLL(void *argument);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
	uint8_t received_data[32];
	HAL_UART_Receive_IT(&huart2, &receive_tmp, 1);
  /* Infinite loop */
  for(;;)
  {
//	  UART_Log("Hello World\n\r");
//	  osDelay(1);
	  if(received_lines > 0){
		  ParserTakeLine(&buffer, received_data);
		  ParserParse((char*)received_data);
		  received_lines--;
	  }
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
	Initialize_Delay();
	ADF5355_Param_Init();
	basic_example_main(&hadf5355);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPLL */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void ADF5355_Param_Init(void){
	hadf5355.spi_init = &hspi3; // Wskaźnik do struktury SPI init
	hadf5355.dev_id = 1; // Identyfikator urządzenia ADF5355
	hadf5355.freq_req = 12000000000; // Żądana częstotliwość wyjściowa w Hz
	hadf5355.freq_req_chan = 0; // Kanał częstotliwości
	hadf5355.clkin_freq = 120000000; // Częstotliwość zegara wejściowego w Hz
	hadf5355.cp_ua = 1000; // Prąd pompy ładunkowej w mikroamperach
	hadf5355.cp_neg_bleed_en = false; // Flaga aktywacji negatywnego prądu wycieku
	hadf5355.cp_gated_bleed_en = false;  // Flaga aktywacji bramkowania prądu wycieku
	hadf5355.cp_bleed_current_polarity_en = false;  // Flaga aktywacji biegunowości prądu wycieku
	hadf5355.mute_till_lock_en = false; // Flaga aktywacji funkcji mute till lock
	hadf5355.outa_en = true;  // Flaga aktywacji wyjścia A
	hadf5355.outb_en = false;  // Flaga aktywacji wyjścia B
	hadf5355.outa_power = 10;  // Moc wyjścia A
	hadf5355.outb_power = 0;  // Moc wyjścia B
	hadf5355.phase_detector_polarity_neg = true;  // Flaga aktywacji negatywnej polaryzacji detektora fazy
	hadf5355.ref_diff_en = false;  // Flaga aktywacji różnicowego wejścia referencyjnego
	hadf5355.mux_out_3v3_en = true;  // Flaga aktywacji wyjścia mux na 3,3V
	hadf5355.ref_doubler_en = false;  // Flaga aktywacji podwajacza częstotliwości referencyjnej
	hadf5355.ref_div2_en = false;  // Flaga aktywacji podzielnika przez 2
	hadf5355.mux_out_sel = 0;  // Wybór wyjścia mux
	hadf5355.outb_sel_fund = false;  // Flaga wyboru częstotliwości podstawowej na wyjściu B
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){
		if(RB_OK == WriteToBuffer(&buffer, receive_tmp)){
			if(receive_tmp == ENDLINE){
				received_lines++;
			}
		}
		HAL_UART_Receive_IT(&huart2, &receive_tmp, 1);
	}
}
/* USER CODE END Application */

