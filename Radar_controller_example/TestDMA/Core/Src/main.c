/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AD7676_CS_ON		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin //HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET)
#define AD7676_CS_OFF		ADC_CS_GPIO_Port->BRR = (uint32_t)ADC_CS_Pin //HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET)
#define AD7676_CNVST_ON		ADC_CNVST_GPIO_Port->BSRR = (uint32_t)ADC_CNVST_Pin
#define AD7676_CNVST_OFF	ADC_CNVST_GPIO_Port->BRR = (uint32_t)ADC_CNVST_Pin
#define AD7676_CONVST_DELAY	for(uint8_t i=0; i<5; i++) __NOP() //12,5ns * 5 for 80MHz RCC clock
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t buf[1000][4];
uint16_t buf_ptr = 0;
uint16_t last_buf_ptr = 0;
uint32_t dma_itr_count = 0;
uint32_t spi_itr_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MX_DMA_Init(void);
void MX_SPI2_Init(void);
static void ad7676_spi_read_raw(uint8_t* buf, uint16_t size);
void DMA1_Channel4_IRQHandler(void);
static void ad7676_dma_enable_stream(uint16_t data_size, uint32_t src_addr, uint32_t dst_addr);
static void ad7676_spi_configuration();
static void ad7676_clock_configuration();
static void ad7676_dma_configuration();
void ad7676_spi_read_raw(uint8_t* buf, uint16_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_DMA_Init();
  MX_SPI2_Init();
  ad7676_clock_configuration();
  ad7676_spi_configuration();
  ad7676_dma_configuration();
  AD7676_CNVST_ON;
  AD7676_CONVST_DELAY;
  AD7676_CNVST_OFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_CNVST_GPIO_Port, ADC_CNVST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : B1_Pin ADC_BUSY_Pin */
  GPIO_InitStruct.Pin = B1_Pin|ADC_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_CNVST_Pin */
  GPIO_InitStruct.Pin = ADC_CNVST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ADC_CNVST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_CS_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MX_SPI2_Init(void){

  HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);

}
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

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

//    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
//	HAL_NVIC_EnableIRQ(SPI2_IRQn);
}

static void ad7676_dma_enable_stream(uint16_t data_size, uint32_t src_addr, uint32_t dst_addr){
	DMA1_Channel4->CNDTR = data_size;
	DMA1_Channel4->CPAR = src_addr;
	DMA1_Channel4->CMAR = dst_addr;
}

void SPI2_IRQHandler(void){
	SPI2->CR1 &= ~SPI_CR1_SPE;
	spi_itr_count++;
	while((SPI2->SR & SPI_CR1_SPE) != 0);
	while((SPI2->SR & SPI_SR_FRLVL) != 0);
	__NVIC_ClearPendingIRQ(SPI2_IRQn);
	SPI2->CR1 |= SPI_CR1_SPE;
}

void DMA1_Channel4_IRQHandler(void) //Remember to comment out this line in stm32l4xx_it.c row 170
{
	if(DMA1->ISR & DMA_ISR_TCIF4){

		SPI2->CR2 &= ~(SPI_CR2_RXDMAEN);
		SPI2->CR1 &= ~(SPI_CR1_SPE);
		while((SPI2->SR & SPI_SR_BSY) != 0);
		DMA1->IFCR |= DMA_IFCR_CTCIF4; // clear interrupt
		SPI2->CR2 &= ~SPI_CR2_RXNEIE;
		AD7676_CS_ON;
		buf_ptr = (buf_ptr+1)%1000;
		dma_itr_count++;
		AD7676_CNVST_ON;
		AD7676_CONVST_DELAY;
		AD7676_CNVST_OFF;
	} //do sth if DMA transfer complete is raised
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ADC_BUSY_Pin){
		if (buf_ptr > last_buf_ptr || buf_ptr == 0){
			  ad7676_spi_read_raw((uint8_t*) buf[buf_ptr], 4);
			  last_buf_ptr = buf_ptr;
		  }
	}
}

static void ad7676_spi_read_raw(uint8_t* buf, uint16_t size){ //try changing to 16_bit
	AD7676_CS_OFF;
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
//	SPI2->CR2 |= SPI_CR2_RXNEIE;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
