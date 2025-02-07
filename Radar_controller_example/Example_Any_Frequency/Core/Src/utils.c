/*
 * utils.c
 *
 *  Created on: Oct 3, 2024
 *      Author: Celelele
 */

#include "main.h"
#include "string.h"
#include "usart.h"
#include "adf5355.h"
#include "adf5355_api.h"
#include "ad7676.h"
#include "utils.h"
#include "tim.h"

extern struct adf5355_init_param hadf5355;
extern data_Collector_TypeDef* ad7676_data;
uint64_t start_time = 0;

void UARTLog(char* message)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}

//void* SetPLL_FF(void* fill_factor){
//
//}

//void* SetPLL_Period(void* period_ms){
//
//}

void* LightLED(void* state){
	static bool ret;
	uint8_t* value = (uint8_t*)state;
	if (*value != 0 && *value != 1) ret = false;
	else {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, *value);
		ret = true;
	}
	return &ret;
}

void* ReadADC(void* samples){
	static bool ret;
	uint32_t* value = (uint16_t*)samples;
	if (*value <= 0 && *value > ad7676_data->data_ptr_max) ret = false;
	else {
		ad7676_read_samples(*value);
		start_time = __HAL_TIM_GET_COUNTER(&htim2);
		ad7676_start_conversion();
		ret = true;
	}
	return &ret;
}
