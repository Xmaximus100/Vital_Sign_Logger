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
extern bool continuous_mode;
uint64_t start_time = 0;
bool raw_data = false;

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

void* SetADCMode(void* state){
	static bool ret;
	uint8_t* value = (uint8_t*)state;
	if (*value != 0 && *value != 1) ret = false;
	else {
		ad7676_read_continuous(*value);
		ret = true;
	}
	return &ret;
}

void* SetADCRange(void* state){
	static bool ret;
	uint16_t* value = (uint16_t*)state;
	if (*value > 2) ret = false;
	else {
		ad7676_data->range_select = *value;
		uint16_t buffer[] = {(uint16_t)(0x03<<8)+(*value<<4)+(*value), (uint16_t)(0x04<<8)+(*value<<4)+(*value), (uint16_t)0x0000};
//		ad7676_write_register(buffer, 3);
		ad7676_spi_write_read_raw(buffer, &ad7676_data->rx_buffer[ad7676_data->rx_buffer_ptr], 3);
		ad7676_data->rx_buffer_ptr = (ad7676_data->rx_buffer_ptr + 3) % ad7676_data->rx_buffer_ptr_max;
		ret = true;
	}
	return &ret;
}

void* ReadADCRange(void* state){
	static bool ret;
	uint16_t buffer[] = {(uint16_t)(0x03<<8)+(uint16_t)(0x1<<14), (uint16_t)(0x04<<8)+(uint16_t)(0x1<<14), (uint16_t)0x0000+(uint16_t)(0x1<<14)};
//	ad7676_write_register(buffer, 3);
	ad7676_spi_write_read_raw(buffer, &ad7676_data->rx_buffer[ad7676_data->rx_buffer_ptr], 3);
	ad7676_data->rx_buffer_ptr = (ad7676_data->rx_buffer_ptr + 3) % ad7676_data->rx_buffer_ptr_max;
	ret = true;
	return &ret;
}

void* ReadADC(void* samples){
	static bool ret;
	uint32_t* value = (uint32_t*)samples;
	if (*value <= 0 && ((*value > ad7676_data->data_ptr_max && continuous_mode == 0) || continuous_mode == 1)) ret = false;
	else {
		ad7676_read_samples(*value);
		start_time = __HAL_TIM_GET_COUNTER(&htim2);
		ret = true;
	}
	return &ret;
}

void* ReadRawADC(void* samples){
	static bool ret;
	uint32_t* value = (uint32_t*)samples;
	if (*value <= 0 && ((*value > ad7676_data->data_ptr_max && continuous_mode == 0) || continuous_mode == 1)) ret = false;
	else {
		ad7676_read_samples(*value);
		raw_data = true;
		start_time = __HAL_TIM_GET_COUNTER(&htim2);
		ret = true;
	}
	return &ret;
}

void* ResetADC(void* arg){
	static bool ret = true;
	ad7676_reset();
	return &ret;
}

