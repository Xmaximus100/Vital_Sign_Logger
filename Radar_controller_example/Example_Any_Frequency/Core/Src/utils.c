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
#include "utils.h"

extern struct adf5355_init_param hadf5355;
extern struct adf5355_dev* dev;

void UARTLog(char* message)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}

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

void* LoadADF5355(void* arg){
	ADF5355_Param_Init();
	static bool ret = false;
	int32_t response = adf5355_init(&dev, &hadf5355);
	if (response == 0) ret = true;
	return &ret;

}
