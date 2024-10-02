/*
 * utils.c
 *
 *  Created on: Oct 3, 2024
 *      Author: Celelele
 */

#include "main.h"
#include "string.h"
#include "usart.h"
#include "utils.h"

void UART_Log(char* message)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}
