/*
 * parser.c
 *
 *  Created on: Oct 3, 2024
 *      Author: Celelele
 */

#include "main.h"
#include "utils.h"
#include "ring_buffer.h"
#include "string.h"
#include "parser.h"

void ParserTakeLine(RingBuffer* buffer, uint8_t* destination){
	uint8_t i = 0;
	uint8_t tmp;
	  do{
		  ReadFromBuffer(buffer, &tmp);
		  if(tmp == ENDLINE){
			  destination[i] = 0;
		  }
		  else{
			  destination[i] = tmp;
		  }
		  i++;
	  }while(tmp != ENDLINE);
}

void ParserParse(char* received_string){
	if(strcmp("LED_ON", received_string) == 0){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		UART_Log("LED ON\n\r");
	}
	else if(strcmp("LED_OFF", received_string) == 0){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		UART_Log("LED OFF\n\r");
	}
}
