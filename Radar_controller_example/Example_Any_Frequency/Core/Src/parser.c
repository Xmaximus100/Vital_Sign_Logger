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
#include "stdio.h"
#include "stdlib.h"
#include "adf5355.h"
#include "adf5355_api.h"
#include "parser.h"

at_Commands_TypeDef at_cmds[] = {
	{"LED", "LED set to %d\n\r", "Type LED 0 or 1\n\r", LightLED, 0},
	{"FREQOut", "FREQ Out set to %d\n\r", "Type FREQOut in MHz\n\r", ADF5355_SetFrequencyOut, 0},
	{"FREQIn", "FREQ In set to %d\n\r", "Type FREQIn in MHz\n\r", ADF5355_SetFrequencyIn, 0},
	{"POW", "POW set to %d\n\r", "Type POW between 0 and 3 in dBm\n\r", ADF5355_SetPower, 0},
	{"CURR", "CURR set to %d\n\r", "Type CURR between 310 and 5000 in mA\n\r", ADF5355_SetCurrent, 0},
	{"MUXOUT", "MUXOUT set to %d\n\r", "Type MUXOUT - THREE_STATE, VDD, GROUND, R_DIV, N_DIV, AN_LD, DIG_LD\n\r", ADF5355_SetMuxOut, 0},
	{"EN", "EN set to %d\n\r", "Type EN 0 or 1\n\r", ADF5355_Enable, 0},
	{"RUN", "New configuration applied\n\r", "New configuration failed to apply\n\r", ADF5355_Run, 1},
	{"SETUP", "Configuration succeed\n\r", "Configuration failed\n\r", ADF5355_Load, 1},
	{"READ", "ADC Read success\n\r", "ADC Read failed\n\r", ReadADC, 0},
	{"READRAW", "1111111111", "0000000000", ReadRawADC, 0}
};

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
	char* endptr;
	char* parse_pointer = strtok(received_string, "=");
	int32_t value = strtol(strtok(NULL,","), &endptr, 10);
	char buffer[100];
	bool cmd_matched = false;
	for (int i=0; i<sizeof(at_cmds)/sizeof(at_Commands_TypeDef); i++){
		if(strcmp(at_cmds[i].command, parse_pointer) == 0){
			bool* result = at_cmds[i].function(&value);
			if (*result == true && (endptr != 0x00 || at_cmds[i].optional_argument == 1)){
				sprintf(buffer, at_cmds[i].responsePositive, value);
			}
			else {
				sprintf(buffer, at_cmds[i].responseNegative, value);
			}
			cmd_matched = true;
		}
	}
	if (!cmd_matched){
		sprintf(buffer, "Available commands are LED, FREQOut, FREQIn, POW, CURR, MUXOUT, EN, RUN, SETUP and READ\n\r");
	}
	UARTLog(buffer);
}
