/*
 * parser.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Celelele
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#define ENDLINE '\n'

//typedef struct __responses {
//
//}responses_TypeDef;

typedef struct __at_Commands {
	char* command;
	char* responsePositive;
	char* responseNegative;
	void* (*function)(void*);
	uint8_t optional_argument;
}at_Commands_TypeDef;

void ParserTakeLine(RingBuffer* buffer, uint8_t* destination);
void ParserParse(char* received_string);

#endif /* INC_PARSER_H_ */
