/*
 * parser.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Celelele
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#define ENDLINE '\n'

void ParserTakeLine(RingBuffer* buffer, uint8_t* destination);
void ParserParse(char* received_string);

#endif /* INC_PARSER_H_ */
