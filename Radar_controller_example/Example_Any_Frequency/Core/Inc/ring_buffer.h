/*
 * ring_buffer.h
 *
 *  Created on: Sep 15, 2024
 *      Author: Celelele
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <inttypes.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"

#define BUFFER_SIZE 32

typedef enum
{
	RB_OK    	= 0x00U,
	RB_ERROR	= 0x01U,
} RB_State_TypeDef;

typedef struct
{
	uint8_t Buffer[BUFFER_SIZE];
	uint16_t Head;
	uint16_t Tail;
} RingBuffer;

uint8_t WriteToBuffer(RingBuffer *Buffer, uint8_t *Data, uint8_t Len);
uint8_t ReadFromBuffer(RingBuffer *Buffer, uint8_t *Data);
void FlushBuffer(RingBuffer *Buffer);


#endif /* INC_RING_BUFFER_H_ */
