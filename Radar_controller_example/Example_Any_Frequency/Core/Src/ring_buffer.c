/*
 * ring_buffer.c
 *
 *  Created on: Sep 15, 2024
 *      Author: Celelele
 */

#include "ring_buffer.h"

uint8_t WriteToBuffer(RingBuffer *Buffer, uint8_t *Data, uint8_t Len)
{
	uint8_t TempHead;

	for(int i=0; i<Len; i++){
		TempHead = (Buffer->Head + 1) % BUFFER_SIZE;

		if( TempHead == Buffer->Tail) // No room for new data
		{
			return RB_ERROR;
		}
		else
		{
			Buffer->Buffer[Buffer->Head] = *(Data+i);

			Buffer->Head++;
			Buffer->Head %= BUFFER_SIZE;
		}
	}
	return RB_OK;
}

uint8_t ReadFromBuffer(RingBuffer *Buffer, uint8_t *Data)
{
	if( Buffer->Tail == Buffer->Head) // No data to read
	{
		return RB_ERROR;
	}
	else
	{
		*Data = Buffer->Buffer[Buffer->Tail];

		Buffer->Tail++;
		Buffer->Tail %= BUFFER_SIZE;
	}
	return RB_OK;
}

void FlushBuffer(RingBuffer *Buffer)
{
	Buffer->Tail = 0;
	Buffer->Head = 0;
}
