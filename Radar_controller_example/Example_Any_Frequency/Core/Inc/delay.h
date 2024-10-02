/*
 * delay.h
 *
 *  Created on: Sep 10, 2024
 *      Author: Celelele
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stdint.h"

void Initialize_Delay(void);

void delay_us(uint32_t us);

void delay_ms(uint32_t ms);

#endif /* INC_DELAY_H_ */
