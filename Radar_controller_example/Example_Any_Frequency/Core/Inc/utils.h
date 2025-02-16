/*
 * utils.h
 *
 *  Created on: Oct 3, 2024
 *      Author: Celelele
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

void UARTLog(char* message);
//void* SetPLL_FF(void* fill_factor);
//void* SetPLL_Period(void* period_ms);
void* LightLED(void* state);
void* ReadADC(void* samples);
void* ReadRawADC(void* samples);

#endif /* INC_UTILS_H_ */
