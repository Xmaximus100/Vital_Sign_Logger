/*
 * adf5355_handler.h
 *
 *  Created on: Oct 15, 2024
 *      Author: Celelele
 */

#ifndef ADF5355_INC_ADF5355_API_H_
#define ADF5355_INC_ADF5355_API_H_

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

void* ADF5355_SetFrequencyOut(void* new_freq);
void* ADF5355_SetFrequencyIn(void* new_freq);
void* ADF5355_SetPower(void* new_pow);
void* ADF5355_Enable(void* state);

void ADF5355_Param_Init(void);

void* ADF5355_Run(void* arg);

#endif /* ADF5355_INC_ADF5355_API_H_ */
