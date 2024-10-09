/*
 * delay.c
 *
 *  Created on: Sep 10, 2024
 *      Author: Celelele
 */

#include "delay.h"
#include <stdlib.h>
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/*****************************Private Variables********************************/
extern TIM_HandleTypeDef htim6;
static TIM_HandleTypeDef s_TimerInstance = {
    .Instance = TIM6
};

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

HAL_StatusTypeDef Initialize_Delay()
{
	return HAL_TIM_Base_Start(&htim6);
}

void delay_us(uint32_t us)
{
//	if (us > 999)
//	{
//		adf5355_delay_ms(ceil(us/1000));
//		return;
//	}
	taskENTER_CRITICAL();
	int timer_val_start = __HAL_TIM_GET_COUNTER(&s_TimerInstance);
	int timer_val = timer_val_start;
	while(abs(timer_val - timer_val_start) < us){
		timer_val = __HAL_TIM_GET_COUNTER(&s_TimerInstance);
	}
	taskEXIT_CRITICAL();
}

void delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}
