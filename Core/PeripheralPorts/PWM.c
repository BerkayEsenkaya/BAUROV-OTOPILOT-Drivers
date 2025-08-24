/*
 * PWM.c
 *
 *  Created on: Aug 23, 2025
 *      Author: Berkay Esenkaya
 */
#include <stdint.h>
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "PWM.h"

PWM_HandleTypeDef_T THRUSTER_Vert_R, THRUSTER_Vert_L, THRUSTER_Horz_R, THRUSTER_Horz_L;

void PWM_Init(PWM_HandleTypeDef_T *handle, void* timerHandle, uint32_t timerChannel, uint16_t PWM_Min, uint16_t PWM_Max, uint16_t PWM_Notr){
	handle->handle = timerHandle;
	handle->TimerChannelNo = timerChannel;
	handle->PWM_Min = PWM_Min;
	handle->PWM_Max = PWM_Max;
	handle->PWM_Notr = PWM_Notr;
    handle->PWM = handle->PWM_Notr;
	PWM_SetPulse(handle);
}

void PWM_SetPulse(PWM_HandleTypeDef_T *handle)
{
    if (handle->PWM < handle->PWM_Min) handle->PWM = handle->PWM_Min;
    if (handle->PWM > handle->PWM_Max) handle->PWM = handle->PWM_Max;
    __HAL_TIM_SET_COMPARE((TIM_HandleTypeDef*)handle->handle, handle->TimerChannelNo, handle->PWM);
}
