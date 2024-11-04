/*
 * Ultrasonic_AJSR04.c
 *
 *  Created on: Nov 4, 2024
 *      Author: julian
 */

#include "Ultrasonic_AJSR04.h"



MeasurementState_t MeasurementState = IDLE;
TriggerSignalGPIOState_t TriggerSignalGPIOState = LOW;



void AJSR04_Init(void)
{
//	MeasurementState_t UltraSonicState = IDLE;
//	TriggerSignalGPIOState_t TriggerSignalGPIOState = LOW;
	MeasurementState = IDLE;
	TriggerSignalGPIOState = LOW;


}


void PulseFunc_FSM(void)
{

	if (TriggerSignalGPIOState == LOW)
	{
		/* enable TIM2 IRQ */
		LL_TIM_EnableIT_UPDATE(TIM2);
		LL_TIM_EnableCounter(TIM2);

		return;
	}
	if (TriggerSignalGPIOState == HIGH)
	{
		TriggerSignalGPIOState = LOW;
		/* set Trigger Signal LOW */
		LL_GPIO_ResetOutputPin(PB7_TRIG_OUT_GPIO_Port, PB7_TRIG_OUT_Pin);
		LL_TIM_DisableCounter(TIM2);
		return;
	}
}
