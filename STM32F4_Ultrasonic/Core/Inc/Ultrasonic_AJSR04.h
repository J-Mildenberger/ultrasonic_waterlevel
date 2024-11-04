/*
 * Ultrasonic_AJSR04.h
 *
 *  Created on: Nov 4, 2024
 *      Author: julia
 */

#ifndef INC_ULTRASONIC_AJSR04_H_
#define INC_ULTRASONIC_AJSR04_H_

#include "main.h"


#define WAIT_TIME_NEXT_MEAS 600000 /* in [ms] */


typedef enum {
    IDLE,
	STARTED,
    ONGOING,
    FINISHED,
} MeasurementState_t;
extern MeasurementState_t MeasurementState;

typedef enum {
	LOW,
	HIGH,
} TriggerSignalGPIOState_t;
extern TriggerSignalGPIOState_t TriggerSignalGPIOState;


void AJSR04_Init(void);
void PulseFunc_FSM(void);

#endif /* INC_ULTRASONIC_AJSR04_H_ */
