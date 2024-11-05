/*
 * Ultrasonic_AJSR04.h
 *
 *  Created on: Nov 4, 2024
 *      Author: julia
 */

#ifndef INC_ULTRASONIC_AJSR04_H_
#define INC_ULTRASONIC_AJSR04_H_

#include "main.h"


#define WAIT_TIME_NEXT_MEAS 100//600000 /* in [ms] */

typedef enum {
    IDLE,
	STARTED,
    ONGOING,
    FINISHED,
} MeasurementState_t;
extern MeasurementState_t MeasurementState;

typedef enum {
    ECHO_IDLE,
    ECHO_RECEIVED,
    ECHO_CALCULATED,
} EchoState_t;
extern EchoState_t EchoState;

typedef enum {
	LOW,
	HIGH,
} TriggerSignalGPIOState_t;
extern TriggerSignalGPIOState_t TriggerSignalGPIOState;

extern uint32_t EchoEdges[2];
extern uint32_t *pEchoEdges;

void AJSR04_Reset(void);
//void PulseFunc_FSM(void);
uint32_t AJSR04_CalcDistanceEcho(uint32_t * risingEdge, uint32_t * fallingEdge);



#endif /* INC_ULTRASONIC_AJSR04_H_ */
