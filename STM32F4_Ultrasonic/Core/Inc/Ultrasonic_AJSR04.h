/*
 * Ultrasonic_AJSR04.h
 *
 *  Created on: Nov 4, 2024
 *      Author: julia
 */

#ifndef INC_ULTRASONIC_AJSR04_H_
#define INC_ULTRASONIC_AJSR04_H_

#include "main.h"


#define WAIT_TIME_NEXT_MEAS 2000//600000 /* in [ms] */

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
uint8_t AJSR04_App(float *distance);
float AJSR04_CalcDistanceEcho(uint32_t * risingEdge, uint32_t * fallingEdge);



#ifdef DEBUG
typedef struct {
    uint32_t timer;
    MeasurementState_t MeasState;
    TriggerSignalGPIOState_t TrigState;
    EchoState_t EchoState;
    uint32_t RisingEdge;
    uint32_t FallingEdge;
} dbg_t;

extern dbg_t buff_dbg[32];
extern dbg_t *pbuff_dbg;

#define DBG_TRACE  {\
    pbuff_dbg->MeasState = MeasurementState;\
    pbuff_dbg->TrigState = TriggerSignalGPIOState;\
    pbuff_dbg->EchoState = EchoState;\
    pbuff_dbg->RisingEdge   = EchoEdges[0];\
    pbuff_dbg->FallingEdge  = EchoEdges[1];\
    pbuff_dbg->timer = LL_TIM_GetCounter(TIM5);\
    pbuff_dbg++;\
    if (pbuff_dbg == &buff_dbg[31]){pbuff_dbg = &buff_dbg[0];}};
#else
#define DBG_TRACE
#endif



#endif /* INC_ULTRASONIC_AJSR04_H_ */
