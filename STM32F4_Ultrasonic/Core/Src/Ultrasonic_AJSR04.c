/*
 * Ultrasonic_AJSR04.c
 *
 *  Created on: Nov 4, 2024
 *      Author: julian
 */

#include "Ultrasonic_AJSR04.h"

EchoState_t EchoState = ECHO_IDLE;
MeasurementState_t MeasurementState = IDLE;
TriggerSignalGPIOState_t TriggerSignalGPIOState = LOW;

uint32_t EchoEdges[2] = {0,0};
uint32_t *pEchoEdges = &EchoEdges[0];
void AJSR04_Reset(void)
{
    MeasurementState = IDLE;
    TriggerSignalGPIOState = LOW;
    EchoState = ECHO_IDLE;
}

uint32_t AJSR04_CalcDistanceEcho(uint32_t * risingEdge, uint32_t * fallingEdge)
{
    uint32_t diff = ((*fallingEdge) - (*risingEdge));
    return diff;
}


