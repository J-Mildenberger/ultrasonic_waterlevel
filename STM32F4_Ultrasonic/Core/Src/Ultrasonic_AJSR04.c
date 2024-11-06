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

dbg_t buff_dbg[32];
dbg_t *pbuff_dbg = &buff_dbg[0];

extern uint64_t UserTick;
extern void Reset_TIM5(void);



void AJSR04_Reset(void)
{
    MeasurementState = IDLE;
    TriggerSignalGPIOState = LOW;
    EchoState = ECHO_IDLE;
    UserTick = (WAIT_TIME_NEXT_MEAS+1);
    pEchoEdges = &EchoEdges[0];
}

uint8_t AJSR04_App(float *distance)
{
    if (MeasurementState == FINISHED) {
        UserTick = 0;
        DBG_TRACE MeasurementState = IDLE;
        Reset_TIM5(); /* prevent timer_cnt overflow */
    }
    if (MeasurementState == IDLE) {
        DBG_TRACE
        if (UserTick >= WAIT_TIME_NEXT_MEAS) {
            MeasurementState = STARTED;
            DBG_TRACE
        }
    } else if (MeasurementState == STARTED) {
        MeasurementState = ONGOING;
        DBG_TRACE
        /* enable TIM2 IRQ */
        LL_TIM_EnableIT_UPDATE(TIM2); /* #TODO improvement: start TIM OnePulseMode */
        LL_TIM_EnableCounter(TIM2);
    } else if (MeasurementState == ONGOING) {
        /* Echo Signal handled @EXTI ISR */
        /* get Rising Edge TIMER Value  */
        /* get Falling Edge TIMER Value */
        /* calc diff  */
        DBG_TRACE
        if (EchoState == ECHO_RECEIVED) {
            *distance = AJSR04_CalcDistanceEcho(&EchoEdges[0], &EchoEdges[1]);
            MeasurementState = FINISHED;
            DBG_TRACE EchoState = ECHO_CALCULATED;
        }
    }
}


float AJSR04_CalcDistanceEcho(uint32_t * risingEdge, uint32_t * fallingEdge)
{
    uint32_t diff = ((*fallingEdge) - (*risingEdge));

    /* Formula: distance = velocity * (time / 2) (Ultrasound is emitted and reflected back)
     * Simplified Ultrasound velocity ~344m/s ; in [cm/us] this is 0.0344cm/us ; in [cm/ms] this is 34.4cm/ms
     * Formula: distance = measured "ECHO" pulsewidth * (0.0344 / 2) */
    //float distance = diff * (0.0344 / 2); //   10^-3s * (0.0172cm/10^-6 s)
    float diff_ms = diff / 10;
    float distance_cm = diff_ms * (34.4 / 2); //   10^-3s * (0.0172cm/10^-6 s)

    return distance_cm;
}


