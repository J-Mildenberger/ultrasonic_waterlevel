/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Ultrasonic_AJSR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */
    extern uint64_t UserTick;
    UserTick++;
    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM2 global interrupt.
 */
uint8_t guard = 0;
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */

    /* USER CODE END TIM2_IRQn 0 */
    /* USER CODE BEGIN TIM2_IRQn 1 */
    LL_TIM_ClearFlag_UPDATE(TIM2);

    if (TriggerSignalGPIOState == LOW) {/* set Trigger Signal HIGH */
        LL_GPIO_SetOutputPin(PB7_TRIG_OUT_GPIO_Port, PB7_TRIG_OUT_Pin);
        TriggerSignalGPIOState = HIGH;
    } else if (TriggerSignalGPIOState == HIGH) {
        LL_GPIO_ResetOutputPin(PB7_TRIG_OUT_GPIO_Port, PB7_TRIG_OUT_Pin);
        NVIC_DisableIRQ(TIM2_IRQn);
        NVIC_ClearPendingIRQ(TIM2_IRQn);
        LL_TIM_DisableCounter(TIM2);
        LL_TIM_DisableIT_UPDATE(TIM2);
        LL_TIM_SetCounter(TIM2, (uint32_t) 0x00);
        TriggerSignalGPIOState = LOW;
        NVIC_EnableIRQ(TIM2_IRQn);
        guard = 1;
    }
    /* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM5 global interrupt.
 */
void TIM5_IRQHandler(void)
{
    /* USER CODE BEGIN TIM5_IRQn 0 */
    LL_TIM_ClearFlag_UPDATE(TIM5);
    LL_TIM_DisableCounter(TIM5);
    LL_TIM_DisableIT_UPDATE(TIM5);
    LL_TIM_SetCounter(TIM5, (uint32_t) 0x00);
    LL_TIM_EnableIT_UPDATE(TIM5);
    LL_TIM_EnableCounter(TIM5);
    /* USER CODE END TIM5_IRQn 0 */
    /* USER CODE BEGIN TIM5_IRQn 1 */

    /* USER CODE END TIM5_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//.word     EXTI0_IRQHandler                  /* EXTI Line0                   */
//.word     EXTI1_IRQHandler                  /* EXTI Line1                   */
//.word     EXTI2_IRQHandler                  /* EXTI Line2                   */
//.word     EXTI3_IRQHandler                  /* EXTI Line3                   */
//.word     EXTI4_IRQHandler                  /* EXTI Line4
//.word     EXTI9_5_IRQHandler                /* External Line[9:5]s
//.word     EXTI15_10_IRQHandler              /* External Line[15:10]s
#define RISING_EDGE     1
#define FALLING_EDGE    0
void EXTI4_IRQHandler(void)
{
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    if (guard) {
        LL_GPIO_SetOutputPin(PB6_DBG_TRIG_GPIO_Port, PB6_DBG_TRIG_Pin);
        *pEchoEdges++ = LL_TIM_GetCounter(TIM5);

        static uint8_t edge = RISING_EDGE;
        /* after measuring rising EDGE, measure falling EDGE */
        if (edge == FALLING_EDGE) {
            guard = 0;
            if (EchoEdges[0] >= EchoEdges[1]) /* Overflow */
            {
                AJSR04_Reset(); /* simply repeat measurement*/
                return;
            }

            EchoState = ECHO_RECEIVED;
            edge = RISING_EDGE;
            pEchoEdges = &EchoEdges[0];
            LL_GPIO_ResetOutputPin(PB6_DBG_TRIG_GPIO_Port, PB6_DBG_TRIG_Pin);
            return;

        }
        edge = FALLING_EDGE;
    }
}

/* USER CODE END 1 */
