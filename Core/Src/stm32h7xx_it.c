#include "storage/sd_spi.h"
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
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
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan.h"
#include "usart.h"
#include "tim.h"
#include "bmi088.h"
#include "blackbox.h"
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

/* Keil Watch: first fault only, reset to zero on the next boot.
 * handler_msp is sampled inside the C handler, not the exception-entry MSP.
 * Do not dereference the fault stack here: it may itself be invalid. */
volatile struct
{
    uint32_t exception_number;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t abfsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t ccr;
    uint32_t psp;
    uint32_t handler_msp;
} g_fault_snapshot;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Fault_SaveSnapshot(void)
{
    /* Fault context must not call FreeRTOS, SPI/UART, or HAL timeout loops. */
    __disable_irq();
    TIM3_HeaterEmergencyOff();
    if (g_fault_snapshot.exception_number == 0U)
    {
        g_fault_snapshot.cfsr = SCB->CFSR;
        g_fault_snapshot.hfsr = SCB->HFSR;
        g_fault_snapshot.abfsr = SCB->ABFSR;
        g_fault_snapshot.bfar = SCB->BFAR;
        g_fault_snapshot.mmfar = SCB->MMFAR;
        g_fault_snapshot.ccr = SCB->CCR;
        g_fault_snapshot.psp = __get_PSP();
        g_fault_snapshot.handler_msp = __get_MSP();
        g_fault_snapshot.exception_number = __get_IPSR();
    }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
TIM3_HeaterEmergencyOff();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
while (1)
{
}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
Fault_SaveSnapshot();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
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
TIM3_HeaterEmergencyOff();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
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
Fault_SaveSnapshot();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
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
TIM3_HeaterEmergencyOff();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
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
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */
HAL_IncTick();
if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
{
    xPortSysTickHandler();
}
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

void FDCAN3_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan3);
}

void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
}

void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void DMA1_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi1_rx); }
void SPI1_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi1_sd); }

void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart10_rx); }
void USART10_IRQHandler(void) { HAL_UART_IRQHandler(&huart10); }

void TIM7_IRQHandler(void) { HAL_TIM_IRQHandler(&htim7); }
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(&htim4); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM7) Bmi088_NotifyFromISR();
  if (timer->Instance == TIM4) Blackbox_NotifyFromISR();
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
