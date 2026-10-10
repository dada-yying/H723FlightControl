/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   ESC/heater PWM and TIM7 BMI088 acquisition declarations.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
HAL_StatusTypeDef MX_TIM3_Init(void); /* Optional BMI088 heater, PB1/CH4. */
HAL_StatusTypeDef MX_TIM4_Init(void); /* Optional independent blackbox trigger. */
HAL_StatusTypeDef MX_TIM7_Init(void); /* Optional BMI088 500 us task trigger. */
void TIM3_HeaterEmergencyOff(void); /* Fault context: registers only. */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
