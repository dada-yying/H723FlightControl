/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM1/TIM2 ESC PWM and TIM3 board BMI088 heater PWM.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "tim.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

HAL_StatusTypeDef MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef channel = {0};
  HAL_StatusTypeDef status;
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  status = HAL_TIM_PWM_Init(&htim3);
  if (status != HAL_OK) return status;
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  status = HAL_TIMEx_MasterConfigSynchronization(&htim3, &master);
  if (status != HAL_OK) return status;
  channel.OCMode = TIM_OCMODE_PWM1;
  channel.Pulse = 0;
  channel.OCPolarity = TIM_OCPOLARITY_HIGH;
  channel.OCFastMode = TIM_OCFAST_DISABLE;
  status = HAL_TIM_PWM_ConfigChannel(&htim3, &channel, TIM_CHANNEL_4);
  if (status != HAL_OK) return status;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  HAL_TIM_MspPostInit(&htim3);
  return HAL_OK;
}

void TIM3_HeaterEmergencyOff(void)
{
  /* No HAL/RTOS, floating point, or wait in fault context. Only this heater. */
  if (__HAL_RCC_TIM3_IS_CLK_ENABLED())
  {
    TIM3->CCER &= ~TIM_CCER_CC4E;
    TIM3->CCR4 = 0U;
    TIM3->EGR = TIM_EGR_UG;
  }
}

void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 239;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if (tim_pwmHandle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
  else if (tim_pwmHandle->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (timHandle->Instance == TIM1)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /* PE9 = TIM1_CH1 (ESC2), PE13 = TIM1_CH3 (ESC1). */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM2)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA0 = TIM2_CH1 (ESC4), PA2 = TIM2_CH3 (ESC3). */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM3)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
  else if (tim_pwmHandle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
  else if (tim_pwmHandle->Instance == TIM3)
  {
    TIM3_HeaterEmergencyOff();
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
}
