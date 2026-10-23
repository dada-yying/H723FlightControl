/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN3_Init(void);

/* USER CODE BEGIN Prototypes */
/* Consistent copy from separate task and ISR diagnostic owners. */
typedef struct
{
    uint32_t kernel_clock_hz;
    uint32_t nominal_bitrate;
    uint32_t init_ok;
    uint32_t start_ok;
    uint32_t hal_error;
    uint32_t task_poll_count;
    uint32_t rx_callback_count;
    uint32_t rx_frame_count;
    uint32_t rejected_frame_count;
    uint32_t last_id;
    uint32_t last_dlc;
    uint32_t rx_error_count;
    uint32_t tx_error_count;
    uint32_t last_protocol_error; /* Latched LEC 1..6; 0=no error seen. */
    uint32_t bus_off;
    uint32_t cccr;
    uint32_t nbtp;
    uint32_t irq_hal_error; /* Independent ISR error observation. */
} FDCAN3_Debug_t;

void FDCAN3_GetDiagnostics(FDCAN3_Debug_t *data);
HAL_StatusTypeDef FDCAN3_StartAtomImuReception(void);
void FDCAN3_UpdateDiagnostics(void); /* Call only from the existing task. */
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

