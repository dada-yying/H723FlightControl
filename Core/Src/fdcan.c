/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   FDCAN3 configuration for ATOM-IMU Classic CAN reception.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "atom_imu.h"
#include "flight_control.h"

#define FDCAN3_EXPECTED_KERNEL_HZ 24000000UL
#define FDCAN3_TARGET_BITRATE 1000000UL
static FDCAN3_Debug_t g_fdcan3_debug, s_fdcan_published;
static volatile FDCAN3_Debug_t s_fdcan_isr;
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan3;

void MX_FDCAN3_Init(void)
{
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;

  /*
   * FDCAN selects the existing 24 MHz HSE, independently of PLL1Q.
   * 24 MHz / 1 / (SyncSeg 1 + Seg1 19 + Seg2 4) = 1 Mbit/s.
   * Sample point = 20/24 = 83.33%; no shared CCU divider is configured.
   */
  hfdcan3.Init.NominalPrescaler = 1U;
  hfdcan3.Init.NominalSyncJumpWidth = 4U;
  hfdcan3.Init.NominalTimeSeg1 = 19U;
  hfdcan3.Init.NominalTimeSeg2 = 4U;
  hfdcan3.Init.DataPrescaler = 1U;
  hfdcan3.Init.DataSyncJumpWidth = 1U;
  hfdcan3.Init.DataTimeSeg1 = 1U;
  hfdcan3.Init.DataTimeSeg2 = 1U;

  hfdcan3.Init.MessageRAMOffset = 0U;
  hfdcan3.Init.StdFiltersNbr = 1U;
  hfdcan3.Init.ExtFiltersNbr = 0U;
  hfdcan3.Init.RxFifo0ElmtsNbr = 8U;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0U;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0U;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0U;
  hfdcan3.Init.TxBuffersNbr = 0U;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 0U;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    g_fdcan3_debug.hal_error = hfdcan3.ErrorCode;
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */
g_fdcan3_debug.kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
g_fdcan3_debug.nominal_bitrate =
    g_fdcan3_debug.kernel_clock_hz /
    (hfdcan3.Init.NominalPrescaler *
     (1U + hfdcan3.Init.NominalTimeSeg1 + hfdcan3.Init.NominalTimeSeg2));
if ((g_fdcan3_debug.kernel_clock_hz != FDCAN3_EXPECTED_KERNEL_HZ) ||
    (g_fdcan3_debug.nominal_bitrate != FDCAN3_TARGET_BITRATE))
{
    Error_Handler();
}
g_fdcan3_debug.init_ok = 1U;
  /* USER CODE END FDCAN3_Init 2 */
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (fdcanHandle->Instance == FDCAN3)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PD12 = FDCAN3_RX, PD13 = FDCAN3_TX. */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_FDCAN3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{
  if (fdcanHandle->Instance == FDCAN3)
  {
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13);
    __HAL_RCC_FDCAN_CLK_DISABLE();
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef FDCAN3_StartAtomImuReception(void)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    /*
   * Dual standard-ID filter. The HAL accepts unshifted 11-bit IDs here,
   * and puts 49/51 into the standard-filter message-RAM slots.
   */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = ATOM_IMU_GYRO_CAN_ID;
    filter.FilterID2 = ATOM_IMU_EULR_CAN_ID;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK)
    {
        g_fdcan3_debug.hal_error = hfdcan3.ErrorCode;
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        g_fdcan3_debug.hal_error = hfdcan3.ErrorCode;
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
    {
        g_fdcan3_debug.hal_error = hfdcan3.ErrorCode;
        return HAL_ERROR;
    }
    status = HAL_FDCAN_Start(&hfdcan3);
    g_fdcan3_debug.start_ok = (status == HAL_OK) ? 1U : 0U;
    g_fdcan3_debug.hal_error = hfdcan3.ErrorCode;
    g_fdcan3_debug.cccr = hfdcan3.Instance->CCCR;
    g_fdcan3_debug.nbtp = hfdcan3.Instance->NBTP;
    return status;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    /* HAL copies the DLC length before our frame validation. */
    uint8_t data[64];

    if ((hfdcan != &hfdcan3) || ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    s_fdcan_isr.rx_callback_count++;
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
        {
            s_fdcan_isr.hal_error = hfdcan->ErrorCode;
            break;
        }

        s_fdcan_isr.rx_frame_count++;
        s_fdcan_isr.last_id = rx_header.Identifier;
        s_fdcan_isr.last_dlc = rx_header.DataLength;
        if ((rx_header.IdType == FDCAN_STANDARD_ID) &&
            (rx_header.RxFrameType == FDCAN_DATA_FRAME) &&
            (rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_8) &&
            ((rx_header.Identifier == ATOM_IMU_GYRO_CAN_ID) ||
             (rx_header.Identifier == ATOM_IMU_EULR_CAN_ID)))
        {
            AtomImu_ProcessCanFrame(rx_header.Identifier, data);
            if (rx_header.Identifier == ATOM_IMU_GYRO_CAN_ID)
            {
                FlightControl_NotifyFromISR();
            }
        }
        else
        {
            s_fdcan_isr.rejected_frame_count++;
        }
    }
}

static void FDCAN3_UpdateDiagnosticsInternal(void)
{
    FDCAN_ProtocolStatusTypeDef protocol;
    FDCAN_ErrorCountersTypeDef errors;

    g_fdcan3_debug.task_poll_count++;
    if (g_fdcan3_debug.init_ok == 0U)
    {
        return;
    }
    g_fdcan3_debug.hal_error = hfdcan3.ErrorCode;
    g_fdcan3_debug.cccr = hfdcan3.Instance->CCCR;
    g_fdcan3_debug.nbtp = hfdcan3.Instance->NBTP;
    /* Reading PSR resets LEC to 7 (no change). Keep the last real error. */
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan3, &protocol) == HAL_OK)
    {
        g_fdcan3_debug.bus_off = protocol.BusOff;
        if ((protocol.LastErrorCode > 0U) && (protocol.LastErrorCode < 7U))
        {
            g_fdcan3_debug.last_protocol_error = protocol.LastErrorCode;
        }
    }
    if (HAL_FDCAN_GetErrorCounters(&hfdcan3, &errors) == HAL_OK)
    {
        g_fdcan3_debug.rx_error_count = errors.RxErrorCnt;
        g_fdcan3_debug.tx_error_count = errors.TxErrorCnt;
    }
}

void FDCAN3_UpdateDiagnostics(void)
{
    FDCAN3_UpdateDiagnosticsInternal();
    /* CAN-C1: task diagnostic publication, no HAL calls inside; 3 us, NOT measured. */
    taskENTER_CRITICAL();
    s_fdcan_published = g_fdcan3_debug;
    taskEXIT_CRITICAL();
}
void FDCAN3_GetDiagnostics(FDCAN3_Debug_t *data)
{
    if (!data) return;
    /* CAN-C2: task snapshot plus ISR single-word diagnostics; 3 us, NOT measured. */
    taskENTER_CRITICAL();
    *data = s_fdcan_published;
    data->rx_callback_count = s_fdcan_isr.rx_callback_count;
    data->rx_frame_count = s_fdcan_isr.rx_frame_count;
    data->rejected_frame_count = s_fdcan_isr.rejected_frame_count;
    data->last_id = s_fdcan_isr.last_id;
    data->last_dlc = s_fdcan_isr.last_dlc;
    data->irq_hal_error = s_fdcan_isr.hal_error;
    taskEXIT_CRITICAL();
}
/* USER CODE END 1 */
