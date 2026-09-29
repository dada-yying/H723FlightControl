/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : CubeMX-managed FreeRTOS application entry
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_main.h"
#include "esc_pwm.h"
#include "flight_control.h"
#include "vofa.h"
#include "fdcan.h"
#include "rc_sbus.h"
#include "ws2812_ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t rcTaskHandle;
const osThreadAttr_t rcTask_attributes = {
  .name = "RcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t flightControlTaskHandle;
const osThreadAttr_t flightControlTask_attributes = {
  .name = "FlightControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t ws2812TaskHandle;
const osThreadAttr_t ws2812_task_attributes = {
  .name = "ws2812",
  .stack_size = 256U * 4,
  .priority = (osPriority_t)osPriorityLow,
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartRcTask(void *argument);
void StartFlightControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  ws2812TaskHandle = osThreadNew(WS2812_ColorTask, NULL, &ws2812_task_attributes);
  rcTaskHandle = osThreadNew(StartRcTask, NULL, &rcTask_attributes);
  flightControlTaskHandle = osThreadNew(StartFlightControlTask, NULL, &flightControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (defaultTaskHandle == NULL)
  {
    Error_Handler();
  }
  if (ws2812TaskHandle == NULL)
  {
    Error_Handler();
  }

  if (rcTaskHandle == NULL)
  {
    Error_Handler();
  }
  if (flightControlTaskHandle == NULL)
  {
    Error_Handler();
  }
  RcSbus_SetTaskHandle((TaskHandle_t)rcTaskHandle);
  FlightControl_SetTaskHandle((TaskHandle_t)flightControlTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  RcSbusData_t rc_data;
  FlightControlState_t flight_data;
  float rates_deg_s[6];
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t vofa_period_ticks = pdMS_TO_TICKS(10U);

  (void)argument;
  for (;;)
  {
    FDCAN3_UpdateDiagnostics();
    RcSbus_GetData(&rc_data);
    FlightControl_GetState(&flight_data);
    rates_deg_s[0] = flight_data.roll_rate_target_rad_s * (180.0f / 3.14159265358979323846f);
    rates_deg_s[1] = flight_data.roll_rate_rad_s * (180.0f / 3.14159265358979323846f);
    rates_deg_s[2] = flight_data.pitch_rate_target_rad_s * (180.0f / 3.14159265358979323846f);
    rates_deg_s[3] = flight_data.pitch_rate_rad_s * (180.0f / 3.14159265358979323846f);
    rates_deg_s[4] = flight_data.yaw_rate_target_rad_s * (180.0f / 3.14159265358979323846f);
    rates_deg_s[5] = flight_data.yaw_rate_rad_s * (180.0f / 3.14159265358979323846f);
    /* Body attitude and rates come from the same controller snapshot. */
    (void)VOFA_SendFlightTelemetry(
        flight_data.roll_rad * (180.0f / 3.14159265358979323846f),
        flight_data.pitch_rad * (180.0f / 3.14159265358979323846f),
        flight_data.yaw_rad * (180.0f / 3.14159265358979323846f),
        rc_data.throttle,
        rc_data.roll_target_rad * (180.0f / 3.14159265358979323846f),
        rc_data.pitch_target_rad * (180.0f / 3.14159265358979323846f),
        rc_data.yaw_rate_target_rad_s * (180.0f / 3.14159265358979323846f), rates_deg_s,
        flight_data.roll_rate_raw_rad_s * (180.0f / 3.14159265358979323846f));
    vTaskDelayUntil(&last_wake_time, vofa_period_ticks);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRcTask */
/**
  * @brief  Function implementing the RcTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRcTask */
void StartRcTask(void *argument)
{
  /* USER CODE BEGIN StartRcTask */
  RcSbusData_t rc_data;

  (void)argument;
  for (;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
    RcSbus_ProcessPending();
    RcSbus_CheckTimeout();
    RcSbus_GetData(&rc_data);
    EscPwm_UpdateFromRc(&rc_data);
  }
  /* USER CODE END StartRcTask */
}

/* USER CODE BEGIN Header_StartFlightControlTask */
/**
  * @brief  Function implementing the FlightControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFlightControlTask */
void StartFlightControlTask(void *argument)
{
  uint32_t pending_gyro_frames = 0U;
  uint32_t received;

  (void)argument;
  for (;;)
  {
    received = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5U));
    if (received == 0U)
    {
      FlightControl_OnTaskTimeout();
      continue;
    }
    pending_gyro_frames += received;
    if (pending_gyro_frames >= 2U)
    {
      pending_gyro_frames = 0U;
      FlightControl_Run();
    }
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
