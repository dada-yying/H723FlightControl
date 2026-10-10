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
#include "blackbox.h"
#include "optical_flow.h"
#include "usart.h"
#include "fdcan.h"
#include "rc_sbus.h"
#include "ws2812_ctrl.h"
#include "bmi088.h"
#include "bmi088_heater.h"
#include "tim.h"
#include "spi.h"
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
osThreadId_t opticalFlowTaskHandle;
osThreadId_t bmi088TaskHandle;
osThreadId_t blackboxTaskHandle;
const osThreadAttr_t blackboxTask_attributes = {
  .name = "BlackboxTask", .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal2,
};
const osThreadAttr_t bmi088Task_attributes = {
  .name = "Bmi088Task", .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal1,
};
const osThreadAttr_t opticalFlowTask_attributes = {
  .name = "OpticalFlowTask", .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};
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
void StartOpticalFlowTask(void *argument);
void StartRcTask(void *argument);
void StartFlightControlTask(void *argument);
void StartBmi088Task(void *argument);
void StartBlackboxTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* Optional sensor: failure does not prevent flight tasks from starting. */
  OpticalFlow_Init(MX_USART10_Init());
  Bmi088_Init(MX_SPI2_Init());
  Bmi088Heater_Init(MX_TIM3_Init());
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
  opticalFlowTaskHandle = osThreadNew(StartOpticalFlowTask, NULL, &opticalFlowTask_attributes);
  OpticalFlow_SetTaskHandle((TaskHandle_t)opticalFlowTaskHandle);
  if (opticalFlowTaskHandle == NULL) OpticalFlow_Init(HAL_ERROR);
  bmi088TaskHandle = osThreadNew(StartBmi088Task, NULL, &bmi088Task_attributes);
  if (bmi088TaskHandle == NULL) Bmi088_OnTaskCreationFailure();
  blackboxTaskHandle = osThreadNew(StartBlackboxTask, NULL, &blackboxTask_attributes);
  if (blackboxTaskHandle == NULL) Blackbox_OnTaskCreationFailure();

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
  EscPwmState_t esc_data;
  OpticalFlowData_t optical_data;
  uint8_t frame[VOFA_FLIGHT_FRAME_SIZE];
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t vofa_period_ticks = pdMS_TO_TICKS(10U);

  (void)argument;
  for (;;)
  {
    Bmi088Heater_CheckTimeout();
    FDCAN3_UpdateDiagnostics();
    RcSbus_GetData(&rc_data);
    FlightControl_GetState(&flight_data);
    EscPwm_GetState(&esc_data);
    OpticalFlow_GetData(&optical_data);
    VOFA_BuildFlightFrame(frame, &flight_data, &rc_data, &esc_data, &optical_data);
    (void)VOFA_SendFrame(frame);
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
void StartOpticalFlowTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    OpticalFlow_ProcessPending();
    g_optical_flow_stats.heap_free_bytes = xPortGetFreeHeapSize();
    g_optical_flow_stats.heap_min_free_bytes = xPortGetMinimumEverFreeHeapSize();
    g_optical_flow_stats.optical_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
    g_optical_flow_stats.default_stack_free_words = uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle);
    g_optical_flow_stats.flight_stack_free_words = uxTaskGetStackHighWaterMark((TaskHandle_t)flightControlTaskHandle);
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
  }
}
/* USER CODE BEGIN Application */
void StartBlackboxTask(void *argument)
{
  (void)argument;
  (void)Blackbox_StartRecording(xTaskGetCurrentTaskHandle());
  for (;;)
  {
    /* 700 Hz TIM4 notifications, independent of FlightControl/IMU updates.
     * Timeout wakes only maintain UART/schema; they never invent DATA slots. */
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
    Blackbox_ProcessPending();
    g_blackbox_tx_stats.stack_free_words = uxTaskGetStackHighWaterMark(NULL);
  }
}

void StartBmi088Task(void *argument)
{
  (void)argument;
  (void)Bmi088_StartSampling(xTaskGetCurrentTaskHandle());
  for (;;)
  {
    /* TIM7 posts 500 us events. Coalesce late events rather than replay reads;
     * timeout wakes only maintain freshness/heater safety, never fake samples. */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U)) != 0U) Bmi088_Process();
    else Bmi088_OnSamplingTimeout();
    Bmi088Heater_Process();
    g_bmi088_stats.stack_free_words = uxTaskGetStackHighWaterMark(NULL);
  }
}

/* USER CODE END Application */
