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
#include "atom_imu.h"
#include "vofa.h"
#include "fdcan.h"
#include "rc_sbus.h"
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t rcTaskHandle;
const osThreadAttr_t rcTask_attributes = {
  .name = "RcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartRcTask(void *argument);

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
  rcTaskHandle = osThreadNew(StartRcTask, NULL, &rcTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (defaultTaskHandle == NULL)
  {
    Error_Handler();
  }
  if (rcTaskHandle == NULL)
  {
    Error_Handler();
  }
  RcSbus_SetTaskHandle((TaskHandle_t)rcTaskHandle);
  App_Main_Start();
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
  AtomImuData_t imu_data;
  RcSbusData_t rc_data;
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t vofa_period_ticks = pdMS_TO_TICKS(10U);

  (void)argument;
  for (;;)
  {
    FDCAN3_UpdateDiagnostics();
    AtomImu_GetData(&imu_data);
    RcSbus_GetData(&rc_data);
    /* IMU axes stay native; any airframe coordinate conversion belongs above this layer. */
    (void)VOFA_SendFlightTelemetry(
        imu_data.roll * (180.0f / 3.14159265358979323846f),
        imu_data.pitch * (180.0f / 3.14159265358979323846f),
        imu_data.yaw * (180.0f / 3.14159265358979323846f),
        rc_data.throttle,
        rc_data.roll_target_rad * (180.0f / 3.14159265358979323846f),
        rc_data.pitch_target_rad * (180.0f / 3.14159265358979323846f),
        rc_data.yaw_rate_target_rad_s * (180.0f / 3.14159265358979323846f));
    vTaskDelayUntil(&last_wake_time, vofa_period_ticks);
  }
  /* USER CODE END StartDefaultTask */
}

void StartRcTask(void *argument)
{
  (void)argument;
  for (;;)
  {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
    RcSbus_ProcessPending();
    RcSbus_CheckTimeout();
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

