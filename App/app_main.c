#include "app_main.h"

#include "cmsis_os2.h"
#include "main.h"
#include "ws2812.h"

#define WS2812_TASK_PERIOD_MS  20U

osThreadId_t ws2812TaskHandle;
volatile uint32_t ws2812_frame_count = 0U;
volatile uint32_t ws2812_task_loop_count = 0U;
volatile uint32_t ws2812_boot_status = 0xFFFFFFFFU;
volatile uint32_t ws2812_task_started = 0U;

static void WS2812_ColorTask(void *argument)
{
    uint8_t red = 64U;
    uint8_t green = 0U;
    uint8_t blue = 0U;

    (void)argument;
    ws2812_task_started = 1U;

    for (;;)
    {
        ws2812_task_loop_count++;
        if (WS2812_Ctrl(red, green, blue) == HAL_OK)
        {
            ws2812_frame_count++;
        }
        red += 1U;
        green += 5U;
        blue += 10U;
        osDelay(WS2812_TASK_PERIOD_MS);
    }
}

void App_Main_Init(void)
{
    WS2812_Init();
    /* Visible red before scheduler startup distinguishes SPI from RTOS faults. */
    ws2812_boot_status = (uint32_t)WS2812_Ctrl(64U, 0U, 0U);
}

void App_Main_Start(void)
{
    static const osThreadAttr_t ws2812_task_attributes = {
        .name = "ws2812",
        .stack_size = 256U * 4U,
        .priority = (osPriority_t)osPriorityLow,
    };

    ws2812TaskHandle = osThreadNew(WS2812_ColorTask, NULL, &ws2812_task_attributes);
    if (ws2812TaskHandle == NULL)
    {
        Error_Handler();
    }
}
