#include "ws2812.h"

#include "ws2812.h"
#include "cmsis_os2.h"

#define WS2812_TASK_PERIOD_MS  20U

volatile uint32_t ws2812_frame_count = 0U;
volatile uint32_t ws2812_task_loop_count = 0U;
volatile uint32_t ws2812_task_started = 0U;

void WS2812_ColorTask(void *argument)
{
    uint8_t red = 255U;
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
        if (red <= 255U && red != 0U && green <= 255U && blue == 0U)
        {
            green += 5;
            red -= 5;
        }
        else if (green <= 255U && green != 0U && blue <= 255U && red == 0U)
        {
            green -= 5;
            blue += 5;
        }
        else if (blue <= 255U && blue != 0U && red <= 255U && green == 0U)
        {
            blue -= 5;
            red += 5;
        }
        osDelay(WS2812_TASK_PERIOD_MS);
    }
}
