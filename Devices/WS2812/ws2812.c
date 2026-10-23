#include "ws2812.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

/* Encoding and reset sequence from the supplied working DM-MC02 example. */
#define WS2812_SPI_ZERO 0x60U
#define WS2812_SPI_ONE 0x78U
#define WS2812_TIMEOUT_MS 5U

static volatile uint32_t ws2812_tx_status;
static volatile uint32_t ws2812_spi_error;
static volatile uint32_t ws2812_error_count;
static volatile uint32_t ws2812_spi_sr;
static volatile uint32_t ws2812_boot_status = 0xFFFFFFFFU;

void WS2812_Init(void)
{
    MX_SPI6_Init();
    ws2812_boot_status = (uint32_t)WS2812_Ctrl(64U, 0U, 0U);
}

HAL_StatusTypeDef WS2812_Ctrl(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t frame[24];
    uint8_t reset = 0U;
    uint32_t i;
    HAL_StatusTypeDef status;
    BaseType_t running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);

    for (i = 0U; i < 8U; i++)
    {
        frame[7U - i] = ((green >> i) & 1U) ? WS2812_SPI_ONE : WS2812_SPI_ZERO;
        frame[15U - i] = ((red >> i) & 1U) ? WS2812_SPI_ONE : WS2812_SPI_ZERO;
        frame[23U - i] = ((blue >> i) & 1U) ? WS2812_SPI_ONE : WS2812_SPI_ZERO;
    }

    /* Prevent task switches during the frame, but leave SysTick enabled so
       HAL timeouts still work. SPI6 is owned exclusively by this driver. */
    if (running)
    {
        vTaskSuspendAll();
    }
    status = HAL_SPI_Transmit(&hspi6, frame, sizeof(frame), WS2812_TIMEOUT_MS);
    /* Keep the reference's 100 single-byte zero transfers (including the
       HAL enable/disable sequence). Its zero-length call is intentionally
       omitted: HAL rejects Size == 0 and that call sends nothing. */
    for (i = 0U; (i < 100U) && (status == HAL_OK); i++)
    {
        status = HAL_SPI_Transmit(&hspi6, &reset, 1U, WS2812_TIMEOUT_MS);
    }
    ws2812_tx_status = (uint32_t)status;
    ws2812_spi_error = hspi6.ErrorCode;
    ws2812_spi_sr = SPI6->SR;
    if (status != HAL_OK)
    {
        ws2812_error_count++;
    }
    if (running)
    {
        (void)xTaskResumeAll();
    }
    return status;
}

#include "cmsis_os2.h"
#define WS2812_TASK_PERIOD_MS 20U

static volatile uint32_t ws2812_frame_count = 0U;
static volatile uint32_t ws2812_task_loop_count = 0U;
static volatile uint32_t ws2812_task_started = 0U;

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
        if (red != 0U && blue == 0U)
        {
            green += 5;
            red -= 5;
        }
        else if (green != 0U && red == 0U)
        {
            green -= 5;
            blue += 5;
        }
        else if (blue != 0U && green == 0U)
        {
            blue -= 5;
            red += 5;
        }
        osDelay(WS2812_TASK_PERIOD_MS);
    }
}

void WS2812_GetStats(WS2812Stats_t *stats)
{
    if (!stats) return;
    /* WS-C1: single-word diagnostic snapshot; 2 us, NOT measured. Normal SPI
     * result publication already occurs with scheduling suspended. Fault-context
     * writes are terminal diagnostics and do not call this task-only getter. */
    taskENTER_CRITICAL();
    stats->tx_status = ws2812_tx_status;
    stats->spi_error = ws2812_spi_error;
    stats->error_count = ws2812_error_count;
    stats->spi_sr = ws2812_spi_sr;
    stats->boot_status = ws2812_boot_status;
    stats->frame_count = ws2812_frame_count;
    stats->task_loop_count = ws2812_task_loop_count;
    stats->task_started = ws2812_task_started;
    taskEXIT_CRITICAL();
}
