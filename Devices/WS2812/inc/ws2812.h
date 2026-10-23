#ifndef WS2812_H
#define WS2812_H

#include "main.h"

void WS2812_ColorTask(void *argument);
void WS2812_Init(void);
HAL_StatusTypeDef WS2812_Ctrl(uint8_t red, uint8_t green, uint8_t blue);





typedef struct
{
    uint32_t tx_status, spi_error, error_count, spi_sr;
    uint32_t boot_status, frame_count, task_loop_count, task_started;
} WS2812Stats_t;
void WS2812_GetStats(WS2812Stats_t *stats);
#endif /* WS2812_H */
