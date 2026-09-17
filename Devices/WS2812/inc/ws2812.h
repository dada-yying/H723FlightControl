#ifndef WS2812_H
#define WS2812_H

#include "main.h"

void WS2812_Init(void);
HAL_StatusTypeDef WS2812_Ctrl(uint8_t red, uint8_t green, uint8_t blue);
extern volatile uint32_t ws2812_tx_status;
extern volatile uint32_t ws2812_spi_error;
extern volatile uint32_t ws2812_error_count;
extern volatile uint32_t ws2812_spi_sr;

#endif /* WS2812_H */
