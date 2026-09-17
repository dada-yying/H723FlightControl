# STM32H723 FreeRTOS Template

This project is a clean STM32H723 + HAL + FreeRTOS starter template.

## Structure

- `Core/Inc`, `Core/Src`: STM32 startup application entry, clock, GPIO, interrupt handlers, HAL config, and FreeRTOS config.
- `App`: user application composition layer.
- `Drivers/CMSIS`: CMSIS device headers and core headers.
- `Drivers/STM32H7xx_HAL_Driver`: STM32H7 HAL driver package.
- `Middlewares/Third_Party/FreeRTOS`: FreeRTOS kernel source.
- `MDK-ARM`: Keil uVision project and startup file.

## Application Entry

- `main()` calls `App_Main_Init()` before scheduler startup to initialize SPI6 and send a red LED frame.
- `MX_FREERTOS_Init()` calls `App_Main_Start()` to create the WS2812 task.
- Keep `Core/Src/main.c` focused on HAL initialization and scheduler startup.

## Current Template Behavior

- Initializes HAL, system clock, GPIO clocks, and FreeRTOS.
- Starts the FreeRTOS scheduler.
- Runs the default task and a WS2812 color task with a 20 ms delay between frames.
- SysTick advances both the HAL timebase and, after scheduler startup, the FreeRTOS tick.

## Notes

- SPI6 and the matching SPI HAL files from the supplied WS2812 example are enabled.
- FDCAN, USART, motor, and PID demos remain outside the application startup path.
- If you add a new peripheral in CubeMX or manually, enable the matching `HAL_xxx_MODULE_ENABLED` macro in `Core/Inc/stm32h7xx_hal_conf.h` and add the HAL source file to the Keil project.
- See `WS2812_CHECK.md` for the checked wiring, diagnostics, rebuild command and regeneration caveats.
