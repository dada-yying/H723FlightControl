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

- Add board-level init code in `App_Main_Init()`.
- Create FreeRTOS tasks, queues, semaphores, timers, and application modules in `App_Main_Start()`.
- Keep `Core/Src/main.c` focused on HAL initialization and scheduler startup.

## Current Template Behavior

- Initializes HAL, system clock, GPIO clocks, and FreeRTOS.
- Starts the FreeRTOS scheduler.
- No user task is created by default, so only the FreeRTOS idle task runs until new tasks are added.

## Notes

- FDCAN, SPI, USART, WS2812, motor, LED, and PID demo code has been removed from the Keil build and application entry path.
- If you add a new peripheral in CubeMX or manually, enable the matching `HAL_xxx_MODULE_ENABLED` macro in `Core/Inc/stm32h7xx_hal_conf.h` and add the HAL source file to the Keil project.
