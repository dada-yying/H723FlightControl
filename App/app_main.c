#include "app_main.h"

#include "main.h"
#include "atom_imu.h"
#include "rc_sbus.h"
#include "ws2812.h"
#include "flight_control.h"

void App_Main_Init(void)
{
    WS2812_Init();
    /* Visible red before scheduler startup distinguishes SPI from RTOS faults. */
    AtomImu_Init();
    RcSbus_Init();
    FlightControl_Init();
}
