#ifndef VOFA_H
#define VOFA_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef VOFA_SendAttitude(float roll_deg, float pitch_deg, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_H */
