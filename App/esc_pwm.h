#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <stdint.h>

#include "rc_sbus.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESC_PWM_MOTOR_COUNT 4U
#define ESC_PWM_MIN_PULSE_US 1000U
#define ESC_PWM_MAX_PULSE_US 2000U

typedef enum
{
    ESC_PWM_MOTOR_1 = 0,
    ESC_PWM_MOTOR_2,
    ESC_PWM_MOTOR_3,
    ESC_PWM_MOTOR_4
} EscPwmMotor_t;

typedef enum
{
    ESC_PWM_INHIBIT_NONE = 0,
    ESC_PWM_INHIBIT_STARTUP,
    ESC_PWM_INHIBIT_SWITCH_LOCKED,
    ESC_PWM_INHIBIT_THROTTLE_HIGH,
    ESC_PWM_INHIBIT_RC_INVALID,
    ESC_PWM_INHIBIT_FAILSAFE,
    ESC_PWM_INHIBIT_RC_TIMEOUT,
    ESC_PWM_INHIBIT_NOT_INITIALIZED,
    ESC_PWM_INHIBIT_INIT_ERROR
} EscPwmInhibitReason_t;

typedef struct
{
    float command[ESC_PWM_MOTOR_COUNT];
    uint16_t pulse_us[ESC_PWM_MOTOR_COUNT];
    uint32_t update_count;
    uint32_t startup_tick;
    uint16_t arm_channel;
    uint8_t initialized;
    uint8_t armed;
    uint8_t arm_middle_seen;
    uint8_t inhibit_reason;
} EscPwmState_t;

extern volatile EscPwmState_t g_esc_pwm_state;

HAL_StatusTypeDef EscPwm_Init(void);
void EscPwm_SetMotorNormalized(EscPwmMotor_t motor, float command);
void EscPwm_SetAllNormalized(float command);
void EscPwm_Disarm(void);
void EscPwm_UpdateFromRc(const RcSbusData_t *rc_data);
void EscPwm_GetState(EscPwmState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ESC_PWM_H */
