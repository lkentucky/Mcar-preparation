#ifndef MCAR_APP_CONTROL_H
#define MCAR_APP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

extern volatile bool motor_run_enabled;
/* Default mode: direct signed PWM in UL, UR, DL, DR order. */
extern volatile bool motor_pwm_test_enabled;
extern volatile int16_t motor_test_pwm[4];
extern volatile float motor_cmd_vx_cmps;
extern volatile float motor_cmd_vy_cmps;
extern volatile float motor_cmd_omega_radps;

void app_control_init(void);
void app_control_motor_tick_10ms(void);

#endif
