#ifndef MCAR_APP_CONTROL_H
#define MCAR_APP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "position_control.h"

extern volatile bool motor_run_enabled;
/* Default is position control with Run off. PWM test must be selected explicitly. */
extern volatile bool motor_pwm_test_enabled;
extern volatile int16_t motor_test_pwm[4];
extern volatile float motor_cmd_vx_cmps;
extern volatile float motor_cmd_vy_cmps;
extern volatile float motor_cmd_omega_radps;
/* Position and direct PWM are exclusive. A mode change cancels Run.
 * Goals are in Navigation/Zero coordinates, cm and degrees. */
extern volatile bool motor_position_enabled;
extern volatile position_goal_t motor_position_goal;
extern volatile position_config_t motor_position_config;
/* Main-loop callers must take this snapshot with interrupts disabled. */
void app_control_get_position_snapshot(position_output_t *out);

void app_control_init(void);
void app_control_motor_tick_10ms(void);

#endif
