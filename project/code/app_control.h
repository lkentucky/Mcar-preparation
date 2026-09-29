#ifndef MCAR_APP_CONTROL_H
#define MCAR_APP_CONTROL_H

#include <stdbool.h>

extern volatile bool motor_run_enabled;
extern volatile float motor_cmd_vx_cmps;
extern volatile float motor_cmd_vy_cmps;
extern volatile float motor_cmd_omega_radps;

void app_control_init(void);
void app_control_motor_tick_10ms(void);

#endif
