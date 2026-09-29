#include "app_control.h"

#include "Motor.h"
#include "PID.h"
#include "PID_config.h"

#include <stdlib.h>

volatile bool motor_run_enabled;
volatile float motor_cmd_vx_cmps;
volatile float motor_cmd_vy_cmps;
volatile float motor_cmd_omega_radps;

static float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

void app_control_init(void)
{
    motor_init();
    encoder_init();
    PID_Init(&ULpid, &ULPidInitStruct);
    PID_Init(&URpid, &URPidInitStruct);
    PID_Init(&DLpid, &DLPidInitStruct);
    PID_Init(&DRpid, &DRPidInitStruct);
    Kinematics_Init();
    motor_run_enabled = false;
}

void app_control_motor_tick_10ms(void)
{
    float body_command[3];

    encoder_get();
    if (motor_run_enabled)
    {
        body_command[0] = clampf(motor_cmd_vx_cmps, -300.0f, 300.0f);
        body_command[1] = clampf(motor_cmd_vy_cmps, -300.0f, 300.0f);
        body_command[2] = clampf(motor_cmd_omega_radps, -20.0f, 20.0f);
        Kinematics_Inverse(body_command, speed_encoder);
        motor_control(speed_encoder);
        return;
    }

    motor_control(car_stop_array);
    if (abs(up_L_all) < 5 && abs(up_R_all) < 5 &&
        abs(down_L_all) < 5 && abs(down_R_all) < 5)
    {
        PID_Clear(&ULpid);
        PID_Clear(&URpid);
        PID_Clear(&DLpid);
        PID_Clear(&DRpid);
        motor_pwm(0, 0, 0, 0);
    }
}
