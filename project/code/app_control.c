#include "app_control.h"

#include "Motor.h"
#include "PID.h"
#include "PID_config.h"

#include <math.h>

volatile bool motor_run_enabled;
volatile bool motor_pwm_test_enabled;
volatile int16_t motor_test_pwm[MOTOR_WHEEL_COUNT];
volatile float motor_cmd_vx_cmps;
volatile float motor_cmd_vy_cmps;
volatile float motor_cmd_omega_radps;

static bool g_previous_pwm_test_enabled;

static void clear_wheel_pid(void)
{
    PID_Clear(&ULpid);
    PID_Clear(&URpid);
    PID_Clear(&DLpid);
    PID_Clear(&DRpid);
}

//限制车体速度
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
    motor_pwm_test_enabled = true;
    g_previous_pwm_test_enabled = true;
    for (unsigned wheel = 0; wheel < MOTOR_WHEEL_COUNT; ++wheel)
        motor_test_pwm[wheel] = 0;
}

void app_control_motor_tick_10ms(void)
{
    float body_command[3];
    bool pwm_test_enabled = motor_pwm_test_enabled;

    encoder_get();
    /* Changing modes stops output and requires Run to be enabled again. */
    if (pwm_test_enabled != g_previous_pwm_test_enabled)
    {
        g_previous_pwm_test_enabled = pwm_test_enabled;
        motor_run_enabled = false;
        clear_wheel_pid();
        motor_pwm(0, 0, 0, 0);
        return;
    }
    if (pwm_test_enabled)
    {
        if (motor_run_enabled)
            motor_pwm(motor_test_pwm[MOTOR_WHEEL_UL], motor_test_pwm[MOTOR_WHEEL_UR],
                      motor_test_pwm[MOTOR_WHEEL_DL], motor_test_pwm[MOTOR_WHEEL_DR]);
        else
            motor_pwm(0, 0, 0, 0);
        return;
    }
    //速度闭环模式
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
    if (fabsf(motor_reference_counts(MOTOR_WHEEL_UL, up_L_all)) < 5.0f &&
        fabsf(motor_reference_counts(MOTOR_WHEEL_UR, up_R_all)) < 5.0f &&
        fabsf(motor_reference_counts(MOTOR_WHEEL_DL, down_L_all)) < 5.0f &&
        fabsf(motor_reference_counts(MOTOR_WHEEL_DR, down_R_all)) < 5.0f)
    {
        clear_wheel_pid();
        motor_pwm(0, 0, 0, 0);
    }
}
