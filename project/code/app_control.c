#include "app_control.h"

#include "Motor.h"
#include "PID.h"
#include "PID_config.h"
#include "app_navigation.h"

#include <math.h>

volatile bool motor_run_enabled;
volatile bool motor_pwm_test_enabled;
volatile int16_t motor_test_pwm[MOTOR_WHEEL_COUNT];
volatile float motor_cmd_vx_cmps;
volatile float motor_cmd_vy_cmps;
volatile float motor_cmd_omega_radps;
volatile bool motor_position_enabled;
volatile position_goal_t motor_position_goal;
volatile position_config_t motor_position_config = POSITION_CONFIG_DEFAULT;

static bool g_previous_pwm_test_enabled;
static bool g_previous_position_enabled;
static bool g_position_was_running;
static position_control_t g_position;

void app_control_get_position_snapshot(position_output_t *out)
{
    *out = g_position.output;
}

static void clear_wheel_pid(void)
{
    PID_Clear(&ULpid);
    PID_Clear(&URpid);
    PID_Clear(&DLpid);
    PID_Clear(&DRpid);
}

static void clear_position_command(void)
{
    motor_cmd_vx_cmps = motor_cmd_vy_cmps = motor_cmd_omega_radps = 0.0f;
}

static void stop_position(void)
{
    motor_run_enabled = false;
    g_position_was_running = false;
    clear_position_command();
    clear_wheel_pid();
    for (unsigned wheel = 0; wheel < MOTOR_WHEEL_COUNT; ++wheel)
        speed_encoder[wheel] = 0;
    motor_pwm(0, 0, 0, 0);
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
    app_navigation_init();
    motor_run_enabled = false;
    motor_pwm_test_enabled = g_previous_pwm_test_enabled = false;
    motor_position_enabled = g_previous_position_enabled = true;
    g_position_was_running = false;
    motor_position_goal = (position_goal_t){0.0f, 0.0f, 0.0f};
    motor_position_config = (position_config_t)POSITION_CONFIG_DEFAULT;
    position_control_reset(&g_position);
    clear_position_command();
    for (unsigned wheel = 0; wheel < MOTOR_WHEEL_COUNT; ++wheel)
        motor_test_pwm[wheel] = 0;
}

void app_control_motor_tick_10ms(void)
{
    float body_command[3];
    bool pwm_test_enabled = motor_pwm_test_enabled;
    bool position_enabled = motor_position_enabled;

    encoder_get();
    app_navigation_encoder_tick_10ms();
    /* Changing modes stops output and requires Run to be enabled again. */
    if (pwm_test_enabled != g_previous_pwm_test_enabled ||
        position_enabled != g_previous_position_enabled)
    {
        bool was_position = g_previous_position_enabled;
        g_previous_pwm_test_enabled = pwm_test_enabled;
        g_previous_position_enabled = position_enabled;
        motor_run_enabled = false;
        g_position_was_running = false;
        position_control_reset(&g_position);
        if (was_position || position_enabled) clear_position_command();
        clear_wheel_pid();
        for (unsigned wheel = 0; wheel < MOTOR_WHEEL_COUNT; ++wheel)
            speed_encoder[wheel] = 0;
        motor_pwm(0, 0, 0, 0);
        return;
    }
    if (position_enabled && pwm_test_enabled)
    {
        /* Reject conflicting requests from code outside the menu as well. */
        position_control_reset(&g_position);
        g_position.output.status = POSITION_BAD_CONFIG;
        stop_position();
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
    if (position_enabled)
    {
        if (!motor_run_enabled)
        {
            position_goal_t goal = motor_position_goal;
            if (g_position_was_running || (g_position.have_previous &&
                (goal.x_cm != g_position.previous_goal.x_cm ||
                 goal.y_cm != g_position.previous_goal.y_cm ||
                 goal.yaw_deg != g_position.previous_goal.yaw_deg)))
                position_control_reset(&g_position);
            stop_position();
            return;
        }
        if (!g_position_was_running) position_control_reset(&g_position);
        g_position_was_running = true;
        {
            navigation_snapshot_t pose;
            position_goal_t goal = motor_position_goal;
            position_config_t config = motor_position_config;
            app_navigation_get_snapshot(&pose);
            position_control_update(&g_position, &config, &goal, &pose, 0.01f);
        }
        if (g_position.output.status < 0 || g_position.output.status == POSITION_REACHED)
        {
            stop_position();
            return;
        }
        motor_cmd_vx_cmps = body_command[0] = g_position.output.vx_cmps;
        motor_cmd_vy_cmps = body_command[1] = g_position.output.vy_cmps;
        motor_cmd_omega_radps = body_command[2] = g_position.output.omega_radps;
        Kinematics_Inverse(body_command, speed_encoder);
        motor_control(speed_encoder);
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
