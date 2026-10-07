/* Exercise the actual menu renderer against a 240x320, 8x16 display boundary. */
#include "Mymenu.h"
#include "Motor.h"
#include "app_control.h"
#include "app_navigation.h"
#include "imu.h"
#include "wifispi.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

volatile bool motor_run_enabled, motor_pwm_test_enabled;
volatile int16_t motor_test_pwm[4];
volatile float motor_cmd_vx_cmps, motor_cmd_vy_cmps, motor_cmd_omega_radps;
volatile bool motor_position_enabled;
volatile position_goal_t motor_position_goal;
volatile position_config_t motor_position_config = POSITION_CONFIG_DEFAULT;
static position_output_t position_snapshot;
int16 up_L_all, up_R_all, down_L_all, down_R_all;
volatile int32 imu_attitude_status;
volatile float imu_roll_deg, imu_pitch_deg, imu_yaw_deg, imu_calibration_percent;
volatile float imu_accel_g[3], imu_gyro_dps[3];
volatile int imu_wifi_status, imu_wifi_last_error;
volatile uint32_t imu_wifi_tx_packets, imu_wifi_init_attempts;
volatile uint32_t wifi_telemetry_channel_count, wifi_telemetry_period_ms;
volatile uint32_t wifi_telemetry_stream_enabled, wifi_telemetry_commands, wifi_telemetry_command_errors;
static key_state_enum events[KEY_NUMBER];
static motor_speed_debug_snapshot_t snapshot;
static unsigned draw_calls, irq_disabled;
void app_control_get_position_snapshot(position_output_t *out)
{ assert(irq_disabled); *out = position_snapshot; }
static char rows[20][31];
volatile float navigation_mount_deg = 180.0f;
volatile bool navigation_yaw_reversed;
volatile float navigation_scale_x = 1.0f, navigation_scale_y = 1.0f;
static unsigned navigation_resets;
static navigation_snapshot_t nav_snapshot;
void app_navigation_get_snapshot(navigation_snapshot_t *out)
{ assert(irq_disabled); *out = nav_snapshot; }
void app_navigation_request_reset(void) { ++navigation_resets; }

void ips200_set_dir(ips200_dir_enum dir) { assert(dir == IPS200_PORTAIT); }
void ips200_set_font(ips200_font_size_enum font) { assert(font == IPS200_8X16_FONT); }
void ips200_set_color(uint16 pen, uint16 background) { (void)pen; (void)background; }
void ips200_init(ips200_type_enum type) { assert(type == IPS200_TYPE_SPI); }
void ips200_clear(void) {}
void ips200_show_string(uint16 x, uint16 y, const char text[])
{
    assert(x + strlen(text) * 8 <= 240);
    assert(y + 16 <= 320);
    assert(strlen(text) <= 30);
    strcpy(rows[y / 16], text);
    ++draw_calls;
}
uint32 interrupt_global_disable(void) { assert(!irq_disabled); irq_disabled = 1; return 0; }
void interrupt_global_enable(uint32 primask) { assert(irq_disabled && primask == 0); irq_disabled = 0; }
void motor_speed_debug_get_snapshot(motor_speed_debug_snapshot_t *out)
{ assert(irq_disabled); *out = snapshot; }
void imu_request_recalibration(void) {}
int Limit_int(int low, int value, int high) { return value < low ? low : value > high ? high : value; }
void key_init(uint32 period) { assert(period == 20); }
key_state_enum key_get_state(key_index_enum key) { return events[key]; }
void key_clear_all_state(void) { memset(events, 0, sizeof(events)); }

static void press(key_index_enum key)
{
    events[key] = KEY_SHORT_PRESS;
    Menu_Switch();
    Menu_Show();
}

int main(void)
{
    for (unsigned wheel = 0; wheel < 4; ++wheel)
    {
        snapshot.raw_counts[wheel] = wheel % 2 ? INT16_MIN : INT16_MAX;
        snapshot.filtered_counts[wheel] = snapshot.raw_counts[wheel];
        snapshot.cumulative_raw_counts[wheel] = wheel % 2 ? INT32_MIN : INT32_MAX;
        snapshot.final_pwm[wheel] = wheel % 2 ? LIMIT_PWM_MIN : LIMIT_PWM_MAX;
    }
    motor_position_enabled = true;
    Menu_Init();
    Menu_Show();
    assert(strstr(rows[0], "Position") != NULL && strstr(rows[1], "On") != NULL);
    assert(!motor_run_enabled && !motor_pwm_test_enabled);
    press(KEY_3); /* Root/Position */
    press(KEY_4); /* PWM_Test */
    press(KEY_4); /* Drive */
    press(KEY_4); /* Encoder */
    press(KEY_1);
    assert(strstr(rows[0], "Encoder") != NULL);
    assert(strstr(rows[1], "Total") != NULL);
    assert(strstr(rows[2], "2147483647") != NULL);
    assert(strstr(rows[3], "-2147483648") != NULL);
    unsigned previous = draw_calls;
    for (unsigned i = 0; i < 4; ++i) { Menu_Tick_20ms(); Menu_Show(); }
    assert(draw_calls == previous);
    Menu_Tick_20ms(); Menu_Show();
    assert(draw_calls > previous);
    press(KEY_3); /* Root */
    press(KEY_4); /* Navigation */
    nav_snapshot.status = NAV_RUNNING;
    nav_snapshot.valid = nav_snapshot.bias_ready = true;
    nav_snapshot.x_m = -1.23f; nav_snapshot.y_m = 4.56f;
    nav_snapshot.vx_mps = 0.12f; nav_snapshot.vy_mps = -0.34f;
    press(KEY_1);
    assert(strstr(rows[0], "Navigation") != NULL);
    assert(strstr(rows[2], "-123.000") != NULL);
    assert(strstr(rows[3], "456.000") != NULL);
    assert(strstr(rows[8], "12.00") != NULL && strstr(rows[8], "-34.00") != NULL);
    assert(strstr(rows[9], "Valid:1") != NULL);
    for (unsigned i = 0; i < 6; ++i) press(KEY_4);
    press(KEY_1); press(KEY_2);
    assert(navigation_resets == 1);
    press(KEY_3); press(KEY_4); /* deselect Zero, select ScaleX */
    assert(strstr(rows[7], "ScaleX") != NULL);
    press(KEY_1); press(KEY_4);
    assert(navigation_scale_x == 0.1f); /* positive calibration bound */
    press(KEY_3); press(KEY_4); /* ScaleY */
    assert(strstr(rows[7], "ScaleY") != NULL);
    press(KEY_1); press(KEY_2);
    assert(navigation_scale_y == 2.0f && navigation_scale_x == 0.1f);
    press(KEY_3); press(KEY_3); /* deselect then Root */
    /* Draw each root folder and its full-width values. */
    for (unsigned i = 0; i < 9; ++i)
    {
        if (i == 4) assert(strstr(rows[7], "PID") != NULL); /* eighth folder scrolls into view */
        press(KEY_1);
        press(KEY_3);
        press(KEY_4);
    }
    /* Root pointer is back at Navigation; Position is five folders ahead. */
    for (unsigned i = 0; i < 5; ++i) press(KEY_4);
    assert(strstr(rows[7], "Position") != NULL);
    press(KEY_1);
    assert(strstr(rows[0], "Position") != NULL);
    motor_pwm_test_enabled = motor_run_enabled = true;
    press(KEY_1); press(KEY_2); /* Enable position */
    assert(motor_position_enabled && !motor_pwm_test_enabled && !motor_run_enabled);
    press(KEY_3);
    press(KEY_4); press(KEY_4); /* TargetX */
    press(KEY_1); press(KEY_4);
    assert(motor_position_goal.x_cm == -1.0f);
    press(KEY_3); press(KEY_4); /* TargetY */
    press(KEY_1); press(KEY_4);
    assert(motor_position_goal.y_cm == -1.0f);
    press(KEY_3); press(KEY_4); /* TargetYaw */
    press(KEY_1); press(KEY_4);
    assert(motor_position_goal.yaw_deg == -1.0f);
    press(KEY_3);
    for (unsigned i = 0; i < 12; ++i) press(KEY_4); /* through every parameter, to ErrYaw */
    assert(strstr(rows[7], "ErrYaw_deg") != NULL);
    press(KEY_1); press(KEY_2); /* Read-only values don't select or modify */
    assert(position_snapshot.yaw_error_deg == 0.0f);
    /* Return from Position to PWM_Test, selecting direct PWM cancels position/Run. */
    press(KEY_3); press(KEY_4); press(KEY_1);
    motor_run_enabled = true;
    press(KEY_1); press(KEY_2);
    assert(motor_pwm_test_enabled && !motor_position_enabled && !motor_run_enabled);
    puts("menu display tests passed: bounds, full signed totals, 100ms refresh, root folders");
    return 0;
}
