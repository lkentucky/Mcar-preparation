/* Exercise the actual menu renderer against a 240x320, 8x16 display boundary. */
#include "Mymenu.h"
#include "Motor.h"
#include "app_control.h"
#include "imu_attitude.h"
#include "imu_wifi_spi.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

volatile bool motor_run_enabled, motor_pwm_test_enabled;
volatile int16_t motor_test_pwm[4];
volatile float motor_cmd_vx_cmps, motor_cmd_vy_cmps, motor_cmd_omega_radps;
int16 up_L_all, up_R_all, down_L_all, down_R_all;
volatile int32 imu_attitude_status;
volatile float imu_roll_deg, imu_pitch_deg, imu_yaw_deg, imu_calibration_percent;
volatile float imu_accel_g[3], imu_gyro_dps[3];
volatile int imu_wifi_status, imu_wifi_last_error;
volatile uint32_t imu_wifi_tx_packets, imu_wifi_init_attempts;
static key_state_enum events[KEY_NUMBER];
static motor_speed_debug_snapshot_t snapshot;
static unsigned draw_calls, irq_disabled;
static char rows[20][31];

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
void imu_attitude_request_recalibration(void) {}
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
    Menu_Init();
    Menu_Show();
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
    /* Draw each root folder and its full-width values. */
    for (unsigned i = 0; i < 7; ++i)
    {
        press(KEY_1);
        press(KEY_3);
        press(KEY_4);
    }
    puts("menu display tests passed: bounds, full signed totals, 100ms refresh, root folders");
    return 0;
}
