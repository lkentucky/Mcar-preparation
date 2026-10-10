#include "app_navigation.h"
#include "navigation_config.h"
#include "imu.h"
#include "Motor.h"
#include "app_control.h"
#include "route_follow.h"
#include "odometry_calibration.h"
#include "zf_common_interrupt.h"
#include <math.h>

#include <string.h>

volatile float navigation_mount_deg;
volatile bool navigation_yaw_reversed;
volatile float navigation_scale_x, navigation_scale_y;
static navigation_fusion_t g_navigation;
static volatile bool g_reset_requested;
static uint32_t g_imu_sequence, g_imu_generation;
static float g_imu_age_s;
static bool g_have_imu;

static void reset_state(void)
{
    navigation_config_t config = {0};
    for (unsigned i = 0; i < 4; ++i)
        config.counts_per_revolution[i] = motor_encoder_counts_per_revolution[i];
    config.wheel_diameter_m = WHEEL_DIAMETER;
    config.forward_scale = navigation_scale_x;
    config.lateral_scale = LATERAL_CORRECTION_FACTOR * navigation_scale_y;
    config.lateral_to_forward = LATERAL_TO_LONGITUDINAL_COUPLING_FACTOR;
    config.imu_mount_yaw_deg = navigation_mount_deg;
    config.imu_yaw_reversed = navigation_yaw_reversed;
    navigation_fusion_init(&g_navigation, &config);
    g_imu_sequence = g_imu_generation = 0;
    g_imu_age_s = 0.0f;
    g_have_imu = false;
}

void app_navigation_init(void)
{
    navigation_mount_deg = 180.0f; /* 实测：IMU +X 朝车尾，+Z 朝上。 */
    navigation_yaw_reversed = false;
    navigation_scale_x = NAV_FORWARD_SCALE_DEFAULT;
    navigation_scale_y = NAV_LEFT_SCALE_DEFAULT;
    g_reset_requested = false;
    reset_state();
}

void app_navigation_request_reset(void)
{
    g_reset_requested = true;
}

int32_t app_navigation_calibrate(unsigned axis, float measured_cm)
{
    float observed, cross, updated, yaw, speed;
    int32_t result=1;
    uint32 primask=interrupt_global_disable();
    if (axis>1 || !isfinite(measured_cm)) result=-1;
    else if (motor_run_enabled || route_run_flag) result=-2;
    else if (g_reset_requested || !g_navigation.output.valid || !g_navigation.output.bias_ready ||
             g_navigation.output.status!=NAV_RUNNING) result=-3;
    else {
        observed=(axis==0?g_navigation.output.x_m:g_navigation.output.y_m)*100;
        cross=(axis==0?g_navigation.output.y_m:g_navigation.output.x_m)*100;
        yaw=remainderf(g_navigation.output.yaw_deg,360);
        speed=hypotf(g_navigation.output.vx_mps,g_navigation.output.vy_mps);
        if (!isfinite(yaw) || fabsf(yaw)>3 || !isfinite(cross) ||
            fabsf(cross)>fmaxf(2,.1f*fabsf(observed)) ||
            !isfinite(speed) || speed>.03f) result=-4;
        else if (!odometry_calibration_scale(axis==0?navigation_scale_x:navigation_scale_y,
                                             observed,measured_cm,&updated)) result=-1;
        else {
            if (axis==0) navigation_scale_x=updated; else navigation_scale_y=updated;
            g_reset_requested=true;
        }
    }
    interrupt_global_enable(primask);
    return result;
}

void app_navigation_get_snapshot(navigation_snapshot_t *snapshot)
{
    if (snapshot != NULL) *snapshot = g_navigation.output;
}

void app_navigation_imu_tick_5ms(void)
{
    imu_navigation_sample_t sample;
    navigation_imu_t input;
    if (!imu_get_navigation_sample(&sample)) return;
    if (g_have_imu && sample.generation != g_imu_generation) {
        /* AHRS 重标定会重置 yaw；旧位置失去航向连续性，必须停车 Zero。 */
        navigation_fusion_invalidate(&g_navigation);
        return;
    }
    if (g_have_imu && sample.sequence == g_imu_sequence) return;
    g_have_imu = true;
    g_imu_sequence = sample.sequence;
    g_imu_generation = sample.generation;
    g_imu_age_s = 0.0f;
    input.roll_deg = sample.roll_deg;
    input.pitch_deg = sample.pitch_deg;
    input.yaw_deg = sample.yaw_deg;
    input.dt_s = sample.dt_s;
    memcpy(input.accel_g, sample.accel_g, sizeof(input.accel_g));
    memcpy(input.gyro_dps, sample.gyro_dps, sizeof(input.gyro_dps));
    navigation_fusion_imu(&g_navigation, &input);
}

void app_navigation_encoder_tick_10ms(void)
{
    const int16_t counts[4] = {
        encoder_data_quaddec1, encoder_data_quaddec2,
        encoder_data_quaddec3, encoder_data_quaddec4
    };
    if (g_reset_requested || navigation_mount_deg != g_navigation.config.imu_mount_yaw_deg ||
        navigation_yaw_reversed != g_navigation.config.imu_yaw_reversed ||
        navigation_scale_x != g_navigation.config.forward_scale ||
        LATERAL_CORRECTION_FACTOR * navigation_scale_y != g_navigation.config.lateral_scale) {
        g_reset_requested = false;
        reset_state();
        app_navigation_imu_tick_5ms();
        return; /* 本周期计数发生在 Zero 之前，不能加入新起点。 */
    }
    g_imu_age_s += 0.01f;
    if (g_have_imu && (imu_attitude_status != IMU_ATTITUDE_RUNNING || g_imu_age_s > NAV_IMU_STALE_S))
        navigation_fusion_invalidate(&g_navigation);
    navigation_fusion_encoder(&g_navigation, counts, 0.01f);
}
