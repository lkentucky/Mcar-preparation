#include "navigation_fusion.h"
#include "navigation_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f
static const int16_t zero[4] = {0};

static navigation_config_t config(void)
{
    navigation_config_t c = {{1000, 1000, 1000, 500}, 1.0f / PI, 1.0f, 1.0f, 0.0f, 0.0f, false};
    return c;
}

static navigation_imu_t sample(void)
{
    navigation_imu_t m = {0};
    m.accel_g[2] = 1.0f;
    m.dt_s = 0.005f;
    return m;
}

static void warmup(navigation_fusion_t *s, navigation_config_t c, navigation_imu_t m)
{
    navigation_fusion_init(s, &c);
    for (unsigned i = 0; i < NAV_ACCEL_BIAS_SAMPLES; ++i) {
        navigation_fusion_imu(s, &m);
        if (i % 2u) navigation_fusion_encoder(s, zero, 0.01f);
    }
    assert(s->output.valid && s->output.bias_ready && s->output.status == NAV_RUNNING);
}

static void tick(navigation_fusion_t *s, const int16_t count[4], navigation_imu_t m)
{
    navigation_fusion_imu(s, &m);
    navigation_fusion_imu(s, &m);
    navigation_fusion_encoder(s, count, 0.01f);
}

static void check_straight_and_strafe(void)
{
    navigation_fusion_t s;
    navigation_imu_t m = sample();
    navigation_config_t c = config();
    int16_t counts[4] = {2, 2, 2, 1};
    warmup(&s, c, m);
    for (unsigned i = 0; i < 100; ++i) tick(&s, counts, m);
    assert(fabsf(s.output.x_m - 0.2f) < 1e-5f && fabsf(s.output.y_m) < 1e-5f);
    assert(fabsf(s.output.encoder_vx_mps - 0.2f) < 1e-5f);
    assert(!s.output.slipping);
    for (unsigned i = 0; i < 4; ++i) counts[i] = (int16_t)-counts[i];
    for (unsigned i = 0; i < 100; ++i) tick(&s, counts, m);
    assert(fabsf(s.output.x_m) < 1e-5f);
    warmup(&s, c, m);
    counts[0] = -2; counts[1] = 2; counts[2] = 2; counts[3] = -1;
    for (unsigned i = 0; i < 100; ++i) tick(&s, counts, m);
    assert(fabsf(s.output.y_m - 0.2f) < 1e-5f && fabsf(s.output.x_m) < 1e-5f);
    c.lateral_scale = 0.8f;
    warmup(&s, c, m);
    for (unsigned i = 0; i < 100; ++i) tick(&s, counts, m);
    assert(fabsf(s.output.y_m - 0.16f) < 1e-5f);
}

static void check_arc_and_spin(void)
{
    navigation_fusion_t s;
    navigation_imu_t m = sample();
    const int16_t counts[4] = {14, 18, 14, 9}; /* forward + rotation; DR half-resolution */
    warmup(&s, config(), m);
    for (unsigned i = 0; i < 100; ++i) {
        m.gyro_dps[2] = 90.0f;
        m.accel_g[1] = (1.6f * PI / 2.0f) / 9.80665f; /* centripetal acceleration */
        m.yaw_deg = (i + 0.5f) * 0.9f;
        navigation_fusion_imu(&s, &m);
        m.yaw_deg = (i + 1.0f) * 0.9f;
        navigation_fusion_imu(&s, &m);
        navigation_fusion_encoder(&s, counts, 0.01f);
    }
    /* v=1.6m/s, omega=pi/2rad/s: after a quarter-circle x=y=v/omega. */
    assert(fabsf(s.output.x_m - 3.2f / PI) < 5e-5f);
    assert(fabsf(s.output.y_m - 3.2f / PI) < 5e-5f);
    assert(fabsf(s.output.yaw_deg - 90.0f) < 1e-3f);
    assert(!s.output.slipping);
    m = sample();
    warmup(&s, config(), m);
    const int16_t spin[4] = {-2, 2, -2, 1};
    for (unsigned i = 0; i < 100; ++i) {
        m.yaw_deg = (i + 1.0f) * 0.9f;
        m.gyro_dps[2] = 90.0f;
        tick(&s, spin, m);
    }
    assert(fabsf(s.output.x_m) < 1e-6f && fabsf(s.output.y_m) < 1e-6f);
    assert(!s.output.stationary);
}

static void check_mount_tilt_wrap_and_bias(void)
{
    navigation_fusion_t s;
    navigation_config_t c = config();
    navigation_imu_t m = sample();
    c.imu_mount_yaw_deg = 180.0f;
    m.roll_deg = 20.0f; m.pitch_deg = -10.0f;
    float sr = sinf(m.roll_deg * PI / 180), cr = cosf(m.roll_deg * PI / 180);
    float sp = sinf(m.pitch_deg * PI / 180), cp = cosf(m.pitch_deg * PI / 180);
    m.accel_g[0] = -sp; m.accel_g[1] = sr * cp; m.accel_g[2] = cr * cp;
    warmup(&s, c, m);
    for (unsigned i = 0; i < 100; ++i) tick(&s, zero, m);
    assert(s.output.stationary && fabsf(s.output.ax_mps2) < 1e-4f && fabsf(s.output.ay_mps2) < 1e-4f);
    /* Add forward 1m/s^2 in chassis coordinates: sensor level X points backward. */
    float lx = -1.0f / 9.80665f;
    m.accel_g[0] = cp * lx - sp;
    m.accel_g[1] = sr * sp * lx + sr * cp;
    m.accel_g[2] = cr * sp * lx + cr * cp;
    navigation_fusion_imu(&s, &m);
    assert(fabsf(s.output.ax_mps2 - 1.0f) < 1e-4f && fabsf(s.output.ay_mps2) < 1e-4f);
    assert(s.velocity[0] > 0.0f);

    m = sample(); m.yaw_deg = 179.0f;
    warmup(&s, config(), m);
    m.yaw_deg = 180.0f; navigation_fusion_imu(&s, &m);
    m.yaw_deg = -179.0f; navigation_fusion_imu(&s, &m);
    assert(s.output.valid && fabsf(s.output.yaw_deg - 2.0f) < 0.001f);
    c = config(); c.imu_yaw_reversed = true;
    m = sample(); warmup(&s, c, m);
    m.yaw_deg = 1.0f; navigation_fusion_imu(&s, &m);
    assert(fabsf(s.output.yaw_deg + 1.0f) < 0.001f);
    m = sample(); m.accel_g[0] = 0.01f;
    warmup(&s, config(), m);
    for (unsigned i = 0; i < 100; ++i) tick(&s, zero, m);
    assert(s.output.stationary && fabsf(s.output.ax_mps2) < 1e-4f);
    assert(fabsf(s.output.accel_bias_forward - 0.0980665f) < 1e-4f);
}

static void check_slip_and_faults(void)
{
    navigation_fusion_t s;
    navigation_imu_t m = sample();
    warmup(&s, config(), m);
    for (unsigned i = 0; i < 4; ++i) tick(&s, zero, m);
    const int16_t slip[4] = {10, 10, 10, 5};
    tick(&s, slip, m); /* wheel reports 1m/s, no corresponding inertial acceleration */
    assert(s.output.slipping && fabsf(s.output.encoder_weight - 0.15f) < 1e-6f);
    assert(s.output.x_m > 0.0f && s.output.x_m < 0.003f);
    for (unsigned i = 0; i < 30; ++i) tick(&s, zero, m);
    assert(s.output.stationary && !s.output.slipping && s.output.vx_mps == 0.0f);
    float last_x = s.output.x_m;
    const int16_t spike[4] = {0, 0, 30000, 0};
    tick(&s, spike, m);
    assert(!s.output.valid && s.output.status == NAV_ENCODER_FAULT);
    assert(s.output.rejected_encoder_samples == 1 && s.output.x_m == last_x);
    tick(&s, slip, m);
    assert(s.output.x_m == last_x); /* latches until explicit reset */
    warmup(&s, config(), m);
    navigation_fusion_invalidate(&s);
    assert(!s.output.valid && s.output.status == NAV_IMU_FAULT);
    warmup(&s, config(), m);
    m.accel_g[0] = NAN;
    navigation_fusion_imu(&s, &m);
    assert(!s.output.valid && s.output.status == NAV_IMU_FAULT);
    m = sample(); warmup(&s, config(), m);
    m.accel_g[0] = 1e38f;
    navigation_fusion_imu(&s, &m);
    assert(!s.output.valid && isfinite(s.output.x_m) && isfinite(s.output.vx_mps));
    navigation_config_t bad = config(); bad.counts_per_revolution[3] = 0.0f;
    navigation_fusion_init(&s, &bad);
    assert(s.output.status == NAV_BAD_INPUT && !s.output.valid);
}

static void check_axis_scales(void)
{
    navigation_fusion_t s;
    navigation_imu_t m=sample();
    navigation_config_t c=config();
    c.forward_scale=0.5f;
    c.lateral_scale=0.6f;
    const int16_t forward[4]={2,2,2,1};
    const int16_t left[4]={-2,2,2,-1};
    const int16_t diagonal[4]={0,4,4,0};
    warmup(&s,c,m);
    for(unsigned i=0;i<100;++i) tick(&s,forward,m);
    assert(fabsf(s.output.x_m-0.1f)<1e-5f && fabsf(s.output.y_m)<1e-5f);
    warmup(&s,c,m);
    for(unsigned i=0;i<100;++i) tick(&s,left,m);
    assert(fabsf(s.output.y_m-0.12f)<1e-5f && fabsf(s.output.x_m)<1e-5f);
    warmup(&s,c,m);
    for(unsigned i=0;i<100;++i) tick(&s,diagonal,m);
    assert(fabsf(s.output.x_m-0.1f)<1e-5f && fabsf(s.output.y_m-0.12f)<1e-5f);
    /* Calibration belongs to chassis axes; it must rotate with measured yaw. */
    warmup(&s,c,m);
    for(unsigned i=0;i<10;++i) { m.yaw_deg+=9; tick(&s,zero,m); }
    for(unsigned i=0;i<100;++i) tick(&s,diagonal,m);
    assert(fabsf(s.output.x_m+0.12f)<1e-5f && fabsf(s.output.y_m-0.1f)<1e-5f);
    assert(fabsf(s.output.vx_mps+0.12f)<1e-5f && fabsf(s.output.vy_mps-0.1f)<1e-5f);
    const int16_t reverse[4]={0,-4,-4,0};
    for(unsigned i=0;i<100;++i) tick(&s,reverse,m);
    assert(fabsf(s.output.x_m)<1e-5f && fabsf(s.output.y_m)<1e-5f);
    c.forward_scale=NAN;
    navigation_fusion_init(&s,&c);
    assert(!s.output.valid && s.output.status==NAV_BAD_INPUT);
    c.forward_scale=0;
    navigation_fusion_init(&s,&c);
    assert(!s.output.valid && s.output.status==NAV_BAD_INPUT);
}

int main(void)
{
    check_straight_and_strafe();
    check_arc_and_spin();
    check_mount_tilt_wrap_and_bias();
    check_slip_and_faults();
    check_axis_scales();
    puts("navigation fusion tests passed: mixed resolutions, strafe, arc, spin, tilt, mount, yaw wrap, bias, slip, faults");
    return 0;
}
