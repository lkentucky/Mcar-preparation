#include "position_control.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f

static navigation_snapshot_t valid_pose(void)
{
    navigation_snapshot_t pose = {0};
    pose.status = NAV_RUNNING;
    pose.valid = pose.bias_ready = true;
    return pose;
}

static void check_directions_and_limits(void)
{
    position_control_t control;
    position_config_t config = POSITION_CONFIG_DEFAULT;
    navigation_snapshot_t pose = valid_pose();
    position_goal_t goal = {100.0f, 100.0f, 90.0f};
    position_control_reset(&control);
    float vx_prev = 0.0f, vy_prev = 0.0f, omega_prev = 0.0f;
    for (unsigned i = 0; i < 100; ++i) {
        position_control_update(&control, &config, &goal, &pose, 0.01f);
        assert(control.output.status == POSITION_MOVING);
        assert(hypotf(control.output.vx_cmps, control.output.vy_cmps) <= 20.00001f);
        assert(hypotf(control.output.vx_cmps - vx_prev,
                      control.output.vy_cmps - vy_prev) <= 0.40001f);
        assert(fabsf(control.output.omega_radps) <= 1.00001f);
        assert(fabsf(control.output.omega_radps - omega_prev) <= 0.02001f);
        vx_prev = control.output.vx_cmps; vy_prev = control.output.vy_cmps;
        omega_prev = control.output.omega_radps;
    }
    assert(control.output.vx_cmps > 0.0f && control.output.vy_cmps > 0.0f);
    assert(control.output.omega_radps > 0.0f);
    /* World +X at yaw +90 is body -Y (right); goal still lies in same world frame. */
    goal.y_cm = 0.0f; pose.yaw_deg = goal.yaw_deg = 90.0f;
    position_control_reset(&control);
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(fabsf(control.output.vx_cmps) < 1e-6f && control.output.vy_cmps < 0.0f);
    goal.x_cm = 0.0f; goal.y_cm = 100.0f;
    position_control_reset(&control);
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.vx_cmps > 0.0f && fabsf(control.output.vy_cmps) < 1e-6f);
    /* Simultaneous translation and rotation. */
    goal.x_cm = 100.0f; goal.y_cm = -100.0f; goal.yaw_deg = 0.0f;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.omega_radps < 0.0f);
    pose.yaw_deg = 179.0f; goal.yaw_deg = -179.0f;
    config.yaw_tolerance_deg = 0.5f;
    position_control_reset(&control);
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(fabsf(control.output.yaw_error_deg - 2.0f) < 0.001f);
    assert(control.output.omega_radps > 0.0f);
    /* Velocity damping must slow the world command when already moving. */
    pose = valid_pose(); goal = (position_goal_t){5.0f, 0.0f, 0.0f};
    config.max_accel_cmps2 = 300.0f;
    pose.vx_mps = 0.4f;
    position_control_reset(&control);
    for (unsigned i = 0; i < 20; ++i)
        position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(fabsf(control.output.vx_cmps - 2.0f) < 0.001f);
}

static void check_arrival_and_faults(void)
{
    position_control_t control;
    position_config_t config = POSITION_CONFIG_DEFAULT;
    navigation_snapshot_t pose = valid_pose();
    position_goal_t goal = {0};
    position_control_reset(&control);
    pose.vx_mps = 0.1f;
    for (unsigned i = 0; i < 30; ++i) {
        position_control_update(&control, &config, &goal, &pose, 0.01f);
        assert(control.output.status == POSITION_SETTLING);
    }
    pose.vx_mps = 0.0f;
    for (unsigned i = 0; i < 19; ++i) {
        position_control_update(&control, &config, &goal, &pose, 0.01f);
        assert(control.output.status == POSITION_SETTLING);
    }
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.status == POSITION_REACHED);
    assert(control.output.vx_cmps == 0.0f && control.output.omega_radps == 0.0f);
    /* Fresh target cancels the previous dwell, including edits within tolerance. */
    goal.x_cm = 1.0f;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.status == POSITION_SETTLING);
    goal.x_cm = 100.0f;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.vx_cmps > 0.0f);
    pose.valid = false;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.status == POSITION_NO_POSE);
    assert(control.output.vx_cmps == 0.0f && control.output.vy_cmps == 0.0f);
    pose = valid_pose(); config.max_speed_cmps = NAN;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.status == POSITION_BAD_CONFIG);
    config = (position_config_t)POSITION_CONFIG_DEFAULT;
    goal.x_cm = FLT_MAX;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.status == POSITION_BAD_CONFIG);
    goal.x_cm = 100.0f; pose.x_m = NAN;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.status == POSITION_NO_POSE);
    pose = valid_pose(); pose.yaw_deg = FLT_MAX;
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(isfinite(control.output.vx_cmps) && isfinite(control.output.yaw_error_deg));
    /* Angle crossing +/-180 must not be interpreted as an almost full turn. */
    pose = valid_pose(); pose.yaw_deg = 179.99f; goal = (position_goal_t){0, 0, 180};
    position_control_reset(&control);
    position_control_update(&control, &config, &goal, &pose, 0.01f);
    pose.yaw_deg = -179.99f;
    for (unsigned i = 0; i < 21; ++i)
        position_control_update(&control, &config, &goal, &pose, 0.01f);
    assert(control.output.status == POSITION_REACHED);
    /* Being in tolerance while still spinning must never finish the dwell. */
    position_control_reset(&control);
    pose.yaw_deg = 0; goal.yaw_deg = 0;
    for (unsigned i = 0; i < 30; ++i) {
        pose.yaw_deg = i % 2 ? 0.1f : -0.1f;
        position_control_update(&control, &config, &goal, &pose, 0.01f);
        assert(control.output.status != POSITION_REACHED);
    }
}

/* A lagged holonomic plant verifies that the full point/yaw loop converges,
 * rather than checking only its first command. It is not hardware tuning. */
static void check_point(float x, float y, float yaw)
{
    position_control_t control;
    position_config_t config = POSITION_CONFIG_DEFAULT;
    navigation_snapshot_t pose = valid_pose();
    position_goal_t goal = {x, y, yaw};
    float actual_omega = 0.0f;
    position_control_reset(&control);
    unsigned i;
    for (i = 0; i < 3000; ++i) {
        position_control_update(&control, &config, &goal, &pose, 0.01f);
        if (control.output.status == POSITION_REACHED) break;
        assert(control.output.status > 0);
        float theta = pose.yaw_deg * PI / 180.0f;
        float vx = (cosf(theta) * control.output.vx_cmps -
                    sinf(theta) * control.output.vy_cmps) * 0.01f;
        float vy = (sinf(theta) * control.output.vx_cmps +
                    cosf(theta) * control.output.vy_cmps) * 0.01f;
        pose.vx_mps += 0.2f * (vx - pose.vx_mps);
        pose.vy_mps += 0.2f * (vy - pose.vy_mps);
        actual_omega += 0.2f * (control.output.omega_radps - actual_omega);
        pose.x_m += pose.vx_mps * 0.01f;
        pose.y_m += pose.vy_mps * 0.01f;
        pose.yaw_deg += actual_omega * 0.01f * 180.0f / PI;
    }
    assert(i < 3000);
    assert(hypotf(x - pose.x_m * 100.0f, y - pose.y_m * 100.0f) <= config.xy_tolerance_cm);
    assert(fabsf(control.output.yaw_error_deg) <= config.yaw_tolerance_deg);
}

int main(void)
{
    check_directions_and_limits();
    check_arrival_and_faults();
    check_point(50, 0, 0);
    check_point(-50, 0, 0);
    check_point(0, 50, 0);
    check_point(0, -50, 0);
    check_point(50, -50, 90);
    check_point(-50, 50, -120);
    check_point(0, 0, 180);
    {
        position_control_t c;
        position_config_t config=POSITION_CONFIG_DEFAULT;
        position_goal_t goal={100,0,0};
        navigation_snapshot_t pose=valid_pose();
        position_control_reset(&c);
        /* A blocked plant is never declared reached just because time expired. */
        for(unsigned i=0;i<1500;++i) position_control_update(&c,&config,&goal,&pose,.01f);
        assert(c.output.profile_phase==8 && c.output.status!=POSITION_REACHED);
        assert(c.output.ref_x_cm==100 && c.output.distance_cm==100);
        assert(c.output.vx_cmps<=5.0001f && c.output.vx_cmps>0);
        for(unsigned i=0;i<2000 && c.output.status>=0;++i)
            position_control_update(&c,&config,&goal,&pose,.01f);
        assert(c.output.status==POSITION_TIMEOUT && c.output.vx_cmps==0);
        position_control_update(&c,&config,&goal,&pose,.01f);
        assert(c.output.status==POSITION_TIMEOUT);
        /* A lateral disturbance is corrected, rather than repeating a frozen direction. */
        position_control_reset(&c);
        pose.y_m=.2f;
        for(unsigned i=0;i<40;++i) position_control_update(&c,&config,&goal,&pose,.01f);
        pose.y_m=.3f;
        position_control_update(&c,&config,&goal,&pose,.01f);
        assert(c.output.vy_cmps<0);
        float old_total=c.output.profile_total_s;
        config.max_speed_cmps=10;
        position_control_update(&c,&config,&goal,&pose,.01f);
        assert(c.output.profile_elapsed_s<.02f && c.output.profile_total_s>old_total);
        config.max_jerk_cmps3=NAN;
        position_control_update(&c,&config,&goal,&pose,.01f);
        assert(c.output.status==POSITION_BAD_CONFIG);
    }
    puts("position control passed: world/body transform, mixed move/yaw convergence, limits, dwell, faults");
    return 0;
}
