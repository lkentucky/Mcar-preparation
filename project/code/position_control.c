#include "position_control.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define POSITION_SETTLE_TIME_S 0.20f
#define POSITION_REST_SPEED_CMPS 3.0f
#define POSITION_REST_OMEGA_RADPS 0.10f

static float clamp(float value, float limit)
{
    return value < -limit ? -limit : (value > limit ? limit : value);
}

static float wrap_degrees(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void limit_vector(float *x, float *y, float limit)
{
    float length = hypotf(*x, *y);
    if (length > limit) {
        *x *= limit / length;
        *y *= limit / length;
    }
}

static bool in_range(float value, float low, float high)
{
    return isfinite(value) && value >= low && value <= high;
}

static bool config_valid(const position_config_t *c, const position_goal_t *g)
{
    return in_range(c->xy_kp, 0.01f, 20.0f) &&
           in_range(c->xy_kd, 0.0f, 5.0f) &&
           in_range(c->yaw_kp, 0.01f, 20.0f) &&
           in_range(c->max_speed_cmps, 1.0f, 500.0f) &&
           in_range(c->max_omega_radps, 0.05f, 3.0f) &&
           in_range(c->max_accel_cmps2, 1.0f, 300.0f) &&
           in_range(c->max_alpha_radps2, 0.05f, 10.0f) &&
           in_range(c->xy_tolerance_cm, 0.5f, 20.0f) &&
           in_range(c->yaw_tolerance_deg, 0.5f, 20.0f) &&
           in_range(c->max_jerk_cmps3, 1.0f, 3000.0f) &&
           in_range(g->x_cm, -10000.0f, 10000.0f) &&
           in_range(g->y_cm, -10000.0f, 10000.0f) &&
           in_range(g->yaw_deg, -180.0f, 180.0f);
}

void position_control_reset(position_control_t *control)
{
    memset(control, 0, sizeof(*control));
}

static void reject(position_control_t *control, int32_t status)
{
    position_control_reset(control);
    control->output.status = status;
}

void position_control_update(position_control_t *control,
                             const position_config_t *config,
                             const position_goal_t *goal,
                             const navigation_snapshot_t *pose, float dt)
{
    float ex, ey, yaw_error, yaw_rate = 0.0f;
    float vx, vy, omega, dvx, dvy, theta, cs, sn;
    bool xy_inside, yaw_inside, resting, goal_changed, profile_complete;
    scurve_sample_t reference = {0};
    if (!config_valid(config, goal) || !in_range(dt, 0.001f, 0.05f)) {
        reject(control, POSITION_BAD_CONFIG);
        return;
    }
    if (!pose->valid || !pose->bias_ready || pose->status != NAV_RUNNING ||
        !in_range(pose->x_m, -1000.0f, 1000.0f) ||
        !in_range(pose->y_m, -1000.0f, 1000.0f) || !isfinite(pose->yaw_deg) ||
        !in_range(pose->vx_mps, -12.0f, 12.0f) ||
        !in_range(pose->vy_mps, -12.0f, 12.0f)) {
        reject(control, POSITION_NO_POSE);
        return;
    }

    ex = goal->x_cm - pose->x_m * 100.0f;
    ey = goal->y_cm - pose->y_m * 100.0f;
    /* Wrap before subtraction to avoid overflowing very large finite yaw. */
    theta = wrap_degrees(pose->yaw_deg);
    yaw_error = wrap_degrees(goal->yaw_deg - theta);
    goal_changed = !control->have_previous ||
                   goal->x_cm != control->previous_goal.x_cm ||
                   goal->y_cm != control->previous_goal.y_cm ||
                   goal->yaw_deg != control->previous_goal.yaw_deg;
    if (control->have_previous)
        yaw_rate = wrap_degrees(theta - control->previous_yaw_deg) * DEG_TO_RAD / dt;
    if (goal_changed) control->settled_seconds = 0.0f;
    if (control->timeout_latched && !goal_changed) return;
    if (goal_changed || config->max_speed_cmps!=control->planned_vmax ||
        config->max_accel_cmps2!=control->planned_amax || config->max_jerk_cmps3!=control->planned_jmax) {
        float distance=hypotf(ex,ey), start_speed;
        control->start_x_cm=pose->x_m*100;
        control->start_y_cm=pose->y_m*100;
        control->dir_x=distance>1e-6f?ex/distance:0;
        control->dir_y=distance>1e-6f?ey/distance:0;
        start_speed=clamp((pose->vx_mps*control->dir_x+pose->vy_mps*control->dir_y)*100,
                          config->max_speed_cmps);
        start_speed=fmaxf(start_speed,0);
        control->profile_time_s=0;
        control->planned_vmax=config->max_speed_cmps;
        control->planned_amax=config->max_accel_cmps2;
        control->planned_jmax=config->max_jerk_cmps3;
        scurve_profile_build(&control->profile,distance,start_speed,0,config->max_speed_cmps,
                              config->max_accel_cmps2,config->max_jerk_cmps3);
        {
            float xy_time=control->profile.valid?control->profile.total_s:
                          distance/fminf(10,config->max_speed_cmps*.25f);
            float xy_tail=logf(fmaxf(1,distance/config->xy_tolerance_cm))/config->xy_kp;
            float yaw_time=0;
            if (fabsf(yaw_error)>config->yaw_tolerance_deg)
                yaw_time=fabsf(yaw_error)*DEG_TO_RAD/config->max_omega_radps+
                         logf(fabsf(yaw_error)/config->yaw_tolerance_deg)/config->yaw_kp+
                         config->max_omega_radps/config->max_alpha_radps2;
            control->motion_elapsed_s=0;
            control->motion_timeout_s=fmaxf(xy_time+xy_tail,yaw_time)+20;
            control->timeout_latched=false;
        }
        control->settled_seconds=0;
    }
    control->previous_goal = *goal;
    control->previous_yaw_deg = theta;
    control->have_previous = true;
    control->motion_elapsed_s+=dt;
    if (control->motion_elapsed_s>control->motion_timeout_s) {
        position_control_reset(control);
        control->previous_goal=*goal; control->have_previous=true;
        control->timeout_latched=true; control->output.status=POSITION_TIMEOUT;
        return;
    }

    control->output.distance_cm = hypotf(ex, ey);
    control->output.yaw_error_deg = yaw_error;
    xy_inside = control->output.distance_cm <= config->xy_tolerance_cm;
    yaw_inside = fabsf(yaw_error) <= config->yaw_tolerance_deg;
    resting = hypotf(pose->vx_mps, pose->vy_mps) * 100.0f <= POSITION_REST_SPEED_CMPS &&
              fabsf(yaw_rate) <= POSITION_REST_OMEGA_RADPS;

    control->profile_time_s=fminf(control->profile_time_s+dt,control->profile.total_s);
    reference=scurve_profile_sample(&control->profile,control->profile_time_s);
    profile_complete=!control->profile.valid || reference.complete;
    control->output.ref_x_cm=profile_complete?goal->x_cm:control->start_x_cm+reference.position_cm*control->dir_x;
    control->output.ref_y_cm=profile_complete?goal->y_cm:control->start_y_cm+reference.position_cm*control->dir_y;
    control->output.ref_speed_cmps=profile_complete?0:reference.velocity_cmps;
    control->output.ref_accel_cmps2=profile_complete?0:reference.acceleration_cmps2;
    control->output.profile_elapsed_s=control->profile_time_s;
    control->output.profile_total_s=control->profile.total_s;
    control->output.profile_phase=reference.phase;
    /* Track the moving reference, not the full endpoint error during the run.
     * Correction is bounded; terminal correction remains closed-loop. If a
     * short retarget cannot fit braking distance, brake/correct at low speed. */
    {
        float ref_vx=control->output.ref_speed_cmps*control->dir_x;
        float ref_vy=control->output.ref_speed_cmps*control->dir_y;
        float correction_x=config->xy_kp*(control->output.ref_x_cm-pose->x_m*100)+
                           config->xy_kd*(ref_vx-pose->vx_mps*100);
        float correction_y=config->xy_kp*(control->output.ref_y_cm-pose->y_m*100)+
                           config->xy_kd*(ref_vy-pose->vy_mps*100);
        float correction_limit=fminf(10,config->max_speed_cmps*(profile_complete?.25f:.5f));
        limit_vector(&correction_x,&correction_y,correction_limit);
        vx=xy_inside?0:ref_vx+correction_x;
        vy=xy_inside?0:ref_vy+correction_y;
    }
    limit_vector(&vx, &vy, config->max_speed_cmps);
    omega = yaw_inside ? 0.0f : clamp(config->yaw_kp * yaw_error * DEG_TO_RAD,
                                     config->max_omega_radps);
    /* Limit world acceleration: rotating the chassis doesn't rotate the goal. */
    dvx = vx - control->world_vx_cmps;
    dvy = vy - control->world_vy_cmps;
    limit_vector(&dvx, &dvy, config->max_accel_cmps2 * dt);
    control->world_vx_cmps += dvx;
    control->world_vy_cmps += dvy;
    limit_vector(&control->world_vx_cmps, &control->world_vy_cmps, config->max_speed_cmps);
    control->output.omega_radps += clamp(omega - control->output.omega_radps,
                                         config->max_alpha_radps2 * dt);
    control->output.omega_radps = clamp(control->output.omega_radps, config->max_omega_radps);

    theta *= DEG_TO_RAD;
    cs = cosf(theta); sn = sinf(theta);
    control->output.vx_cmps = cs * control->world_vx_cmps + sn * control->world_vy_cmps;
    control->output.vy_cmps = -sn * control->world_vx_cmps + cs * control->world_vy_cmps;
    control->output.status = xy_inside && yaw_inside ? POSITION_SETTLING : POSITION_MOVING;
    /* Also wait until the ramped command has reached zero. */
    if (profile_complete && xy_inside && yaw_inside && resting &&
        hypotf(control->world_vx_cmps, control->world_vy_cmps) < 0.001f &&
        fabsf(control->output.omega_radps) < 0.001f) {
        control->settled_seconds += dt;
        if (control->settled_seconds >= POSITION_SETTLE_TIME_S - 0.00001f) {
            control->output.status = POSITION_REACHED;
            control->output.vx_cmps = control->output.vy_cmps = control->output.omega_radps = 0.0f;
        }
    } else {
        control->settled_seconds = 0.0f;
    }
}
