#ifndef MCAR_POSITION_CONTROL_H
#define MCAR_POSITION_CONTROL_H

#include "navigation_fusion.h"

/* Goal XY uses the fixed Navigation/Zero frame; yaw is relative to that origin.
 * +X forward at Zero, +Y left, positive yaw counter-clockwise. */
typedef struct {
    float x_cm, y_cm, yaw_deg;
} position_goal_t;

typedef struct {
    float xy_kp;             /* 1/s */
    float xy_kd;             /* measured world velocity damping */
    float yaw_kp;            /* 1/s, angle error in radians */
    float max_speed_cmps;
    float max_omega_radps;
    float max_accel_cmps2;
    float max_alpha_radps2;
    float xy_tolerance_cm;
    float yaw_tolerance_deg;
} position_config_t;

#define POSITION_CONFIG_DEFAULT {2.0f, 0.2f, 2.0f, 20.0f, 1.0f, 40.0f, 2.0f, 2.0f, 3.0f}

enum {
    POSITION_BAD_CONFIG = -2,
    POSITION_NO_POSE = -1,
    POSITION_IDLE = 0,
    POSITION_MOVING = 1,
    POSITION_SETTLING = 2,
    POSITION_REACHED = 3
};

typedef struct {
    int32_t status;
    float distance_cm, yaw_error_deg;
    float vx_cmps, vy_cmps, omega_radps; /* effective Drive body command */
} position_output_t;

typedef struct {
    position_output_t output;
    position_goal_t previous_goal;
    float world_vx_cmps, world_vy_cmps, previous_yaw_deg, settled_seconds;
    bool have_previous;
} position_control_t;

void position_control_reset(position_control_t *control);
/* Pure controller, called once per encoder tick. Invalid input returns zero
 * command. Arrival requires low measured speed for 0.2s; caller latches Run off. */
void position_control_update(position_control_t *control,
                             const position_config_t *config,
                             const position_goal_t *goal,
                             const navigation_snapshot_t *pose, float dt);

#endif
