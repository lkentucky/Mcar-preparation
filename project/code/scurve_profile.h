#ifndef MCAR_SCURVE_PROFILE_H
#define MCAR_SCURVE_PROFILE_H
#include <stdbool.h>

/* One-dimensional, seven-segment, zero-boundary-acceleration profile.
 * Units: cm, cm/s, cm/s^2, cm/s^3, seconds. No hardware dependencies. */
typedef struct {
    float duration[7], start_s[7], start_v[7], start_a[7], jerk[7];
    float total_s, distance_cm, end_speed_cmps;
    bool valid;
} scurve_profile_t;
typedef struct {
    float position_cm, velocity_cmps, acceleration_cmps2;
    unsigned phase; /* 1..7; 8 = complete; 0 = invalid */
    bool complete;
} scurve_sample_t;
bool scurve_profile_build(scurve_profile_t *profile, float distance_cm,
                          float start_speed_cmps, float end_speed_cmps,
                          float max_speed_cmps, float max_accel_cmps2,
                          float max_jerk_cmps3);
scurve_sample_t scurve_profile_sample(const scurve_profile_t *profile, float time_s);
#endif
