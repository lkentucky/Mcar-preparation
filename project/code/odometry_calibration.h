#ifndef MCAR_ODOMETRY_CALIBRATION_H
#define MCAR_ODOMETRY_CALIBRATION_H
#include <stdbool.h>
/* Tape-measured distance and odometry distance must be the same straight run.
 * Scale corrects body-axis odometry only; it does not change encoder/PID units. */
bool odometry_calibration_scale(float current_scale, float odometry_cm,
                                float measured_cm, float *new_scale);
#endif
