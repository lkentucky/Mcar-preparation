#include "odometry_calibration.h"
#include <math.h>
bool odometry_calibration_scale(float current, float odometry, float measured, float *result)
{
    float next;
    if (!result || !isfinite(current) || !isfinite(odometry) || !isfinite(measured) ||
        current<.1f || current>5 || fabsf(odometry)<10 || fabsf(measured)<10 ||
        fabsf(odometry)>10000 || fabsf(measured)>10000) return false;
    next=current*fabsf(measured/odometry);
    if (!isfinite(next) || next<.1f || next>5) return false;
    *result=next; return true;
}
