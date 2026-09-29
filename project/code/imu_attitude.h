#ifndef MCAR_IMU_ATTITUDE_H
#define MCAR_IMU_ATTITUDE_H

#include "zf_common_typedef.h"

enum
{
    IMU_ATTITUDE_INIT_FAILED = -1,
    IMU_ATTITUDE_TIMEOUT = -2,
    IMU_ATTITUDE_BAD_DT = -3,
    IMU_ATTITUDE_FILTER_ERROR = -4,
    IMU_ATTITUDE_CALIBRATING = 0,
    IMU_ATTITUDE_RUNNING = 1
};

extern volatile int32 imu_attitude_status;
extern volatile float imu_roll_deg;
extern volatile float imu_pitch_deg;
extern volatile float imu_yaw_deg;
extern volatile float imu_accel_norm_g;
extern volatile float imu_calibration_percent;
extern volatile float imu_accel_g[3];
extern volatile float imu_gyro_dps[3];
extern const uint8 imu_attitude_method;

void imu_attitude_init(void);
void imu_attitude_update_5ms(void);
void imu_attitude_request_recalibration(void);
void imu_attitude_service(void);

#endif
