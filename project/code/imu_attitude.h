#ifndef MCAR_IMU_ATTITUDE_H
#define MCAR_IMU_ATTITUDE_H

#include "zf_common_typedef.h"
#include <stdbool.h>

/* 仅发布完成姿态更新的同一帧数据，陀螺已扣静止标定零偏。 */
typedef struct {
    uint32 sequence, generation;
    float roll_deg, pitch_deg, yaw_deg, dt_s;
    float accel_g[3], gyro_dps[3];
} imu_navigation_sample_t;

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
/* 同一控制中断内读取；主循环如需读取，先短暂关闭中断。 */
bool imu_attitude_get_navigation_sample(imu_navigation_sample_t *sample);

/* Mahony/Madgwick 共用的几何接口，实现统一放在 imu_attitude.c。
 * 输入四元数顺序 w,x,y,z，加速度单位 g，欧拉角输出单位度。 */
int attitude_init(float q[4], float *accel_norm, const float accel_g[3]);
void attitude_euler(const float q[4], float rpy_deg[3]);
void attitude_cube_euler(const float q[4], float xyz_deg[3]);

#endif
