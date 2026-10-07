/** IMU660RA 采样、状态机与姿态发布；不依赖 WiFi 或显示驱动。 */
#ifndef MCAR_IMU_H
#define MCAR_IMU_H
#include "zf_common_typedef.h"
#ifdef __cplusplus
extern "C" {
#endif

enum {
    IMU_ATTITUDE_INIT_FAILED=-1, IMU_ATTITUDE_TIMEOUT=-2,
    IMU_ATTITUDE_BAD_DT=-3, IMU_ATTITUDE_FILTER_ERROR=-4,
    IMU_ATTITUDE_CALIBRATING=0, IMU_ATTITUDE_RUNNING=1
};
/* 保留原变量名，Mymenu 不需改变数据绑定。多变量一致快照需短暂关中断。 */
extern volatile int32 imu_attitude_status;
/* 常规 ZYX 欧拉角，单位：度。六轴 yaw 长期会漂移。 */
extern volatile float imu_roll_deg;
extern volatile float imu_pitch_deg;
extern volatile float imu_yaw_deg;
extern volatile float imu_accel_norm_g; /* 加速度模长，g */
extern volatile float imu_calibration_percent; /* 静止标定进度，0~100 */
extern volatile float imu_accel_g[3]; /* 原始 X/Y/Z 加速度，g */
extern volatile float imu_gyro_dps[3]; /* 未扣静止零偏的角速度，deg/s */
extern const uint8 imu_attitude_method;

/* 同一成功采样帧的姿态、加速度和扣零偏角速度，供定位融合使用。
 * 主循环调用 getter 时须短暂关中断；PIT 内可直接读取。 */
typedef struct {
    uint32 sequence, generation;
    float roll_deg, pitch_deg, yaw_deg, dt_s;
    float accel_g[3], gyro_dps[3];
} imu_navigation_sample_t;
bool imu_get_navigation_sample(imu_navigation_sample_t *out);

void imu_init(void); /* 启动阶段，先于 WiFi/定时采样启用 */
void imu_update_5ms(void); /* 5ms 周期入口 */
void imu_request_recalibration(void); /* 只置请求标志 */
void imu_service(void); /* 主循环处理重标定 */

#ifdef __cplusplus
}
#endif
#endif
