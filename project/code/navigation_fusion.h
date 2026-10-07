#ifndef MCAR_NAVIGATION_FUSION_H
#define MCAR_NAVIGATION_FUSION_H

#include <stdbool.h>
#include <stdint.h>

/* 固定坐标：起点车头为 +X，左侧为 +Y，逆时针航向为正。 */
enum {
    NAV_ENCODER_FAULT = -3,
    NAV_IMU_FAULT = -2,
    NAV_BAD_INPUT = -1,
    NAV_WAIT_IMU = 0,
    NAV_CALIBRATING = 1,
    NAV_RUNNING = 2
};

typedef struct {
    float counts_per_revolution[4]; /* UL、UR、DL、DR，实际计数 */
    float wheel_diameter_m;
    float lateral_scale;
    float lateral_to_forward;
    float imu_mount_yaw_deg;        /* 水平传感器 +X 相对车头的逆时针角度 */
    bool imu_yaw_reversed;
} navigation_config_t;

typedef struct {
    float roll_deg, pitch_deg, yaw_deg;
    float accel_g[3];
    float gyro_dps[3];              /* 已扣除陀螺零偏 */
    float dt_s;
} navigation_imu_t;

typedef struct {
    int32_t status;
    float x_m, y_m, yaw_deg;
    float vx_mps, vy_mps;            /* 固定坐标速度 */
    float ax_mps2, ay_mps2;
    float encoder_vx_mps, encoder_vy_mps;
    float accel_bias_forward, accel_bias_left;
    float encoder_weight;
    uint32_t rejected_encoder_samples;
    bool valid, bias_ready, stationary, slipping;
} navigation_snapshot_t;

/* 单写者：5ms IMU 与 10ms 编码器更新应在同一控制中断内串行调用。 */
typedef struct {
    navigation_config_t config;
    navigation_snapshot_t output;
    float meter_per_count[4];
    float heading_rad, last_yaw_rad, wheel_heading_rad;
    float mount_cos, mount_sin;
    float body_acc[2], bias_sum[2], body_bias[2];
    float velocity[2], predicted_delta[2], previous_encoder_velocity[2];
    float gyro_norm_dps, max_wheel_speed;
    unsigned bias_samples, stationary_ticks, encoder_samples, slip_ticks;
    bool have_heading, fault_latched;
} navigation_fusion_t;

void navigation_fusion_init(navigation_fusion_t *state, const navigation_config_t *config);
void navigation_fusion_imu(navigation_fusion_t *state, const navigation_imu_t *sample);
void navigation_fusion_encoder(navigation_fusion_t *state, const int16_t counts[4], float dt_s);
void navigation_fusion_invalidate(navigation_fusion_t *state);

#endif
