#include "navigation_fusion.h"
#include "navigation_config.h"

#include <math.h>
#include <string.h>

/* 独立实现编码器/IMU互补融合；算法参考见 README 的来源说明。 */
#define NAV_PI 3.14159265358979323846f
#define NAV_DEG_RAD (NAV_PI / 180.0f)
#define NAV_GRAVITY 9.80665f

static float wrap_pi(float value)
{
    value = fmodf(value + NAV_PI, 2.0f * NAV_PI);
    if (value < 0.0f) value += 2.0f * NAV_PI;
    return value - NAV_PI;
}

static float norm2(float x, float y)
{
    return sqrtf(x * x + y * y);
}

static void fault(navigation_fusion_t *s, int32_t status)
{
    s->fault_latched = true;
    s->output.status = status;
    s->output.valid = false;
    s->output.stationary = false;
    s->output.slipping = false;
    s->velocity[0] = s->velocity[1] = 0.0f;
    s->predicted_delta[0] = s->predicted_delta[1] = 0.0f;
    s->output.vx_mps = s->output.vy_mps = 0.0f;
}

void navigation_fusion_invalidate(navigation_fusion_t *s)
{
    fault(s, NAV_IMU_FAULT);
}

void navigation_fusion_init(navigation_fusion_t *s, const navigation_config_t *config)
{
    memset(s, 0, sizeof(*s));
    s->config = *config;
    s->output.status = NAV_WAIT_IMU;
    if (!isfinite(config->wheel_diameter_m) || config->wheel_diameter_m <= 0.0f || config->wheel_diameter_m > 2.0f ||
        !isfinite(config->lateral_scale) || config->lateral_scale <= 0.0f || config->lateral_scale > 5.0f ||
        !isfinite(config->lateral_to_forward) || fabsf(config->lateral_to_forward) > 5.0f ||
        !isfinite(config->imu_mount_yaw_deg) || fabsf(config->imu_mount_yaw_deg) > 180.0f) {
        fault(s, NAV_BAD_INPUT);
        return;
    }
    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(config->counts_per_revolution[i]) || config->counts_per_revolution[i] < 1.0f ||
            config->counts_per_revolution[i] > 10000000.0f) {
            fault(s, NAV_BAD_INPUT);
            return;
        }
        s->meter_per_count[i] = NAV_PI * config->wheel_diameter_m / config->counts_per_revolution[i];
    }
    s->mount_cos = cosf(config->imu_mount_yaw_deg * NAV_DEG_RAD);
    s->mount_sin = sinf(config->imu_mount_yaw_deg * NAV_DEG_RAD);
}

void navigation_fusion_imu(navigation_fusion_t *s, const navigation_imu_t *m)
{
    float roll, pitch, cr, sr, cp, sp, level_x, level_y, az, acc_norm, yaw, delta;
    float bx, by, c, sn, ax, ay;
    if (s->fault_latched) return;
    if (!isfinite(m->dt_s) || m->dt_s <= 0.0f || m->dt_s > NAV_IMU_MAX_DT_S ||
        !isfinite(m->roll_deg) || !isfinite(m->pitch_deg) || !isfinite(m->yaw_deg)) {
        fault(s, NAV_IMU_FAULT);
        return;
    }
    for (unsigned i = 0; i < 3; ++i) {
        /* 防止有限但超量程的值在平方/积分中溢出；当前硬件为 ±8g/±2000dps。 */
        if (!isfinite(m->accel_g[i]) || fabsf(m->accel_g[i]) > 16.0f ||
            !isfinite(m->gyro_dps[i]) || fabsf(m->gyro_dps[i]) > 2200.0f) {
            fault(s, NAV_IMU_FAULT);
            return;
        }
    }
    yaw = m->yaw_deg * NAV_DEG_RAD;
    if (!s->have_heading) {
        s->last_yaw_rad = yaw;
        s->have_heading = true;
        s->output.status = NAV_CALIBRATING;
    }
    delta = wrap_pi(yaw - s->last_yaw_rad);
    if (fabsf(delta) > NAV_MAX_YAW_RADPS * m->dt_s) {
        fault(s, NAV_IMU_FAULT);
        return;
    }
    s->last_yaw_rad = yaw;
    s->heading_rad += s->config.imu_yaw_reversed ? -delta : delta;
    s->output.yaw_deg = s->heading_rad / NAV_DEG_RAD;

    /* Ry(pitch)*Rx(roll) 去倾斜，重力仅保留在 Z，再按安装角映射水平轴。 */
    roll = m->roll_deg * NAV_DEG_RAD;
    pitch = m->pitch_deg * NAV_DEG_RAD;
    cr = cosf(roll); sr = sinf(roll); cp = cosf(pitch); sp = sinf(pitch);
    level_x = cp * m->accel_g[0] + sp * sr * m->accel_g[1] + sp * cr * m->accel_g[2];
    level_y = cr * m->accel_g[1] - sr * m->accel_g[2];
    az = -sp * m->accel_g[0] + cp * sr * m->accel_g[1] + cp * cr * m->accel_g[2];
    acc_norm = sqrtf(level_x * level_x + level_y * level_y + az * az);
    s->body_acc[0] = NAV_GRAVITY * (s->mount_cos * level_x - s->mount_sin * level_y);
    s->body_acc[1] = NAV_GRAVITY * (s->mount_sin * level_x + s->mount_cos * level_y);
    s->gyro_norm_dps = sqrtf(m->gyro_dps[0] * m->gyro_dps[0] +
                                m->gyro_dps[1] * m->gyro_dps[1] + m->gyro_dps[2] * m->gyro_dps[2]);
    if (!s->output.bias_ready) {
        if (s->max_wheel_speed < NAV_STATIC_WHEEL_MPS && s->gyro_norm_dps < NAV_STATIC_GYRO_DPS &&
            acc_norm > 0.9f && acc_norm < 1.1f && norm2(s->body_acc[0], s->body_acc[1]) < 0.5f) {
            s->bias_sum[0] += s->body_acc[0];
            s->bias_sum[1] += s->body_acc[1];
            if (++s->bias_samples >= NAV_ACCEL_BIAS_SAMPLES) {
                s->body_bias[0] = s->bias_sum[0] / (float)s->bias_samples;
                s->body_bias[1] = s->bias_sum[1] / (float)s->bias_samples;
                s->output.bias_ready = s->output.valid = true;
                s->output.status = NAV_RUNNING;
                s->wheel_heading_rad = s->heading_rad;
            }
        } else {
            s->bias_samples = 0;
            s->bias_sum[0] = s->bias_sum[1] = 0.0f;
        }
        return;
    }
    bx = s->body_acc[0] - s->body_bias[0];
    by = s->body_acc[1] - s->body_bias[1];
    c = cosf(s->heading_rad); sn = sinf(s->heading_rad);
    ax = c * bx - sn * by;
    ay = sn * bx + c * by;
    s->output.ax_mps2 = ax;
    s->output.ay_mps2 = ay;
    /* 200Hz 惯性预测；100Hz 编码器修正速度与本周期位移。 */
    s->predicted_delta[0] += s->velocity[0] * m->dt_s + 0.5f * ax * m->dt_s * m->dt_s;
    s->predicted_delta[1] += s->velocity[1] * m->dt_s + 0.5f * ay * m->dt_s * m->dt_s;
    s->velocity[0] += ax * m->dt_s;
    s->velocity[1] += ay * m->dt_s;
}

void navigation_fusion_encoder(navigation_fusion_t *s, const int16_t counts[4], float dt_s)
{
    float wheel[4], dx, dy, half, arc_scale, mid, c, sn, enc_dx, enc_dy;
    float vx, vy, innovation, accel_diff, alpha, ax, ay;
    if (s->fault_latched) return;
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f) {
        fault(s, NAV_BAD_INPUT);
        return;
    }
    s->max_wheel_speed = 0.0f;
    for (unsigned i = 0; i < 4; ++i) {
        wheel[i] = counts[i] * s->meter_per_count[i];
        s->max_wheel_speed = fmaxf(s->max_wheel_speed, fabsf(wheel[i] / dt_s));
    }
    if (s->max_wheel_speed > NAV_MAX_WHEEL_MPS) {
        ++s->output.rejected_encoder_samples;
        fault(s, NAV_ENCODER_FAULT);
        return;
    }
    if (!s->output.bias_ready) {
        if (s->max_wheel_speed >= NAV_STATIC_WHEEL_MPS) {
            s->bias_samples = 0;
            s->bias_sum[0] = s->bias_sum[1] = 0.0f;
        }
        return;
    }
    dx = 0.25f * (wheel[0] + wheel[1] + wheel[2] + wheel[3]);
    dy = 0.25f * (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) * s->config.lateral_scale;
    dx += s->config.lateral_to_forward * dy;
    half = 0.5f * (s->heading_rad - s->wheel_heading_rad);
    mid = s->wheel_heading_rad + half;
    arc_scale = fabsf(half) < 1e-5f ? 1.0f - half * half / 6.0f : sinf(half) / half;
    c = cosf(mid); sn = sinf(mid);
    enc_dx = arc_scale * (c * dx - sn * dy);
    enc_dy = arc_scale * (sn * dx + c * dy);
    s->wheel_heading_rad = s->heading_rad;

    /* 速度比较用周期末车体方向，防止转弯时坐标旋转被误认为打滑。 */
    c = cosf(s->heading_rad); sn = sinf(s->heading_rad);
    vx = (c * dx - sn * dy) / dt_s;
    vy = (sn * dx + c * dy) / dt_s;
    s->output.encoder_vx_mps = vx;
    s->output.encoder_vy_mps = vy;
    ax = s->body_acc[0] - s->body_bias[0];
    ay = s->body_acc[1] - s->body_bias[1];
    if (s->max_wheel_speed < NAV_STATIC_WHEEL_MPS && s->gyro_norm_dps < NAV_STATIC_GYRO_DPS &&
        norm2(ax, ay) < NAV_STATIC_ACCEL_MPS2) {
        if (s->stationary_ticks < NAV_STATIC_CONFIRM_TICKS) ++s->stationary_ticks;
    } else {
        s->stationary_ticks = 0;
    }
    s->output.stationary = s->stationary_ticks >= NAV_STATIC_CONFIRM_TICKS;
    if (s->output.stationary) {
        s->body_bias[0] += NAV_STATIC_BIAS_ALPHA * ax;
        s->body_bias[1] += NAV_STATIC_BIAS_ALPHA * ay;
        s->velocity[0] = s->velocity[1] = 0.0f;
        s->predicted_delta[0] = s->predicted_delta[1] = 0.0f;
        s->slip_ticks = 0;
    } else {
        innovation = norm2(vx - s->velocity[0], vy - s->velocity[1]);
        accel_diff = norm2((vx - s->previous_encoder_velocity[0]) / dt_s - s->output.ax_mps2,
                          (vy - s->previous_encoder_velocity[1]) / dt_s - s->output.ay_mps2);
        /* 首次测量先建立速度基准，避免启动瞬间误触发。 */
        if (s->encoder_samples >= 3u && innovation > NAV_SLIP_VELOCITY_MPS && accel_diff > NAV_SLIP_ACCEL_MPS2)
            s->slip_ticks = NAV_SLIP_HOLD_TICKS;
    }
    alpha = s->slip_ticks ? NAV_SLIP_ENCODER_WEIGHT : NAV_ENCODER_WEIGHT;
    s->output.slipping = s->slip_ticks != 0u;
    s->output.encoder_weight = alpha;
    s->output.x_m += alpha * enc_dx + (1.0f - alpha) * s->predicted_delta[0];
    s->output.y_m += alpha * enc_dy + (1.0f - alpha) * s->predicted_delta[1];
    s->velocity[0] += alpha * (vx - s->velocity[0]);
    s->velocity[1] += alpha * (vy - s->velocity[1]);
    s->output.vx_mps = s->velocity[0];
    s->output.vy_mps = s->velocity[1];
    s->output.accel_bias_forward = s->body_bias[0];
    s->output.accel_bias_left = s->body_bias[1];
    s->previous_encoder_velocity[0] = vx;
    s->previous_encoder_velocity[1] = vy;
    s->predicted_delta[0] = s->predicted_delta[1] = 0.0f;
    if (s->encoder_samples < 3u) ++s->encoder_samples;
    if (s->slip_ticks) --s->slip_ticks;
}
