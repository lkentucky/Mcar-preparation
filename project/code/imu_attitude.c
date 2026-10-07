#include "imu_attitude.h"

#include "ahrs6.h"
#include "config.h"
#include "imu_numeric.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#include <math.h>
#include <string.h>

#define IMU_STATUS_REG 0x03u

/* 静止标定：累计陀螺零偏与初始重力向量，运动时丢弃当前窗口。 */
typedef struct {
    unsigned count;
    float sum_g[3], sum_a[3], min_g[3], max_g[3], min_a[3], max_a[3];
    float bias_dps[3], initial_accel[3];
} ImuCalibration;

static void calibration_reset(ImuCalibration *c)
{
    memset(c, 0, sizeof(*c));
}

/* 输入 gyro 为 deg/s、accel 为 g，收满 CALIBRATION_SAMPLES 后返回 1。
 * 这是静止检查，无法识别所有匀速运动。 */
static int calibration_push(ImuCalibration *c, const float g[3], const float a[3])
{
    unsigned i;
    float n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    /* 加速度模长偏离 1g，本次标定作废。 */
    if (!imu_finite(n) || n < 0.9f || n > 1.1f) {
        calibration_reset(c); return 0;
    }
    for (i=0; i<3; ++i) {
        /* 陀螺数值非法或超出静止阈值，同样作废重来。 */
        if (!imu_finite(g[i]) || fabsf(g[i]) > CAL_GYRO_LIMIT_DPS) {
            calibration_reset(c); return 0;
        }
        /* 记录各轴极值，用于峰峰值判据。 */
        if (!c->count) {
            c->min_g[i]=c->max_g[i]=g[i]; c->min_a[i]=c->max_a[i]=a[i];
        }
        if (g[i]<c->min_g[i]) c->min_g[i]=g[i];
        if (g[i]>c->max_g[i]) c->max_g[i]=g[i];
        if (a[i]<c->min_a[i]) c->min_a[i]=a[i];
        if (a[i]>c->max_a[i]) c->max_a[i]=a[i];
        /* 峰峰值超限说明窗口内出现过运动，整个窗口重新计数。 */
        if (c->max_g[i]-c->min_g[i]>CAL_GYRO_SPAN_DPS ||
            c->max_a[i]-c->min_a[i]>CAL_ACCEL_SPAN_G) {
            calibration_reset(c); return 0;
        }
    }
    for (i=0; i<3; ++i) { c->sum_g[i]+=g[i]; c->sum_a[i]+=a[i]; }
    if (++c->count < CALIBRATION_SAMPLES) return 0;
    /* 零偏用于扣零漂，初始重力用于求初始姿态。 */
    for (i=0; i<3; ++i) {
        c->bias_dps[i]=c->sum_g[i]/c->count;
        c->initial_accel[i]=c->sum_a[i]/c->count;
    }
    return 1;
}

/* 两种融合算法共用的姿态几何工具；坐标系为传感器坐标到参考坐标。
 * 静止 +Z 加速度为 +1g，初始 yaw 约定为 0，欧拉角输出单位度。 */
static float attitude_clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* 由静止重力求初始四元数 w,x,y,z；输入非法时返回 0。 */
int attitude_init(float q[4], float *accel_norm, const float a[3])
{
    float norm, roll, pitch, cr, sr, cp, sp;
    if (!imu_finite(a[0]) || !imu_finite(a[1]) || !imu_finite(a[2])) return 0;
    norm = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (!imu_finite(norm) || norm < ACC_NORM_MIN || norm > ACC_NORM_MAX) return 0;
    roll = atan2f(a[1], a[2]);
    pitch = atan2f(-a[0], sqrtf(a[1]*a[1] + a[2]*a[2]));
    cr = cosf(roll*0.5f); sr = sinf(roll*0.5f);
    cp = cosf(pitch*0.5f); sp = sinf(pitch*0.5f);
    q[0] = cr*cp; q[1] = sr*cp;
    q[2] = cr*sp; q[3] = -sr*sp;
    *accel_norm = norm;
    return 1;
}

/* ZYX 欧拉角：R=Rz(yaw)*Ry(pitch)*Rx(roll)。
 * pitch 接近 ±90 度时，roll/yaw 不唯一。 */
void attitude_euler(const float q[4], float e[3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    e[0]=atan2f(2.0f*(w*x+y*z), 1.0f-2.0f*(x*x+y*y))*RAD_TO_DEG;
    e[1]=asinf(attitude_clamp(2.0f*(w*y-z*x), -1.0f, 1.0f))*RAD_TO_DEG;
    e[2]=atan2f(2.0f*(w*z+x*y), 1.0f-2.0f*(y*y+z*z))*RAD_TO_DEG;
}

/* Qt3D Cube 专用欧拉角：R=Ry(Y)*Rx(X)*Rz(Z)，与 ZYX 不可混用。 */
void attitude_cube_euler(const float q[4], float e[3])
{
    float w=q[0], x=q[1], y=q[2], z=q[3];
    float r00=1.0f-2.0f*(y*y+z*z), r02=2.0f*(x*z+w*y);
    float r10=2.0f*(x*y+w*z), r11=1.0f-2.0f*(x*x+z*z);
    float r12=2.0f*(y*z-w*x), r20=2.0f*(x*z-w*y);
    float r22=1.0f-2.0f*(x*x+y*y);
    float cx=sqrtf(r10*r10+r11*r11);
    e[0]=atan2f(-r12,cx)*RAD_TO_DEG;
    if (cx>1e-5f) {
        e[1]=atan2f(r02,r22)*RAD_TO_DEG;
        e[2]=atan2f(r10,r11)*RAD_TO_DEG;
    } else {
        /* X接近±90度时Y/Z不唯一，取Z=0，保留等效姿态。 */
        e[1]=atan2f(-r20,r00)*RAD_TO_DEG;
        e[2]=0;
    }
}

volatile int32 imu_attitude_status = IMU_ATTITUDE_INIT_FAILED;
volatile float imu_roll_deg;
volatile float imu_pitch_deg;
volatile float imu_yaw_deg;
volatile float imu_accel_norm_g;
volatile float imu_calibration_percent;
volatile float imu_accel_g[3];
volatile float imu_gyro_dps[3];
const uint8 imu_attitude_method = AHRS_METHOD;

static Ahrs6 g_attitude;
static ImuCalibration g_calibration;
static uint32 g_last_sample_ticks;
static uint32 g_last_success_ticks;
static uint8 g_have_sample;
static uint8 g_calibrated;
static volatile uint8 g_update_enabled;
static volatile uint8 g_recalibration_requested;
static imu_navigation_sample_t g_navigation_sample;
static uint32 g_navigation_generation;

static void imu_publish_navigation(const float gyro[3], const float accel[3],
                                   const float euler[3], float dt)
{
    g_navigation_sample.roll_deg = euler[0];
    g_navigation_sample.pitch_deg = euler[1];
    g_navigation_sample.yaw_deg = euler[2];
    g_navigation_sample.dt_s = dt;
    for (unsigned i = 0; i < 3; ++i) {
        g_navigation_sample.accel_g[i] = accel[i];
        g_navigation_sample.gyro_dps[i] = gyro[i] - g_calibration.bias_dps[i];
    }
    g_navigation_sample.generation = g_navigation_generation;
    ++g_navigation_sample.sequence;
}

bool imu_attitude_get_navigation_sample(imu_navigation_sample_t *sample)
{
    if (sample == NULL || imu_attitude_status != IMU_ATTITUDE_RUNNING ||
        g_navigation_sample.sequence == 0u) return false;
    *sample = g_navigation_sample;
    return true;
}

static uint32 imu_ticks(void)
{
    return DWT->CYCCNT;
}

static float imu_seconds(uint32 elapsed)
{
    return (float)elapsed / (float)system_clock;
}

static uint32 imu_ms_ticks(uint32 ms)
{
    return (system_clock / 1000u) * ms;
}

static void imu_reg_write(uint8 reg, uint8 value)
{
    IMU660RA_CS(0);
    spi_write_8bit_register(IMU660RA_SPI, reg | IMU660RA_SPI_W, value);
    IMU660RA_CS(1);
    system_delay_ms(1);
}

static uint8 imu_reg_read(uint8 reg)
{
    uint8 data[2];
    IMU660RA_CS(0);
    spi_read_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_R, data, 2);
    IMU660RA_CS(1);
    return data[1];
}

static uint8 imu_read_sample(float gyro_dps[3], float accel_g[3])
{
    uint8 data[13];
    uint8 status = imu_reg_read(IMU_STATUS_REG);
    uint8 i;

    if (status == 0xFFu || (status & 0xC0u) != 0xC0u)
    {
        return 0u;
    }

    IMU660RA_CS(0);
    spi_read_8bit_registers(IMU660RA_SPI,
                            IMU660RA_ACC_ADDRESS | IMU660RA_SPI_R,
                            data,
                            13);
    IMU660RA_CS(1);

    for (i = 0u; i < 3u; ++i)
    {
        int16 acc_raw = (int16)((uint16)data[1u + 2u * i] |
                                ((uint16)data[2u + 2u * i] << 8));
        int16 gyro_raw = (int16)((uint16)data[7u + 2u * i] |
                                 ((uint16)data[8u + 2u * i] << 8));
        accel_g[i] = (float)acc_raw / 4096.0f;
        gyro_dps[i] = (float)gyro_raw / 16.4f;
    }
    return 1u;
}

static uint8 imu_hardware_init(void)
{
    if (imu660ra_init() != 0u)
    {
        return 0u;
    }

    /* Match rt1064_imu_vofa: accelerometer and gyro both at 200 Hz. */
    imu_reg_write(IMU660RA_ACC_CONF, 0xA9u);
    imu_reg_write(IMU660RA_GYR_CONF, 0xA9u);
    imu_reg_write(IMU660RA_ACC_RANGE, 0x02u);
    imu_reg_write(IMU660RA_GYR_RANGE, 0x00u);
    return (imu_reg_read(IMU660RA_ACC_CONF) == 0xA9u &&
            imu_reg_read(IMU660RA_GYR_CONF) == 0xA9u &&
            imu_reg_read(IMU660RA_ACC_RANGE) == 0x02u &&
            imu_reg_read(IMU660RA_GYR_RANGE) == 0x00u) ? 1u : 0u;
}

static void imu_reset_pipeline(void)
{
    ++g_navigation_generation;
    memset(&g_navigation_sample, 0, sizeof(g_navigation_sample));
    memset(&g_attitude, 0, sizeof(g_attitude));
    calibration_reset(&g_calibration);
    g_have_sample = 0u;
    g_calibrated = 0u;
    imu_roll_deg = 0.0f;
    imu_pitch_deg = 0.0f;
    imu_yaw_deg = 0.0f;
    imu_accel_norm_g = 0.0f;
    imu_calibration_percent = 0.0f;
    g_last_sample_ticks = imu_ticks();
    g_last_success_ticks = g_last_sample_ticks;
}

void imu_attitude_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();

    g_update_enabled = 0u;
    imu_reset_pipeline();
    if (imu_hardware_init())
    {
        imu_attitude_status = IMU_ATTITUDE_CALIBRATING;
        g_update_enabled = 1u;
    }
    else
    {
        imu_attitude_status = IMU_ATTITUDE_INIT_FAILED;
    }
}

void imu_attitude_update_5ms(void)
{
    float gyro_dps[3];
    float accel_g[3];
    float gyro_rad_s[3];
    float euler[3];
    float dt;
    uint32 now;
    uint8 i;

    if (!g_update_enabled ||
        imu_attitude_status == IMU_ATTITUDE_BAD_DT ||
        imu_attitude_status == IMU_ATTITUDE_FILTER_ERROR)
    {
        return;
    }

    now = imu_ticks();
    if (!imu_read_sample(gyro_dps, accel_g))
    {
        if ((uint32)(now - g_last_success_ticks) > imu_ms_ticks(IMU_TIMEOUT_MS))
        {
            imu_attitude_status = IMU_ATTITUDE_TIMEOUT;
        }
        return;
    }

    g_last_success_ticks = now;
    dt = imu_seconds((uint32)(now - g_last_sample_ticks));
    g_last_sample_ticks = now;
    for (i = 0u; i < 3u; ++i)
    {
        imu_gyro_dps[i] = gyro_dps[i];
        imu_accel_g[i] = accel_g[i];
    }
    imu_accel_norm_g = sqrtf(accel_g[0] * accel_g[0] +
                             accel_g[1] * accel_g[1] +
                             accel_g[2] * accel_g[2]);

    if (!g_have_sample)
    {
        g_have_sample = 1u;
        return;
    }
    if (dt <= 0.0f || dt > MAX_SAMPLE_DT)
    {
        imu_attitude_status = IMU_ATTITUDE_BAD_DT;
        return;
    }

    if (!g_calibrated)
    {
        if (calibration_push(&g_calibration, gyro_dps, accel_g))
        {
            g_calibrated = (uint8)ahrs6_init(&g_attitude,
                                             g_calibration.initial_accel);
            if (!g_calibrated)
            {
                imu_attitude_status = IMU_ATTITUDE_FILTER_ERROR;
                return;
            }
            ahrs6_euler(&g_attitude, euler);
            imu_roll_deg = euler[0];
            imu_pitch_deg = euler[1];
            imu_yaw_deg = euler[2];
            imu_publish_navigation(gyro_dps, accel_g, euler, dt);
            imu_attitude_status = IMU_ATTITUDE_RUNNING;
        }
        imu_calibration_percent =
            100.0f * (float)g_calibration.count / (float)CALIBRATION_SAMPLES;
        return;
    }

    for (i = 0u; i < 3u; ++i)
    {
        gyro_rad_s[i] = (gyro_dps[i] - g_calibration.bias_dps[i]) * DEG_TO_RAD;
    }
    if (!ahrs6_update(&g_attitude, gyro_rad_s, accel_g, dt))
    {
        imu_attitude_status = IMU_ATTITUDE_FILTER_ERROR;
        return;
    }

    ahrs6_euler(&g_attitude, euler);
    imu_roll_deg = euler[0];
    imu_pitch_deg = euler[1];
    imu_yaw_deg = euler[2];
    imu_accel_norm_g = g_attitude.accel_norm;
    imu_publish_navigation(gyro_dps, accel_g, euler, dt);
    imu_attitude_status = IMU_ATTITUDE_RUNNING;
}

void imu_attitude_request_recalibration(void)
{
    g_recalibration_requested = 1u;
}

void imu_attitude_service(void)
{
    uint32 primask;
    uint8 need_hardware_init;

    if (!g_recalibration_requested)
    {
        return;
    }

    primask = interrupt_global_disable();
    g_recalibration_requested = 0u;
    g_update_enabled = 0u;
    need_hardware_init = (imu_attitude_status == IMU_ATTITUDE_INIT_FAILED) ? 1u : 0u;
    interrupt_global_enable(primask);

    if (need_hardware_init && !imu_hardware_init())
    {
        imu_attitude_status = IMU_ATTITUDE_INIT_FAILED;
        return;
    }

    primask = interrupt_global_disable();
    imu_reset_pipeline();
    imu_attitude_status = IMU_ATTITUDE_CALIBRATING;
    g_update_enabled = 1u;
    interrupt_global_enable(primask);
}
