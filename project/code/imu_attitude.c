#include "imu_attitude.h"

#include "ahrs6.h"
#include "calibration.h"
#include "config.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

#include <math.h>
#include <string.h>

#define IMU_STATUS_REG 0x03u

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
