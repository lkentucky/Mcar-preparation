#include "app_wifi_telemetry.h"

#include "config.h"
#include "imu_attitude.h"
#include "imu_wifi_spi.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "fsl_common.h"

static uint32 g_last_wifi_tx_ticks;

void app_wifi_telemetry_init(void)
{
    (void)imu_wifi_init();
    g_last_wifi_tx_ticks = DWT->CYCCNT;
}

void app_wifi_telemetry_service(void)
{
    uint32 now = DWT->CYCCNT;
    uint32 period_ticks = (system_clock / 1000u) * IMU_WIFI_PERIOD_MS;
    float angles[3];
    uint32 primask;

    if ((uint32)(now - g_last_wifi_tx_ticks) < period_ticks)
    {
        return;
    }
    g_last_wifi_tx_ticks = now;

    primask = interrupt_global_disable();
    angles[0] = imu_roll_deg;
    angles[1] = imu_pitch_deg;
    angles[2] = imu_yaw_deg;
    interrupt_global_enable(primask);
    (void)imu_wifi_send_angles(angles);
}
